from flask import Flask, render_template, request, jsonify, Response
from flask_socketio import SocketIO
import serial
import serial.tools.list_ports
import threading
import time
import collections
import csv
import io
from datetime import datetime

app = Flask(__name__)
app.config['SECRET_KEY'] = 'shear_dev_key_42'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

UNIT_CONV = {
    'N':   (1.0,       'N'),
    'kN':  (1e-3,      'kN'),
    'lbf': (0.224809,  'lbf'),
    'kgf': (0.101972,  'kgf'),
}

# ── Global state ──────────────────────────────────────────────────────────────
g = {
    'ser':          None,
    'status':       'Not connected',

    # Stepper
    'rpm':          10.0,
    'spr':          200.0,
    'dir_state':    0,

    # Caliper (still received / displayed on caliper tab)
    'cal_value':    0.0,
    'caliper_str':  '--',

    # Force
    'force_value':  0.0,
    'force_str':    '--',
    'force_unit':   'N',
    'force_history': [],       # list of [elapsed_s, raw_N]
    'force_t0':     None,
    'ma_enabled':   False,
    'ma_window':    10,
    'ma_buf':       collections.deque(),

    # Shear test — displacement derived from steps, NOT caliper
    'test_running':     False,
    'test_data':        [],    # list of [displacement_mm, force_N]
    'test_start_time':  None,
    'test_rpm':         10.0,
    'test_spr':         200.0,
    'steps_per_mm':     100.0,
}


# ── Helpers ───────────────────────────────────────────────────────────────────
def _apply_ma(raw_N: float) -> float:
    if g['ma_enabled']:
        win = max(1, g['ma_window'])
        g['ma_buf'].append(raw_N)
        while len(g['ma_buf']) > win:
            g['ma_buf'].popleft()
        return sum(g['ma_buf']) / len(g['ma_buf'])
    g['ma_buf'].clear()
    return raw_N


def _serial_send(line: str):
    ser = g['ser']
    if ser and ser.is_open:
        try:
            ser.write((line.strip() + '\n').encode('utf-8'))
        except Exception as e:
            socketio.emit('error', {'msg': str(e)})


def _record_shear_point():
    """Compute displacement from stepper steps (time × step_rate / steps_per_mm)."""
    if not g['test_running'] or g['test_start_time'] is None:
        return
    elapsed   = time.time() - g['test_start_time']
    step_rate = g['test_rpm'] / 60.0 * g['test_spr']   # steps per second
    disp_mm   = step_rate * elapsed / g['steps_per_mm']
    force     = g['force_value']
    g['test_data'].append([disp_mm, force])
    socketio.emit('test_point', {
        'displacement': round(disp_mm, 4),
        'force':        round(force, 4),
        'count':        len(g['test_data']),
    })


# ── Serial polling thread ─────────────────────────────────────────────────────
_poll_active = True

def _poll_loop():
    while _poll_active:
        try:
            ser = g['ser']
            if ser and ser.is_open:
                while ser.in_waiting:
                    raw  = ser.readline()
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    if line.startswith('CAL '):
                        parts = line.split()
                        if len(parts) == 3:
                            try:
                                g['cal_value']   = float(parts[1])
                                g['caliper_str'] = f"{parts[1]} {parts[2]}"
                                socketio.emit('caliper', {
                                    'value': g['cal_value'],
                                    'str':   g['caliper_str'],
                                })
                            except ValueError:
                                pass

                    elif line.startswith('FORCE '):
                        parts = line.split()
                        if len(parts) == 3:
                            try:
                                raw_N = float(parts[1])
                                g['force_value'] = raw_N

                                now = time.time()
                                if g['force_t0'] is None:
                                    g['force_t0'] = now
                                elapsed = now - g['force_t0']
                                g['force_history'].append([elapsed, raw_N])

                                disp_N = _apply_ma(raw_N)
                                factor, unit = UNIT_CONV[g['force_unit']]
                                g['force_str'] = f"{disp_N * factor:.3f} {unit}"

                                socketio.emit('force', {
                                    'raw':     raw_N,
                                    'display': round(disp_N * factor, 4),
                                    'unit':    unit,
                                    'elapsed': round(elapsed, 3),
                                    'str':     g['force_str'],
                                })

                                if g['test_running']:
                                    _record_shear_point()
                            except ValueError:
                                pass
        except Exception:
            pass
        time.sleep(0.05)


# ── Flask routes ──────────────────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/ports')
def api_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return jsonify({'ports': ports})


@app.route('/api/connect', methods=['POST'])
def api_connect():
    data = request.json or {}
    port = data.get('port', '').strip()
    if not port:
        return jsonify({'ok': False, 'error': 'No port selected'})
    try:
        if g['ser'] and g['ser'].is_open:
            g['ser'].close()
        g['ser'] = serial.Serial(port, 115200, timeout=0.05)
        g['status'] = f'Connected: {port}'
        _serial_send(f"RPM {g['rpm']}")
        _serial_send(f"DIR {g['dir_state']}")
        _serial_send('STOP')
        socketio.emit('status', {'status': g['status']})
        return jsonify({'ok': True, 'status': g['status']})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)})


@app.route('/api/disconnect', methods=['POST'])
def api_disconnect():
    if g['test_running']:
        g['test_running'] = False
        _serial_send('STOP')
    try:
        if g['ser']:
            g['ser'].close()
    except Exception:
        pass
    g['ser']    = None
    g['status'] = 'Not connected'
    socketio.emit('status', {'status': g['status']})
    return jsonify({'ok': True})


@app.route('/api/stepper/rpm', methods=['POST'])
def api_set_rpm():
    try:
        g['rpm'] = abs(float((request.json or {}).get('rpm', 10)))
        _serial_send(f"RPM {g['rpm']}")
        return jsonify({'ok': True})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)})


@app.route('/api/stepper/spr', methods=['POST'])
def api_set_spr():
    try:
        g['spr'] = float((request.json or {}).get('spr', 200))
        _serial_send(f"SPR {g['spr']}")
        return jsonify({'ok': True})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)})


@app.route('/api/stepper/dir', methods=['POST'])
def api_toggle_dir():
    g['dir_state'] = 1 - g['dir_state']
    _serial_send(f"DIR {g['dir_state']}")
    socketio.emit('dir', {'dir': g['dir_state']})
    return jsonify({'ok': True, 'dir': g['dir_state']})


@app.route('/api/stepper/run', methods=['POST'])
def api_run():
    _serial_send(f"RPM {g['rpm']}")
    _serial_send('RUN 1')
    return jsonify({'ok': True})


@app.route('/api/stepper/stop', methods=['POST'])
def api_stop():
    _serial_send('STOP')
    return jsonify({'ok': True})


@app.route('/api/force/tare', methods=['POST'])
def api_tare():
    _serial_send('TARE')
    return jsonify({'ok': True})


@app.route('/api/force/calfactor', methods=['POST'])
def api_calfactor():
    try:
        f = float((request.json or {}).get('factor', 1))
        if f == 0:
            raise ValueError
        _serial_send(f"CALFACTOR {f}")
        return jsonify({'ok': True})
    except Exception:
        return jsonify({'ok': False, 'error': 'Invalid factor'})


@app.route('/api/force/settings', methods=['POST'])
def api_force_settings():
    data = request.json or {}
    g['force_unit'] = data.get('unit', 'N')
    g['ma_enabled'] = bool(data.get('ma_enabled', False))
    try:
        g['ma_window'] = max(1, int(data.get('ma_window', 10)))
    except Exception:
        g['ma_window'] = 10
    g['ma_buf'].clear()
    return jsonify({'ok': True})


@app.route('/api/force/clear', methods=['POST'])
def api_force_clear():
    g['force_history'].clear()
    g['force_t0'] = None
    g['ma_buf'].clear()
    return jsonify({'ok': True})


@app.route('/api/shear/start', methods=['POST'])
def api_shear_start():
    if not g['ser'] or not g['ser'].is_open:
        return jsonify({'ok': False, 'error': 'Not connected'})
    data = request.json or {}

    try:
        rpm = abs(float(data.get('rpm', 10)))
    except Exception:
        rpm = 10.0
    try:
        steps_per_mm = float(data.get('steps_per_mm', 100))
        if steps_per_mm <= 0:
            raise ValueError
    except Exception:
        steps_per_mm = 100.0

    dir_val = 0 if data.get('direction', 'Forward') == 'Forward' else 1

    g['test_data']       = []
    g['test_running']    = True
    g['test_start_time'] = time.time()
    g['test_rpm']        = rpm
    g['test_spr']        = g['spr']
    g['steps_per_mm']    = steps_per_mm

    _serial_send(f"DIR {dir_val}")
    _serial_send(f"RPM {rpm}")
    _serial_send('RUN 1')

    socketio.emit('test_status', {'running': True})
    return jsonify({'ok': True})


@app.route('/api/shear/stop', methods=['POST'])
def api_shear_stop():
    g['test_running'] = False
    _serial_send('STOP')
    socketio.emit('test_status', {'running': False})
    return jsonify({'ok': True})


@app.route('/api/shear/clear', methods=['POST'])
def api_shear_clear():
    g['test_data'] = []
    socketio.emit('test_clear', {})
    return jsonify({'ok': True})


@app.route('/api/shear/export')
def api_shear_export():
    if not g['test_data']:
        return Response('No data', status=400)
    out = io.StringIO()
    w   = csv.writer(out)
    w.writerow(['Displacement (mm)', 'Shear Force (N)'])
    w.writerows(g['test_data'])
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    return Response(
        out.getvalue(),
        mimetype='text/csv',
        headers={'Content-Disposition': f'attachment; filename=shear_test_{ts}.csv'},
    )


if __name__ == '__main__':
    import webbrowser
    t = threading.Thread(target=_poll_loop, daemon=True)
    t.start()
    threading.Timer(1.0, lambda: webbrowser.open('http://127.0.0.1:5000')).start()
    socketio.run(app, host='127.0.0.1', port=5000, debug=False, allow_unsafe_werkzeug=True)
