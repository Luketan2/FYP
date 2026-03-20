import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import csv
import time
import collections
from datetime import datetime

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


class App(tk.Tk):

    _UNIT_CONVERSIONS = {
        "N":   (1.0,       "N"),
        "kN":  (1e-3,      "kN"),
        "lbf": (0.224809,  "lbf"),
        "kgf": (0.101972,  "kgf"),
    }

    def __init__(self):
        super().__init__()
        self.title("Direct Shear Strain Device")
        self.geometry("860x640")
        self.resizable(True, True)

        self.ser = None

        # Connection
        self.port_var   = tk.StringVar()
        self.status_var = tk.StringVar(value="Not connected")

        # Stepper (shared state, used by both stepper tab and shear test)
        self.rpm_var      = tk.StringVar(value="10")
        self.spr_var      = tk.StringVar(value="200")
        self.dir_state    = 0

        # Caliper
        self.caliper_var  = tk.StringVar(value="--")
        self.cal_value    = 0.0   # latest numeric value in mm

        # Force
        self.force_var    = tk.StringVar(value="--")
        self.force_value  = 0.0   # latest numeric value in N (raw, unfiltered)

        # Force display / filtering
        self.force_unit_var       = tk.StringVar(value="N")
        self.filter_ma_var        = tk.BooleanVar(value=False)
        self.filter_ma_window_var = tk.StringVar(value="10")
        self.force_history        = []            # list of (elapsed_s, raw_N)
        self._force_t0            = None          # epoch of first reading
        self._ma_buf              = collections.deque()

        # Shear test
        self.test_running    = False
        self.test_data       = []   # list of (displacement_mm, force_N)
        self.test_start_disp = 0.0

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._poll_serial)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _build_ui(self):
        # ── Connection bar ──────────────────────────────────────────────
        conn = ttk.Frame(self)
        conn.pack(fill="x", padx=10, pady=(8, 2))

        ttk.Label(conn, text="Port:").pack(side="left")
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var,
                                       width=14, state="readonly")
        self.port_combo.pack(side="left", padx=4)
        ttk.Button(conn, text="Refresh",    command=self._refresh_ports).pack(side="left", padx=2)
        ttk.Button(conn, text="Connect",    command=self.connect).pack(side="left", padx=2)
        ttk.Button(conn, text="Disconnect", command=self.disconnect).pack(side="left", padx=2)
        ttk.Label(conn, textvariable=self.status_var,
                  foreground="navy").pack(side="left", padx=12)

        # ── Tabs ────────────────────────────────────────────────────────
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=10, pady=6)

        self._build_shear_tab(nb)
        self._build_stepper_tab(nb)
        self._build_caliper_tab(nb)
        self._build_force_tab(nb)

    # ── Tab 1: Direct Shear Test ───────────────────────────────────────
    def _build_shear_tab(self, nb):
        frame = ttk.Frame(nb)
        nb.add(frame, text="  Direct Shear Test  ")

        # Controls row
        ctrl = ttk.LabelFrame(frame, text="Test Setup")
        ctrl.pack(fill="x", padx=8, pady=(8, 4))

        ttk.Label(ctrl, text="Shear Rate (RPM):").grid(row=0, column=0, padx=8, pady=6, sticky="w")
        self.shear_rpm_var = tk.StringVar(value="10")
        ttk.Entry(ctrl, textvariable=self.shear_rpm_var, width=8).grid(row=0, column=1, sticky="w")

        ttk.Label(ctrl, text="Direction:").grid(row=0, column=2, padx=(16, 4), sticky="w")
        self.shear_dir_var = tk.StringVar(value="Forward")
        ttk.Combobox(ctrl, textvariable=self.shear_dir_var,
                     values=["Forward", "Reverse"],
                     width=9, state="readonly").grid(row=0, column=3, sticky="w")

        self.shear_start_btn = ttk.Button(ctrl, text="▶  START TEST",
                                          command=self._start_shear_test)
        self.shear_start_btn.grid(row=0, column=4, padx=16)

        self.shear_stop_btn = ttk.Button(ctrl, text="■  STOP",
                                         command=self._stop_shear_test,
                                         state="disabled")
        self.shear_stop_btn.grid(row=0, column=5, padx=4)

        ttk.Button(ctrl, text="Export CSV",
                   command=self._export_csv).grid(row=0, column=6, padx=8)

        ttk.Button(ctrl, text="Clear Data",
                   command=self._clear_data).grid(row=0, column=7, padx=4)

        # Live readings bar
        live = ttk.LabelFrame(frame, text="Live Readings")
        live.pack(fill="x", padx=8, pady=4)

        ttk.Label(live, text="Displacement:").grid(row=0, column=0, padx=10, pady=6, sticky="w")
        ttk.Label(live, textvariable=self.caliper_var,
                  font=("Segoe UI", 14, "bold"),
                  foreground="#0055cc", width=14).grid(row=0, column=1, sticky="w")

        ttk.Label(live, text="Shear Force:").grid(row=0, column=2, padx=(20, 10), sticky="w")
        ttk.Label(live, textvariable=self.force_var,
                  font=("Segoe UI", 14, "bold"),
                  foreground="#cc2200", width=14).grid(row=0, column=3, sticky="w")

        self.point_count_var = tk.StringVar(value="Points: 0")
        ttk.Label(live, textvariable=self.point_count_var,
                  foreground="gray").grid(row=0, column=4, padx=20, sticky="w")

        # Plot
        if HAS_MPL:
            fig_frame = ttk.LabelFrame(frame, text="Force vs Displacement")
            fig_frame.pack(fill="both", expand=True, padx=8, pady=4)

            self.fig = Figure(figsize=(7, 3.2), dpi=92)
            self.ax  = self.fig.add_subplot(111)
            self.ax.set_xlabel("Displacement (mm)")
            self.ax.set_ylabel("Shear Force (N)")
            self.ax.grid(True, alpha=0.3)
            self.line, = self.ax.plot([], [], "b-o", markersize=3, linewidth=1.2)
            self.fig.tight_layout(pad=1.5)

            self.canvas = FigureCanvasTkAgg(self.fig, master=fig_frame)
            self.canvas.get_tk_widget().pack(fill="both", expand=True)
        else:
            ttk.Label(frame,
                      text="Install matplotlib for live plotting:  pip install matplotlib",
                      foreground="gray").pack(pady=8)
            self.canvas = None

    # ── Tab 2: Stepper Test ────────────────────────────────────────────
    def _build_stepper_tab(self, nb):
        frame = ttk.Frame(nb)
        nb.add(frame, text="  Stepper Test  ")

        ctrl = ttk.LabelFrame(frame, text="Stepper Control")
        ctrl.pack(fill="x", padx=16, pady=16)

        ttk.Label(ctrl, text="RPM:").grid(row=0, column=0, padx=10, pady=8, sticky="w")
        ttk.Entry(ctrl, textvariable=self.rpm_var, width=10).grid(row=0, column=1, sticky="w")
        ttk.Button(ctrl, text="Apply RPM", command=self._apply_rpm).grid(row=0, column=2, padx=8)

        ttk.Label(ctrl, text="Steps / Rev:").grid(row=1, column=0, padx=10, pady=6, sticky="w")
        ttk.Entry(ctrl, textvariable=self.spr_var, width=10).grid(row=1, column=1, sticky="w")
        ttk.Button(ctrl, text="Apply SPR", command=self._apply_spr).grid(row=1, column=2, padx=8)

        btns = ttk.Frame(ctrl)
        btns.grid(row=2, column=0, columnspan=4, padx=8, pady=10, sticky="w")

        self.step_dir_label = ttk.Label(btns, text="DIR: LOW (0)", width=14)
        self.step_dir_label.pack(side="left", padx=4)
        ttk.Button(btns, text="Toggle DIR", command=self._toggle_dir).pack(side="left", padx=6)
        ttk.Button(btns, text="RUN",        command=self._run_stepper).pack(side="left", padx=6)
        ttk.Button(btns, text="STOP",       command=self._stop_stepper).pack(side="left", padx=6)

        ttk.Label(frame,
                  text="Note: stepper uses STEP=GPIO16, DIR=GPIO17  (safe, non-strap pins)",
                  foreground="gray").pack(anchor="w", padx=16, pady=6)

    # ── Tab 3: Caliper Test ────────────────────────────────────────────
    def _build_caliper_tab(self, nb):
        frame = ttk.Frame(nb)
        nb.add(frame, text="  Caliper Test  ")

        cal = ttk.LabelFrame(frame, text="Digital Caliper Readout")
        cal.pack(fill="x", padx=16, pady=16)

        ttk.Label(cal, text="Current Reading:").pack(anchor="w", padx=10, pady=(8, 2))
        ttk.Label(cal, textvariable=self.caliper_var,
                  font=("Segoe UI", 32, "bold"),
                  foreground="#0055cc").pack(anchor="w", padx=10, pady=8)
        ttk.Label(cal,
                  text="Caliper: CLK=GPIO27, DATA=GPIO26\n"
                       "Reading updates automatically ~10 Hz when caliper is active.",
                  foreground="gray").pack(anchor="w", padx=10, pady=6)

    # ── Tab 4: Force Gauge Test ────────────────────────────────────────
    def _build_force_tab(self, nb):
        frame = ttk.Frame(nb)
        nb.add(frame, text="  Force Gauge Test  ")

        fg = ttk.LabelFrame(frame, text="Load Cell  (HX711)")
        fg.pack(fill="x", padx=16, pady=(16, 4))

        # ── Current reading ──────────────────────────────────────────
        ttk.Label(fg, text="Current Force:").pack(anchor="w", padx=10, pady=(8, 2))
        ttk.Label(fg, textvariable=self.force_var,
                  font=("Segoe UI", 32, "bold"),
                  foreground="#cc2200").pack(anchor="w", padx=10, pady=(0, 6))

        # ── Units row ────────────────────────────────────────────────
        unit_row = ttk.Frame(fg)
        unit_row.pack(anchor="w", padx=10, pady=(0, 4))
        ttk.Label(unit_row, text="Display Unit:").pack(side="left")
        unit_cb = ttk.Combobox(unit_row, textvariable=self.force_unit_var,
                               values=["N", "kN", "lbf", "kgf"],
                               width=6, state="readonly")
        unit_cb.pack(side="left", padx=6)
        unit_cb.bind("<<ComboboxSelected>>", lambda _e: self._refresh_force_display())

        # ── Filters ──────────────────────────────────────────────────
        filt_lf = ttk.LabelFrame(fg, text="Filters")
        filt_lf.pack(fill="x", padx=10, pady=4)

        ma_row = ttk.Frame(filt_lf)
        ma_row.pack(anchor="w", padx=8, pady=4)
        ttk.Checkbutton(ma_row, text="Moving Average",
                        variable=self.filter_ma_var,
                        command=self._on_filter_change).pack(side="left")
        ttk.Label(ma_row, text="Window (samples):").pack(side="left", padx=(12, 4))
        ttk.Entry(ma_row, textvariable=self.filter_ma_window_var, width=6).pack(side="left")
        ttk.Label(ma_row,
                  text="  — smooths noise; larger window = more smoothing",
                  foreground="gray").pack(side="left", padx=6)

        # ── Action buttons ───────────────────────────────────────────
        btns = ttk.Frame(fg)
        btns.pack(anchor="w", padx=10, pady=4)
        ttk.Button(btns, text="Tare (Zero)",
                   command=self._tare).pack(side="left", padx=4)
        ttk.Button(btns, text="Clear History",
                   command=self._clear_force_history).pack(side="left", padx=4)

        # ── Calibration ──────────────────────────────────────────────
        sep = ttk.Separator(fg, orient="horizontal")
        sep.pack(fill="x", padx=10, pady=8)

        cal_row = ttk.Frame(fg)
        cal_row.pack(anchor="w", padx=10, pady=4)
        ttk.Label(cal_row, text="Calibration Factor  (raw counts / N):").pack(side="left")
        self.cal_factor_var = tk.StringVar(value="1.0")
        ttk.Entry(cal_row, textvariable=self.cal_factor_var, width=12).pack(side="left", padx=6)
        ttk.Button(cal_row, text="Set Factor",
                   command=self._set_cal_factor).pack(side="left", padx=4)

        ttk.Label(fg,
                  text="To calibrate: place a known weight, note the raw reading,\n"
                       "then set factor = raw_reading / weight_in_newtons.\n"
                       "HX711: DOUT=GPIO32, SCK=GPIO33",
                  foreground="gray").pack(anchor="w", padx=10, pady=(4, 10))

        # ── Force vs Time plot ───────────────────────────────────────
        if HAS_MPL:
            plot_lf = ttk.LabelFrame(frame, text="Force vs Time")
            plot_lf.pack(fill="both", expand=True, padx=16, pady=(4, 8))

            self.force_fig = Figure(figsize=(7, 2.8), dpi=92)
            self.force_ax  = self.force_fig.add_subplot(111)
            self.force_ax.set_xlabel("Time (s)")
            self.force_ax.set_ylabel("Force (N)")
            self.force_ax.grid(True, alpha=0.3)
            self.force_line_raw, = self.force_ax.plot([], [], color="#cccccc",
                                                      linewidth=1.0, label="Raw")
            self.force_line_filt, = self.force_ax.plot([], [], color="#cc2200",
                                                       linewidth=1.5, label="Filtered")
            self.force_ax.legend(loc="upper right", fontsize=8)
            self.force_fig.tight_layout(pad=1.5)

            self.force_canvas = FigureCanvasTkAgg(self.force_fig, master=plot_lf)
            self.force_canvas.get_tk_widget().pack(fill="both", expand=True)
        else:
            self.force_canvas = None

    # ------------------------------------------------------------------
    # Serial helpers
    # ------------------------------------------------------------------
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Error", "Select a port first.")
            return
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.05)
            self.status_var.set(f"Connected: {port}")
            self.send(f"RPM {self.rpm_var.get()}")
            self.send(f"DIR {self.dir_state}")
            self.send("STOP")
        except Exception as e:
            messagebox.showerror("Connection failed", str(e))

    def disconnect(self):
        if self.test_running:
            self._stop_shear_test()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.status_var.set("Not connected")

    def send(self, line: str):
        if not self.ser or not self.ser.is_open:
            return
        try:
            self.ser.write((line.strip() + "\n").encode("utf-8"))
        except Exception as e:
            messagebox.showerror("Serial error", str(e))

    def _poll_serial(self):
        try:
            if self.ser and self.ser.is_open:
                while self.ser.in_waiting > 0:
                    raw = self.ser.readline()
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue

                    if line.startswith("CAL "):
                        parts = line.split()
                        if len(parts) == 3:
                            try:
                                self.cal_value = float(parts[1])
                                self.caliper_var.set(f"{parts[1]} {parts[2]}")
                            except ValueError:
                                pass

                    elif line.startswith("FORCE "):
                        parts = line.split()
                        if len(parts) == 3:
                            try:
                                raw_N = float(parts[1])
                                self.force_value = raw_N

                                # Record timestamp and raw value
                                now = time.time()
                                if self._force_t0 is None:
                                    self._force_t0 = now
                                elapsed = now - self._force_t0
                                self.force_history.append((elapsed, raw_N))

                                # Apply moving average for live display
                                display_N = self._apply_ma(raw_N)

                                # Convert to selected unit and update label
                                factor, unit = self._UNIT_CONVERSIONS[self.force_unit_var.get()]
                                self.force_var.set(f"{display_N * factor:.3f} {unit}")

                                # Throttle plot redraws to every 5 readings
                                if len(self.force_history) % 5 == 0:
                                    self._update_force_plot()

                                if self.test_running:
                                    self._record_point()
                            except ValueError:
                                pass
        except Exception:
            pass

        self.after(50, self._poll_serial)

    # ------------------------------------------------------------------
    # Force tab helpers
    # ------------------------------------------------------------------
    def _apply_ma(self, raw_N: float) -> float:
        """Apply moving average to raw_N and return filtered value."""
        if self.filter_ma_var.get():
            try:
                win = max(1, int(self.filter_ma_window_var.get()))
            except ValueError:
                win = 10
            self._ma_buf.append(raw_N)
            while len(self._ma_buf) > win:
                self._ma_buf.popleft()
            return sum(self._ma_buf) / len(self._ma_buf)
        else:
            self._ma_buf.clear()
            return raw_N

    def _compute_ma_series(self, values):
        """Return moving-average-filtered series from a list of raw values."""
        try:
            win = max(1, int(self.filter_ma_window_var.get()))
        except ValueError:
            win = 10
        result = []
        buf = collections.deque()
        for v in values:
            buf.append(v)
            if len(buf) > win:
                buf.popleft()
            result.append(sum(buf) / len(buf))
        return result

    def _update_force_plot(self):
        if not (hasattr(self, "force_canvas") and self.force_canvas):
            return
        if len(self.force_history) < 2:
            return

        ts   = [p[0] for p in self.force_history]
        raws = [p[1] for p in self.force_history]

        factor, unit = self._UNIT_CONVERSIONS[self.force_unit_var.get()]
        raw_ys = [v * factor for v in raws]

        self.force_line_raw.set_data(ts, raw_ys)

        if self.filter_ma_var.get():
            filt_ys = [v * factor for v in self._compute_ma_series(raws)]
            self.force_line_filt.set_data(ts, filt_ys)
            self.force_line_raw.set_alpha(0.4)
        else:
            self.force_line_filt.set_data([], [])
            self.force_line_raw.set_alpha(1.0)

        self.force_ax.set_ylabel(f"Force ({unit})")
        self.force_ax.relim()
        self.force_ax.autoscale_view()
        self.force_canvas.draw_idle()

    def _refresh_force_display(self):
        """Re-render current reading label and plot after unit/filter change."""
        factor, unit = self._UNIT_CONVERSIONS[self.force_unit_var.get()]
        if self.filter_ma_var.get() and self._ma_buf:
            val = (sum(self._ma_buf) / len(self._ma_buf)) * factor
        else:
            val = self.force_value * factor
        self.force_var.set(f"{val:.3f} {unit}")
        self._update_force_plot()

    def _on_filter_change(self):
        """Called when a filter checkbox is toggled."""
        self._ma_buf.clear()
        self._refresh_force_display()

    def _clear_force_history(self):
        self.force_history.clear()
        self._force_t0 = None
        self._ma_buf.clear()
        if hasattr(self, "force_canvas") and self.force_canvas:
            unit = self.force_unit_var.get()
            self.force_ax.cla()
            self.force_ax.set_xlabel("Time (s)")
            self.force_ax.set_ylabel(f"Force ({unit})")
            self.force_ax.grid(True, alpha=0.3)
            self.force_line_raw,  = self.force_ax.plot([], [], color="#cccccc",
                                                       linewidth=1.0, label="Raw")
            self.force_line_filt, = self.force_ax.plot([], [], color="#cc2200",
                                                       linewidth=1.5, label="Filtered")
            self.force_ax.legend(loc="upper right", fontsize=8)
            self.force_canvas.draw()

    # ------------------------------------------------------------------
    # Stepper controls
    # ------------------------------------------------------------------
    def _apply_rpm(self):
        try:
            rpm = abs(float(self.rpm_var.get()))
        except Exception:
            messagebox.showerror("Invalid RPM", "Enter a valid number.")
            return
        self.send(f"RPM {rpm}")

    def _apply_spr(self):
        try:
            spr = float(self.spr_var.get())
        except Exception:
            messagebox.showerror("Invalid SPR", "Enter a valid number.")
            return
        self.send(f"SPR {spr}")

    def _toggle_dir(self):
        self.dir_state = 1 - self.dir_state
        self.send(f"DIR {self.dir_state}")
        self.step_dir_label.config(
            text=f"DIR: {'HIGH (1)' if self.dir_state else 'LOW (0)'}")

    def _run_stepper(self):
        self._apply_rpm()
        self.send("RUN 1")

    def _stop_stepper(self):
        self.send("STOP")

    # ------------------------------------------------------------------
    # Force gauge controls
    # ------------------------------------------------------------------
    def _tare(self):
        self.send("TARE")

    def _set_cal_factor(self):
        try:
            f = float(self.cal_factor_var.get())
            if f == 0:
                raise ValueError
        except Exception:
            messagebox.showerror("Invalid", "Enter a non-zero calibration factor.")
            return
        self.send(f"CALFACTOR {f}")

    # ------------------------------------------------------------------
    # Shear test logic
    # ------------------------------------------------------------------
    def _start_shear_test(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Error", "Not connected.")
            return

        self.test_data       = []
        self.test_running    = True
        self.test_start_disp = self.cal_value
        self.point_count_var.set("Points: 0")

        if self.canvas:
            self.ax.cla()
            self.ax.set_xlabel("Displacement (mm)")
            self.ax.set_ylabel("Shear Force (N)")
            self.ax.grid(True, alpha=0.3)
            self.line, = self.ax.plot([], [], "b-o", markersize=3, linewidth=1.2)
            self.canvas.draw()

        dir_val = 0 if self.shear_dir_var.get() == "Forward" else 1
        self.send(f"DIR {dir_val}")

        try:
            rpm = abs(float(self.shear_rpm_var.get()))
        except Exception:
            rpm = 10.0
        self.send(f"RPM {rpm}")
        self.send("RUN 1")

        self.shear_start_btn.config(state="disabled")
        self.shear_stop_btn.config(state="normal")

    def _stop_shear_test(self):
        self.test_running = False
        self.send("STOP")
        self.shear_start_btn.config(state="normal")
        self.shear_stop_btn.config(state="disabled")

    def _record_point(self):
        disp  = self.cal_value - self.test_start_disp
        force = self.force_value
        self.test_data.append((disp, force))
        self.point_count_var.set(f"Points: {len(self.test_data)}")

        if self.canvas and len(self.test_data) > 1:
            xs = [d[0] for d in self.test_data]
            ys = [d[1] for d in self.test_data]
            self.line.set_data(xs, ys)
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw_idle()

    def _clear_data(self):
        self.test_data = []
        self.point_count_var.set("Points: 0")
        if self.canvas:
            self.ax.cla()
            self.ax.set_xlabel("Displacement (mm)")
            self.ax.set_ylabel("Shear Force (N)")
            self.ax.grid(True, alpha=0.3)
            self.line, = self.ax.plot([], [], "b-o", markersize=3, linewidth=1.2)
            self.canvas.draw()

    def _export_csv(self):
        if not self.test_data:
            messagebox.showinfo("No data", "Run a test first.")
            return
        fname = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=f"shear_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        if not fname:
            return
        with open(fname, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Displacement (mm)", "Shear Force (N)"])
            writer.writerows(self.test_data)
        messagebox.showinfo("Exported", f"Saved {len(self.test_data)} rows to:\n{fname}")


if __name__ == "__main__":
    App().mainloop()
