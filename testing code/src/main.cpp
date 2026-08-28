
#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>

// =====================================================================
// Pin assignments  — all safe GPIOs, no strap / flash / PSRAM pins
//
//  AVOID: 0 (boot), 2 (strap), 6-11 (internal flash), 12 (MTDI strap),
//         15 (MTDO strap)
// =====================================================================

// Stepper
static const int PIN_STEP = 16;
static const int PIN_DIR  = 17;
static const int PIN_EN   = 4;   // TMC5160 enable (active LOW)
static const int PIN_CS   = 22;  // TMC5160 SPI chip select (GPIO22, no boot conflict)

// TMC5160 SPI uses ESP32 VSPI bus defaults:
//   SCK  = GPIO18,  MISO = GPIO19,  MOSI = GPIO23
// These are wired directly to the driver SDI/SDO/SCK pins.

// TMC5160 Pro sense resistor value (Mellow Fly Pro V1.5 = 0.075 Ω)
static const float R_SENSE = 0.075f;

// Caliper  (interrupt-driven)
static const int CAL_CLK_PIN  = 27;
static const int CAL_DATA_PIN = 26;

// HX711 load-cell amplifier
static const int HX711_DOUT = 32;
static const int HX711_SCK  = 33;

// Built-in LED (GPIO 2 on most DOIT DevKit boards)
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
static const int PIN_LED = LED_BUILTIN;

static inline void flash_led(uint32_t ms = 20) {
  digitalWrite(PIN_LED, HIGH);
  delay(ms);
  digitalWrite(PIN_LED, LOW);
}

// =====================================================================
// TMC5160 driver object
// =====================================================================
TMC5160Stepper driver(PIN_CS, R_SENSE);

// Per-motor run current (mA rms). The shear axis and the sample-prep vibration
// table use different motors, swapped by hand on the same driver, so each has
// its own stored current. Never assume the value left over from the other mode.
static uint16_t shear_mA = 1000;
static uint16_t vib_mA   = 800;

void initTMC5160() {
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);   // hold disabled while we configure

  // ── Raw SPI diagnostic (bypass TMCStepper) ───────────────────────
  // Check MISO line state before SPI starts
  pinMode(19, INPUT_PULLUP);
  delay(5);
  Serial.print("MISO line (GPIO19) before SPI: ");
  Serial.println(digitalRead(19) ? "HIGH (ok / floating)" : "LOW (pulled down — check VIO & wiring)");

  // Manually clock out a TMC5160 IOIN read (address 0x04).
  // TMC5160 pipelines reads: first transaction loads address,
  // second transaction clocks out the data.
  SPI.begin(18, 19, 23, 22);
  delay(5);
  uint8_t raw[5];
  for (int pass = 0; pass < 2; pass++) {
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(500);
    raw[0] = SPI.transfer(0x04);   // IOIN register address (read)
    raw[1] = SPI.transfer(0x00);
    raw[2] = SPI.transfer(0x00);
    raw[3] = SPI.transfer(0x00);
    raw[4] = SPI.transfer(0x00);
    delayMicroseconds(500);
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
    delay(5);
    Serial.printf("Raw SPI pass %d: %02X %02X %02X %02X %02X\n",
                  pass + 1, raw[0], raw[1], raw[2], raw[3], raw[4]);
  }
  // pass 2 bytes 1-4 should be IOIN = 0x30000040 if driver is alive
  // All 00 = MISO stuck LOW (VIO missing or MISO wire broken)
  // All FF = MISO stuck HIGH (MISO not reaching driver)
  // ────────────────────────────────────────────────────────────────

  driver.setSPISpeed(500000);   // 500 kHz — robust over dupont wires
  driver.begin();

  // Chopper: SpreadCycle baseline, overridden to StealthChop below
  driver.toff(5);
  driver.blank_time(24);

  // Current: rms_current() sets both IRUN and IHOLD (50 % hold by default)
  driver.rms_current(shear_mA);

  // Microstepping: 16× physical, internally interpolated to 256× by the driver.
  // INTPOL uses the driver's own precision clock so motion is smoother than
  // sending 256× pulses from the ESP32 timer.
  driver.microsteps(16);
  driver.intpol(true);

  // StealthChop2: near-silent at the slow speeds used in direct shear tests
  driver.en_pwm_mode(true);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);

  // Leave driver disabled at startup — EN goes LOW only when RUN 1 is received.
  // This means no current flows through the motor until a test is started.
  digitalWrite(PIN_EN, HIGH);

  // SPI sanity check — read IOIN register (contains driver version field).
  // Good response: 0x30000040  (version byte = 0x30 for TMC5160)
  // Bad response:  0x00000000 or 0xFFFFFFFF → SPI not reaching driver, recheck wiring.
  uint32_t ioin = driver.IOIN();
  Serial.print("TMC5160 IOIN: 0x");
  Serial.println(ioin, HEX);
  if ((ioin >> 24) == 0x30) {
    Serial.println("TMC5160 SPI OK — driver detected");
  } else {
    Serial.println("TMC5160 SPI FAIL — check SCK/SDI/SDO/CS wiring");
  }
}

// =====================================================================
// Stepper motion
// =====================================================================
// With 16× microsteps + INTPOL, SPR seen by this firmware = 200 × 16 = 3200.
volatile float stepsPerRev = 3200.0f;
volatile float rpm         = 0.0f;
volatile bool  running     = false;

// Declared here (used by updateTimerFromParams); vibration logic lives below.
volatile bool  vibActive   = false;

hw_timer_t*  timer0   = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool stepLevel = false;

void updateTimerFromParams() {
  // Vibration owns timer0 while active — an RPM/SPR/DIR command arriving mid
  // vibration must not disable the alarm out from under it.
  if (vibActive) return;

  float localRpm, localSpr;
  portENTER_CRITICAL(&timerMux);
  localRpm = rpm;
  localSpr = stepsPerRev;
  portEXIT_CRITICAL(&timerMux);

  float stepsPerSec = (localRpm * localSpr) / 60.0f;

  if (!running || stepsPerSec <= 0.0f) {
    timerAlarmDisable(timer0);
    digitalWrite(PIN_STEP, LOW);
    stepLevel = false;
    return;
  }

  // Toggle STEP each interrupt → rising edges at stepsPerSec
  float isrHz = 2.0f * stepsPerSec;
  if (isrHz > 200000.0f) isrHz = 200000.0f;
  if (isrHz < 1.0f)      isrHz = 1.0f;

  uint32_t intervalUs = (uint32_t)(1000000.0f / isrHz);
  if (intervalUs < 5) intervalUs = 5;

  timerAlarmWrite(timer0, intervalUs, true);
  timerAlarmEnable(timer0);
}

// =====================================================================
// Vibration mode  (sample-prep table — separate motor, swapped in by hand)
//
// The vibration motor is bolted to the table with a bare shaft, so it acts
// as a reaction-mass shaker: the rotor oscillates, and the table receives the
// reaction to the rotor's angular acceleration as a torque couple about the
// shaft axis.
//
// Both STEP and DIR are derived from the SAME timer, so the reversal point is
// exact — a second independent clock (e.g. LEDC for STEP) would slowly drift
// in phase against the DIR flip and smear the amplitude.
//
//   steps per cycle = 2 × vibHalfSteps      (out and back)
//   ISR rate        = 4 × f × vibHalfSteps  (2 toggles per step)
// =====================================================================
// (vibActive is declared with the stepper globals above)
volatile uint32_t vibHalfSteps = 10;    // microsteps per half-cycle = p-p amplitude
volatile uint32_t vibStepCount = 0;
volatile bool     vibDirState  = false;

// Ceiling on ISR rate. Above this the ISR starves the main loop; amplitude is
// reduced to fit rather than silently missing steps.
static const float VIB_MAX_ISR_HZ = 100000.0f;

void IRAM_ATTR onTimer() {
  stepLevel = !stepLevel;
  digitalWrite(PIN_STEP, stepLevel ? HIGH : LOW);

  // Flip DIR on the falling edge: gives the driver a full half-toggle period
  // of setup time before the next rising edge latches a step in the new
  // direction (TMC5160 needs ~20 ns; this is orders of magnitude more).
  if (vibActive && !stepLevel) {
    if (++vibStepCount >= vibHalfSteps) {
      vibStepCount = 0;
      vibDirState  = !vibDirState;
      digitalWrite(PIN_DIR, vibDirState ? HIGH : LOW);
    }
  }
}

// Driver config for rapid reversal. StealthChop regulates from back-EMF and
// cannot track a direction change 2f times per second; the 256× interpolator
// extrapolates from the previous step interval and smears every reversal.
// Both must go for vibration, and both come back for shear.
void applyVibProfile() {
  driver.en_pwm_mode(false);     // SpreadCycle
  driver.intpol(false);
  driver.rms_current(vib_mA);
}

void applyShearProfile() {
  driver.intpol(true);
  driver.en_pwm_mode(true);      // StealthChop2 — quiet at shear speeds
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  driver.rms_current(shear_mA);
}

void stopVibration() {
  vibActive = false;
  timerAlarmDisable(timer0);
  digitalWrite(PIN_STEP, LOW);
  stepLevel = false;
  digitalWrite(PIN_EN, HIGH);    // de-energise — required before a motor swap
  applyShearProfile();
}

// Returns the amplitude actually used (may be clamped below the request).
uint32_t startVibration(float hz, uint32_t amp) {
  // Vibration and shear are mutually exclusive: different motors.
  running = false;
  timerAlarmDisable(timer0);

  if (hz  < 1.0f)   hz  = 1.0f;
  if (hz  > 500.0f) hz  = 500.0f;
  if (amp < 1)      amp = 1;

  // Clamp amplitude so the ISR rate stays sane at the requested frequency
  uint32_t ampMax = (uint32_t)(VIB_MAX_ISR_HZ / (4.0f * hz));
  if (ampMax < 1)   ampMax = 1;
  if (amp > ampMax) amp = ampMax;

  float isrHz = 4.0f * hz * (float)amp;
  uint32_t intervalUs = (uint32_t)(1000000.0f / isrHz + 0.5f);
  if (intervalUs < 5) intervalUs = 5;

  applyVibProfile();

  vibHalfSteps = amp;
  vibStepCount = 0;
  vibDirState  = false;
  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_STEP, LOW);
  stepLevel    = false;
  vibActive    = true;

  digitalWrite(PIN_EN, LOW);     // energise
  delay(1);
  timerAlarmWrite(timer0, intervalUs, true);
  timerAlarmEnable(timer0);
  return amp;
}

// =====================================================================
// Caliper decode  (interrupt-driven, falling-edge on clock)
//
// 24-bit frame:
//   bits 1-20  : magnitude (LSB first)
//   bit  21    : sign (1 = negative)
//   bits 22-23 : unused
//   bit  24    : unit (1 = inch, 0 = mm)
// =====================================================================
volatile uint8_t  cal_bits[25]      = {};
volatile int      cal_bit_index     = 0;
volatile uint32_t cal_last_fall_us  = 0;

volatile bool    cal_frame_ready = false;
volatile int32_t cal_raw_value   = 0;
volatile int8_t  cal_sign        = 1;
volatile bool    cal_is_inch     = false;

static const uint32_t CAL_GAP_US = 500;

void IRAM_ATTR onCalClockFall() {
  uint32_t now = micros();
  uint32_t dt  = now - cal_last_fall_us;
  cal_last_fall_us = now;

  if (dt > CAL_GAP_US) {
    cal_bit_index = 1;  // new frame
  }

  if (cal_bit_index >= 1 && cal_bit_index <= 24) {
    cal_bits[cal_bit_index] = (uint8_t)digitalRead(CAL_DATA_PIN);
    cal_bit_index++;

    if (cal_bit_index == 25) {
      int32_t value = 0;
      for (int i = 1; i <= 20; i++) {
        value += ((int32_t)cal_bits[i] << (i - 1));
      }
      cal_raw_value   = value;
      cal_sign        = (cal_bits[21] == 1) ? -1 : 1;
      cal_is_inch     = (cal_bits[24] == 1);
      cal_frame_ready = true;
    }
  }
}

// =====================================================================
// HX711 load-cell amplifier  (polled, main loop)
// =====================================================================
static float hx711_offset     = 0.0f;  // tare offset (raw counts)
static float hx711_cal_factor = 1.0f;  // raw counts per Newton

// PGA select. The gain applies to the NEXT conversion and is chosen purely by
// the number of SCK pulses sent after the 24 data bits:
//   1 pulse  -> channel A, gain 128  (+/-20 mV full scale)
//   2 pulses -> channel B, gain 32   (+/-80 mV, separate input pair)
//   3 pulses -> channel A, gain 64   (+/-40 mV)
// Lower gain buys headroom before the 24-bit rail at the cost of one bit of
// resolution, which sits far below the noise floor here. The chip powers up in
// channel A / gain 128, so that is the default this firmware keeps.
static uint8_t hx711_pulses = 1;
static uint8_t hx711_gain   = 128;

bool hx711_ready() {
  return digitalRead(HX711_DOUT) == LOW;
}

// Read 24-bit signed value, leave chip set for gain-128 channel-A next read
int32_t hx711_read_raw() {
  int32_t val = 0;
  noInterrupts();   // prevent step-timer ISR stretching SCK pulses mid-transfer
  for (int i = 0; i < 24; i++) {
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    val = (val << 1) | (digitalRead(HX711_DOUT) ? 1 : 0);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);
  }
  // Trailing pulses select channel and gain for the NEXT conversion
  for (uint8_t i = 0; i < hx711_pulses; i++) {
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);
  }
  interrupts();

  // Sign-extend 24-bit → 32-bit
  if (val & 0x800000) val |= 0xFF000000;
  return val;
}

// Auto-tare: average 8 readings at startup
void hx711_tare(int samples = 8) {
  long sum = 0;
  int  got = 0;
  uint32_t deadline = millis() + 2000;
  while (got < samples && millis() < deadline) {
    if (hx711_ready()) {
      sum += hx711_read_raw();
      got++;
    }
    delay(5);
  }
  if (got > 0) hx711_offset = (float)sum / got;
}

bool hx711_wait_ready(uint32_t timeout_ms = 500) {
  uint32_t deadline = millis() + timeout_ms;
  while (millis() < deadline) {
    if (hx711_ready()) return true;
    delay(1);
  }
  return false;
}

// Switch the PGA. Returns false for an unsupported gain, in which case nothing
// changes. The conversion already in flight was configured by the previous
// read's trailing pulses, so the first sample after the switch still arrives at
// the OLD gain -- it is read and discarded. The re-tare is mandatory: the offset
// is held in raw counts, and those counts have just been rescaled.
bool hx711_set_gain(uint8_t gain) {
  uint8_t pulses;
  if      (gain == 128) pulses = 1;
  else if (gain == 64)  pulses = 3;
  else if (gain == 32)  pulses = 2;
  else return false;

  hx711_pulses = pulses;
  hx711_gain   = gain;

  if (hx711_wait_ready()) hx711_read_raw();   // stale sample at the old gain
  hx711_tare();
  return true;
}

// Returns force in Newtons
float hx711_get_newtons() {
  int32_t raw = hx711_read_raw();
  return ((float)raw - hx711_offset) / hx711_cal_factor;
}

// =====================================================================
// Serial protocol
//
// PC → ESP32 (commands):
//   DIR 0/1           set direction
//   RPM <float>       set speed
//   SPR <float>       set steps/rev (use 3200 for 16× microstep + INTPOL)
//   RUN 0/1           start/stop motor
//   STOP              stop motor (also stops vibration, de-energises driver)
//   TARE              tare load cell
//   CALFACTOR <f>     set calibration factor (raw counts/N)
//   GAIN <128|64|32>  set HX711 PGA gain (re-tares; 32 uses channel B)
//   VIB <hz> <amp>    start vibration: amp = microsteps peak-to-peak
//   VIB 0             stop vibration
//   VIBCUR <mA>       run current for the vibration-table motor
//   SHEARCUR <mA>     run current for the direct-shear motor
//   STATUS            report mode and stored currents
//
// ESP32 → PC (output):
//   CAL <value> mm|in     caliper reading
//   FORCE <value> N       load-cell reading in Newtons
//   OK <CMD>              command acknowledged
//   OK VIB <hz> <amp> <stepHz>   vibration started with ACTUAL params
//   STATUS <mode> <shear_mA> <vib_mA> <hx711_gain>
//   OK GAIN <gain>        PGA re-ranged and re-tared
//   ERR GAIN              rejected: gain must be 128, 64 or 32
//   ERR                   unrecognised command
//   ERR VIB_ACTIVE        rejected: stop vibration before driving the shear axis
// =====================================================================
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("DIR")) {
    if (vibActive) {                  // DIR is driven by the vibration ISR
      Serial.println("ERR VIB_ACTIVE");
      return;
    }
    int v = line.substring(3).toInt();
    bool wasRunning = running;
    running = false;
    updateTimerFromParams();
    digitalWrite(PIN_STEP, LOW);
    stepLevel = false;
    digitalWrite(PIN_DIR, v ? LOW : HIGH);
    delayMicroseconds(20);
    running = wasRunning;
    updateTimerFromParams();
    Serial.println("OK DIR");
    flash_led();
    return;
  }

  if (line.startsWith("RPM")) {
    float v = line.substring(3).toFloat();
    if (v < 0.0f) v = -v;
    portENTER_CRITICAL(&timerMux);
    rpm = v;
    portEXIT_CRITICAL(&timerMux);
    updateTimerFromParams();
    Serial.println("OK RPM");
    flash_led();
    return;
  }

  if (line.startsWith("SPR")) {
    float v = line.substring(3).toFloat();
    if (v < 1.0f) v = 1.0f;
    portENTER_CRITICAL(&timerMux);
    stepsPerRev = v;
    portEXIT_CRITICAL(&timerMux);
    updateTimerFromParams();
    Serial.println("OK SPR");
    flash_led();
    return;
  }

  // ── Vibration (sample-prep table) ────────────────────────────────
  // VIB <hz> <amp>  — amp is microsteps peak-to-peak.  VIB 0 stops.
  if (line.startsWith("VIB ") || line == "VIB") {
    String args = line.substring(3);
    args.trim();
    float    hz  = args.toFloat();
    uint32_t amp = 0;
    int sp = args.indexOf(' ');
    if (sp > 0) amp = (uint32_t)args.substring(sp + 1).toInt();

    if (hz <= 0.0f || amp == 0) {
      stopVibration();
      Serial.println("OK VIB 0");
    } else {
      uint32_t used = startVibration(hz, amp);
      // Report what was ACTUALLY set — amplitude may have been clamped.
      Serial.printf("OK VIB %.2f %u %.1f\n", hz, used, 2.0f * hz * (float)used);
    }
    flash_led();
    return;
  }

  if (line.startsWith("VIBCUR")) {
    int v = line.substring(6).toInt();
    if (v >= 100 && v <= 3000) {
      vib_mA = (uint16_t)v;
      if (vibActive) driver.rms_current(vib_mA);   // live adjust while running
      Serial.println("OK VIBCUR");
    } else {
      Serial.println("ERR");
    }
    flash_led();
    return;
  }

  if (line.startsWith("SHEARCUR")) {
    int v = line.substring(8).toInt();
    if (v >= 100 && v <= 3000) {
      shear_mA = (uint16_t)v;
      if (!vibActive) driver.rms_current(shear_mA);
      Serial.println("OK SHEARCUR");
    } else {
      Serial.println("ERR");
    }
    flash_led();
    return;
  }

  if (line == "STATUS") {
    Serial.printf("STATUS %s %u %u %u\n",
                  vibActive ? "VIB" : (running ? "SHEAR" : "IDLE"),
                  shear_mA, vib_mA, hx711_gain);
    return;
  }

  if (line.startsWith("RUN")) {
    int v = line.substring(3).toInt();
    // Refuse rather than silently switching modes: the wrong motor may be
    // plugged in, and the vibration motor is not rated for the shear current.
    if (v != 0 && vibActive) {
      Serial.println("ERR VIB_ACTIVE");
      return;
    }
    if (v != 0) {
      digitalWrite(PIN_EN, LOW);   // enable driver before stepping
      delay(1);                    // brief settle before first step pulse
      running = true;
    } else {
      running = false;
      updateTimerFromParams();
      digitalWrite(PIN_EN, HIGH);  // disable driver — no current when idle
    }
    updateTimerFromParams();
    Serial.println("OK RUN");
    flash_led();
    return;
  }

  if (line == "STOP") {
    running = false;
    if (vibActive) stopVibration();   // also drops EN and restores shear profile
    updateTimerFromParams();
    digitalWrite(PIN_EN, HIGH);    // disable driver — no current when idle
    Serial.println("OK STOP");
    flash_led();
    return;
  }

  if (line == "TARE") {
    hx711_tare();
    Serial.println("OK TARE");
    flash_led();
    return;
  }

  // GAIN 128|64|32 -- re-ranges the amplifier and re-tares. Any calibration
  // taken at another gain stops applying: counts per Newton scale with it.
  if (line.startsWith("GAIN")) {
    int v = line.substring(4).toInt();
    if (hx711_set_gain((uint8_t)v)) {
      Serial.printf("OK GAIN %u\n", hx711_gain);
    } else {
      Serial.println("ERR GAIN");
    }
    flash_led();
    return;
  }

  if (line.startsWith("CALFACTOR")) {
    float v = line.substring(9).toFloat();
    if (v != 0.0f) hx711_cal_factor = v;
    Serial.println("OK CALFACTOR");
    flash_led();
    return;
  }

  Serial.println("ERR");
}

String rx;

// =====================================================================
// Setup
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Stepper step/dir pins
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR,  HIGH);

  // TMC5160 via SPI
  initTMC5160();

  // Step timer: 1 µs tick, prescaler 80
  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &onTimer, true);
  timerAlarmWrite(timer0, 1000000, true);
  timerAlarmDisable(timer0);

  // Caliper
  pinMode(CAL_CLK_PIN,  INPUT);
  pinMode(CAL_DATA_PIN, INPUT);
  cal_last_fall_us = micros();
  attachInterrupt(digitalPinToInterrupt(CAL_CLK_PIN), onCalClockFall, FALLING);

  // HX711
  pinMode(HX711_DOUT, INPUT);
  pinMode(HX711_SCK,  OUTPUT);
  digitalWrite(HX711_SCK, LOW);
  delay(500);
  hx711_tare();

  Serial.println("READY");
  Serial.println("Driver: TMC5160 Pro  Microsteps: 16x+INTPOL(256)  SPR: 3200");
  Serial.println("Pins: STEP=16 DIR=17 EN=4 CS=22 SCK=18 MISO=19 MOSI=23");
  Serial.println("Pins: CAL_CLK=27 CAL_DATA=26 HX711_DOUT=32 HX711_SCK=33");
  Serial.printf("HX711: channel %c, gain %u\n", hx711_gain == 32 ? 'B' : 'A', hx711_gain);
}

// =====================================================================
// Loop
// =====================================================================
static uint32_t last_force_ms = 0;

void loop() {
  // Serial command handling
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleLine(rx);
      rx = "";
    } else if (c != '\r') {
      rx += c;
      if (rx.length() > 120) rx = "";
    }
  }

  // Caliper output
  if (cal_frame_ready) {
    noInterrupts();
    int32_t raw  = cal_raw_value;
    int8_t  sign = cal_sign;
    bool    inch = cal_is_inch;
    cal_frame_ready = false;
    interrupts();

    if (inch) {
      Serial.print("CAL ");
      Serial.print((raw * sign) / 2000.0f, 3);
      Serial.println(" in");
    } else {
      Serial.print("CAL ");
      Serial.print((raw * sign) / 100.0f, 2);
      Serial.println(" mm");
    }
    flash_led(10);
  }

  uint32_t now = millis();

  // Force output — poll HX711 at ~10 Hz
  if (now - last_force_ms >= 100 && hx711_ready()) {
    last_force_ms = now;
    float newtons = hx711_get_newtons();
    Serial.print("FORCE ");
    Serial.print(newtons, 3);
    Serial.println(" N");
  }

}
