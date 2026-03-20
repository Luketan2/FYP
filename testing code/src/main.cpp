
#include <Arduino.h>

// =====================================================================
// Pin assignments  — all safe GPIOs, no strap / flash / PSRAM pins
//
//  AVOID: 0 (boot), 2 (strap), 6-11 (internal flash), 12 (MTDI strap),
//         15 (MTDO strap)
// =====================================================================

// Stepper
static const int PIN_STEP = 16;
static const int PIN_DIR  = 17;

// Caliper  (interrupt-driven)
static const int CAL_CLK_PIN  = 27;
static const int CAL_DATA_PIN = 26;

// HX711 load-cell amplifier
static const int HX711_DOUT = 32;
static const int HX711_SCK  = 33;

// Built-in LED (GPIO 2 on most DOIT DevKit boards — fine after reset)
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
// Stepper motion
// =====================================================================
volatile float stepsPerRev = 200.0f;
volatile float rpm         = 0.0f;
volatile bool  running     = false;

hw_timer_t*  timer0   = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool stepLevel = false;

void updateTimerFromParams() {
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

void IRAM_ATTR onTimer() {
  stepLevel = !stepLevel;
  digitalWrite(PIN_STEP, stepLevel ? HIGH : LOW);
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

bool hx711_ready() {
  return digitalRead(HX711_DOUT) == LOW;
}

// Read 24-bit signed value, leave chip set for gain-128 channel-A next read
int32_t hx711_read_raw() {
  int32_t val = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    val = (val << 1) | (digitalRead(HX711_DOUT) ? 1 : 0);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);
  }
  // One extra pulse → gain 128, channel A for next conversion
  digitalWrite(HX711_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(HX711_SCK, LOW);
  delayMicroseconds(1);

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
  if (got > 0) hx711_offset = (float)(sum / got);
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
//   DIR 0/1        set direction
//   RPM <float>    set speed
//   SPR <float>    set steps/rev
//   RUN 0/1        start/stop motor
//   STOP           stop motor
//   TARE           tare load cell
//   CALFACTOR <f>  set calibration factor (raw counts/N)
//
// ESP32 → PC (output):
//   CAL <value> mm|in    caliper reading
//   FORCE <value> N      load-cell reading in Newtons
//   OK <CMD>             command acknowledged
//   ERR                  unrecognised command
// =====================================================================
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("DIR")) {
    int v = line.substring(3).toInt();
    bool wasRunning = running;
    running = false;
    updateTimerFromParams();
    digitalWrite(PIN_STEP, LOW);
    stepLevel = false;
    digitalWrite(PIN_DIR, v ? HIGH : LOW);
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

  if (line.startsWith("RUN")) {
    int v = line.substring(3).toInt();
    running = (v != 0);
    updateTimerFromParams();
    Serial.println("OK RUN");
    flash_led();
    return;
  }

  if (line == "STOP") {
    running = false;
    updateTimerFromParams();
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

  // Stepper
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR,  LOW);

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
  delay(500);          // let HX711 power up
  hx711_tare();        // zero at startup

  Serial.println("READY");
  Serial.println("Pins: STEP=16 DIR=17 CAL_CLK=27 CAL_DATA=26 HX711_DOUT=32 HX711_SCK=33");
}

// =====================================================================
// Loop
// =====================================================================

// Send a force reading every ~100 ms (matches HX711 10 Hz output rate)
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

  // Force output — poll HX711 at ~10 Hz
  uint32_t now = millis();
  if (now - last_force_ms >= 100 && hx711_ready()) {
    last_force_ms = now;
    float newtons = hx711_get_newtons();
    Serial.print("FORCE ");
    Serial.print(newtons, 3);
    Serial.println(" N");
  }
}
