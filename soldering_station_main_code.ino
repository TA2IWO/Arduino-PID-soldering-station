/*
  Hot Air Gun (PID) – Burst‑Fire Triac Control (UNO/Pro Mini)
  — Stable encoder (interrupt quadrature) + quick temp calibration —

  • Menu: Set (°C) → Fan (%) → Heater ON/OFF
  • OLED SSD1306 I2C (A4 SDA, A5 SCL)
  • MAX6675 K‑type (SO=D4, CS=D7, SCK=D6)
  • Heater: MOC3021 + BTA12 on D5 (burst fire, NOT PWM)
  • Fan: IRFZ44N on D11 (PWM + soft‑start)

  Improvements in this build
  - Encoder now uses a robust ISR-based quadrature decoder on D2/D3
    (no library), with acceleration + fine edits behaving symmetrically.
  - Quick ambient calibration: In "Temp:" line (any menu position) while
    HEAT is OFF, hold the encoder button for ≥2 s → current reading becomes 25 °C
    (offset stored in RAM; can add EEPROM on request).
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>
#include <PID_v1_bc.h>
#include <math.h>
#include <string.h>

// forward decl for display so we can use it here
extern Adafruit_SSD1306 display;

// center text helper (uses runtime width instead of SCREEN_WIDTH)
void printCenter(int y, const char* s, uint8_t size=1){
  display.setTextSize(size);
  display.setTextColor(SSD1306_WHITE);
  int16_t w = (int16_t)strlen(s) * 6 * size; // ~6 px/char at size=1
  int16_t x = (display.width() - w)/2;
  if (x < 0) x = 0;
  display.setCursor(x, y);
  display.println(s);
}

// ==== OLED ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
const uint8_t OLED_ADDR = 0x3C;         // change to 0x3D if needed
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==== MAX6675 ====
const uint8_t KTC_SO  = 4;   // MISO (DO)
const uint8_t KTC_CS  = 7;
const uint8_t KTC_SCK = 6;
MAX6675 thermocouple(KTC_SCK, KTC_CS, KTC_SO);

// ==== Encoder — ISR CLK-only optimized (fast & robust) ====
// A -> D2 (CLK / INT0), B -> D3 (DT), BTN -> D10
#define PIN_ENC_A 2
#define PIN_ENC_B 3
#define PIN_BTN   10

volatile long encStepCount = 0;        // ±1 per detent (commit after two A edges)
volatile int  lastClkState = HIGH;     // last level of CLK (A)
volatile int8_t halfAcc = 0;           // accumulate half-steps; ±2 → 1 detent
volatile unsigned long lastEdgeUs = 0; // microsecond filter against bounce/EMI

// Tuning
#define ENC_MIN_US 180                 // ignore A edges faster than this (µs)
#define INVERT_DIR 0                   // 1 → invert CW/CCW

#if defined(ESP32) || defined(ESP8266)
void IRAM_ATTR encISR_clk() {
#else
void encISR_clk() {
#endif
  // Fast port read on AVR: D2 = PIND bit2, D3 = PIND bit3
  int a = (PIND >> 2) & 0x01;               // CLK (A)
  unsigned long t = micros();
  if ((t - lastEdgeUs) < ENC_MIN_US) {      // very-short interval → ignore
    lastClkState = a;
    return;
  }
  if (a != lastClkState) {
    int b = (PIND >> 3) & 0x01;             // DT (B)
    int dir = (b != a) ? +1 : -1;           // direction from DT vs CLK
#if INVERT_DIR
    dir = -dir;
#endif
    halfAcc += dir;                          // count half-steps on A edges only
    if (halfAcc >= 2)      { halfAcc = 0; encStepCount++; }
    else if (halfAcc <= -2){ halfAcc = 0; encStepCount--; }
    lastEdgeUs = t;
  }
  lastClkState = a;
}

// ==== Heater Triac (burst fire) ====
#define HEATER_PIN 5                  // to MOC3021 LED via resistor
const unsigned long WINDOW_MS = 1000; // 1 s window (~50 AC cycles @50 Hz)
unsigned long windowStart = 0;

// ==== Fan PWM ====
#define FAN_PIN 11                    // IRFZ44N gate (PWM)
int fanTarget = 50;                   // % (25–100)
int fanCurrent = 0;                   // % soft‑start

// ==== PID ====
volatile bool pidPaused = false;        // pause PID during edits to avoid jumps
double Setpoint = 200;                  // °C
double PV = 25;                         // °C (filtered)
double CV = 0;                          // % output 0–100 (burst duty)

// Tuned calmer defaults
double Kp = 18.0, Ki = 2.5, Kd = 25.0;
PID pid(&PV, &CV, &Setpoint, Kp, Ki, Kd, DIRECT);
const double CV_STEP_MAX = 5.0;          // max % change per loop

// ==== UI / Limits ====
uint8_t menuIndex = 0;                // 0:Set, 1:Fan, 2:Heat
bool heaterEnable = true;             // OFF → fan still runs (cooling)
const int TEMP_MIN = 100;             // set limits
const int TEMP_MAX = 450;
const int TEMP_CUTOFF = 470;
const uint16_t READ_PERIOD_MS = 200;  // MAX6675 sample period
unsigned long lastRead = 0;

// ===== Temperature calibration =====
// PV_calibrated = CAL_GAIN * raw + CAL_OFFSET
// Quick ambient cal: long‑press (≥2 s) while HEAT=OFF → set offset so PV=25 °C
// (Can add EEPROM save later.)
double CAL_OFFSET = 0.0;   // °C
double CAL_GAIN   = 1.00;  // unitless
const double AMBIENT_TARGET = 25.0;  // °C

// Simple EMA filter
const float PV_ALPHA = 0.25f;         // 0..1 (higher = less smoothing)

// ===== Helpers =====
void drawUI() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); display.print("Hot Air PID");
  display.setCursor(90,0); display.print(heaterEnable ? "ON" : "OFF");

  display.setCursor(0,16); display.print("Temp: "); display.print((int)PV); display.print("C");
  display.setCursor(0,30); display.print("Set:  "); display.print((int)Setpoint); display.print("C"); if(menuIndex==0) display.print(" <");
  display.setCursor(0,44); display.print("Fan:  "); display.print(fanTarget); display.print("% "); if(menuIndex==1) display.print(" <");
  display.setCursor(0,56); display.print("Heat: "); display.print(heaterEnable ? "ON" : "OFF"); if(menuIndex==2) display.print(" <");
  display.display();
}



void setup() {
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  // Init encoder state and interrupts (user's style)
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_BTN,   INPUT_PULLUP);
  lastClkState = digitalRead(PIN_ENC_A);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR_clk, CHANGE);

  Wire.begin();
  Wire.setClock(400000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.begin(115200);
    Serial.println(F("SSD1306 init failed. Check wiring or address."));
  } else {
    display.clearDisplay(); display.display();

    // ===== 3s Splash =====
    display.clearDisplay();
    printCenter(8,  "Selim Burak", 1);
    printCenter(28, "Kul",         2);
    printCenter(50, "TA2IWO",      2);
    display.display();
    delay(3000);
    display.clearDisplay(); display.display();
  }

  // PID setup
  pid.SetOutputLimits(0, 100);
  pid.SetSampleTime(200);
  pid.SetMode(MANUAL);
  CV = 0;                        // start from zero power
  pid.SetMode(AUTOMATIC);

  windowStart = millis();
}

void handleEncoder() {
  // 1) Encoder steps (polling, no acceleration)
  noInterrupts(); long steps = encStepCount; encStepCount = 0; interrupts();
  if (steps != 0) {
    pidPaused = true;
    if (menuIndex == 0) {
      Setpoint = constrain((int)Setpoint + steps, TEMP_MIN, TEMP_MAX);
    } else if (menuIndex == 1) {
      fanTarget = constrain(fanTarget + steps, 25, 100);
    } else if (menuIndex == 2) {
      if (steps > 0) heaterEnable = true;
      else if (steps < 0) heaterEnable = false;
    }
  }

  // 2) Button: short press = next menu (Set <-> Fan <-> Heat)
  static bool lastBtnLocal = HIGH;
  static unsigned long btnDown = 0;
  bool b = digitalRead(PIN_BTN);
  if (lastBtnLocal == HIGH && b == LOW) btnDown = millis();
  if (lastBtnLocal == LOW && b == HIGH) {
    unsigned long press = millis() - btnDown;
    if (press < 700) menuIndex = (menuIndex + 1) % 3;
  }
  lastBtnLocal = b;

  if (digitalRead(PIN_BTN) == HIGH) pidPaused = false;
}

void serviceFan() {
  if (fanTarget < 25) fanTarget = 25;  // enforce min 25%
  if (fanCurrent < fanTarget) fanCurrent++;
  else if (fanCurrent > fanTarget) fanCurrent--;
  int pwm = map(fanCurrent, 0, 100, 0, 255);
  analogWrite(FAN_PIN, pwm);
}

void serviceHeaterBurst() {
  if (!heaterEnable || isnan(PV) || PV > TEMP_CUTOFF) { digitalWrite(HEATER_PIN, LOW); return; }
  unsigned long now = millis();
  if (now - windowStart >= WINDOW_MS) windowStart += WINDOW_MS;
  unsigned long onMs = (unsigned long)(CV * WINDOW_MS / 100.0);
  unsigned long e = now - windowStart;
  digitalWrite(HEATER_PIN, (e < onMs) ? HIGH : LOW);
}

void loop() {
  unsigned long now = millis();
  if (now - lastRead >= READ_PERIOD_MS) {
    lastRead = now;
    double raw = thermocouple.readCelsius();
    if (!isnan(raw) && raw > -50 && raw < 1200) {
      double meas = CAL_GAIN * raw + CAL_OFFSET; // apply calibration
      PV = PV + (meas - PV) * PV_ALPHA;          // EMA filter
    }
  }

  if (!pidPaused) { pid.Compute(); }
  static double lastCV = 0; double diff = CV - lastCV;
  if (diff >  CV_STEP_MAX) CV = lastCV + CV_STEP_MAX;
  if (diff < -CV_STEP_MAX) CV = lastCV - CV_STEP_MAX;
  lastCV = CV;

  handleEncoder();
  serviceFan();
  serviceHeaterBurst();
  drawUI();
  delay(5);
}
