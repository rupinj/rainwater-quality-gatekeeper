/*
  ============================================================================
  Self-Calibrating Rainwater Harvesting Quality Gatekeeper
  Multi-Parameter Water Sensing | ESP32 Firmware
  ============================================================================
  Project: A Self-Calibrating Rainwater Harvesting Quality Gatekeeper Using
           Multi-Parameter Water Sensing
  Board:   ESP32 Dev Module

  What this firmware does
  ------------------------
  1. Reads turbidity, pH, TDS/conductivity and flow sensors every SAMPLE_MS.
  2. Applies a 5-sample median filter to each analog sensor to suppress noise.
  3. Tracks turbidity stability (|T(k) - T(k-1)| < EPSILON for N consecutive
     readings) to detect an adaptive first-flush endpoint instead of using a
     fixed timer.
  4. Computes a multi-parameter confidence score (0-100) that penalises
     invalid readings, unrealistic drift, missing calibration and
     (optionally) disagreement between two redundant turbidity sensors.
  5. Runs the decision routine (REJECT / GARDEN / STORAGE / HOLD) and drives
     a 3-way servo valve accordingly, always failing safe to REJECT/HOLD
     when confidence is low or a sensor fault is detected.
  6. Shows live status + confidence on a 0.96" SSD1306 OLED.
  7. Logs every sample and decision, with a timestamp, to a microSD card.
  8. Sounds a buzzer / lights an RGB LED on faults or low-confidence events.

  Required libraries (install via Library Manager)
  ---------------------------------------------------
    - Adafruit GFX Library
    - Adafruit SSD1306
    - ESP32Servo
    - SD (bundled with ESP32 core)

  Wiring (default pin map - change in the CONFIG block below if different)
  ---------------------------------------------------------------------------
    Turbidity sensor 1   -> GPIO34 (ADC1_CH6, analog in)
    Turbidity sensor 2*  -> GPIO35 (ADC1_CH7, analog in)   *redundancy option
    pH sensor            -> GPIO32 (ADC1_CH4, analog in)
    TDS/conductivity     -> GPIO33 (ADC1_CH5, analog in)
    Flow sensor (pulse)  -> GPIO4  (digital in, interrupt)
    Servo (3-way valve)  -> GPIO13 (PWM out)
    OLED SDA             -> GPIO21
    OLED SCL             -> GPIO22
    Buzzer               -> GPIO25
    RGB LED  R / G / B   -> GPIO14 / GPIO12 / GPIO27
    SD card  CS          -> GPIO5  (default VSPI: MOSI 23, MISO 19, SCK 18)

  NOTE: The thresholds below are PROTOTYPE values for a tabletop demonstration.
  They are NOT drinking-water certification limits. Calibrate against your
  own reference solutions before relying on the numbers for anything beyond
  a classroom/micro-project demo.
  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <SD.h>

// ============================== CONFIG ====================================

// --- Pin map ---
#define PIN_TURBIDITY_1   34
#define PIN_TURBIDITY_2   35   // set USE_REDUNDANT_TURBIDITY to true to use
#define PIN_PH            32
#define PIN_TDS           33
#define PIN_FLOW          4
#define PIN_SERVO         13
#define PIN_BUZZER        25
#define PIN_LED_R         14
#define PIN_LED_G         12
#define PIN_LED_B         27
#define PIN_SD_CS         5

// --- Feature flags ---
#define USE_REDUNDANT_TURBIDITY  true   // ILO52: redundancy / cross-validation
#define USE_SD_LOGGING           true
#define USE_OLED                 true

// --- Timing ---
const unsigned long SAMPLE_MS        = 5000;  // read every 5 seconds (as per design)
const unsigned long STANDBY_FLOW_MS  = 60000; // no-flow time before standby

// --- Median filter ---
const int FILTER_WINDOW = 5;

// --- Adaptive stability check (turbidity) ---
const float TURBIDITY_CLEAN_THRESHOLD = 10.0;  // NTU-equivalent
const float TURBIDITY_EPSILON         = 1.5;   // max allowed delta for "stable"
const int   STABLE_READINGS_REQUIRED  = 3;     // consecutive stable samples

// --- Decision thresholds (prototype values, Table 1 of the proposal) ---
const float TURBIDITY_REJECT_ABOVE   = 40.0;
const float TURBIDITY_GARDEN_LOW     = 10.0;
const float TURBIDITY_GARDEN_HIGH    = 40.0;

const float PH_REJECT_LOW   = 5.5;
const float PH_REJECT_HIGH  = 8.5;
const float PH_STORE_LOW    = 6.0;
const float PH_STORE_HIGH   = 8.0;

const float TDS_BASELINE          = 300.0;  // ppm, calibrate on site
const float TDS_SUDDEN_CHANGE_PPM = 150.0;  // |delta| beyond this => "sudden"

const int CONFIDENCE_REJECT_BELOW = 60;
const int CONFIDENCE_STORE_ABOVE  = 80;

// --- Confidence score penalties ---
const int PENALTY_INVALID_READING   = 40;
const int PENALTY_UNREALISTIC_DRIFT = 20;
const int PENALTY_NOT_CALIBRATED    = 15;
const int PENALTY_SENSOR_DISAGREE   = 25;

// --- Calibration flags (set true once each sensor has been calibrated) ---
bool turbidityCalibrated = true;
bool phCalibrated        = true;
bool tdsCalibrated       = true;

// --- Valve/servo angles (3-way diverter) ---
const int SERVO_ANGLE_REJECT  = 0;
const int SERVO_ANGLE_GARDEN  = 90;
const int SERVO_ANGLE_STORAGE = 170;

// ============================================================================

Servo valveServo;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

volatile unsigned long flowPulseCount = 0;
unsigned long lastFlowPulseTime = 0;

float turbidityHistory1[FILTER_WINDOW] = {0};
float turbidityHistory2[FILTER_WINDOW] = {0};
float phHistory[FILTER_WINDOW]         = {0};
float tdsHistory[FILTER_WINDOW]        = {0};
int historyIndex = 0;
bool historyFilled = false;

float lastFilteredTurbidity = -1;
int stableCount = 0;

enum Route { ROUTE_REJECT, ROUTE_GARDEN, ROUTE_STORAGE, ROUTE_HOLD, ROUTE_STANDBY };
Route currentRoute = ROUTE_REJECT;

File logFile;

// ----------------------------- Interrupt -----------------------------------
void IRAM_ATTR onFlowPulse() {
  flowPulseCount++;
}

// ----------------------------- Utility: median ------------------------------
float median5(float arr[]) {
  float tmp[FILTER_WINDOW];
  memcpy(tmp, arr, sizeof(tmp));
  // simple insertion sort - fine for 5 elements
  for (int i = 1; i < FILTER_WINDOW; i++) {
    float key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  return tmp[FILTER_WINDOW / 2];
}

// ----------------------------- Sensor reads ---------------------------------
// Convert raw ADC counts (0-4095) to an engineering-unit estimate.
// Replace these with your own calibration curves once you have reference
// standards; the linear maps below are placeholders for the demo.

float readTurbidityRaw(int pin) {
  int raw = analogRead(pin);
  // Higher turbidity -> lower voltage for most low-cost IR turbidity modules.
  // Map 0-4095 to an NTU-equivalent 0-200 scale (placeholder calibration).
  float ntu = map(raw, 4095, 0, 0, 200);
  return ntu;
}

float readPHRaw(int pin) {
  int raw = analogRead(pin);
  float voltage = raw * (3.3 / 4095.0);
  // Placeholder linear calibration: adjust slope/intercept after calibrating
  // against pH 4 / 7 / 9 buffer solutions.
  float ph = 7.0 + ((2.5 - voltage) / 0.18);
  return ph;
}

float readTDSRaw(int pin) {
  int raw = analogRead(pin);
  float voltage = raw * (3.3 / 4095.0);
  // Placeholder TDS curve (ppm); recalibrate with a known-TDS solution.
  float tds = (133.42 * voltage * voltage * voltage
               - 255.86 * voltage * voltage
               + 857.39 * voltage) * 0.5;
  return tds;
}

bool isValidReading(float v, float minV, float maxV) {
  return (v >= minV && v <= maxV) && !isnan(v);
}

// ----------------------------- Confidence score ------------------------------
int computeConfidence(bool turbidityValid, bool phValid, bool tdsValid,
                       bool unrealisticDrift, bool sensorsDisagree) {
  int confidence = 100;
  if (!turbidityValid || !phValid || !tdsValid) confidence -= PENALTY_INVALID_READING;
  if (unrealisticDrift) confidence -= PENALTY_UNREALISTIC_DRIFT;
  if (!turbidityCalibrated || !phCalibrated || !tdsCalibrated) confidence -= PENALTY_NOT_CALIBRATED;
  if (sensorsDisagree) confidence -= PENALTY_SENSOR_DISAGREE;
  if (confidence < 0) confidence = 0;
  return confidence;
}

// ----------------------------- Decision logic --------------------------------
Route decideRoute(float filteredTurbidity, float filteredPH, float filteredTDS,
                   int confidence, bool stable, bool flowDetected,
                   String &reason) {

  if (!flowDetected) {
    reason = "No flow - standby";
    return ROUTE_STANDBY;
  }

  if (confidence < CONFIDENCE_REJECT_BELOW) {
    reason = "Sensor validation failed / low confidence";
    return ROUTE_HOLD;
  }

  if (filteredTurbidity > TURBIDITY_REJECT_ABOVE) {
    reason = "First flush: dirty runoff";
    return ROUTE_REJECT;
  }

  bool turbidityOkForStore = (filteredTurbidity < TURBIDITY_GARDEN_LOW) && stable;
  bool phOkForStore  = (filteredPH >= PH_STORE_LOW && filteredPH <= PH_STORE_HIGH);
  bool phRejectRange = (filteredPH < PH_REJECT_LOW || filteredPH > PH_REJECT_HIGH);

  if (phRejectRange) {
    reason = "pH outside safe range";
    return ROUTE_REJECT;
  }

  if (turbidityOkForStore && phOkForStore && confidence > CONFIDENCE_STORE_ABOVE) {
    reason = "Water accepted for storage";
    return ROUTE_STORAGE;
  }

  reason = "Intermediate-quality water";
  return ROUTE_GARDEN;
}

// ----------------------------- Actuation --------------------------------------
void driveValve(Route route) {
  switch (route) {
    case ROUTE_REJECT:
    case ROUTE_HOLD:
      valveServo.write(SERVO_ANGLE_REJECT);
      break;
    case ROUTE_GARDEN:
      valveServo.write(SERVO_ANGLE_GARDEN);
      break;
    case ROUTE_STORAGE:
      valveServo.write(SERVO_ANGLE_STORAGE);
      break;
    case ROUTE_STANDBY:
      // leave valve in last safe (reject) position
      valveServo.write(SERVO_ANGLE_REJECT);
      break;
  }
}

void alertFault() {
  digitalWrite(PIN_LED_R, HIGH);
  digitalWrite(PIN_LED_G, LOW);
  digitalWrite(PIN_LED_B, LOW);
  tone(PIN_BUZZER, 2000, 300);
}

void alertOk(Route route) {
  noTone(PIN_BUZZER);
  switch (route) {
    case ROUTE_STORAGE:
      digitalWrite(PIN_LED_R, LOW);
      digitalWrite(PIN_LED_G, HIGH);
      digitalWrite(PIN_LED_B, LOW);
      break;
    case ROUTE_GARDEN:
      digitalWrite(PIN_LED_R, HIGH);
      digitalWrite(PIN_LED_G, HIGH);
      digitalWrite(PIN_LED_B, LOW);
      break;
    default:
      digitalWrite(PIN_LED_R, LOW);
      digitalWrite(PIN_LED_G, LOW);
      digitalWrite(PIN_LED_B, HIGH);
      break;
  }
}

// ----------------------------- Display ----------------------------------------
String routeToString(Route r) {
  switch (r) {
    case ROUTE_REJECT:  return "REJECT";
    case ROUTE_GARDEN:  return "GARDEN";
    case ROUTE_STORAGE: return "STORAGE";
    case ROUTE_HOLD:    return "HOLD (FAULT)";
    case ROUTE_STANDBY: return "STANDBY";
  }
  return "?";
}

void updateDisplay(Route route, int confidence, String reason,
                    float turbidity, float ph, float tds) {
#if USE_OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Rainwater Gatekeeper");
  display.print("Status: "); display.println(routeToString(route));
  display.print("Confidence: "); display.print(confidence); display.println("%");
  display.print("Turb: "); display.print(turbidity, 1); display.println(" NTU");
  display.print("pH: "); display.println(ph, 2);
  display.print("TDS: "); display.print(tds, 0); display.println(" ppm");
  display.println(reason);
  display.display();
#endif
}

// ----------------------------- SD logging ---------------------------------------
void logToSD(unsigned long timestamp, float turbidity, float ph, float tds,
             float flowRate, int confidence, Route route, String reason) {
#if USE_SD_LOGGING
  logFile = SD.open("/log.csv", FILE_APPEND);
  if (logFile) {
    logFile.print(timestamp); logFile.print(",");
    logFile.print(turbidity, 2); logFile.print(",");
    logFile.print(ph, 2); logFile.print(",");
    logFile.print(tds, 2); logFile.print(",");
    logFile.print(flowRate, 2); logFile.print(",");
    logFile.print(confidence); logFile.print(",");
    logFile.print(routeToString(route)); logFile.print(",");
    logFile.println(reason);
    logFile.close();
  } else {
    Serial.println("ERROR: could not open /log.csv for append");
  }
#endif
}

// ----------------------------- Setup --------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Rainwater Quality Gatekeeper - booting...");

  analogReadResolution(12); // 0-4095

  pinMode(PIN_FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), onFlowPulse, RISING);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);

  valveServo.setPeriodHertz(50);
  valveServo.attach(PIN_SERVO, 500, 2400);
  valveServo.write(SERVO_ANGLE_REJECT); // fail-safe default on boot

#if USE_OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("WARNING: OLED not found - continuing without display");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Gatekeeper booting...");
    display.display();
  }
#endif

#if USE_SD_LOGGING
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("WARNING: SD card not found - logging disabled for this run");
  } else {
    if (!SD.exists("/log.csv")) {
      logFile = SD.open("/log.csv", FILE_WRITE);
      if (logFile) {
        logFile.println("timestamp_ms,turbidity_ntu,ph,tds_ppm,flow_rate,confidence,route,reason");
        logFile.close();
      }
    }
    Serial.println("SD logging ready: /log.csv");
  }
#endif

  Serial.println("Setup complete. Entering main loop.");
}

// ----------------------------- Main loop -----------------------------------------
unsigned long lastSampleTime = 0;

void loop() {
  unsigned long now = millis();

  // Track flow for standby detection
  static unsigned long lastPulseCountCheck = 0;
  static unsigned long lastKnownPulseCount = 0;
  if (flowPulseCount != lastKnownPulseCount) {
    lastKnownPulseCount = flowPulseCount;
    lastFlowPulseTime = now;
  }
  bool flowDetected = (now - lastFlowPulseTime) < STANDBY_FLOW_MS;

  if (now - lastSampleTime < SAMPLE_MS) {
    return; // non-blocking wait until next sample window
  }
  lastSampleTime = now;

  // ---- 1. Read raw sensors ----
  float rawTurb1 = readTurbidityRaw(PIN_TURBIDITY_1);
  float rawTurb2 = USE_REDUNDANT_TURBIDITY ? readTurbidityRaw(PIN_TURBIDITY_2) : rawTurb1;
  float rawPH    = readPHRaw(PIN_PH);
  float rawTDS   = readTDSRaw(PIN_TDS);

  // flow rate estimate: pulses since last sample / sample interval
  static unsigned long lastFlowCountForRate = 0;
  unsigned long pulsesThisWindow = flowPulseCount - lastFlowCountForRate;
  lastFlowCountForRate = flowPulseCount;
  // YF-S201 typical: 450 pulses/litre -> litres/min
  float flowRate = (pulsesThisWindow / 450.0) * (60000.0 / SAMPLE_MS);

  // ---- 2. Push into rolling history buffers ----
  turbidityHistory1[historyIndex] = rawTurb1;
  turbidityHistory2[historyIndex] = rawTurb2;
  phHistory[historyIndex]         = rawPH;
  tdsHistory[historyIndex]        = rawTDS;
  historyIndex = (historyIndex + 1) % FILTER_WINDOW;
  if (historyIndex == 0) historyFilled = true;

  // ---- 3. Median filter ----
  float filteredTurb1 = median5(turbidityHistory1);
  float filteredTurb2 = median5(turbidityHistory2);
  float filteredTurb  = USE_REDUNDANT_TURBIDITY
                           ? (filteredTurb1 + filteredTurb2) / 2.0
                           : filteredTurb1;
  float filteredPH  = median5(phHistory);
  float filteredTDS = median5(tdsHistory);

  // ---- 4. Validate readings ----
  bool turbValid = isValidReading(filteredTurb, 0, 200);
  bool phValid   = isValidReading(filteredPH, 0, 14);
  bool tdsValid  = isValidReading(filteredTDS, 0, 2000);

  bool unrealisticDrift = false;
  if (lastFilteredTurbidity >= 0) {
    float delta = fabs(filteredTurb - lastFilteredTurbidity);
    if (delta > 80) unrealisticDrift = true; // implausible single-step jump
  }

  bool sensorsDisagree = USE_REDUNDANT_TURBIDITY &&
                          (fabs(filteredTurb1 - filteredTurb2) > 15.0);

  // ---- 5. Stability check (adaptive first-flush endpoint) ----
  bool stable = false;
  if (lastFilteredTurbidity >= 0) {
    float delta = fabs(filteredTurb - lastFilteredTurbidity);
    if (filteredTurb < TURBIDITY_CLEAN_THRESHOLD && delta < TURBIDITY_EPSILON) {
      stableCount++;
    } else {
      stableCount = 0;
    }
  }
  stable = (stableCount >= STABLE_READINGS_REQUIRED);
  lastFilteredTurbidity = filteredTurb;

  // ---- 6. Confidence score ----
  int confidence = computeConfidence(turbValid, phValid, tdsValid,
                                      unrealisticDrift, sensorsDisagree);

  // ---- 7. Decision ----
  String reason;
  Route route = decideRoute(filteredTurb, filteredPH, filteredTDS,
                             confidence, stable, flowDetected, reason);
  currentRoute = route;

  // ---- 8. Actuate, display, alert, log ----
  driveValve(route);

  if (route == ROUTE_HOLD) {
    alertFault();
  } else {
    alertOk(route);
  }

  updateDisplay(route, confidence, reason, filteredTurb, filteredPH, filteredTDS);
  logToSD(now, filteredTurb, filteredPH, filteredTDS, flowRate, confidence, route, reason);

  // ---- 9. Serial debug ----
  Serial.printf("[%lu ms] Turb=%.1f NTU  pH=%.2f  TDS=%.0f ppm  Flow=%.2f L/min  "
                "Conf=%d%%  Route=%s  (%s)\n",
                now, filteredTurb, filteredPH, filteredTDS, flowRate,
                confidence, routeToString(route).c_str(), reason.c_str());
}
