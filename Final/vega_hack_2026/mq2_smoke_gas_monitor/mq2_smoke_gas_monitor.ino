/**
 * ==============================================================================
 *  AI-Based Infant Monitoring Cradle — MQ-2 Smoke & Flammable Gas System
 *  Microcontroller : VEGA ARIES V2.0 (THEJAS32 RISC-V SoC)
 *  Sensor          : Flying-Fish MQ-2 Smoke & Combustible Gas Sensor (5.0V Power)
 *  ADC Protection  : 2.2kΩ / 1kΩ Voltage Divider (0 - 5.0V -> 0 - 1.56V)
 *  Target Gases    : Smoke Detection & LPG / Combustible Gas Leaks
 *  Baud Rate       : 115200 Baud
 *  Web Dashboard   : http://localhost:5000/
 * ==============================================================================
 *
 *  HARDWARE WIRING (Same as MQ-135):
 *  -----------------------------------------------------------------------------
 *  1. MQ-2 VCC   ──>  VEGA ARIES 5V   (Powers the ~300°C heater coil)
 *  2. MQ-2 GND   ──>  VEGA ARIES GND  (Common Ground)
 *  3. MQ-2 AO    ──>  [2.2 kΩ Resistor] ──┬──> VEGA ARIES Pin A0 (Safe 0 - 1.56V)
 *                                         │
 *                                      [1 kΩ Resistor]
 *                                         │
 *                                      VEGA GND
 *  4. Alert Output ──>  VEGA ARIES Pin 13 (LED / Buzzer)
 * ==============================================================================
 */

#include <math.h>

// ==============================================================================
//  1. PIN CONFIGURATIONS
// ==============================================================================
const int MQ2_ANALOG_PIN   = A0;  // Analog input connected to voltage divider midpoint
const int ALERT_OUTPUT_PIN = 13;  // Digital output for Alert LED / Active Buzzer
const int MQ2_DIGITAL_PIN  = 7;   // Optional DOUT (not required)

// ==============================================================================
//  2. ADC & VOLTAGE CALCULATION CONSTANTS
// ==============================================================================
const float ADC_REF_VOLTAGE  = 3.3f;    // VEGA ARIES V2.0 ADC reference voltage (3.3V)
const int   ADC_RESOLUTION   = 1023;    // 10-bit ADC max resolution (0 to 1023)
const float DIVIDER_RATIO    = 0.3125f; // Voltage divider ratio: 1k / (2.2k + 1k) = 0.3125
const float RL_VALUE         = 10.0f;   // 10 kΩ load resistor on MQ-2 module

// ==============================================================================
//  3. FILTERING & TIMING PARAMETERS
// ==============================================================================
const int   FILTER_WINDOW_SIZE     = 16;   // 16-sample Moving Average Filter
const unsigned long SAMPLE_INTERVAL_MS = 200;  // Sample every 200 ms (5 Hz sampling)
const unsigned long PRINT_INTERVAL_MS  = 1000; // Update Serial Monitor every 1 second

// ==============================================================================
//  4. CONFIGURABLE SOFTWARE THRESHOLDS (RELATIVE TO CLEAN ROOM BASELINE)
// ==============================================================================
const int THRESHOLD_WARN_OFFSET = 35;  // ADC rise for WARNING
const int THRESHOLD_SMOKE_ALERT = 80;  // ADC rise for SMOKE / GAS HAZARD ALERT

// ==============================================================================
//  5. GLOBAL VARIABLES & STATE
// ==============================================================================
int   rawAdcValue       = 0;
int   averagedAdcValue  = 0;
float calculatedVoltage = 0.0f;
int   baselineAdcValue  = 0;
float rZero             = 9.83f; // Clean-air baseline sensor resistance R0 for MQ-2

// Real-time gas concentrations in ppm
float ppmSmoke = 0.0f;
float ppmLPG   = 0.0f;
float ppmCH4   = 0.0f;

// Moving average circular buffer
int   adcHistory[FILTER_WINDOW_SIZE];
int   historyIndex = 0;
long  historySum   = 0;

// Non-blocking timer variables
unsigned long lastSampleTime = 0;
unsigned long lastPrintTime  = 0;

// --- Function Declarations ---
int   performWarmupAndCalibration();
int   updateMovingAverage(int newSample);
float calculateSensorVoltage(int adcVal);
void  calculateMQ2GasConcentrations(float vSensor);
void  classifySmokeAndGas(int currentAdc, int baseline, String &statusText, bool &isAlert);
void  printFormattedReport(int raw, int avg, float volts, int baseline, String status, bool alert);

// ==============================================================================
//  SETUP FUNCTION
// ==============================================================================
void setup() {
  pinMode(ALERT_OUTPUT_PIN, OUTPUT);
  digitalWrite(ALERT_OUTPUT_PIN, LOW); // Start with alert OFF

  pinMode(MQ2_DIGITAL_PIN, INPUT);

  Serial.begin(115200);
  delay(1200); // Allow UART and power rails to stabilize

  Serial.println("\n==================================================");
  Serial.println("  VEGA ARIES V2.0 — INFANT CRADLE SAFETY MONITOR  ");
  Serial.println("  Sensor : Flying-Fish MQ-2 (Smoke & LPG Detector)");
  Serial.println("==================================================");

  // 1. Sensor Warmup & Baseline Calibration Phase
  baselineAdcValue = performWarmupAndCalibration();

  // 2. Pre-fill moving average buffer with clean air baseline
  for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
    adcHistory[i] = baselineAdcValue;
  }
  historySum = (long)baselineAdcValue * FILTER_WINDOW_SIZE;

  Serial.println("\n[SYSTEM READY] Real-time Smoke & Gas Hazard Monitoring Active.\n");
}

// ==============================================================================
//  LOOP FUNCTION (NON-BLOCKING)
// ==============================================================================
void loop() {
  unsigned long currentMillis = millis();

  // Task 1: Periodic Sampling & Moving Average Filter (every 200 ms)
  if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = currentMillis;

    // Read ADC on VEGA ARIES Pin A0
    rawAdcValue = analogRead(MQ2_ANALOG_PIN);

    // Apply moving average noise reduction
    averagedAdcValue = updateMovingAverage(rawAdcValue);

    // Calculate actual sensor output voltage
    calculatedVoltage = calculateSensorVoltage(averagedAdcValue);

    // Calculate real changing Smoke & LPG ppm values
    calculateMQ2GasConcentrations(calculatedVoltage);
  }

  // Task 2: Air Quality Classification, Alerting & Report (every 1000 ms)
  if (currentMillis - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = currentMillis;

    String hazardStatus = "NORMAL";
    bool   alertActive  = false;

    classifySmokeAndGas(averagedAdcValue, baselineAdcValue, hazardStatus, alertActive);

    // Activate Alert Pin (LED / Buzzer)
    digitalWrite(ALERT_OUTPUT_PIN, alertActive ? HIGH : LOW);

    // Print clean diagnostic report
    printFormattedReport(rawAdcValue, averagedAdcValue, calculatedVoltage, baselineAdcValue, hazardStatus, alertActive);
  }
}

// ==============================================================================
//  CALIBRATION / BASELINE ROUTINE (MQ-2)
// ==============================================================================
int performWarmupAndCalibration() {
  const int CALIBRATION_SAMPLES = 30;
  long calibrationSum = 0;

  Serial.println("\n>>> STARTING MQ-2 BASELINE CALIBRATION <<<");
  Serial.println("[INFO] Ensure the infant cradle area is free of smoke or gas.");
  Serial.print("[INFO] Collecting baseline samples: ");

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int sample = analogRead(MQ2_ANALOG_PIN);
    calibrationSum += sample;
    Serial.print(".");
    delay(200); // 6-second baseline capture
  }
  Serial.println(" DONE!");

  int baseline = (int)(calibrationSum / CALIBRATION_SAMPLES);
  if (baseline < 10) baseline = 100; // Safe fallback

  float vPin = ((float)baseline / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE;
  float vSens = vPin / DIVIDER_RATIO;
  if (vSens < 0.1f) vSens = 0.1f;
  if (vSens > 4.9f) vSens = 4.9f;

  float rsAir = ((5.0f - vSens) / vSens) * RL_VALUE;
  rZero = rsAir / 9.83f; // Standard clean air ratio factor for MQ-2 (9.83)

  Serial.println("--------------------------------------------------");
  Serial.print(">> Clean Air Baseline (ADC) : "); Serial.println(baseline);
  Serial.print(">> Baseline R0 Resistance   : "); Serial.print(rZero, 2); Serial.println(" kOhm");
  Serial.print(">> Smoke Warning Limit (ADC): "); Serial.println(baseline + THRESHOLD_WARN_OFFSET);
  Serial.print(">> Smoke Hazard Alert (ADC) : "); Serial.println(baseline + THRESHOLD_SMOKE_ALERT);
  Serial.println("--------------------------------------------------");
  
  return baseline;
}

// ==============================================================================
//  MOVING AVERAGE FILTER
// ==============================================================================
int updateMovingAverage(int newSample) {
  historySum -= adcHistory[historyIndex];
  adcHistory[historyIndex] = newSample;
  historySum += newSample;
  historyIndex = (historyIndex + 1) % FILTER_WINDOW_SIZE;
  return (int)(historySum / FILTER_WINDOW_SIZE);
}

// ==============================================================================
//  VOLTAGE CALCULATION
// ==============================================================================
float calculateSensorVoltage(int adcVal) {
  float pinVoltage = ((float)adcVal / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE;
  float sensorOutVoltage = pinVoltage / DIVIDER_RATIO;
  if (sensorOutVoltage < 0.05f) sensorOutVoltage = 0.05f;
  if (sensorOutVoltage > 4.95f) sensorOutVoltage = 4.95f;
  return sensorOutVoltage;
}

// ==============================================================================
//  MQ-2 GAS CONCENTRATION CALCULATIONS (SMOKE & LPG IN PPM)
// ==============================================================================
void calculateMQ2GasConcentrations(float vSensor) {
  float rs = ((5.0f - vSensor) / vSensor) * RL_VALUE;
  float ratio = rs / rZero;
  if (ratio < 0.05f) ratio = 0.05f;

  // Power law regression models from MQ-2 datasheet curves:
  // 1. Smoke (ppm): ppm = 3436.7 * (Rs/R0)^(-3.369)
  ppmSmoke = 3436.7f * pow(ratio, -3.369f);
  if (ppmSmoke < 0.0f) ppmSmoke = 0.0f;
  if (ppmSmoke > 10000.0f) ppmSmoke = 10000.0f;

  // 2. LPG (Combustible Cooking Gas): ppm = 574.25 * (Rs/R0)^(-2.222)
  ppmLPG = 574.25f * pow(ratio, -2.222f);
  if (ppmLPG < 0.0f) ppmLPG = 0.0f;
  if (ppmLPG > 10000.0f) ppmLPG = 10000.0f;

  // 3. Methane (CH4): ppm = 4594.1 * (Rs/R0)^(-2.686)
  ppmCH4 = 4594.1f * pow(ratio, -2.686f);
  if (ppmCH4 < 0.0f) ppmCH4 = 0.0f;
  if (ppmCH4 > 10000.0f) ppmCH4 = 10000.0f;
}

// ==============================================================================
//  SMOKE & GAS CLASSIFICATION
// ==============================================================================
void classifySmokeAndGas(int currentAdc, int baseline, String &statusText, bool &isAlert) {
  int warnLimit = baseline + THRESHOLD_WARN_OFFSET;
  int alertLimit = baseline + THRESHOLD_SMOKE_ALERT;

  if (currentAdc < warnLimit) {
    statusText = "NORMAL (Safe Air)";
    isAlert = false;
  } else if (currentAdc < alertLimit) {
    statusText = "WARNING (Trace Smoke / Gas Detected)";
    isAlert = false;
  } else {
    statusText = "HAZARD ALERT (SMOKE / GAS LEAK DETECTED!)";
    isAlert = true; // Trigger LED / Buzzer
  }
}

// ==============================================================================
//  SERIAL MONITOR REPORT & DASHBOARD STREAM
// ==============================================================================
void printFormattedReport(int raw, int avg, float volts, int baseline, String status, bool alert) {
  // Stream single-line telemetry feed for Web Dashboard (http://localhost:5000/)
  Serial.print("Smoke: ");
  Serial.print(ppmSmoke, 1);
  Serial.print(" ppm | LPG: ");
  Serial.print(ppmLPG, 1);
  Serial.print(" ppm | GasStatus: ");
  Serial.print(status);
  Serial.print(" | HazardAlert: ");
  Serial.println(alert ? "YES" : "NO");

  // Multi-line human readable report
  Serial.println("=========================");
  Serial.println("    MQ-2 SMOKE & GAS     ");
  Serial.println("=========================");
  Serial.print("Raw ADC       : "); Serial.println(raw);
  Serial.print("Average ADC   : "); Serial.println(avg);
  Serial.print("Voltage       : "); Serial.print(volts, 2); Serial.println(" V");
  Serial.print("Smoke (ppm)   : "); Serial.print(ppmSmoke, 1); Serial.println(" ppm");
  Serial.print("LPG Gas (ppm) : "); Serial.print(ppmLPG, 1); Serial.println(" ppm");
  Serial.print("Methane (CH4) : "); Serial.print(ppmCH4, 1); Serial.println(" ppm");
  Serial.print("Baseline ADC  : "); Serial.println(baseline);
  Serial.print("Status        : "); Serial.println(status);
  Serial.print("Alert         : "); Serial.println(alert ? "YES [HAZARD TRIGGERED]" : "NO");
  Serial.println("=========================\n");
}
