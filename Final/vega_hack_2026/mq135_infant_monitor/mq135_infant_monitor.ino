/**
 * ==============================================================================
 *  VEGA ARIES V2.0 — DUAL SMART SENSING PLATFORM
 *  1. AMG8833 8x8 Grid-EYE Thermal Camera (I2C-1 on SDA1/SCL1)
 *  2. MQ-135 Multi-Gas / Air Quality Sensor (Analog A0 + Alert Pin 13)
 *  3. ST7735 1.8" SPI TFT LCD (24x24 Seamless Thermal Heatmap + Air Quality Stats)
 *  4. Real-Time Web Dashboard at http://localhost:5000/ (Dual Telemetry Stream)
 * ==============================================================================
 *
 *  HARDWARE SETUP & WIRING:
 *  -----------------------------------------------------------------------------
 *  1. AMG8833 Thermal Camera   ──>  VEGA Aries V2
 *     - VCC                    ──>  3.3V
 *     - GND                    ──>  GND
 *     - SDA                    ──>  SDA1 (I2C-1)
 *     - SCL                    ──>  SCL1 (I2C-1)
 *     - AD0                    ──>  GND (Address = 0x68)
 *
 *  2. ST7735 1.8" SPI TFT LCD   ──>  VEGA Aries V2
 *     - VCC                    ──>  3.3V
 *     - GND                    ──>  GND
 *     - LED (Backlight)        ──>  3.3V
 *     - SCK (Clock)            ──>  SCLK0 (Dedicated SPI pad)
 *     - SDA (MOSI)             ──>  MOSI0 (Dedicated SPI pad)
 *     - AO (DC / Command)      ──>  Pin 8 (GPIO-8)
 *     - RESET                  ──>  Pin 9 (GPIO-9)
 *     - CS                     ──>  Pin 10 (GPIO-10)
 *
 *  3. MQ-135 Gas Sensor        ──>  VEGA Aries V2
 *     - VCC                    ──>  5.0V (or 3.3V)
 *     - GND                    ──>  GND
 *     - AOUT                   ──>  [2.2kΩ] ──┬──> VEGA Pin A0
 *                                             │
 *                                           [1kΩ]
 *                                             │
 *                                           GND
 *     - Alert LED / Buzzer     ──>  Pin 13 (GPIO-13)
 * ==============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <math.h>

// --- Display Dimension Constants ---
#define DISP_W        128  // Display Width (128 pixels)
#define DISP_H        160  // Display Height (160 pixels)

// --- LCD Display Pin Mapping (Hardware SPI0) ---
//  ⚠  These pin numbers MUST match your physical wiring.
//     If colours look wrong, change INIT_TAB to INITR_GREENTAB
//     (or whichever tab worked in tft_diag.ino).
#define TFT_CS         10  // Pin 10 (GPIO-10) — CS
#define TFT_DC          9  // Pin 9  (GPIO-9)  — DC / RS
#define TFT_RST         8  // Pin 8  (GPIO-8)  — RST
#define INIT_TAB      INITR_BLACKTAB  // ← change to INITR_GREENTAB if screen stays white

// --- Hardware Bus Instantiations for VEGA Aries V2 ---
//  Use a NAMED SPIClass instance and pass its pointer to the TFT constructor.
//  This is the pattern that works on VEGA ARIES V2 (see vega_hack_2026_lcd.ino).
SPIClass SPI(0);            // Linker dependency — keeps Adafruit_GFX happy
SPIClass tftSPI(0);         // Actual SPI0 instance driving SCLK0 / MOSI0
TwoWire amgWire(1);         // I2C-1 on SDA1 / SCL1 for AMG8833
Adafruit_ST7735 tft(&tftSPI, TFT_CS, TFT_DC, TFT_RST);  // pointer-form constructor

// --- AMG8833 I2C Addresses & Registers ---
#define AMG8833_ADDR_LOW    0x68  // Address when AD0 is GND
#define AMG8833_PCTL        0x00  // Power Control Register
#define AMG8833_RST         0x01  // Reset Register
#define AMG8833_FPS         0x02  // Frame Rate Register
#define AMG8833_TTHL        0x0E  // Thermistor Register LSB
#define AMG8833_PIXEL_BASE  0x80  // Pixel 1 LSB Register

uint8_t amgAddress = AMG8833_ADDR_LOW;
float pixels[64];

// --- 24x24 Interpolation Geometry ---
#define GRID_N         24   // 24x24 interpolated tiles
#define TILE_PX         4   // 4x4 pixels per tile
#define BOX_X          16   // Centered horizontally: (128 - 96) / 2 = 16
#define BOX_Y          16   // Below 14px header
#define BOX_SIZE       96   // 24 * 4 = 96 pixels

float interp24[GRID_N * GRID_N];

// --- MQ-135 Gas Sensor Configuration ---
const int   MQ135_ANALOG_PIN       = A0;    // Analog input A0
const int   ALERT_OUTPUT_PIN       = 13;    // Digital alert output (LED / Buzzer)
const float ADC_REF_VOLTAGE        = 3.3f;  // 3.3V reference
const int   ADC_RESOLUTION         = 1023;  // 10-bit ADC
const float DIVIDER_RATIO          = 0.3125f; // 1k / (2.2k + 1k)
const float RL_VALUE               = 10.0f; // 10 kΩ load resistor

const int   FILTER_WINDOW_SIZE     = 16;
const int   THRESHOLD_WARN_OFFSET  = 40;
const int   THRESHOLD_POOR_OFFSET  = 90;

int   rawAdcValue       = 0;
int   averagedAdcValue  = 0;
float calculatedVoltage = 0.0f;
int   baselineAdcValue  = 120;
float rZero             = 76.63f;

float ppmCO2 = 412.0f;
float ppmNH3 = 0.80f;
String airStatus = "NORMAL";
bool alertActive = false;

int   adcHistory[FILTER_WINDOW_SIZE];
int   historyIndex = 0;
long  historySum   = 0;

// --- Exact Thermal Color Palette Matching Dashboard ---
struct PaletteStop { uint8_t pos, r, g, b; };
static const PaletteStop PALETTE[] PROGMEM = {
  {   0, 165, 125, 187 }, // Violet / Purple (Cold Background)
  {  42,  30, 136, 229 }, // Sky Blue
  {  85,   0, 188, 212 }, // Cyan
  { 130,  76, 175,  80 }, // Green (Body)
  { 175, 205, 220,  57 }, // Yellow-Green
  { 215, 255, 167,  38 }, // Warm Orange
  { 255, 216,  67,  21 }  // Hot Red-Orange (Hot Spot)
};
#define PALETTE_SIZE (sizeof(PALETTE) / sizeof(PALETTE[0]))

uint16_t getHeatColor(float norm) {
  if (norm < 0.0f) norm = 0.0f;
  if (norm > 1.0f) norm = 1.0f;
  uint8_t v = (uint8_t)(norm * 255.0f + 0.5f);

  uint8_t i = 0;
  for (; i < PALETTE_SIZE - 2; i++) {
    if (v <= pgm_read_byte(&PALETTE[i + 1].pos)) break;
  }

  uint8_t loPos = pgm_read_byte(&PALETTE[i].pos);
  uint8_t hiPos = pgm_read_byte(&PALETTE[i + 1].pos);
  float span = (float)(hiPos - loPos);
  float frac = (span < 0.001f) ? 0.0f : (float)(v - loPos) / span;

  uint8_t r = pgm_read_byte(&PALETTE[i].r) + frac * (int8_t)(pgm_read_byte(&PALETTE[i + 1].r) - pgm_read_byte(&PALETTE[i].r));
  uint8_t g = pgm_read_byte(&PALETTE[i].g) + frac * (int8_t)(pgm_read_byte(&PALETTE[i + 1].g) - pgm_read_byte(&PALETTE[i].g));
  uint8_t b = pgm_read_byte(&PALETTE[i].b) + frac * (int8_t)(pgm_read_byte(&PALETTE[i + 1].b) - pgm_read_byte(&PALETTE[i].b));

  return tft.color565(r, g, b);
}

// --- Function Declarations ---
bool amg8833_init();
void amg8833_reset();
float amg8833_readThermistor();
bool amg8833_readPixels(float *pixelArray);
void writeRegister(uint8_t reg, uint8_t value);
void interpolate8to24(const float *src, float *dst);

int   performWarmupAndCalibration();
int   updateMovingAverage(int newSample);
float calculateSensorVoltage(int adcVal);
void  calculateGasConcentrations(float vSensor);
void  classifyAirQuality(int currentAdc, int baseline);

void drawLcdHeader();
void drawLcdGrid(float minT, float maxT);
void drawLcdCombinedStats(float minT, float maxT, float co2, float nh3, String status, bool alert);
void printFullSerialReport(float minT, float maxT, float avgT, float thm, float *pixelArray);

// ==============================================================================
//  SETUP
// ==============================================================================
void setup() {
  pinMode(ALERT_OUTPUT_PIN, OUTPUT);
  digitalWrite(ALERT_OUTPUT_PIN, LOW);

  delay(1000);
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("\n==================================================");
  Serial.println("  VEGA ARIES V2 — DUAL THERMAL & AIR MONITOR      ");
  Serial.println("  AMG8833 Thermal Camera + MQ-135 Gas Sensor      ");
  Serial.println("==================================================");

  // 1. Initialise TFT LCD Screen
  //    Call begin() on the NAMED tftSPI instance — this is what actually
  //    starts the SCLK0/MOSI0 hardware and cures the "all-white" symptom.
  tftSPI.begin();
  tft.initR(INIT_TAB);
  tft.setRotation(0); // Portrait mode (128 wide × 160 tall)
  tft.fillScreen(ST77XX_BLACK);

  // Draw thermal box border
  tft.drawRect(BOX_X - 1, BOX_Y - 1, BOX_SIZE + 2, BOX_SIZE + 2, ST77XX_WHITE);
  drawLcdHeader();

  // 2. Initialise I2C-1 (SDA1/SCL1) for AMG8833
  amgWire.begin();
  delay(100);

  // 3. Connect to AMG8833 Sensor
  if (!amg8833_init()) {
    Serial.println("[WARN] AMG8833 initializing retry...");
  }

  // 4. MQ-135 Baseline Calibration
  baselineAdcValue = performWarmupAndCalibration();
  for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
    adcHistory[i] = baselineAdcValue;
  }
  historySum = (long)baselineAdcValue * FILTER_WINDOW_SIZE;

  Serial.println("[SYSTEM READY] Real-time Dual Monitoring active.\n");
}

// ==============================================================================
//  LOOP
// ==============================================================================
void loop() {
  // --- 1. Read MQ-135 Gas Sensor ---
  rawAdcValue = analogRead(MQ135_ANALOG_PIN);
  averagedAdcValue = updateMovingAverage(rawAdcValue);
  calculatedVoltage = calculateSensorVoltage(averagedAdcValue);
  calculateGasConcentrations(calculatedVoltage);
  classifyAirQuality(averagedAdcValue, baselineAdcValue);
  digitalWrite(ALERT_OUTPUT_PIN, alertActive ? HIGH : LOW);

  // --- 2. Read AMG8833 Thermal Camera ---
  float thermistorTemp = amg8833_readThermistor();
  float minTemp = 24.0f, maxTemp = 35.0f, avgTemp = 28.0f;

  if (amg8833_readPixels(pixels)) {
    minTemp = pixels[0];
    maxTemp = pixels[0];
    float sumTemp = 0;

    for (int i = 0; i < 64; i++) {
      if (pixels[i] < minTemp) minTemp = pixels[i];
      if (pixels[i] > maxTemp) maxTemp = pixels[i];
      sumTemp += pixels[i];
    }
    avgTemp = sumTemp / 64.0f;

    // 24x24 Bilinear Interpolation
    interpolate8to24(pixels, interp24);

    // Draw Smooth Seamless 24x24 Thermal Image
    drawLcdGrid(minTemp, maxTemp);
  }

  // --- 3. Update LCD Multi-Sensor Stats Bar ---
  drawLcdCombinedStats(minTemp, maxTemp, ppmCO2, ppmNH3, airStatus, alertActive);

  // --- 4. Send Unified Telemetry to Web Dashboard (http://localhost:5000/) ---
  printFullSerialReport(minTemp, maxTemp, avgTemp, thermistorTemp, pixels);

  delay(200); // ~5 FPS smooth real-time update
}

// ==============================================================================
//  AMG8833 SENSOR FUNCTIONS
// ==============================================================================
bool amg8833_init() {
  amgAddress = AMG8833_ADDR_LOW;
  writeRegister(AMG8833_PCTL, 0x00);
  delay(10);
  amg8833_reset();
  writeRegister(AMG8833_FPS, 0x00);
  delay(10);
  return true;
}

void amg8833_reset() {
  writeRegister(AMG8833_RST, 0x3F);
  delay(10);
  writeRegister(AMG8833_RST, 0x39);
  delay(10);
}

float amg8833_readThermistor() {
  amgWire.beginTransmission(amgAddress);
  amgWire.write(AMG8833_TTHL);
  if (amgWire.endTransmission() != 0) return 0.0f;

  amgWire.requestFrom((int)amgAddress, 2);
  if (amgWire.available() >= 2) {
    uint8_t lsb = amgWire.read();
    uint8_t msb = amgWire.read();
    int16_t raw = ((uint16_t)msb << 8) | lsb;
    if (raw & 0x0800) raw |= 0xF000;
    return raw * 0.0625f;
  }
  return 0.0f;
}

bool amg8833_readPixels(float *pixelArray) {
  uint8_t rawBuf[128];
  for (int chunk = 0; chunk < 8; chunk++) {
    uint8_t regAddr = AMG8833_PIXEL_BASE + (chunk * 16);
    amgWire.beginTransmission(amgAddress);
    amgWire.write(regAddr);
    if (amgWire.endTransmission() != 0) return false;

    amgWire.requestFrom((int)amgAddress, 16);
    int idx = chunk * 16;
    for (int i = 0; i < 16; i++) {
      if (amgWire.available()) {
        rawBuf[idx + i] = amgWire.read();
      } else {
        return false;
      }
    }
  }

  for (int i = 0; i < 64; i++) {
    uint8_t lsb = rawBuf[i * 2];
    uint8_t msb = rawBuf[i * 2 + 1];
    int16_t val = ((uint16_t)msb << 8) | lsb;
    if (val & 0x0800) val |= 0xF000;
    pixelArray[i] = val * 0.25f;
  }
  return true;
}

void writeRegister(uint8_t reg, uint8_t value) {
  amgWire.beginTransmission(amgAddress);
  amgWire.write(reg);
  amgWire.write(value);
  amgWire.endTransmission();
}

void interpolate8to24(const float *src, float *dst) {
  float scale = 7.0f / 23.0f;
  for (int dy = 0; dy < 24; dy++) {
    float gy = dy * scale;
    int y1 = (int)gy;
    int y2 = (y1 + 1 < 8) ? y1 + 1 : y1;
    float fy = gy - y1;

    for (int dx = 0; dx < 24; dx++) {
      float gx = dx * scale;
      int x1 = (int)gx;
      int x2 = (x1 + 1 < 8) ? x1 + 1 : x1;
      float fx = gx - x1;

      dst[dy * 24 + dx] = src[y1 * 8 + x1] * (1.0f - fx) * (1.0f - fy)
                        + src[y1 * 8 + x2] * fx * (1.0f - fy)
                        + src[y2 * 8 + x1] * (1.0f - fx) * fy
                        + src[y2 * 8 + x2] * fx * fy;
    }
  }
}

// ==============================================================================
//  MQ-135 AIR QUALITY FUNCTIONS
// ==============================================================================
int performWarmupAndCalibration() {
  const int SAMPLES = 20;
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(MQ135_ANALOG_PIN);
    delay(100);
  }
  int baseline = (int)(sum / SAMPLES);
  if (baseline < 10) baseline = 100;

  float vPin = ((float)baseline / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE;
  float vSens = vPin / DIVIDER_RATIO;
  if (vSens < 0.1f) vSens = 0.1f;
  if (vSens > 4.9f) vSens = 4.9f;

  float rsAir = ((5.0f - vSens) / vSens) * RL_VALUE;
  rZero = rsAir / 3.6f;
  return baseline;
}

int updateMovingAverage(int newSample) {
  historySum -= adcHistory[historyIndex];
  adcHistory[historyIndex] = newSample;
  historySum += newSample;
  historyIndex = (historyIndex + 1) % FILTER_WINDOW_SIZE;
  return (int)(historySum / FILTER_WINDOW_SIZE);
}

float calculateSensorVoltage(int adcVal) {
  float pinVoltage = ((float)adcVal / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE;
  float sensorOutVoltage = pinVoltage / DIVIDER_RATIO;
  if (sensorOutVoltage < 0.05f) sensorOutVoltage = 0.05f;
  if (sensorOutVoltage > 4.95f) sensorOutVoltage = 4.95f;
  return sensorOutVoltage;
}

void calculateGasConcentrations(float vSensor) {
  float rs = ((5.0f - vSensor) / vSensor) * RL_VALUE;
  float ratio = rs / rZero;
  if (ratio < 0.05f) ratio = 0.05f;

  ppmCO2 = 110.47f * pow(ratio, -2.862f) + 380.0f;
  if (ppmCO2 < 400.0f) ppmCO2 = 400.0f;
  if (ppmCO2 > 5000.0f) ppmCO2 = 5000.0f;

  ppmNH3 = 102.2f * pow(ratio, -2.473f);
  if (ppmNH3 < 0.5f) ppmNH3 = 0.5f;
  if (ppmNH3 > 200.0f) ppmNH3 = 200.0f;
}

void classifyAirQuality(int currentAdc, int baseline) {
  int warnLimit = baseline + THRESHOLD_WARN_OFFSET;
  int poorLimit = baseline + THRESHOLD_POOR_OFFSET;

  if (currentAdc < warnLimit) {
    airStatus = "NORMAL (Clean Air)";
    alertActive = false;
  } else if (currentAdc < poorLimit) {
    airStatus = "WARNING (Elevated Gas)";
    alertActive = false;
  } else {
    airStatus = "POOR AIR (ALERT!)";
    alertActive = true;
  }
}

// ==============================================================================
//  TFT LCD RENDERING (Seamless Heatmap + Air Quality Telemetry)
// ==============================================================================
void drawLcdHeader() {
  tft.fillRect(0, 0, DISP_W, 14, tft.color565(10, 15, 30));
  tft.fillCircle(5, 7, 3, tft.color565(0, 210, 80));

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(12, 3);
  tft.print("VEGA CAM+AIR");

  tft.setTextColor(tft.color565(0, 210, 80));
  tft.setCursor(96, 3);
  tft.print("LIVE");
}

void drawLcdGrid(float minT, float maxT) {
  float range = maxT - minT;
  if (range < 1.0f) range = 1.0f;

  for (int row = 0; row < GRID_N; row++) {
    for (int col = 0; col < GRID_N; col++) {
      float norm = (interp24[row * GRID_N + col] - minT) / range;
      uint16_t col565 = getHeatColor(norm);

      int px = BOX_X + (col * TILE_PX);
      int py = BOX_Y + (row * TILE_PX);

      // Solid 4x4 fill with NO border lines (seamless continuous thermal image)
      tft.fillRect(px, py, TILE_PX, TILE_PX, col565);
    }
  }
}

void drawLcdCombinedStats(float minT, float maxT, float co2, float nh3, String status, bool alert) {
  int16_t sy = BOX_Y + BOX_SIZE + 2; // y = 114
  tft.fillRect(0, sy, DISP_W, 46, tft.color565(8, 12, 24));

  tft.setTextSize(1);

  // Line 1: Thermal Max & Min
  tft.setCursor(2, sy + 2);
  tft.setTextColor(tft.color565(255, 85, 65));
  tft.print("Mx:");
  tft.setTextColor(ST77XX_WHITE);
  tft.print(maxT, 1);

  tft.setCursor(66, sy + 2);
  tft.setTextColor(tft.color565(70, 145, 255));
  tft.print("Mn:");
  tft.setTextColor(ST77XX_WHITE);
  tft.print(minT, 1);

  // Line 2: Air Quality (CO2 & NH3)
  tft.setCursor(2, sy + 13);
  tft.setTextColor(tft.color565(59, 130, 246));
  tft.print("CO2:");
  tft.setTextColor(ST77XX_WHITE);
  tft.print((int)co2);

  tft.setCursor(66, sy + 13);
  tft.setTextColor(tft.color565(16, 185, 129));
  tft.print("NH3:");
  tft.setTextColor(ST77XX_WHITE);
  tft.print(nh3, 2);

  // Line 3: Air Quality Status Badge
  uint16_t badgeBg = alert ? tft.color565(239, 68, 68) : (co2 > 800 ? tft.color565(245, 158, 11) : tft.color565(16, 185, 129));
  const char *badgeTxt = alert ? "AIR: ALERT!" : (co2 > 800 ? "AIR: WARN" : "AIR: CLEAN");

  tft.fillRoundRect(2, sy + 26, 124, 14, 3, badgeBg);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(26, sy + 29);
  tft.print(badgeTxt);
}

// ==============================================================================
//  SERIAL OUTPUT FOR DASHBOARD (Both Thermal & Air Quality)
// ==============================================================================
void printFullSerialReport(float minT, float maxT, float avgT, float thm, float *pixelArray) {
  // 1. MQ-135 Telemetry Feed (Parsed by app.js)
  Serial.print("CO2: ");
  Serial.print(ppmCO2, 1);
  Serial.print(" ppm | NH3: ");
  Serial.print(ppmNH3, 2);
  Serial.print(" ppm | AirStatus: ");
  Serial.print(airStatus);
  Serial.print(" | AirAlert: ");
  Serial.println(alertActive ? "YES" : "NO");

  // 2. AMG8833 Thermal Telemetry Feed (Parsed by app.js)
  Serial.println("--- AMG8833 8x8 Thermal Data ---");
  Serial.print("Internal Thermistor Temp: ");
  Serial.print(thm, 2);
  Serial.println(" °C");

  Serial.print("Min Temp: ");
  Serial.print(minT, 2);
  Serial.print(" °C | Max Temp: ");
  Serial.print(maxT, 2);
  Serial.print(" °C | Avg Temp: ");
  Serial.print(avgT, 2);
  Serial.println(" °C\n");

  Serial.println("Numerical Temperature Matrix (8x8 in °C):");
  for (int row = 0; row < 8; row++) {
    Serial.print("[ ");
    for (int col = 0; col < 8; col++) {
      int index = row * 8 + col;
      if (pixelArray[index] < 10.0f && pixelArray[index] >= 0.0f) {
        Serial.print(" ");
      }
      Serial.print(pixelArray[index], 1);
      if (col < 7) Serial.print(", ");
    }
    Serial.println(" ]");
  }
}
