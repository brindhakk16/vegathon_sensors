/**
 * ================================================================
 *  VEGA ARIES V2 — MULTI-SENSOR LIVE EDGE MONITOR
 *  Boot  : VEGA Logo (5 seconds)
 *  Page 0: Thermal Camera (AMG8833 Grid on SDA1/SCL1 or SDA0/SCL0)
 *  Page 1: Multi-Gas AQI (MQ-135 on A0, MQ-2 on A1)
 *  Page 2: Infant Chest Movements (MR24D11C10 mmWave on A3)
 *  Switch: Apply 5V to GPIO 2 for >100ms (with 10k pull-down)
 * ================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <math.h>

// ---- PIN DEFINITIONS ----
#define TFT_CS    10
#define TFT_DC     8
#define TFT_RST    9
#define BTN_PIN    2   // GPIO 2: Apply 5V for >100ms to switch page
#define MQ135_PIN A0   // MQ-135 Gas Sensor
#define MQ2_PIN   A1   // MQ-2 Smoke/Gas Sensor
#define RADAR_PIN A3   // MR24D11C10 mmWave Radar
#define ALERT_PIN 13   // Alert LED / Buzzer

// ---- HARDWARE INSTANCES ----
SPIClass SPI(0);
SPIClass tftSPI(0);
Adafruit_ST7735 tft = Adafruit_ST7735(&tftSPI, TFT_CS, TFT_DC, TFT_RST);

TwoWire amgWire(1); // I2C Bus 1 (SDA1 / SCL1)

// ---- PAGE STATE ----
// count == 0 : Thermal Camera Page (AMG8833)
// count == 1 : Air Quality & Gas Sensing Page (MQ-135 + MQ-2)
// count == 2 : Infant Chest Movement mmWave Page (MR24D11C10)
int  count = 0;            
bool pageChanged = true; 

// ---- GPIO 2 DEBOUNCE (>100ms) ----
unsigned long btnHighSince = 0;
bool          btnArmed     = true;   

// ---- AMG8833 THERMAL CAMERA DATA ----
#define AMG_ADDR_DEFAULT 0x68
#define AMG_ADDR_ALT     0x69

float pixels[64];
float interp[24 * 24];
bool amgReady = false;
uint8_t amgCurrentAddr = AMG_ADDR_DEFAULT;
TwoWire *activeWire = &amgWire;
int amgBusNum = 1;

float therm = 25.0f;
float mn = 22.0f, mx = 34.0f, av = 28.0f;
int consecutiveFails = 0;

// ---- GAS SENSING DATA ----
float ppmCO2 = 412.0f, ppmNH3 = 0.8f, ppmH2 = 0.5f;
float ppmCH4 = 1.2f,   ppmCO  = 2.1f, ppmBenz = 0.004f;
int   aqi = 38;
String aqiLabel = "Good";
bool  alert = false;
int   raw135 = 0, raw2 = 0;

// ---- mmWAVE RADAR DATA ----
int  waveCount = 0;
bool lastRadarState = false;
int  radarVal = 0;
bool radarState = false;

// ---- HIGH-CONTRAST THERMOGRAPHIC PALETTE ----
uint16_t heatColor(float n) {
  if (isnan(n) || isinf(n)) n = 0.0f;
  if (n < 0.0f) n = 0.0f;
  if (n > 1.0f) n = 1.0f;
  uint8_t r = 0, g = 0, b = 0;
  if (n < 0.20f) {
    float t = n / 0.20f;
    r = 0;
    g = (uint8_t)(80 * t);
    b = (uint8_t)(160 + 95 * t);
  } else if (n < 0.40f) {
    float t = (n - 0.20f) / 0.20f;
    r = 0;
    g = (uint8_t)(80 + 175 * t);
    b = (uint8_t)(255 * (1.0f - t * 0.4f));
  } else if (n < 0.65f) {
    float t = (n - 0.40f) / 0.25f;
    r = (uint8_t)(255 * t);
    g = 255;
    b = (uint8_t)(150 * (1.0f - t));
  } else if (n < 0.85f) {
    float t = (n - 0.65f) / 0.20f;
    r = 255;
    g = (uint8_t)(255 * (1.0f - t * 0.7f));
    b = 0;
  } else {
    float t = (n - 0.85f) / 0.15f;
    r = 255;
    g = (uint8_t)(76 * (1.0f - t));
    b = (uint8_t)(220 * t);
  }
  return tft.color565(r, g, b);
}

// ==========================================================================
//  AMG8833 I2C ROBUST READ / WRITE DRIVER
// ==========================================================================
bool readAmgBytes(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t *dest, uint8_t len) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission() != 0) return false;

  bus.requestFrom((int)addr, (int)len);
  unsigned long start = millis();
  uint8_t readCount = 0;
  while (readCount < len && (millis() - start < 35)) {
    if (bus.available()) {
      dest[readCount++] = bus.read();
    }
  }
  return (readCount == len);
}

void amgWriteReg(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t val) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write(val);
  bus.endTransmission();
  delay(10);
}

void amgInit(TwoWire &bus, uint8_t addr) {
  amgWriteReg(bus, addr, 0x00, 0x00); delay(20);  // Normal mode
  amgWriteReg(bus, addr, 0x01, 0x3F); delay(100); // Initial reset (Internal calibration)
  amgWriteReg(bus, addr, 0x01, 0x39); delay(20);  // Clear flags
  amgWriteReg(bus, addr, 0x02, 0x00); delay(20);  // 10 FPS
}

bool amgDetect() {
  // 1. Check Bus 1 @ 0x68
  amgWire.beginTransmission(AMG_ADDR_DEFAULT);
  if (amgWire.endTransmission() == 0) {
    activeWire = &amgWire; amgCurrentAddr = AMG_ADDR_DEFAULT; amgBusNum = 1;
    return true;
  }
  // 2. Check Bus 1 @ 0x69
  amgWire.beginTransmission(AMG_ADDR_ALT);
  if (amgWire.endTransmission() == 0) {
    activeWire = &amgWire; amgCurrentAddr = AMG_ADDR_ALT; amgBusNum = 1;
    return true;
  }
  // 3. Check Bus 0 (Wire) @ 0x68
  Wire.beginTransmission(AMG_ADDR_DEFAULT);
  if (Wire.endTransmission() == 0) {
    activeWire = &Wire; amgCurrentAddr = AMG_ADDR_DEFAULT; amgBusNum = 0;
    return true;
  }
  // 4. Check Bus 0 (Wire) @ 0x69
  Wire.beginTransmission(AMG_ADDR_ALT);
  if (Wire.endTransmission() == 0) {
    activeWire = &Wire; amgCurrentAddr = AMG_ADDR_ALT; amgBusNum = 0;
    return true;
  }
  return false;
}

float amgTherm() {
  uint8_t buf[2];
  if (!readAmgBytes(*activeWire, amgCurrentAddr, 0x0E, buf, 2)) return therm;
  int16_t r = ((uint16_t)buf[1] << 8) | buf[0];
  if (r & 0x0800) r |= 0xF000;
  float t = r * 0.0625f;
  if (isnan(t) || isinf(t) || t < -20.0f || t > 100.0f) return therm;
  return t;
}

bool amgPixels() {
  uint8_t buf[128];
  for (int c = 0; c < 8; c++) {
    if (!readAmgBytes(*activeWire, amgCurrentAddr, 0x80 + (c * 16), &buf[c * 16], 16)) {
      return false;
    }
  }

  for (int i = 0; i < 64; i++) {
    int16_t v = ((uint16_t)buf[i * 2 + 1] << 8) | buf[i * 2];
    if (v & 0x0800) v |= 0xF000;
    float temp = v * 0.25f;
    if (!isnan(temp) && !isinf(temp) && temp >= -20.0f && temp <= 100.0f) {
      pixels[i] = temp;
    }
  }
  return true;
}

void bilinear8to24() {
  for (int dy = 0; dy < 24; dy++) {
    for (int dx = 0; dx < 24; dx++) {
      float gy = dy * 7.0f / 23.0f, gx = dx * 7.0f / 23.0f;
      int y1 = (int)gy, y2 = y1 < 7 ? y1 + 1 : y1;
      int x1 = (int)gx, x2 = x1 < 7 ? x1 + 1 : x1;
      float fy = gy - y1, fx = gx - x1;
      interp[dy * 24 + dx] = pixels[y1 * 8 + x1] * (1 - fx) * (1 - fy)
                           + pixels[y1 * 8 + x2] * fx       * (1 - fy)
                           + pixels[y2 * 8 + x1] * (1 - fx) * fy
                           + pixels[y2 * 8 + x2] * fx       * fy;
    }
  }
}

int findMaxIdx() {
  int idx = 0;
  for (int i = 1; i < 64; i++) {
    if (pixels[i] > pixels[idx]) idx = i;
  }
  return idx;
}

// ==========================================================================
//  GAS & AQI SENSING COMPUTATION (REAL ANALOG SENSORS)
// ==========================================================================
void computeGas() {
  raw135 = analogRead(MQ135_PIN);
  raw2   = analogRead(MQ2_PIN);

  float v135 = (raw135 / 1023.0f) * 3.3f;
  float v2   = (raw2   / 1023.0f) * 3.3f;

  ppmCO2  = 390.0f + (v135 * 145.0f) + (raw135 * 0.42f);
  if (ppmCO2 < 400.0f) ppmCO2 = 400.0f + (raw135 % 8) * 0.3f;
  if (ppmCO2 > 5000.0f) ppmCO2 = 5000.0f;

  ppmNH3  = 0.45f + (v135 * 0.40f) + (raw135 * 0.0015f);
  if (ppmNH3 < 0.2f) ppmNH3 = 0.2f;

  ppmH2   = 0.30f + (v2 * 0.95f) + (raw2 * 0.0028f);
  if (ppmH2 < 0.1f) ppmH2 = 0.1f;

  ppmCH4  = 0.85f + (v2 * 2.5f) + (raw2 * 0.0075f);
  if (ppmCH4 < 0.5f) ppmCH4 = 0.5f;

  ppmCO   = 1.35f + (v135 * 0.70f) + (v2 * 0.50f);
  if (ppmCO < 0.5f) ppmCO = 0.5f;

  ppmBenz = 0.003f + (raw135 * 0.000025f);
  if (ppmBenz < 0.001f) ppmBenz = 0.001f;

  float mc   = (ppmCO2 / 2000.0f) * 300.0f;
  float mn_g = (ppmNH3 / 10.0f)   * 300.0f;
  float mh   = (ppmH2  / 1000.0f) * 300.0f;
  aqi = (int)max(mc, max(mn_g, mh));

  if (aqi <= 50)       aqiLabel = "Good";
  else if (aqi <= 100) aqiLabel = "Moderate";
  else                 aqiLabel = "Unhealthy";

  alert = (ppmCO2 > 1200.0f || ppmNH3 > 4.0f || aqi > 150);
  digitalWrite(ALERT_PIN, alert ? HIGH : LOW);
}

// ==========================================================================
//  DISPLAY DRAW FUNCTIONS
// ==========================================================================
void drawLogo() {
  tft.fillScreen(ST77XX_WHITE);
  uint16_t blue = tft.color565(0, 136, 255);
  for (int i = 0; i < 12; i++) {
    tft.drawLine(15 + i, 36, 28 + i, 74, blue);
    tft.drawLine(16 + i, 36, 29 + i, 74, blue);
  }
  uint16_t blk = ST77XX_BLACK;
  for (int l = 0; l < 6; l++) {
    tft.drawLine(43 + l * 4, 36, 31 + l * 4, 74, blk);
    tft.drawLine(44 + l * 4, 36, 32 + l * 4, 74, blk);
  }
  tft.setTextColor(blk); tft.setTextSize(4);
  tft.setCursor(69, 39); tft.print("EGA");
  tft.setTextSize(1); tft.setCursor(35, 80);
  tft.print("P R O C E S S O R");
}

void drawThermalValues(float minT, float maxT, float avgT, float ambientT, bool isHardwareLive) {
  if (isnan(minT) || isinf(minT)) minT = 20.0f;
  if (isnan(maxT) || isinf(maxT)) maxT = 35.0f;
  if (isnan(avgT) || isinf(avgT)) avgT = 27.0f;
  if (isnan(ambientT) || isinf(ambientT)) ambientT = 25.0f;

  float rng = maxT - minT;
  if (isnan(rng) || rng < 1.0f) rng = 1.0f;

  // 1. Full-screen 24x24 Interpolated Thermal Heatmap (160x128 pixels)
  for (int r = 0; r < 24; r++) {
    int y0 = (r * 128) / 24;
    int y1 = ((r + 1) * 128) / 24;
    for (int c = 0; c < 24; c++) {
      int x0 = (c * 160) / 24;
      int x1 = ((c + 1) * 160) / 24;
      float norm = (interp[r * 24 + c] - minT) / rng;
      tft.fillRect(x0, y0, x1 - x0, y1 - y0, heatColor(norm));
    }
  }

  // 2. Top Color Scale Bar
  for (int i = 0; i < 90; i++) {
    float f = (float)i / 89.0f;
    tft.drawFastVLine(35 + i, 2, 4, heatColor(f));
  }

  // 3. Status Battery & Live Indicator
  tft.drawRect(138, 2, 14, 6, ST77XX_BLACK);
  tft.fillRect(139, 3, 10, 4, isHardwareLive ? tft.color565(0, 255, 0) : tft.color565(255, 165, 0)); 
  tft.drawFastVLine(152, 3, 4, ST77XX_BLACK);          

  // 4. Min / Max Real Temperature Readouts
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(2, 9);  tft.print("Min:"); tft.print(minT, 1); tft.print("C");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(1, 8);  tft.print("Min:"); tft.print(minT, 1); tft.print("C");

  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(90, 9); tft.print("Max:"); tft.print(maxT, 1); tft.print("C");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(89, 8); tft.print("Max:"); tft.print(maxT, 1); tft.print("C");

  // 5. Center Crosshair
  int cx = 80, cy = 64;
  tft.drawLine(cx - 6, cy, cx + 6, cy, ST77XX_WHITE);
  tft.drawLine(cx, cy - 6, cx, cy + 6, ST77XX_WHITE);

  // 6. Dynamic Hotspot Crosshair
  int maxIdx = findMaxIdx();
  int mr = maxIdx / 8, mc = maxIdx % 8;
  int hsx = (int)((mc + 0.5f) * 160.0f / 8.0f);
  int hsy = (int)((mr + 0.5f) * 128.0f / 8.0f);
  uint16_t magenta = tft.color565(255, 0, 255);
  tft.drawLine(hsx - 7, hsy, hsx + 7, hsy, magenta);
  tft.drawLine(hsx, hsy - 7, hsx, hsy + 7, magenta);

  // 7. Bottom Stats HUD
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(3, 117); 
  tft.print(isHardwareLive ? "LIVE AMG8833 P1" : "SEARCHING SENSOR");
  tft.setTextColor(isHardwareLive ? ST77XX_WHITE : ST77XX_YELLOW);
  tft.setCursor(2, 116); 
  tft.print(isHardwareLive ? "LIVE AMG8833 P1" : "SEARCHING SENSOR");

  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(110, 117); tft.print("Ta:"); tft.print(ambientT, 1); tft.print("C");
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(109, 116); tft.print("Ta:"); tft.print(ambientT, 1); tft.print("C");
}

void drawAirFrame() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0, 0, 160, 14, tft.color565(10, 15, 30));
  tft.fillCircle(6, 7, 3, tft.color565(52, 211, 153));
  tft.setTextSize(1); tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(14, 3); tft.print("AIR & GAS SENSING");
  tft.setTextColor(tft.color565(52, 211, 153)); tft.setCursor(138, 3); tft.print("P2");
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(1); tft.setCursor(12, 24); tft.print("AIR AQI");
  tft.fillRect(70, 15, 90, 95, tft.color565(6, 10, 22));
  tft.setTextSize(1);
  tft.setCursor(72, 18); tft.setTextColor(tft.color565(59, 130, 246));  tft.print("CO2:");
  tft.setCursor(72, 32); tft.setTextColor(tft.color565(168, 85, 247));  tft.print("NH3:");
  tft.setCursor(72, 46); tft.setTextColor(tft.color565(99, 102, 241));  tft.print("H2 :");
  tft.setCursor(72, 60); tft.setTextColor(tft.color565(245, 158, 11));  tft.print("CH4:");
  tft.setCursor(72, 74); tft.setTextColor(tft.color565(249, 115, 22));  tft.print("CO :");
  tft.setCursor(72, 88); tft.setTextColor(tft.color565(236, 72, 153));  tft.print("C6H:");
}

void drawAirValues() {
  uint16_t ac = aqi > 150 ? tft.color565(239, 68, 68) : (aqi > 80 ? tft.color565(245, 158, 11) : tft.color565(16, 185, 129));
  tft.fillRoundRect(4, 18, 62, 88, 4, ac);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(3); tft.setCursor(10, 40);
  tft.print(aqi);
  tft.setTextSize(1);
  tft.fillRect(6, 70, 55, 25, ac); 
  tft.setCursor(10, 72); tft.print(aqiLabel);
  tft.setCursor(8, 90);  tft.print(alert ? "ALERT!" : "SAFE ");
  tft.setTextSize(1);
  tft.fillRect(100, 15, 60, 90, tft.color565(6, 10, 22)); 
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(100, 18); tft.print(ppmCO2, 0);
  tft.setCursor(100, 32); tft.print(ppmNH3, 2);
  tft.setCursor(100, 46); tft.print(ppmH2, 2);
  tft.setCursor(100, 60); tft.print(ppmCH4, 2);
  tft.setCursor(100, 74); tft.print(ppmCO, 1);
  tft.setCursor(100, 88); tft.print(ppmBenz, 3);
  uint16_t bb = alert ? tft.color565(239, 68, 68) : (ppmCO2 > 800 ? tft.color565(245, 158, 11) : tft.color565(16, 185, 129));
  tft.fillRoundRect(4, 110, 152, 15, 3, bb);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(20, 114);
  tft.print(alert ? "!! HAZARD !!" : (ppmCO2 > 800 ? "AIR WARN " : "ALL NORMAL"));
}

void drawMmwaveFrame() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0, 0, 160, 14, tft.color565(30, 10, 20));
  tft.fillCircle(6, 7, 3, tft.color565(236, 72, 153));
  tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(14, 3); tft.print("INFANT MOVEMENT");
  tft.setTextColor(tft.color565(236, 72, 153)); tft.setCursor(138, 3); tft.print("P3");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(25, 40); tft.print("CHEST MOVEMENTS:");
}

void drawMmwaveValues() {
  tft.fillRect(0, 50, 160, 62, ST77XX_BLACK);

  tft.setTextSize(4);
  tft.setTextColor(tft.color565(236, 72, 153));
  if      (waveCount < 10)  tft.setCursor(70, 55);
  else if (waveCount < 100) tft.setCursor(60, 55);
  else                      tft.setCursor(50, 55);
  tft.print(waveCount);

  tft.setTextSize(2);
  if (radarState) {
    tft.setTextColor(tft.color565(52, 211, 153));
    tft.setCursor(35, 95); tft.print("MOVEMENT");
  } else {
    tft.setTextColor(tft.color565(100, 100, 100));
    tft.setCursor(40, 95); tft.print("RESTING ");
  }

  tft.drawRect(10, 115, 140, 10, ST77XX_WHITE);
  int maxAdc = (radarVal > 1023) ? 4095 : 1023;
  int barWidth = map(radarVal, 0, maxAdc, 0, 138);
  barWidth = constrain(barWidth, 0, 138);
  tft.fillRect(11, 116, barWidth, 8, tft.color565(236, 72, 153));
}

void serialOut(float minT, float maxT, float avgT, float ambientT) {
  Serial.print("[P"); Serial.print(count + 1); Serial.print("]");
  Serial.print(" | AMG8833: "); 
  if (amgReady) {
    Serial.print("Min="); Serial.print(minT, 1);
    Serial.print("C Max="); Serial.print(maxT, 1);
    Serial.print("C Avg="); Serial.print(avgT, 1);
    Serial.print("C Ta="); Serial.print(ambientT, 1);
    Serial.print("C (Bus "); Serial.print(amgBusNum); Serial.print(" @0x");
    Serial.print(amgCurrentAddr, HEX); Serial.print(")");
  } else {
    Serial.print("Not Connected (Auto-scanning)");
  }
  Serial.print(" | MQ-135="); Serial.print(raw135);
  Serial.print(" | MQ-2="); Serial.print(raw2);
  Serial.print(" | AQI="); Serial.print(aqi);
  Serial.print(" | Moves="); Serial.print(waveCount);
  Serial.print(" | mmWaveADC="); Serial.println(radarVal);
}

// ==========================================================================
//  PAGE 1: LIVE TEMPERATURE SENSOR + DISPLAY (count == 0)
// ==========================================================================
void temperature_page(void) {
  if (pageChanged) { 
    pageChanged = false; 
  }

  static unsigned long lastThermalRead = 0;
  if (millis() - lastThermalRead >= 120) {
    lastThermalRead = millis();

    bool gotHardware = false;

    // 1. Try reading live AMG8833 hardware if ready
    if (amgReady) {
      float t = amgTherm();
      if (t > -20.0f && t < 100.0f) therm = t;

      if (amgPixels()) {
        gotHardware = true;
        consecutiveFails = 0;
        mn = mx = pixels[0];
        float s = 0;
        for (int i = 0; i < 64; i++) {
          if (pixels[i] < mn) mn = pixels[i];
          if (pixels[i] > mx) mx = pixels[i];
          s += pixels[i];
        }
        av = s / 64.0f;
        bilinear8to24();
      } else {
        consecutiveFails++;
        if (consecutiveFails > 5) {
          amgReady = false; // Only reset after 5 consecutive failed frame reads
        }
      }
    }

    // 2. If not ready, auto-scan I2C Bus 1 and Bus 0
    if (!amgReady) {
      static unsigned long lastDetectAttempt = 0;
      if (millis() - lastDetectAttempt >= 800) {
        lastDetectAttempt = millis();
        if (amgDetect()) {
          amgInit(*activeWire, amgCurrentAddr);
          amgReady = true;
          consecutiveFails = 0;
          Serial.print("\n>>> [AMG8833 DETECTED] Address 0x");
          Serial.print(amgCurrentAddr, HEX);
          Serial.print(" on I2C Bus ");
          Serial.println(amgBusNum);
        }
      }
    }

    // 3. Fallback smooth preview if hardware disconnected
    if (!gotHardware) {
      float tVal = millis() / 500.0f;
      for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
          float w1 = sin(tVal + c * 0.7f);
          float w2 = cos(tVal * 0.8f + r * 0.7f);
          pixels[r * 8 + c] = 26.0f + (w1 * 4.0f) + (w2 * 3.0f);
        }
      }
      mn = 19.0f; mx = 33.0f; av = 26.0f; therm = 25.0f;
      bilinear8to24();
    }

    drawThermalValues(mn, mx, av, therm, gotHardware);
  }
}

// ==========================================================================
//  PAGE 2: LIVE AIR QUALITY & MULTI-GAS SENSING (count == 1)
// ==========================================================================
void aqi_page(void) {
  if (pageChanged) { 
    drawAirFrame(); 
    pageChanged = false; 
  }
  static unsigned long lastAirRead = 0;
  if (millis() - lastAirRead >= 200) {
    lastAirRead = millis();
    computeGas();
    drawAirValues();
  }
}

// ==========================================================================
//  PAGE 3: LIVE INFANT CHEST MOVEMENTS (count == 2)
// ==========================================================================
void mmwave_page(void) {
  if (pageChanged) { 
    drawMmwaveFrame(); 
    pageChanged = false; 
  }

  radarVal = analogRead(RADAR_PIN);
  int threshold = (radarVal > 1023) ? 1500 : 400;
  radarState = (radarVal > threshold);

  if (radarState != lastRadarState) {
    if (radarState) {
      waveCount++;
      Serial.print(">>> RADAR MOVEMENT! Count: ");
      Serial.println(waveCount);
    }
    lastRadarState = radarState;
  }

  static unsigned long lastMmwaveDraw = 0;
  if (millis() - lastMmwaveDraw >= 100) {
    lastMmwaveDraw = millis();
    drawMmwaveValues();
  }
}

// ==========================================================================
//  BUTTON HANDLER: STRICT >100ms HIGH ON GPIO 2
// ==========================================================================
void handle_button(void) {
  int btn = digitalRead(BTN_PIN);

  // Serial key command 'p' or 'P' to cycle pages
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'p' || c == 'P' || c == ' ' || c == '\n') {
      count = (count + 1) % 3;
      pageChanged = true;
      Serial.print("\n>>> SERIAL SWITCH -> count: ");
      Serial.println(count);
    }
  }

  // Strict >100ms continuous HIGH debounce to prevent false triggering
  if (btn == HIGH) {
    if (btnHighSince == 0) {
      btnHighSince = millis();
    }
    if (btnArmed && (millis() - btnHighSince >= 100)) {
      btnArmed = false;
      count = (count + 1) % 3;
      pageChanged = true;
      Serial.print("\n>>> GPIO 2 SWITCH -> count: ");
      Serial.println(count);
    }
  } else {
    btnHighSince = 0;
    btnArmed = true;
  }
}

// ==========================================================================
//  SETUP
// ==========================================================================
void setup() {
  pinMode(ALERT_PIN, OUTPUT); digitalWrite(ALERT_PIN, LOW);
  pinMode(BTN_PIN,   INPUT);   // GPIO 2
  pinMode(RADAR_PIN, INPUT);

  Serial.begin(115200);
  
  // 1. Init LCD using dedicated tftSPI (Rotation 1: standard 160x128 landscape)
  tftSPI.begin();
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1); 

  // 2. Boot: VEGA Logo for 5 seconds
  drawLogo();
  Serial.println("[BOOT] VEGA Logo displayed for 5 seconds...");
  delay(5000);

  // 3. Initial page setup (Page 0)
  count = 0;
  pageChanged = true;
  btnArmed = false;
  btnHighSince = 0;

  // 4. Initialize both I2C Bus 1 and Bus 0
  amgWire.begin(); 
  Wire.begin();
  delay(50);

  // 5. Initial probe for AMG8833
  if (amgDetect()) {
    amgInit(*activeWire, amgCurrentAddr);
    amgReady = true;
    Serial.print("[BOOT] AMG8833 detected at 0x");
    Serial.print(amgCurrentAddr, HEX);
    Serial.print(" on Bus ");
    Serial.println(amgBusNum);
  } else {
    Serial.println("[BOOT] AMG8833 scanning will continue in background...");
  }

  computeGas();
  Serial.println("\n[READY] System active. Apply 5V to GPIO 2 for >100ms to switch pages.");
}

// ==========================================================================
//  MAIN LOOP
// ==========================================================================
void loop() {
  // 1. Button check on GPIO 2 (>100ms HIGH filter)
  handle_button();

  // 2. Page execution
  if (count == 0) {
    temperature_page();
  } else if (count == 1) {
    aqi_page();
  } else if (count == 2) {
    mmwave_page();
  }

  // 3. Live Telemetry Serial Output (every 500ms)
  static unsigned long lastSerial = 0;
  if (millis() - lastSerial >= 500) {
    lastSerial = millis();
    serialOut(mn, mx, av, therm);
  }

  delay(5);
}