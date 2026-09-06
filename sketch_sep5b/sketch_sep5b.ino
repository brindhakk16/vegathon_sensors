/**
 * ============================================================================
 * VEGA ARIES V2 - MULTI-SENSOR LIVE DISPLAY
 * Pages:
 *   0 = VEGA Boot Logo (5 seconds)
 *   1 = AMG8833 Thermal Camera Heatmap
 *   2 = MQ-135 & MQ-2 Air Quality & Gas Monitoring
 *   3 = MR24D11C10 mmWave Radar Baby Cradle Chest Movement / Hand Wave Monitor
 * 
 * Page Switching:
 *   - GPIO2 HIGH > 500ms (connect 5V to Pin 2 for >0.5s; 10k or 20k resistor to GND)
 *   - Or send 'p', '1', '2', or '3' in the Serial Monitor (115200 baud)
 * ============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <math.h>

// ---- PINS ----
#define TFT_CS    10
#define TFT_DC     8
#define TFT_RST    9
#define BTN_PIN    2   // Connect 10k or 20k from Pin 2 to GND; apply 5V to switch
#define MQ135_PIN A0
#define MQ2_PIN   A1
#define ALERT_PIN 13

// Hardware Peripherals
SPIClass SPI(0);
TwoWire Wire(1);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
HardwareSerial radarSerial(1); // UART1 for MR24D11C10 mmWave Radar (TX1/RX1)

// ---- PAGE STATE ----
int page = 1;            // Current page (1=Thermal, 2=Air, 3=Cradle mmWave)
bool pageChanged = true; // True = redraw static frame this iteration

// ---- GPIO2 TRIGGER STATE (strict: must be HIGH for >500ms) ----
unsigned long btnHighSince = 0;
bool          btnArmed     = true;

// ---- AMG8833 THERMAL SENSOR ----
#define AMG_ADDR  0x68
float pixels[64];
float interp[24 * 24];

// ---- GAS SENSORS ----
float ppmCO2 = 412, ppmNH3 = 0.8, ppmH2 = 0.5, ppmCH4 = 1.2, ppmCO = 2.1, ppmBenz = 0.004;
int   aqi = 38;
String aqiLabel = "Good";
bool  alert = false;
int   raw135 = 0, raw2 = 0;

// ---- MR24D11C10 mmWave RADAR: BABY CRADLE / HAND WAVE TRACKING ----
int           waveCount         = 0;       // Total baby chest movements / hand waves
bool          isWaveActive      = false;   // True during active wave / chest rise
int           motionEnergy      = 0;       // 0-100 live Doppler energy
int           radarPresence     = 0;       // 0: None, 1: Present
int           radarMotion       = 0;       // 0: None, 1: Breathing/Micro, 2: Active
float         breathingRate     = 0.0f;    // Breaths / Movements Per Minute (BPM)
unsigned long lastMotionPeak    = 0;       // Timestamp of last motion peak
unsigned long lastWaveCounted   = 0;       // Timestamp of last counted wave
unsigned long totalRadarBytes   = 0;       // Total raw bytes received from radar
unsigned long totalRadarPackets = 0;       // Total valid packets decoded
String        babyState         = "CALM";  // "WAVING", "BREATHING", "CALM", "NO MOTION"

// ============================================================
// IRON RAINBOW PALETTE (Matches Handheld Thermal Imager)
// ============================================================
uint16_t heatColor(float n) {
  if (n < 0.0f) n = 0.0f;
  if (n > 1.0f) n = 1.0f;
  uint8_t r, g, b;
  if (n < 0.20f) {
    float t = n / 0.20f;
    r = (uint8_t)(70 * (1.0f - t));
    g = 0;
    b = (uint8_t)(140 + 115 * t);
  } else if (n < 0.40f) {
    float t = (n - 0.20f) / 0.20f;
    r = 0;
    g = (uint8_t)(255 * t);
    b = 255;
  } else if (n < 0.60f) {
    float t = (n - 0.40f) / 0.20f;
    r = 0;
    g = 255;
    b = (uint8_t)(255 * (1.0f - t));
  } else if (n < 0.80f) {
    float t = (n - 0.60f) / 0.20f;
    r = (uint8_t)(255 * t);
    g = 255;
    b = 0;
  } else {
    float t = (n - 0.80f) / 0.20f;
    r = 255;
    g = (uint8_t)(255 * (1.0f - t));
    b = 0;
  }
  return tft.color565(r, g, b);
}

// ============================================================
// AMG8833 LOW LEVEL DRIVER (EXACT ORIGINAL LOGIC)
// ============================================================
void amgWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AMG_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

void amgInit() {
  amgWrite(0x00, 0x00); delay(10);
  amgWrite(0x01, 0x3F); delay(10);
  amgWrite(0x01, 0x39); delay(10);
  amgWrite(0x02, 0x00); delay(10);
}

float amgTherm() {
  Wire.beginTransmission(AMG_ADDR); Wire.write(0x0E); Wire.endTransmission();
  Wire.requestFrom((int)AMG_ADDR, 2);
  if (Wire.available() < 2) return 0.0f;
  uint8_t l = Wire.read(), m = Wire.read();
  int16_t r = ((uint16_t)m << 8) | l;
  if (r & 0x0800) r |= 0xF000;
  return r * 0.0625f;
}

bool amgPixels() {
  uint8_t buf[128];
  for (int c = 0; c < 8; c++) {
    Wire.beginTransmission(AMG_ADDR); Wire.write(0x80 + c * 16);
    if (Wire.endTransmission() != 0) return false;
    Wire.requestFrom((int)AMG_ADDR, 16);
    for (int i = 0; i < 16; i++) buf[c * 16 + i] = Wire.available() ? Wire.read() : 0;
  }
  for (int i = 0; i < 64; i++) {
    int16_t v = ((uint16_t)buf[i * 2 + 1] << 8) | buf[i * 2];
    if (v & 0x0800) v |= 0xF000;
    pixels[i] = v * 0.25f;
  }
  return true;
}

void bilinear8to24() {
  for (int dy = 0; dy < 24; dy++) {
    for (int dx = 0; dx < 24; dx++) {
      float gy = dy * 7.0f / 23.0f, gx = dx * 7.0f / 23.0f;
      int y1 = (int)gy, y2 = y1 < 7 ? y1 + 1 : y1, x1 = (int)gx, x2 = x1 < 7 ? x1 + 1 : x1;
      float fy = gy - y1, fx = gx - x1;
      interp[dy * 24 + dx] = pixels[y1 * 8 + x1] * (1 - fx) * (1 - fy) +
                             pixels[y1 * 8 + x2] * fx * (1 - fy) +
                             pixels[y2 * 8 + x1] * (1 - fx) * fy +
                             pixels[y2 * 8 + x2] * fx * fy;
    }
  }
}

// ============================================================
// GAS PROCESSING (EXACT ORIGINAL LOGIC)
// ============================================================
void computeGas() {
  raw135 = analogRead(MQ135_PIN);
  raw2   = analogRead(MQ2_PIN);
  float v135 = raw135 / 1023.0f * 3.3f;
  float v2   = raw2   / 1023.0f * 3.3f;

  ppmCO2 = 390 + (v135 * 135) + (raw135 * 0.40f);
  if (ppmCO2 < 400)  ppmCO2 = 400 + (raw135 % 7) * 0.3f;
  if (ppmCO2 > 5000) ppmCO2 = 5000;

  ppmNH3 = 0.5 + (v135 * 0.38) + (raw135 * 0.0012);
  if (ppmNH3 < 0.2) ppmNH3 = 0.2;

  ppmH2  = 0.35 + (v2 * 0.9) + (raw2 * 0.0025) + (raw2 * raw2 * 0.00004);
  if (ppmH2 < 0.1) ppmH2 = 0.1;

  ppmCH4 = 0.9 + (v2 * 2.4) + (raw2 * 0.007) + (raw2 * raw2 * 0.00012);
  if (ppmCH4 < 0.5) ppmCH4 = 0.5;

  ppmCO  = 1.4 + (v135 * 0.65) + (v2 * 0.45);
  if (ppmCO < 0.5) ppmCO = 0.5;

  ppmBenz = 0.003 + (raw135 * 0.00002);
  if (ppmBenz < 0.001) ppmBenz = 0.001;

  float mc = ppmCO2 / 2000.0f * 300;
  float mn = ppmNH3 / 10.0f * 300;
  float mh = ppmH2  / 1000.0f * 300;
  aqi = (int)max(mc, max(mn, mh));

  if (aqi <= 50)       aqiLabel = "Good";
  else if (aqi <= 100) aqiLabel = "Moderate";
  else                 aqiLabel = "Unhealthy";

  alert = (ppmCO2 > 1200 || ppmNH3 > 4 || aqi > 150);
  digitalWrite(ALERT_PIN, alert ? HIGH : LOW);
}

// ============================================================
// MR24D11C10 mmWave RADAR: BABY CRADLE / HAND WAVE SENSING
// ============================================================
void sendRadarCommand(const uint8_t *cmd, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    radarSerial.write(cmd[i]);
  }
}

void activateRadarStreaming() {
  // Command 1: Open Underlying Message (activates continuous Doppler dynamic energy reporting)
  const uint8_t open_underlying[10] = {0x53, 0x59, 0x08, 0x00, 0x00, 0x01, 0x01, 0xB6, 0x54, 0x43};
  sendRadarCommand(open_underlying, 10);
  delay(10);

  // Command 2: Query movement state
  const uint8_t query_movement[10]  = {0x53, 0x59, 0x80, 0x02, 0x00, 0x01, 0x0F, 0xEE, 0x54, 0x43};
  sendRadarCommand(query_movement, 10);
  delay(10);

  // Command 3: Query body signs
  const uint8_t query_signs[10]     = {0x53, 0x59, 0x80, 0x03, 0x00, 0x01, 0x0F, 0xEF, 0x54, 0x43};
  sendRadarCommand(query_signs, 10);
}

void initRadar() {
  radarSerial.begin(115200);
  delay(50);
  activateRadarStreaming();
}

/**
 * Robust, full-spectrum MR24D11C10 frame parser.
 * Supports standard frames (0x53 0x59), presence (0x80), detail status (0x08),
 * respiration (0x81), motion signs (0x85), and alternative frames (0x55).
 */
void parseRadarByte(uint8_t b) {
  static uint8_t rxState = 0;
  static uint8_t rxBuf[48];
  static uint8_t rxIdx = 0;
  static uint8_t dataLen = 0;

  totalRadarBytes++;

  switch (rxState) {
    case 0: // Header 1
      if (b == 0x53) {
        rxBuf[0] = b;
        rxIdx = 1;
        rxState = 1;
      } else if (b == 0x55) {
        // Alternative packet header
        motionEnergy = max(motionEnergy, 35);
        lastMotionPeak = millis();
        totalRadarPackets++;
      }
      break;

    case 1: // Header 2
      if (b == 0x59) {
        rxBuf[1] = b;
        rxIdx = 2;
        rxState = 2;
      } else {
        rxState = 0;
      }
      break;

    case 2: // Control Byte (0x80, 0x08, 0x85, etc.)
      rxBuf[rxIdx++] = b;
      rxState = 3;
      break;

    case 3: // Command Byte
      rxBuf[rxIdx++] = b;
      rxState = 4;
      break;

    case 4: // Length High Byte
      rxBuf[rxIdx++] = b;
      rxState = 5;
      break;

    case 5: // Length Low Byte
      rxBuf[rxIdx++] = b;
      dataLen = b;
      if (dataLen > 36) { // Safeguard against corrupted lengths
        rxState = 0;
      } else {
        rxState = (dataLen == 0) ? 7 : 6;
      }
      break;

    case 6: // Payload Data
      rxBuf[rxIdx++] = b;
      if (rxIdx >= (6 + dataLen)) {
        rxState = 7;
      }
      break;

    case 7: // Checksum byte
      rxBuf[rxIdx++] = b;
      rxState = 8;
      break;

    case 8: // Tail 1 (0x54)
      if (b == 0x54) {
        rxBuf[rxIdx++] = b;
        rxState = 9;
      } else {
        rxState = 0;
      }
      break;

    case 9: // Tail 2 (0x43)
      if (b == 0x43) {
        totalRadarPackets++;
        uint8_t control = rxBuf[2];
        uint8_t command = rxBuf[3];

        // 1. Human Presence / Movement Information (0x80)
        if (control == 0x80) {
          if (command == 0x01) {
            radarPresence = rxBuf[6]; // 0=Nobody, 1=Somebody
          } else if (command == 0x02) {
            radarMotion = rxBuf[6];   // 0=None, 1=Stationary/Breathing, 2=Moving
            if (radarMotion == 2) {
              motionEnergy = max(motionEnergy, 75);
              lastMotionPeak = millis();
            } else if (radarMotion == 1) {
              motionEnergy = max(motionEnergy, 25);
              lastMotionPeak = millis();
            }
          } else if (command == 0x03) {
            motionEnergy = rxBuf[6];  // Body Signs Parameter (0-100)
            if (motionEnergy > 5) lastMotionPeak = millis();
          }
        }
        // 2. Underlying Detailed Movement Status (0x08)
        else if (control == 0x08) {
          if (command == 0x01 && dataLen >= 4) {
            int dynVal = (dataLen >= 4) ? rxBuf[9] : rxBuf[6];
            motionEnergy = constrain(dynVal, 0, 100);
            if (motionEnergy > 5) lastMotionPeak = millis();
          } else if (command == 0x07) {
            motionEnergy = rxBuf[6];  // Detail body sign
            if (motionEnergy > 5) lastMotionPeak = millis();
          }
        }
        // 3. Respiration Rate (0x81)
        else if (control == 0x81) {
          if (command == 0x02 && rxBuf[6] >= 10 && rxBuf[6] <= 60) {
            breathingRate = (float)rxBuf[6];
          }
        }
        // 4. Movement Signs (0x85)
        else if (control == 0x85) {
          motionEnergy = rxBuf[6];
          if (motionEnergy > 5) lastMotionPeak = millis();
        }
      }
      rxState = 0;
      break;

    default:
      rxState = 0;
      break;
  }
}

/**
 * Counts wave cycles (hand waves or baby chest/abdomen movements)
 * and calculates Breaths / Movements Per Minute (BPM)
 */
void processMovementLogic() {
  unsigned long now = millis();

  // Decay motion energy gradually if no recent Doppler bursts
  if (now - lastMotionPeak > 200 && motionEnergy > 0) {
    motionEnergy -= 3;
    if (motionEnergy < 0) motionEnergy = 0;
  }

  // Active motion threshold:
  // Motion energy >= 15, or radar reports active motion (2)
  bool currentMoving = (motionEnergy >= 15 || radarMotion == 2);

  // Transition: Calm -> Active Wave
  if (currentMoving && !isWaveActive) {
    isWaveActive = true;
    lastMotionPeak = now;
  }
  // Transition: Active -> Completed Wave Cycle
  else if (!currentMoving && isWaveActive && (now - lastMotionPeak >= 300)) {
    isWaveActive = false;

    // Debounce 400ms between wave cycles
    if (now - lastWaveCounted >= 400) {
      waveCount++;
      unsigned long dt = now - lastWaveCounted;

      if (lastWaveCounted > 0 && dt >= 400 && dt <= 10000) {
        float instBpm = 60000.0f / (float)dt;
        if (breathingRate < 1.0f) {
          breathingRate = instBpm;
        } else {
          breathingRate = (breathingRate * 0.6f) + (instBpm * 0.4f);
        }
      }
      lastWaveCounted = now;

      Serial.print(">>> [WAVE/CHEST #");
      Serial.print(waveCount);
      Serial.print("] Energy: ");
      Serial.print(motionEnergy);
      Serial.print(" | BPM: ");
      Serial.println(breathingRate, 1);
    }
  }

  // Update Status
  if (isWaveActive || motionEnergy >= 30) {
    babyState = "WAVING";
  } else if (motionEnergy >= 8 || radarMotion == 1) {
    babyState = "BREATHING";
  } else if (now - lastWaveCounted < 5000) {
    babyState = "CALM";
  } else {
    babyState = "NO MOTION";
    if (now - lastWaveCounted > 10000 && breathingRate > 0) {
      breathingRate *= 0.95f;
      if (breathingRate < 2.0f) breathingRate = 0.0f;
    }
  }
}

void readRadar() {
  while (radarSerial.available() > 0) {
    uint8_t b = radarSerial.read();
    parseRadarByte(b);
  }
  processMovementLogic();
}

// ============================================================
// LCD DRAW FUNCTIONS: LOGO
// ============================================================
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

// ============================================================
// PAGE 1: AMG8833 THERMAL CAMERA PAGE (EXACT ORIGINAL LOGIC)
// ============================================================
int findMaxIdx() {
  int idx = 0;
  for (int i = 1; i < 64; i++) {
    if (pixels[i] > pixels[idx]) idx = i;
  }
  return idx;
}

void drawThermalFrame() {
  tft.fillScreen(ST77XX_BLACK);
}

void drawThermalValues(float mn, float mx, float av, float th) {
  float rng = mx - mn;
  if (rng < 1.0f) rng = 1.0f;

  // 1. FULL-SCREEN BACKGROUND HEATMAP (160x128)
  for (int r = 0; r < 24; r++) {
    int y0 = (r * 128) / 24;
    int y1 = ((r + 1) * 128) / 24;
    for (int c = 0; c < 24; c++) {
      int x0 = (c * 160) / 24;
      int x1 = ((c + 1) * 160) / 24;
      float norm = (interp[r * 24 + c] - mn) / rng;
      tft.fillRect(x0, y0, x1 - x0, y1 - y0, heatColor(norm));
    }
  }

  // 2. TOP COLOR PALETTE BAR
  for (int i = 0; i < 90; i++) {
    float f = (float)i / 89.0f;
    tft.drawFastVLine(35 + i, 2, 4, heatColor(f));
  }

  // 3. BATTERY ICON
  tft.drawRect(138, 2, 14, 6, ST77XX_BLACK);
  tft.fillRect(139, 3, 10, 4, tft.color565(0, 255, 0));
  tft.drawFastVLine(152, 3, 4, ST77XX_BLACK);

  // 4. TOP MIN & MAX READINGS
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(2, 9);  tft.print("Min:"); tft.print(mn, 1); tft.print(" C");
  tft.setTextColor(tft.color565(10, 30, 180));
  tft.setCursor(1, 8);  tft.print("Min:"); tft.print(mn, 1); tft.print(" C");

  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(90, 9); tft.print("Max:"); tft.print(mx, 1); tft.print(" C");
  tft.setTextColor(tft.color565(10, 30, 180));
  tft.setCursor(89, 8); tft.print("Max:"); tft.print(mx, 1); tft.print(" C");

  // 5. CENTER WHITE CROSSHAIR (+)
  int cx = 80, cy = 64;
  tft.drawLine(cx - 5, cy, cx + 5, cy, ST77XX_WHITE);
  tft.drawLine(cx, cy - 5, cx, cy + 5, ST77XX_WHITE);

  // 6. HOTSPOT MAGENTA CROSSHAIR (+)
  int maxIdx = findMaxIdx();
  int mr = maxIdx / 8, mc = maxIdx % 8;
  int hsx = (int)((mc + 0.5f) * 160.0f / 8.0f);
  int hsy = (int)((mr + 0.5f) * 128.0f / 8.0f);
  uint16_t magenta = tft.color565(255, 0, 255);
  tft.drawLine(hsx - 6, hsy, hsx + 6, hsy, magenta);
  tft.drawLine(hsx, hsy - 6, hsx, hsy + 6, magenta);

  // 7. BOTTOM STATUS BAR
  float centerTemp = interp[12 * 24 + 12];

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(3, 117); tft.print("e=0.95");
  tft.setTextColor(tft.color565(0, 240, 0));
  tft.setCursor(2, 116); tft.print("e=0.95");

  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(61, 117); tft.print(centerTemp, 1); tft.print(" C");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(60, 116); tft.print(centerTemp, 1); tft.print(" C");

  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(107, 117); tft.print("Ta:"); tft.print(th, 1); tft.print(" C");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(106, 116); tft.print("Ta:"); tft.print(th, 1); tft.print(" C");
}

// ============================================================
// PAGE 2: AIR QUALITY & GAS SENSING (EXACT ORIGINAL LOGIC)
// ============================================================
void drawAirFrame() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0, 0, 160, 14, tft.color565(10, 15, 30));
  tft.fillCircle(6, 7, 3, tft.color565(52, 211, 153));
  tft.setTextSize(1); tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(14, 3); tft.print("AIR & GAS SENSING");
  tft.setTextColor(tft.color565(52, 211, 153)); tft.setCursor(138, 3); tft.print("P2");
  
  tft.setTextColor(ST77XX_BLACK); tft.setTextSize(1); tft.setCursor(12, 24); tft.print("AIR AQI");
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
  tft.setTextColor(ST77XX_BLACK); tft.setCursor(20, 114);
  tft.print(alert ? "!! HAZARD !!" : (ppmCO2 > 800 ? "AIR WARN " : "ALL NORMAL"));
}

// ============================================================
// PAGE 3: BABY CRADLE mmWAVE MONITOR UI
// ============================================================
void drawCradleFrame() {
  tft.fillScreen(ST77XX_BLACK);

  // Top Header Bar
  tft.fillRect(0, 0, 160, 14, tft.color565(10, 15, 30));
  tft.fillCircle(6, 7, 3, tft.color565(0, 220, 255));
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(14, 3);
  tft.print("CRADLE mmWAVE MONITOR");
  tft.setTextColor(tft.color565(52, 211, 153));
  tft.setCursor(144, 3);
  tft.print("P3");

  // Right Side Panel Static Labels
  tft.fillRect(78, 16, 78, 90, tft.color565(6, 10, 22));
  tft.drawRoundRect(77, 15, 80, 92, 4, tft.color565(30, 50, 90));
  tft.setTextSize(1);
  tft.setTextColor(tft.color565(255, 200, 50));
  tft.setCursor(82, 19); tft.print("RATE:");
  tft.setTextColor(tft.color565(100, 200, 255));
  tft.setCursor(82, 38); tft.print("STATE:");
  tft.setTextColor(tft.color565(200, 200, 200));
  tft.setCursor(82, 57); tft.print("ENERGY:");
}

void drawCradleValues() {
  // 1. LEFT CARD: HUGE BOLD CHEST MOVEMENT / HAND WAVE COUNT
  uint16_t cardBg = isWaveActive ? tft.color565(15, 110, 90) : tft.color565(15, 30, 65);
  tft.fillRoundRect(4, 15, 70, 92, 4, cardBg);
  tft.drawRoundRect(4, 15, 70, 92, 4, isWaveActive ? tft.color565(0, 255, 180) : tft.color565(40, 70, 130));

  tft.setTextColor(tft.color565(180, 220, 255));
  tft.setTextSize(1);
  tft.setCursor(7, 19);
  tft.print("CHEST MOVE");

  // Big Wave Count Number
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  if (waveCount < 10)       tft.setCursor(28, 38);
  else if (waveCount < 100) tft.setCursor(18, 38);
  else                      tft.setCursor(8, 38);
  tft.print(waveCount);

  // Subtitle
  tft.setTextSize(1);
  tft.setTextColor(isWaveActive ? tft.color565(100, 255, 200) : tft.color565(120, 150, 200));
  tft.setCursor(15, 72);
  tft.print("COUNT");
  tft.setCursor(8, 86);
  tft.print(isWaveActive ? "*WAVING*" : "CYCLES");

  // 2. RIGHT PANEL: LIVE TELEMETRY
  // Rate (BPM)
  tft.fillRect(116, 18, 38, 12, tft.color565(6, 10, 22));
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(116, 19);
  if (breathingRate > 0.0f) {
    tft.print((int)breathingRate);
    tft.print("m");
  } else {
    tft.print("--");
  }

  // Motion State
  tft.fillRect(82, 48, 70, 9, tft.color565(6, 10, 22));
  tft.setCursor(82, 48);
  if (babyState == "WAVING") {
    tft.setTextColor(tft.color565(0, 255, 150));
    tft.print("WAVING !");
  } else if (babyState == "BREATHING") {
    tft.setTextColor(tft.color565(50, 200, 255));
    tft.print("BREATHING");
  } else if (babyState == "CALM") {
    tft.setTextColor(tft.color565(200, 220, 240));
    tft.print("CALM");
  } else {
    tft.setTextColor(tft.color565(255, 100, 100));
    tft.print("NO MOTION");
  }

  // Energy Bar
  int barW = constrain((motionEnergy * 68) / 100, 0, 68);
  tft.fillRect(82, 68, 70, 8, tft.color565(20, 25, 40));
  uint16_t barCol = (motionEnergy > 50) ? tft.color565(255, 80, 80) : ((motionEnergy > 20) ? tft.color565(0, 255, 150) : tft.color565(0, 180, 255));
  if (barW > 0) {
    tft.fillRect(82, 68, barW, 8, barCol);
  }

  // Live Radar Packet / Byte Diagnostic
  tft.fillRect(82, 82, 70, 20, tft.color565(6, 10, 22));
  tft.setTextSize(1);
  tft.setTextColor(tft.color565(120, 160, 220));
  tft.setCursor(82, 82);
  tft.print("E:");
  tft.print(motionEnergy);
  tft.print("% ");
  tft.setCursor(82, 93);
  tft.print("PKT:");
  tft.print(totalRadarPackets);

  // 3. BOTTOM SAFETY STATUS BAR
  uint16_t sbCol;
  String   sbText;
  if (babyState == "WAVING") {
    sbCol  = tft.color565(0, 200, 120);
    sbText = "ACTIVE MOVEMENT DETECTED";
  } else if (babyState == "BREATHING") {
    sbCol  = tft.color565(16, 185, 129);
    sbText = "CHEST RHYTHM: HEALTHY";
  } else if (babyState == "CALM") {
    sbCol  = tft.color565(0, 140, 220);
    sbText = "BABY RESTING QUIETLY";
  } else {
    sbCol  = tft.color565(239, 68, 68);
    sbText = "!! STILL - CHECK CRADLE !!";
  }

  tft.fillRoundRect(4, 110, 152, 15, 3, sbCol);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(6, 114);
  tft.print(sbText);
}

// ============================================================
// SERIAL OUTPUT (ORIGINAL FORMAT + mmWAVE)
// ============================================================
void serialOut(float mn, float mx, float av, float th) {
  Serial.print("ActivePage: "); Serial.println(page);
  Serial.print("CO2: "); Serial.print(ppmCO2, 1);
  Serial.print(" NH3: "); Serial.print(ppmNH3, 2);
  Serial.print(" AQI: "); Serial.println(aqi);
  Serial.print("mmWave Waves: "); Serial.print(waveCount);
  Serial.print(" | BPM: "); Serial.print(breathingRate, 1);
  Serial.print(" | State: "); Serial.print(babyState);
  Serial.print(" | Energy: "); Serial.print(motionEnergy);
  Serial.print("% | Pkts: "); Serial.print(totalRadarPackets);
  Serial.print(" | Bytes: "); Serial.println(totalRadarBytes);
  Serial.println("--- AMG8833 8x8 Thermal Data ---");
  Serial.print("Min Temp: "); Serial.print(mn, 1);
  Serial.print(" Max Temp: "); Serial.print(mx, 1);
  Serial.print(" Avg Temp: "); Serial.println(av, 1);
  for (int r = 0; r < 8; r++) {
    Serial.print("[ ");
    for (int c = 0; c < 8; c++) {
      Serial.print(pixels[r * 8 + c], 1);
      if (c < 7) Serial.print(", ");
    }
    Serial.println(" ]");
  }
}

// ============================================================
// SETUP (EXACT ORIGINAL SEQUENCE + mmWAVE INIT)
// ============================================================
void setup() {
  pinMode(ALERT_PIN, OUTPUT);
  digitalWrite(ALERT_PIN, LOW);
  pinMode(BTN_PIN, INPUT);   // Plain INPUT — needs 10k or 20k to GND externally

  Serial.begin(115200);
  SPI.begin();
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);        // Landscape 160x128

  // ---- LOGO for 5 seconds ----
  drawLogo();
  Serial.println("ActivePage: 0");
  Serial.println("[BOOT] VEGA Logo 5 seconds...");
  delay(5000);

  // ---- Init sensors ----
  Wire.begin();
  delay(50);
  amgInit();
  computeGas();
  initRadar();

  // ---- Switch to Page 1 ----
  page = 1;
  pageChanged = true;        // Trigger frame draw on first loop
  btnArmed = true;
  btnHighSince = 0;
  Serial.println("ActivePage: 1");
  Serial.println("[READY] Page 1 LOCKED. Apply 5V to Pin2 for >500ms to switch.");
}

// ============================================================
// MAIN LOOP (NON-BLOCKING CONTINUOUS SENSING)
// ============================================================
void loop() {
  unsigned long now = millis();

  // ============================================================
  // 1. DRAIN mmWave RADAR BYTES CONTINUOUSLY (ZERO FIFO OVERFLOW)
  // ============================================================
  readRadar();

  // ============================================================
  // 2. GPIO2 & SERIAL TRIGGER: 3-PAGE CYCLING (1 -> 2 -> 3 -> 1)
  // ============================================================
  int btn = digitalRead(BTN_PIN);

  if (btn == HIGH) {
    if (btnHighSince == 0) {
      btnHighSince = now;
    }
    unsigned long held = now - btnHighSince;
    if (btnArmed && held >= 500) {
      btnArmed = false;
      page++;
      if (page > 3) page = 1;
      pageChanged = true;
      Serial.print("\n>>> PIN2 HIGH >500ms -> Page: ");
      Serial.println(page);
    }
  } else {
    btnHighSince = 0;
    btnArmed = true;
  }

  // Serial Monitor Command switch fallback ('p', '1', '2', '3')
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'p' || c == 'P' || c == '\n') {
      page++;
      if (page > 3) page = 1;
      pageChanged = true;
      Serial.print("\n>>> [SERIAL] Switched to Page: ");
      Serial.println(page);
    } else if (c == '1' || c == '2' || c == '3') {
      page = c - '0';
      pageChanged = true;
      Serial.print("\n>>> [SERIAL] Switched to Page: ");
      Serial.println(page);
    }
  }

  // ============================================================
  // 3. PERIODIC SENSOR READ & LCD RENDER (EVERY 90ms)
  // ============================================================
  static unsigned long lastSensorTick = 0;
  static float lastMn = 25.0f, lastMx = 35.0f, lastAv = 30.0f, lastTherm = 28.0f;

  if (now - lastSensorTick >= 90) {
    lastSensorTick = now;

    computeGas();

    lastTherm = amgTherm();
    if (amgPixels()) {
      lastMn = lastMx = pixels[0];
      float s = 0.0f;
      for (int i = 0; i < 64; i++) {
        if (pixels[i] < lastMn) lastMn = pixels[i];
        if (pixels[i] > lastMx) lastMx = pixels[i];
        s += pixels[i];
      }
      lastAv = s / 64.0f;
      bilinear8to24();
    }

    // DRAW: Frame (once on page change) + Values (every loop)
    if (page == 1) {
      if (pageChanged) { drawThermalFrame(); pageChanged = false; }
      drawThermalValues(lastMn, lastMx, lastAv, lastTherm);
    } else if (page == 2) {
      if (pageChanged) { drawAirFrame(); pageChanged = false; }
      drawAirValues();
    } else if (page == 3) {
      if (pageChanged) { drawCradleFrame(); pageChanged = false; }
      drawCradleValues();
    }
  }

  // ============================================================
  // 4. PERIODIC SERIAL TELEMETRY & RADAR KEEP-ALIVE (EVERY 1 SEC)
  // ============================================================
  static unsigned long lastLog = 0;
  if (now - lastLog >= 1000) {
    lastLog = now;
    serialOut(lastMn, lastMx, lastAv, lastTherm);
    // Refresh radar keep-alive query
    activateRadarStreaming();
  }
}