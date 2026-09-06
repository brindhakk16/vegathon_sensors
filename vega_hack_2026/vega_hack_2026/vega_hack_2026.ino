/**
 * ================================================================
 *  VEGA ARIES V2 — MULTI-SENSOR EDGE DISPLAY
 *  Boot  : VEGA Logo (5 seconds)
 *  Page 0: Temperature (AMG8833 Thermal Camera on SDA1/SCL1)
 *  Page 1: Air Quality (MQ-135 on A0, MQ-2 on A1)
 *  Page 2: mmWave      (MR24D11C10 Radar on A3)
 *  Switch: Apply 5V to GPIO 2 for >100ms (10k pull-down to GND)
 * ================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <math.h>

// ---- PINS ----
#define TFT_CS    10
#define TFT_DC     8
#define TFT_RST    9
#define BTN_PIN    2   // GPIO 2: Apply 5V for >100ms to switch count
#define RADAR_PIN A3   // mmWave Radar MR24D11C10 on A3
#define MQ135_PIN A0
#define MQ2_PIN   A1
#define ALERT_PIN 13

// ---- HARDWARE INSTANCES ----
// 1. Linker dependency: Adafruit_GFX expects a global 'SPI' to exist.
SPIClass SPI(0);

// 2. Actual SPI used for TFT on VEGA (SCLK0 / MOSI0)
SPIClass tftSPI(0);
Adafruit_ST7735 tft = Adafruit_ST7735(&tftSPI, TFT_CS, TFT_DC, TFT_RST);

// 3. I2C Bus-1 for AMG8833 (SDA1 / SCL1)
TwoWire amgWire(1);

// ---- PAGE COUNT STATE ----
// count == 0 : Temperature Page (AMG8833)
// count == 1 : Air Quality Page (MQ-135 + MQ-2)
// count == 2 : mmWave Page (MR24D11C10)
int  count = 0;            
bool pageChanged = true; 

// ---- GPIO 2 TRIGGER STATE (>100ms debounce) ----
unsigned long btnHighSince = 0;
bool          btnArmed     = true;   

// ---- AMG8833 ----
#define AMG_ADDR  0x68
float pixels[64];
float interp[24*24];
bool amgReady = false;  // Prevents I2C hang if sensor disconnected

// ---- GAS ----
float ppmCO2=412, ppmNH3=0.8, ppmH2=0.5, ppmCH4=1.2, ppmCO=2.1, ppmBenz=0.004;
int   aqi=38;
String aqiLabel="Good";
bool  alert=false;
int   raw135=0, raw2=0;

// ---- mmWAVE ----
int  waveCount = 0;
bool lastRadarState = false;
int  radarVal = 0;
bool radarState = false;

// ---- IRON RAINBOW PALETTE ----
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

// ---- AMG8833 LOW LEVEL ----
void amgWrite(uint8_t reg, uint8_t val) {
  amgWire.beginTransmission(AMG_ADDR); amgWire.write(reg); amgWire.write(val); amgWire.endTransmission();
}
bool amgDetect() {
  amgWire.beginTransmission(AMG_ADDR);
  return (amgWire.endTransmission() == 0);
}
void amgInit() {
  amgWrite(0x00,0x00); delay(10);
  amgWrite(0x01,0x3F); delay(10);
  amgWrite(0x01,0x39); delay(10);
  amgWrite(0x02,0x00); delay(10);
}
float amgTherm() {
  amgWire.beginTransmission(AMG_ADDR); amgWire.write(0x0E); amgWire.endTransmission();
  amgWire.requestFrom((int)AMG_ADDR,2);
  if(amgWire.available()<2) return 0;
  uint8_t l=amgWire.read(), m=amgWire.read();
  int16_t r=((uint16_t)m<<8)|l; if(r&0x0800)r|=0xF000;
  return r*0.0625f;
}
bool amgPixels() {
  uint8_t buf[128];
  for(int c=0;c<8;c++){
    amgWire.beginTransmission(AMG_ADDR); amgWire.write(0x80+c*16);
    if(amgWire.endTransmission()!=0) return false;
    amgWire.requestFrom((int)AMG_ADDR,16);
    for(int i=0;i<16;i++) buf[c*16+i]=amgWire.available()?amgWire.read():0;
  }
  for(int i=0;i<64;i++){
    int16_t v=((uint16_t)buf[i*2+1]<<8)|buf[i*2];
    if(v&0x0800)v|=0xF000;
    pixels[i]=v*0.25f;
  }
  return true;
}
void bilinear8to24() {
  for(int dy=0;dy<24;dy++) for(int dx=0;dx<24;dx++){
    float gy=dy*7.0f/23.0f, gx=dx*7.0f/23.0f;
    int y1=(int)gy,y2=y1<7?y1+1:y1,x1=(int)gx,x2=x1<7?x1+1:x1;
    float fy=gy-y1,fx=gx-x1;
    interp[dy*24+dx]=pixels[y1*8+x1]*(1-fx)*(1-fy)+pixels[y1*8+x2]*fx*(1-fy)
                    +pixels[y2*8+x1]*(1-fx)*fy   +pixels[y2*8+x2]*fx*fy;
  }
}

// ---- GAS ----
void computeGas(){
  raw135=analogRead(MQ135_PIN); raw2=analogRead(MQ2_PIN);
  float v135=raw135/1023.0f*3.3f, v2=raw2/1023.0f*3.3f;
  ppmCO2=390+(v135*135)+(raw135*0.40f); if(ppmCO2<400)ppmCO2=400+(raw135%7)*0.3f; if(ppmCO2>5000)ppmCO2=5000;
  ppmNH3=0.5+(v135*0.38)+(raw135*0.0012); if(ppmNH3<0.2)ppmNH3=0.2;
  ppmH2=0.35+(v2*0.9)+(raw2*0.0025)+(raw2*raw2*0.00004); if(ppmH2<0.1)ppmH2=0.1;
  ppmCH4=0.9+(v2*2.4)+(raw2*0.007)+(raw2*raw2*0.00012); if(ppmCH4<0.5)ppmCH4=0.5;
  ppmCO=1.4+(v135*0.65)+(v2*0.45); if(ppmCO<0.5)ppmCO=0.5;
  ppmBenz=0.003+(raw135*0.00002); if(ppmBenz<0.001)ppmBenz=0.001;
  float mc=ppmCO2/2000.0f*300; float mn=ppmNH3/10.0f*300; float mh=ppmH2/1000.0f*300;
  aqi=(int)max(mc,max(mn,mh));
  if(aqi<=50)aqiLabel="Good"; else if(aqi<=100)aqiLabel="Moderate"; else aqiLabel="Unhealthy";
  alert=(ppmCO2>1200||ppmNH3>4||aqi>150);
  digitalWrite(ALERT_PIN,alert?HIGH:LOW);
}

// ---- DRAW FUNCTIONS ----
void drawLogo(){
  tft.fillScreen(ST77XX_WHITE);
  uint16_t blue=tft.color565(0,136,255);
  for(int i=0;i<12;i++){
    tft.drawLine(15+i,36,28+i,74,blue);
    tft.drawLine(16+i,36,29+i,74,blue);
  }
  uint16_t blk=ST77XX_BLACK;
  for(int l=0;l<6;l++){
    tft.drawLine(43+l*4,36,31+l*4,74,blk);
    tft.drawLine(44+l*4,36,32+l*4,74,blk);
  }
  tft.setTextColor(blk); tft.setTextSize(4);
  tft.setCursor(69,39); tft.print("EGA");
  tft.setTextSize(1); tft.setCursor(35,80);
  tft.print("P R O C E S S O R");
}

int findMaxIdx(){
  int idx = 0;
  for(int i = 1; i < 64; i++) {
    if(pixels[i] > pixels[idx]) idx = i;
  }
  return idx;
}

void drawThermalFrame(){
  tft.fillScreen(ST77XX_BLACK);
}

void drawThermalValues(float mn, float mx, float av, float th){
  float rng = mx - mn;
  if(rng < 1.0f) rng = 1.0f;

  for(int r = 0; r < 24; r++){
    int y0 = (r * 128) / 24;
    int y1 = ((r + 1) * 128) / 24;
    for(int c = 0; c < 24; c++){
      int x0 = (c * 160) / 24;
      int x1 = ((c + 1) * 160) / 24;
      float norm = (interp[r * 24 + c] - mn) / rng;
      tft.fillRect(x0, y0, x1 - x0, y1 - y0, heatColor(norm));
    }
  }

  for(int i = 0; i < 90; i++){
    float f = (float)i / 89.0f;
    tft.drawFastVLine(35 + i, 2, 4, heatColor(f));
  }

  tft.drawRect(138, 2, 14, 6, ST77XX_BLACK);
  tft.fillRect(139, 3, 10, 4, tft.color565(0, 255, 0)); 
  tft.drawFastVLine(152, 3, 4, ST77XX_BLACK);          

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(2, 9);  tft.print("Min:"); tft.print(mn, 1); tft.print(" C");
  tft.setTextColor(tft.color565(10, 30, 180));
  tft.setCursor(1, 8);  tft.print("Min:"); tft.print(mn, 1); tft.print(" C");

  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(90, 9); tft.print("Max:"); tft.print(mx, 1); tft.print(" C");
  tft.setTextColor(tft.color565(10, 30, 180));
  tft.setCursor(89, 8); tft.print("Max:"); tft.print(mx, 1); tft.print(" C");

  int cx = 80, cy = 64;
  tft.drawLine(cx - 5, cy, cx + 5, cy, ST77XX_WHITE);
  tft.drawLine(cx, cy - 5, cx, cy + 5, ST77XX_WHITE);

  int maxIdx = findMaxIdx();
  int mr = maxIdx / 8, mc = maxIdx % 8;
  int hsx = (int)((mc + 0.5f) * 160.0f / 8.0f);
  int hsy = (int)((mr + 0.5f) * 128.0f / 8.0f);
  uint16_t magenta = tft.color565(255, 0, 255);
  tft.drawLine(hsx - 6, hsy, hsx + 6, hsy, magenta);
  tft.drawLine(hsx, hsy - 6, hsx, hsy + 6, magenta);

  float centerTemp = interp[12 * 24 + 12];
  if (!amgReady) centerTemp = 0.0f; // Indicate missing sensor

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
  
  if(!amgReady) {
    tft.setTextColor(ST77XX_BLACK);
    tft.setCursor(41, 31); tft.print("SENSOR NOT FOUND");
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(40, 30); tft.print("SENSOR NOT FOUND");
  }
}

void drawAirFrame(){
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0,0,160,14,tft.color565(10,15,30));
  tft.fillCircle(6,7,3,tft.color565(52,211,153));
  tft.setTextSize(1); tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(14,3); tft.print("AIR & GAS SENSING");
  tft.setTextColor(tft.color565(52,211,153)); tft.setCursor(138,3); tft.print("P2");
  tft.setTextColor(ST77XX_BLACK); tft.setTextSize(1); tft.setCursor(12,24); tft.print("AIR AQI");
  tft.fillRect(70,15,90,95,tft.color565(6,10,22));
  tft.setTextSize(1);
  tft.setCursor(72,18); tft.setTextColor(tft.color565(59,130,246));  tft.print("CO2:");
  tft.setCursor(72,32); tft.setTextColor(tft.color565(168,85,247));  tft.print("NH3:");
  tft.setCursor(72,46); tft.setTextColor(tft.color565(99,102,241));  tft.print("H2 :");
  tft.setCursor(72,60); tft.setTextColor(tft.color565(245,158,11));  tft.print("CH4:");
  tft.setCursor(72,74); tft.setTextColor(tft.color565(249,115,22));  tft.print("CO :");
  tft.setCursor(72,88); tft.setTextColor(tft.color565(236,72,153));  tft.print("C6H:");
}

void drawAirValues(){
  uint16_t ac=aqi>150?tft.color565(239,68,68):(aqi>80?tft.color565(245,158,11):tft.color565(16,185,129));
  tft.fillRoundRect(4,18,62,88,4,ac);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(3); tft.setCursor(10,40);
  tft.print(aqi);
  tft.setTextSize(1);
  tft.fillRect(6,70,55,25,ac); 
  tft.setCursor(10,72); tft.print(aqiLabel);
  tft.setCursor(8,90);  tft.print(alert?"ALERT!" : "SAFE ");
  tft.setTextSize(1);
  tft.fillRect(100,15,60,90,tft.color565(6,10,22)); 
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(100,18); tft.print(ppmCO2,0);
  tft.setCursor(100,32); tft.print(ppmNH3,2);
  tft.setCursor(100,46); tft.print(ppmH2,2);
  tft.setCursor(100,60); tft.print(ppmCH4,2);
  tft.setCursor(100,74); tft.print(ppmCO,1);
  tft.setCursor(100,88); tft.print(ppmBenz,3);
  uint16_t bb=alert?tft.color565(239,68,68):(ppmCO2>800?tft.color565(245,158,11):tft.color565(16,185,129));
  tft.fillRoundRect(4,110,152,15,3,bb);
  tft.setTextColor(ST77XX_BLACK); tft.setCursor(20,114);
  tft.print(alert?"!! HAZARD !!" : (ppmCO2>800?"AIR WARN " : "ALL NORMAL"));
}

void drawMmwaveFrame(){
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0,0,160,14,tft.color565(30,10,20));
  tft.fillCircle(6,7,3,tft.color565(236,72,153));
  tft.setTextSize(1); tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(14,3); tft.print("INFANT MOVEMENT");
  tft.setTextColor(tft.color565(236,72,153)); tft.setCursor(138,3); tft.print("P3");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(25,40); tft.print("CHEST MOVEMENTS:");
}

void drawMmwaveValues(){
  tft.fillRect(0, 50, 160, 78, ST77XX_BLACK);

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
  int barWidth = map(radarVal, 0, 4095, 0, 138);
  barWidth = constrain(barWidth, 0, 138);
  tft.fillRect(11, 116, barWidth, 8, tft.color565(236, 72, 153));
}

void serialOut(float mn,float mx,float av,float th){
  Serial.print("count: "); Serial.print(count);
  Serial.print(" | CO2: "); Serial.print(ppmCO2,1);
  Serial.print(" | NH3: "); Serial.print(ppmNH3,2);
  Serial.print(" | AQI: "); Serial.print(aqi);
  Serial.print(" | Moves: "); Serial.print(waveCount);
  Serial.print(" | Temp: "); Serial.println(av,1);
}

// ==========================================================================
//  FUNCTION 1: TEMPERATURE SENSOR + TEMPERATURE DISPLAY (count == 0)
// ==========================================================================
void temperature_page(void) {
  if (pageChanged) { 
    drawThermalFrame(); 
    pageChanged = false; 
  }

  float therm = 0;
  float mn = 25, mx = 35, av = 30;

  // ONLY read I2C if sensor was detected to prevent HANGING
  if (amgReady) {
    therm = amgTherm();
    if (amgPixels()) {
      mn = mx = pixels[0]; float s = 0;
      for(int i = 0; i < 64; i++){
        if(pixels[i] < mn) mn = pixels[i];
        if(pixels[i] > mx) mx = pixels[i];
        s += pixels[i];
      }
      av = s / 64.0f;
      bilinear8to24();
    }
  } else {
    // Fill dummy interpolation so screen isn't broken
    for(int i = 0; i < 576; i++) interp[i] = 25.0f;
  }

  drawThermalValues(mn, mx, av, therm);
}

// ==========================================================================
//  FUNCTION 2: GAS SENSOR + AQI DISPLAY (count == 1)
// ==========================================================================
void aqi_page(void) {
  if (pageChanged) { 
    drawAirFrame(); 
    pageChanged = false; 
  }
  computeGas();
  drawAirValues();
}

// ==========================================================================
//  FUNCTION 3: mmWAVE SENSOR + mmWAVE DISPLAY (count == 2)
// ==========================================================================
void mmwave_page(void) {
  radarVal = analogRead(RADAR_PIN);
  radarState = (radarVal > 2000);

  if (radarState != lastRadarState) {
    if (radarState) {
      waveCount++;
      Serial.print(">>> RADAR MOVEMENT! Count: ");
      Serial.println(waveCount);
    }
    lastRadarState = radarState;
  }

  if (pageChanged) { 
    drawMmwaveFrame(); 
    pageChanged = false; 
  }
  drawMmwaveValues();
}

// ==========================================================================
//  BUTTON HANDLER: GPIO 2 (>100ms HIGH) -> count switching
// ==========================================================================
void handle_button(void) {
  // Read GPIO 2 (and also allow Pin 31 / A2 as fallback)
  int btn = digitalRead(BTN_PIN);
  int btn31 = digitalRead(31);
  int btnA = analogRead(A2);
  bool isPressed = (btn == HIGH || btn31 == HIGH || btnA > 500);

  // Serial command 'p' to switch pages
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'p' || c == 'P' || c == '\n') {
      count++;
      if (count > 2) count = 0;
      pageChanged = true;
      Serial.print("\n>>> SERIAL SWITCH -> count: ");
      Serial.println(count);
    }
  }

  // Button debounce > 100ms
  if (isPressed) {
    if (btnHighSince == 0) {
      btnHighSince = millis();
    }
    unsigned long held = millis() - btnHighSince;
    if (btnArmed && held >= 100) {       
      btnArmed = false;                  
      count++;
      if (count > 2) count = 0;       
      pageChanged = true;               
      Serial.print("\n>>> GPIO 2 PRESSED (>100ms) -> count: ");
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
  
  // 1. Init LCD using dedicated tftSPI instance
  tftSPI.begin();
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1); 

  // 2. Boot: VEGA Logo for 5 seconds
  drawLogo();
  Serial.println("[BOOT] VEGA Logo displayed for 5 seconds...");
  delay(5000);

  // 3. Init AMG8833 on amgWire
  amgWire.begin(); 
  delay(50);
  if (amgDetect()) {
    amgReady = true;
    amgInit();
    Serial.println("[AMG] Sensor initialized successfully.");
  } else {
    amgReady = false;
    Serial.println("[AMG] SENSOR NOT DETECTED! Check SDA1/SCL1 wiring.");
  }

  computeGas();

  // 4. Set count = 0 (Temperature page)
  count = 0;
  pageChanged = true;  
  btnArmed = true;
  btnHighSince = 0;
  Serial.println("\n[READY] count = 0 (Temperature Page) active. Apply 5V to GPIO 2 >100ms to switch.");
}

// ==========================================================================
//  LOOP
// ==========================================================================
void loop() {
  // 1. Check Button on GPIO 2 (>100ms)
  handle_button();

  // 2. Count-based 3-function switching
  if (count == 0) {
    temperature_page();
  } else if (count == 1) {
    aqi_page();
  } else if (count == 2) {
    mmwave_page();
  }

  // 3. Telemetry Output
  float therm = 0;
  float mn = 25, mx = 35, av = 30;
  serialOut(mn, mx, av, therm);
  
  delay(150);
}