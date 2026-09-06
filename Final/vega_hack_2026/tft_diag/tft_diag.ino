/**
 * tft_diag.ino — ST7735 1.8" Display Diagnostic for VEGA Aries V2
 *
 * ═══════════════════════════════════════════════════════════════
 *  IMPORTANT — RE-WIRE YOUR DISPLAY TO THESE PINS:
 *
 *  Display pin │ Connect to on VEGA Aries V2 board
 *  ────────────┼──────────────────────────────────────────────
 *  VCC         │ 3.3V
 *  GND         │ GND
 *  SCK / CLK   │ SCLK-0  (the pad labelled "SCLK" or "SCLK0")
 *  MOSI / SDA  │ MOSI-0  (the pad labelled "MOSI" or "MOSI0")
 *  CS          │ Pin 10  (GPIO-10, can stay as-is)
 *  DC / RS     │ Pin 9   (GPIO-9,  can stay as-is)
 *  RST         │ Pin 8   (GPIO-8,  can stay as-is)
 *  BL / LED    │ 3.3V
 *
 *  ⚠ Pins 7 and 11 on the VEGA Aries V2 digital header are
 *    plain GPIO — they are NOT connected to the SPI hardware.
 *    You MUST move SCK and MOSI to the dedicated SCLK-0 / MOSI-0
 *    pads (usually on the same header, look for the silkscreen
 *    labels "MOSI0" and "SCLK0").
 * ═══════════════════════════════════════════════════════════════
 *
 * This sketch uses HARDWARE SPI (SPI.begin()) and tries every
 * possible init tab so you can confirm the display is working.
 * Watch the screen: you should see RED → GREEN → BLUE cycling.
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// ── CS / DC / RST can stay on any GPIO ────────────────────────
#define TFT_CS   10
#define TFT_DC    9
#define TFT_RST   8

// Hardware-SPI constructor (MOSI & SCK come from the SPI peripheral)
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

// ── Manual hardware reset ──────────────────────────────────────
void hardReset() {
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH); delay(20);
  digitalWrite(TFT_RST, LOW);  delay(20);
  digitalWrite(TFT_RST, HIGH); delay(200);
}

// ── Flash three colours and print which test is running ────────
void flashTest(uint8_t tab, const char* name) {
  Serial.print("Testing initR tab: ");
  Serial.println(name);

  hardReset();
  SPI.begin();               // ensure HW SPI is active before each init
  tft.initR(tab);
  tft.setRotation(0);

  tft.fillScreen(ST77XX_RED);
  Serial.println("  -> RED");
  delay(700);

  tft.fillScreen(ST77XX_GREEN);
  Serial.println("  -> GREEN");
  delay(700);

  tft.fillScreen(ST77XX_BLUE);
  Serial.println("  -> BLUE");
  delay(700);

  tft.fillScreen(ST77XX_BLACK);
  delay(300);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("====================================");
  Serial.println(" VEGA Aries V2 — ST7735 DIAGNOSTIC");
  Serial.println("====================================");
  Serial.println("Make sure SCK -> SCLK-0 and MOSI -> MOSI-0!");
  Serial.println();

  // Try all common init tabs in sequence
  flashTest(INITR_GREENTAB,   "INITR_GREENTAB");
  flashTest(INITR_BLACKTAB,   "INITR_BLACKTAB");
  flashTest(INITR_REDTAB,     "INITR_REDTAB");
  flashTest(INITR_18GREENTAB, "INITR_18GREENTAB");
  flashTest(INITR_MINI160x80, "INITR_MINI160x80");

  Serial.println();
  Serial.println("All tests done.");
  Serial.println("Tell us which test showed colour on screen.");
}

void loop() { /* nothing */ }
