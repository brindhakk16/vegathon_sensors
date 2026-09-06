# Vegathon Sensors

Repository containing Arduino sketches, sensor integrations, firmware, and real-time monitoring web dashboard developed for the VEGA ARIES v2 board.

## Repository Contents

- **`Final/vega_hack_2026/`**
  - **Web Dashboard**: Real-time interface (`index.html`, `app.js`, `style.css`, `server.js`) and visual assets for monitoring sensor metrics.
  - **`mq135_infant_monitor/`**: Air quality and infant respiratory environment monitoring firmware.
  - **`mq2_smoke_gas_monitor/`**: Smoke, flammable gas, and LPG detection firmware.
  - **`tft_diag/`**: Diagnostic utility for ST7735 TFT display.
  - **`vega_hack_2026/`**: Multi-sensor integration firmware.
  - **`vega_hack_2026_lcd/`**: LCD display integration firmware.
- **`sketch_sep5b/`**
  - **`sketch_sep5b.ino`**: Multi-sensor display firmware integration (thermal camera AMG8833, air quality MQ135 & MQ2, breath counter with mmWave radar).
- **`vega_hack_2026/`**
  - Source dashboard assets and firmware sketches.

## Hardware Platform

- **MCU Board**: VEGA ARIES v2 (RISC-V)
- **Sensors**:
  - AMG8833 Grid-EYE Thermal Camera
  - MR24D11C10 mmWave Radar (Breath Detection)
  - MQ-135 Air Quality Sensor
  - MQ-2 Smoke/Gas Sensor
  - ST7735 Color TFT LCD Display
