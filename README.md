<h1 align="center"><em>Currently in development and not fully functional yet.</em></h1>

<h2 align="center">ESP32 Filament Dryer Controller</h2>


Firmware for a custom **4-spool filament storage/dryer** with a target chamber temperature of up to **75 °C**, running on an **ESP32-3248S035C** development board with a **3.5" ST7796 TFT capacitive touchscreen (GT911)**.


This project uses:

- [LVGL 9.x](https://lvgl.io/) for GUI
- [LovyanGFX](https://github.com/lovyan03/LovyanGFX) for fast TFT drawing
- Custom GT911 I²C driver for capacitive touch
- MOSFET-switched **PTC heater bank** and PWM fan control

---

## 📐 Features

- Capacitive touchscreen interface with **Presets / Manual / Settings** UI
- LVGL-driven **real-time temperature & humidity display**
- PWM control of:
  - **Heater bank** (GPIO22)
  - **Circulation fan** (GPIO26)
- Safety-focused:
  - Adaptive LVGL DMA buffer allocation (avoids boot fails)
  - Heater control via isolated MOSFET drivers
- Expandable:
  - Add humidity/temperature sensors via I²C
  - Add filament feedthrough ports (PTFE couplers)

---

## 🖥️ Hardware

### Main Controller

- **Board:** ESP32-3248S035C (ST7796 TFT + GT911 Touch)
- **Power:** Mean Well LRS-350-24 PSU (24 V, 14.6 A)
- **Logic:** 3.3 V native, tolerant for peripherals

### Heating

- **4x PTC heater strips** (24 V) in parallel
- MOSFET driver module (e.g. XY-MOSFET) for high-side switching

### Cooling

- **24 V blower fan** for air circulation
- Driven by ESP32 PWM via MOSFET

### Display/Touch Pins

| Function  | ESP32 Pin |
| --------- | --------- |
| TFT MOSI  | 13        |
| TFT MISO  | 12        |
| TFT SCLK  | 14        |
| TFT CS    | 15        |
| TFT DC    | 2         |
| TFT RST   | -1 (tied) |
| TFT BL    | 27        |
| I²C SDA   | 33        |
| I²C SCL   | 32        |
| Touch RST | 25        |
| Touch INT | 21        |

### Control Outputs

| Function   | ESP32 Pin |
| ---------- | --------- |
| Heater PWM | 22        |
| Fan PWM    | 26        |

---

## 📊 Wiring Topology

```text
     +---------------------+
     | Mean Well 24V PSU   |
     |  LRS-350-24         |
     +---------------------+
            | 24V
            v
    +-----------------+         +------------------+
    | PTC Heater x4   |<------->| MOSFET Driver    |
    | (parallel bank) |         | (XY-MOS)         |
    +-----------------+         +------------------+
            ^
            | 24V FAN
    +-----------------+
    | 24V Fan         |
    +-----------------+

    +-------------------------------+
    | ESP32-3248S035C Board         |
    |  - ST7796 TFT (480x320)       |
    |  - GT911 Capacitive Touch     |
    |                               |
    | GPIO22 -> Heater MOSFET TRIG  |
    | GPIO26 -> Fan MOSFET TRIG     |
    | I²C (32=SCL, 33=SDA) -> GT911 |
    +-------------------------------+
```

---

## ⚙️ Software Overview

### LVGL GUI

- **Home screen**:
  - Title + live chamber temperature/humidity readout
  - 3 buttons: **Presets / Manual / Settings**
- **Touch calibration**:
  - GT911 raw coords transformed to 480×320 LVGL space
  - Configurable `SWAP_XY`, `INVERT_X`, `INVERT_Y` flags

### PWM Control

- `ledcSetup()` + `ledcWrite()` used for:
  - Heater (1 kHz)
  - Fan (25 kHz for silent running)

### Safety

- Adaptive LVGL buffer allocation ensures DMA fits internal RAM
- If DMA allocation fails, fallback to single-buffer mode

---

## 🛠️ Building

### Prerequisites

- [VS Code](https://code.visualstudio.com/) + [PlatformIO](https://platformio.org/)
- USB data cable for ESP32-3248S035C

### Clone & Open

```bash
git clone https://github.com/gouthro/ESP32-Filament-Dryer-Controller
cd ESP32-Filament-Dryer-Controller/code
```

### Dependencies

Declared in `platformio.ini`:

```ini
lib_deps =
  lvgl/lvgl @ ^9.3.0
  lovyan03/LovyanGFX @ ^1.1.16
  Wire
  SPI
```

### Build & Upload

```bash
pio run -t upload -e esp32-3248s035c
```

### Monitor Serial

```bash
pio device monitor -b 115200
```

---

## 📱 UI Flow

1. **Boot**
   - Shows “Filament Dryer” title, live chamber readout
2. **Touch Interaction**
   - Tap **Presets** → future expansion: choose PLA/PETG/ABS profiles
   - Tap **Manual** → manual heater/fan slider control
   - Tap **Settings** → calibration, OTA updates, safety cutoffs
3. **Toast messages**
   - Tap logs visible feedback in Serial + screen popup

---

## 🧩 Next Steps / Upgrades

- Add **SHT3x / BME280** sensor integration for live humidity
- Add **preset profiles**:
  - PLA: 45 °C, 4h
  - PETG: 65 °C, 6h
  - ABS: 70 °C, 8h
- Add **OTA updates** via WiFi
- Add **graphing** (LVGL chart) of temp/RH over time

---

## 📐 License

MIT License — use, modify, and build upon freely.

