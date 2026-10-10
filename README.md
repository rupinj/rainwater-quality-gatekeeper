# Self-Calibrating Rainwater Harvesting Quality Gatekeeper

Multi-parameter, ESP32-based first-flush controller for rooftop rainwater
harvesting. Instead of discarding the first N minutes of runoff on a fixed
timer, this system measures turbidity, pH, TDS/conductivity and flow in
real time and routes water to **Reject**, **Garden-use**, or **Storage**
based on live, filtered, confidence-scored sensor data.

> Prototype thresholds are illustrative only and are **not** drinking-water
> certification limits. Calibrate every sensor against known reference
> standards before drawing any conclusions about water safety.

## Repository structure

```
rainwater-quality-gatekeeper/
├── firmware/
│   └── rainwater_gatekeeper.ino   # Main ESP32 firmware (Arduino)
├── README.md
└── docs/
    └── progress_report.docx       # (not committed — see note below)
```

## Hardware

| Component | Pin (ESP32) | Notes |
|---|---|---|
| Turbidity sensor 1 | GPIO34 | Essential |
| Turbidity sensor 2 | GPIO35 | Optional — redundancy/cross-validation |
| pH probe | GPIO32 | Recommended |
| TDS/conductivity sensor | GPIO33 | Recommended |
| Flow sensor (YF-S201 or similar) | GPIO4 | Interrupt pin |
| 3-way servo valve | GPIO13 | PWM |
| OLED (SSD1306, I2C) | SDA 21 / SCL 22 | 0.96" |
| Buzzer | GPIO25 | Fault alert |
| RGB LED | R14 / G12 / B27 | Status indicator |
| microSD module | CS 5 (VSPI) | Data logging |

## Software status: complete (prototype firmware)

The firmware in `firmware/rainwater_gatekeeper.ino` implements:

- [x] Multi-sensor analog reading (turbidity ×2, pH, TDS) + digital flow pulse counting
- [x] 5-sample median filtering per sensor
- [x] Adaptive turbidity stability check (ΔT < ε for N consecutive samples) in place of a fixed timer
- [x] Multi-parameter confidence scoring with penalties for invalid readings, unrealistic drift, missing calibration, and sensor disagreement
- [x] Decision routine: REJECT / GARDEN / STORAGE / HOLD (fail-safe on low confidence or flow loss)
- [x] Servo-driven 3-way valve actuation
- [x] OLED live status + confidence display
- [x] microSD CSV logging (timestamp, all readings, confidence, route, reason)
- [x] Buzzer + RGB LED fault/status alerts

## Required Arduino libraries

Install from the Arduino IDE Library Manager:

- `Adafruit GFX Library`
- `Adafruit SSD1306`
- `ESP32Servo`
- `SD` (bundled with the ESP32 board package)

Board package: **ESP32 by Espressif Systems** (install via Boards Manager,
then select an "ESP32 Dev Module" board).

## Calibrating the sensors

The `readTurbidityRaw()`, `readPHRaw()`, and `readTDSRaw()` functions near
the top of the `.ino` file contain placeholder linear/polynomial maps from
raw ADC counts to engineering units. Replace these with your own fitted
curves after taking readings against known reference standards (e.g. pH
4/7/9 buffer solutions, a known-NTU turbidity standard, and a known-TDS
solution), then set the corresponding `*Calibrated` flag to `true`.

## Flashing

1. Open `firmware/rainwater_gatekeeper.ino` in the Arduino IDE.
2. Select **Tools → Board → ESP32 Dev Module**.
3. Select the correct **Port**.
4. Click **Upload**.
5. Open **Tools → Serial Monitor** at 115200 baud to see live readings and routing decisions.

## GitHub repository

**Link:** _[to be added]_
