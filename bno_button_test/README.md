# BNO055 + Button Test

Tiny standalone ESP-IDF test app for the XIAO ESP32-C6 wiring.

## Wiring

- BNO055 SDA -> D4
- BNO055 SCL -> D5
- BNO055 VCC -> 3V3
- BNO055 GND -> GND
- Button -> D0 with pull-down

## What it does

- Prints BNO055 calibration state, linear acceleration, gyro, and quaternion over serial.
- Prints button HIGH/LOW transitions over serial.
- Uses no WLED code and no web UI.

## Build / flash

```bash
cd bno_button_test
/home/zinc/.platformio/penv/bin/platformio run
/home/zinc/.platformio/penv/bin/platformio run -t upload
/home/zinc/.platformio/penv/bin/platformio device monitor
```
