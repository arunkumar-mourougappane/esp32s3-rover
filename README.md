# esp32s3-rover

[![CI](https://github.com/arunkumar-mourougappane/esp32s3-rover/actions/workflows/ci.yml/badge.svg)](https://github.com/arunkumar-mourougappane/esp32s3-rover/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![Target](https://img.shields.io/badge/target-ESP32--S3-red)
![Framework](https://img.shields.io/badge/framework-Arduino-00979D)
![Build](https://img.shields.io/badge/build-PlatformIO-orange)
![Python](https://img.shields.io/badge/host_tool-Python_3.10%2B-3776AB)

An ESP32-S3 Feather TFT based rover, controlled over its own WiFi access point. WiFi credentials are handed out over Bluetooth LE after the user enters a PIN shown on the rover's screen.

## Supported platforms

**Firmware target**

| Item | Supported |
| --- | --- |
| Board | Adafruit Feather ESP32-S3 TFT (`adafruit_feather_esp32s3_tft`), 4 MB flash |
| Build system | PlatformIO, `espressif32@6.10.0` |
| Framework | Arduino-ESP32 2.0.17 (ESP-IDF 4.4) |
| Python for PlatformIO | 3.10 to 3.13 (3.14 is rejected by PlatformIO 6.1) |
| IMU | MPU6050 on the STEMMA QT port (0x68 or 0x69) |

Other ESP32-S3 boards are untested. They would need the pin map in `src/config.h` and the display and NeoPixel pins changed.

**Host tool (`tools/rover_tool.py`)**

| OS | Scan / pair | Auto-join rover WiFi | GUI |
| --- | --- | --- | --- |
| macOS | yes (tested) | yes, via `networksetup` | yes, needs `python-tk` |
| Linux (BlueZ) | yes, untested | yes, via `nmcli` | yes, needs `python3-tk` |
| Windows | yes, untested | no, join manually | yes, untested |

CI compiles the host tool on Python 3.10 to 3.13. It was run by hand on Python 3.12 only. The BLE and WebSocket paths need hardware, so CI does not exercise them.

**Dashboard**: any browser with WebSocket and canvas support. Tested on desktop Chrome.

## CI

GitHub Actions (`.github/workflows/ci.yml`) builds the firmware with PlatformIO on every push to `main` and every pull request, and uploads `firmware.bin` and `partitions.bin` as a build artifact. A second job compiles the host tool on Python 3.10 to 3.13.

## Hardware

| Function | Pin | Feather silkscreen | Notes |
| --- | --- | --- | --- |
| Left motor IN1 (forward PWM) | GPIO 5 | D5 | LED for now |
| Left motor IN2 (reverse PWM) | GPIO 6 | D6 | LED for now |
| Right motor IN1 (forward PWM) | GPIO 9 | D9 | LED for now |
| Right motor IN2 (reverse PWM) | GPIO 10 | D10 | LED for now |
| MPU6050 | GPIO 42 SDA / 41 SCL | STEMMA QT | 0x68 or 0x69 |
| Battery gauge (optional) | I2C 0x36 | on-board MAX17048 | shows `n/a` if absent |
| Pairing button | GPIO 0 | BOOT | |

Each motor uses two PWM pins (20 kHz, 10 bit). Forward drives IN1 and holds IN2 low; reverse is the opposite. This works with DRV8833, DRV8871, TB6612 (PWM pin tied high) and L298N (EN tied high). Pins are in `src/config.h`.

LEDs: put a ~330 ohm resistor in series, anode to the GPIO, cathode to GND. Forward brightness lights the IN1 LED, reverse lights the IN2 LED. Never use the GPIOs to drive a motor directly. Use a driver board.

Free pins for later: D11 (GPIO 11), D12 (GPIO 12) for driver enable/STBY, A0-A5 for encoders.

## Pairing flow

1. On boot (or after pressing **BOOT**) the rover opens a 3 minute pairing window: it advertises over BLE as `Rover-XXXX` and shows a 6 digit PIN on the TFT.
2. The client connects, proves it knows the PIN with an HMAC challenge-response, and receives the WiFi SSID and password encrypted with AES-256-GCM (key derived from the PIN and both nonces). Protocol details are in `src/pairing.h`.
3. Three wrong PINs close the window. Press BOOT to reopen it with a new PIN.
4. The client joins the `Rover-XXXX` WiFi AP (WPA2, random 12 character password generated on first boot and stored in NVS) and opens `http://192.168.4.1/` or `ws://192.168.4.1:81/`.

Limitation: the PIN is the only secret on the BLE link, and 6 digits can be brute-forced offline by someone who sniffs a full pairing exchange. Pair somewhere you trust. Anyone on the WiFi AP can drive the rover.

## NeoPixel modes

| Colour | Meaning |
| --- | --- |
| White | booting |
| Amber, slow breathe | AP up, nobody connected |
| Dim green | WebSocket client connected |
| Bright green | motors commanded |
| Blue, fast pulse | pairing window open |
| Cyan | BLE client connected to the pairing service |
| Green flashing (2 s) | pairing succeeded |
| Red flashing (2 s) | wrong PIN / lockout |
| Orange fast blink | drive failsafe tripped (no command for 400 ms) |
| Red double blink | IMU not found |

## Interfaces

- `http://192.168.4.1/` dashboard: top-down heading view, side and rear tilt, g-meter, motor bars, strip chart, touch/keyboard drive pad. The rover is drawn with tyres, lights and a sensor mast. Tyres in the top view fill green (forward) or amber (reverse) with wheel speed. The tilt views pivot on the tyre that stays on the ground, and turn amber past 20 degrees and red past 35.
- `http://192.168.4.1/json` one telemetry snapshot.
- `ws://192.168.4.1:81/` telemetry at 20 Hz; accepts JSON commands:
  - `{"cmd":"drive","left":-1..1,"right":-1..1}` (resend at least every 400 ms)
  - `{"cmd":"stop"}`, `{"cmd":"zero_yaw"}`, `{"cmd":"calibrate"}` (keep the rover still, about 1 s)

Telemetry: `imu` (accel m/s^2, gyro deg/s), `att` (pitch, roll, yaw in degrees), `motors` (left/right -1..1 plus the four raw PWM duties), `temp`, `vbat`, `failsafe`, `clients`, `uptime_s`, `heap`, `pairing`.

IMU convention: chip X forward, Y left, Z up. Pitch is nose-up positive, roll is left-side-up positive, yaw is counter-clockwise positive. Yaw comes from the gyro only, so it is relative and drifts. If the IMU is mounted differently, change the axes in `src/imu.cpp`.

## Build and flash

```bash
python3.12 -m platformio run -t upload
python3.12 -m platformio device monitor
```

(PlatformIO's `pio` fails on Python 3.14; use 3.12 or 3.13.)

## Host tool

```bash
cd tools
python3.12 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
python rover_tool.py scan
python rover_tool.py pair --pin 123456 --join   # --join switches your WiFi to the rover
python rover_tool.py watch                      # print telemetry (--host IP[:port])
python rover_tool.py gui                        # pair, join, drive, live panels; needs tkinter
```

The GUI shows the same live panels as the dashboard: heading, side and rear tilt, g-meter and motor bars. Drive with WASD or the arrow keys, or hold the on-screen buttons. Space stops.

macOS GUI needs `brew install python-tk@3.12`, and the terminal needs Bluetooth permission in System Settings.
