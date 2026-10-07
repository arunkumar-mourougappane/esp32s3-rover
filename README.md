# esp32s3-rover
An ESP32-S3 Feather TFT based rover, controlled over its own WiFi access point. WiFi credentials are handed out over Bluetooth LE after the user enters a PIN shown on the rover's screen.

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

- `http://192.168.4.1/` dashboard: top-down heading view, side and rear tilt, g-meter, motor bars, strip chart, touch/keyboard drive pad.
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
python rover_tool.py watch                      # print telemetry
python rover_tool.py gui                        # needs tkinter
```

macOS GUI needs `brew install python-tk@3.12`, and the terminal needs Bluetooth permission in System Settings.
