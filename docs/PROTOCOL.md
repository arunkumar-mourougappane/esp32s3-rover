# Telemetry and command protocol

How to talk to the rover once you are on its WiFi (see the README for pairing). All payloads are UTF-8 JSON. There is no authentication beyond the WPA2 network itself: any client on the AP can drive.

| Transport | Endpoint | Direction | Rate |
| --- | --- | --- | --- |
| WebSocket | `ws://192.168.4.1:81/` | rover to client: telemetry; client to rover: commands | telemetry 20 Hz (every 50 ms) |
| HTTP GET | `http://192.168.4.1/json` | one telemetry snapshot, same schema | on demand |
| HTTP GET | `http://192.168.4.1/` | dashboard page | on demand |

Telemetry is broadcast to every connected WebSocket client, and only while at least one is connected. The rate is driven by the main loop, so treat 20 Hz as nominal.

## Telemetry (rover to client)

One JSON object per text frame. Example:

```json
{
  "seq": 1042,
  "imu": {"ax": 0.12, "ay": -0.03, "az": 9.79, "gx": 0.1, "gy": -0.2, "gz": 0.0},
  "att": {"pitch": 1.2, "roll": -0.4, "yaw": 12.5, "acc_pitch": 1.4, "acc_roll": -0.5},
  "filt": {"trust": 1.0, "still": true},
  "motors": {"left": 0.5, "right": 0.5, "pwm": [512, 0, 512, 0]},
  "temp": 27.1,
  "vbat": 4.12,
  "failsafe": false,
  "clients": 1,
  "uptime_s": 124,
  "heap": 212340,
  "pairing": {"open": false, "secs": 0}
}
```

| Field | Type | Unit / range | Notes |
| --- | --- | --- | --- |
| `seq` | integer | 0..2^32-1 | Frame counter, increments per frame built (including `/json` requests), wraps. A gap on the WebSocket means dropped frames or a `/json` call in between. |
| `imu.ax/ay/az` | float | m/s^2, 2 decimals | Raw accelerometer, includes gravity. Not low-passed beyond the sensor's own 44 Hz filter. |
| `imu.gx/gy/gz` | float | deg/s, 1 decimal | Gyro with the estimated bias removed. |
| `att.pitch` | float | deg, 1 decimal | Fused. Nose-up positive. |
| `att.roll` | float | deg, 1 decimal | Fused. Left-side-up positive. |
| `att.yaw` | float | deg, -180..180 | Gyro-integrated, counter-clockwise positive, relative to boot or the last `zero_yaw`. Drifts while moving (no magnetometer); frozen while `filt.still`. |
| `att.acc_pitch`, `att.acc_roll` | float | deg, 1 decimal | Angles from the accelerometer alone, for comparison with the fused values. Noisy and wrong under acceleration. |
| `filt.trust` | float | 0..1 | How much the accelerometer is believed this sample. 1 at rest; falls to 0 when `|a|` is 0.25 g or more away from 1 g. |
| `filt.still` | bool | | Motors idle, gyro quiet and `|a|` near 1 g for long enough. Gyro bias is tracking and yaw is frozen. |
| `motors.left`, `motors.right` | float | -1..1, 2 decimals | Commanded speed after clamping. Positive is forward. |
| `motors.pwm` | array of 4 integers | 0..1023 (10 bit) | Raw duty per output in the order left forward (GPIO5), left reverse (GPIO6), right forward (GPIO9), right reverse (GPIO10). |
| `temp` | float or null | deg C | MPU6050 die temperature. `null` if the IMU is missing. |
| `vbat` | float or null | V, 2 decimals | MAX17048 cell voltage, refreshed about once a second. `null` if no gauge or the read failed. |
| `failsafe` | bool | | True after the drive failsafe stopped the motors. Cleared by the next `drive` or `stop`. |
| `clients` | integer | | Connected WebSocket clients. |
| `uptime_s` | integer | s | Since boot. |
| `heap` | integer | bytes | Free heap. |
| `pairing.open` | bool | | BLE pairing window is open. |
| `pairing.secs` | integer | s | Seconds left in the window, 0 when closed. |

Axis convention: chip X forward, Y left, Z up. If the IMU is mounted differently, change the axes in `src/imu.cpp`.

While a `calibrate` is running (about 1 s) the IMU fields hold their last values; sampling resumes afterwards.

## Commands (client to rover)

One JSON object per text frame, at most 256 bytes. The rover sends no reply or acknowledgement; watch the telemetry for the effect. Malformed JSON, oversized frames and unknown `cmd` values are ignored silently.

| Command | Payload | Effect |
| --- | --- | --- |
| `drive` | `{"cmd":"drive","left":-1..1,"right":-1..1}` | Set both sides. Values are clamped to -1..1 and a missing or non-numeric value counts as 0. Also feeds the failsafe timer. |
| `stop` | `{"cmd":"stop"}` | Both sides to 0 and clear `failsafe`. |
| `zero_yaw` | `{"cmd":"zero_yaw"}` | Reset `att.yaw` to 0. Applied by the IMU task on its next sample. |
| `calibrate` | `{"cmd":"calibrate"}` | Stop the motors and re-measure the gyro bias. Keep the rover still for about 1 s. |

### Safety behaviour

- **Failsafe:** if the motors are running and no `drive` has arrived for 400 ms, the motors stop, `failsafe` becomes true and the NeoPixel blinks orange. Clients that want to keep moving must resend `drive` at least every 400 ms (the dashboard and GUI send at 10 Hz while a control is held).
- **Disconnect:** when the last WebSocket client disconnects, the motors stop.
- **Calibration:** `calibrate` stops the motors first.

## Minimal clients

```python
import asyncio, json, websockets

async def main():
    async with websockets.connect("ws://192.168.4.1:81/") as ws:
        await ws.send(json.dumps({"cmd": "drive", "left": 0.3, "right": 0.3}))
        async for msg in ws:
            t = json.loads(msg)
            print(t["att"]["pitch"], t["att"]["roll"], t["motors"]["pwm"])

asyncio.run(main())
```

```js
const ws = new WebSocket('ws://192.168.4.1:81/');
ws.onmessage = e => { const t = JSON.parse(e.data); console.log(t.att.pitch, t.filt.still); };
ws.onopen = () => ws.send(JSON.stringify({cmd: 'drive', left: 0.3, right: 0.3}));
```

```sh
curl http://192.168.4.1/json
```

The sample clients above send one `drive` and then stop, so the failsafe will halt the motors after 400 ms. A real client resends in a loop.

`tools/rover_tool.py watch` prints a live summary, and `tools/rover_tool.py gui` drives and visualises the same stream.
