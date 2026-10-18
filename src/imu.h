#pragma once
#include <Arduino.h>

// Assumed mounting: chip X = forward, Y = left, Z = up.
// pitch: nose-up positive. roll: left-side-up positive. yaw: CCW (left turn)
// positive and relative to the last zero; it drifts (no magnetometer).
namespace imu {
struct Data {
  float ax, ay, az;     // m/s^2, includes gravity
  float gx, gy, gz;     // deg/s, bias removed
  float pitch, roll, yaw;  // deg
  float temp;           // C
  float trust;          // 0..1, how much the accel is believed (1 = at rest)
};

// A FreeRTOS task (core 1) samples at 200 Hz and owns the I2C bus, including
// the optional MAX17048 fuel gauge. All accessors are safe from any task.
bool begin();           // init + gyro bias calibration (~1 s, keep still), starts the task
bool ok();
void calibrate();       // request gyro bias recalibration (~1 s, keep still)
void zeroYaw();         // request yaw reset
Data data();            // consistent snapshot
bool hasFuelGauge();
float batteryVolts();   // NaN if no gauge or read failed
}  // namespace imu
