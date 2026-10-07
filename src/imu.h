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
};

bool begin();           // init + gyro bias calibration (~1 s, keep still)
bool ok();
void update();          // call every loop; samples at 100 Hz internally
void calibrate();       // re-run gyro bias calibration (~1 s, keep still)
void zeroYaw();
const Data& data();
}  // namespace imu
