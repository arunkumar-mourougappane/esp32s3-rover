#include "imu.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

namespace {
constexpr uint32_t SAMPLE_US = 10000;  // 100 Hz
constexpr float ALPHA = 0.98f;         // gyro weight in the complementary filter

Adafruit_MPU6050 mpu;
bool gOk = false;
uint32_t gLastUs = 0;
float gBiasX = 0, gBiasY = 0, gBiasZ = 0;  // deg/s
imu::Data gData = {};

void wrap180(float& a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
}

void read(sensors_event_t& a, sensors_event_t& g, sensors_event_t& t) {
  mpu.getEvent(&a, &g, &t);
}

void accelAngles(const sensors_event_t& a, float& pitch, float& roll) {
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  roll  = atan2f(ay, az) * RAD_TO_DEG;
}
}  // namespace

namespace imu {
bool begin() {
  gOk = mpu.begin(0x68, &Wire) || mpu.begin(0x69, &Wire);
  if (!gOk) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  calibrate();
  return true;
}

bool ok() { return gOk; }

void calibrate() {
  if (!gOk) return;
  constexpr int N = 200;
  float sx = 0, sy = 0, sz = 0;
  sensors_event_t a, g, t;
  for (int i = 0; i < N; i++) {
    read(a, g, t);
    sx += g.gyro.x; sy += g.gyro.y; sz += g.gyro.z;
    delay(5);
  }
  gBiasX = sx / N * RAD_TO_DEG;
  gBiasY = sy / N * RAD_TO_DEG;
  gBiasZ = sz / N * RAD_TO_DEG;
  // seed attitude from gravity
  read(a, g, t);
  accelAngles(a, gData.pitch, gData.roll);
  gData.yaw = 0;
  gLastUs = micros();
}

void zeroYaw() { gData.yaw = 0; }

void update() {
  if (!gOk) return;
  uint32_t now = micros();
  if (now - gLastUs < SAMPLE_US) return;
  float dt = (now - gLastUs) * 1e-6f;
  gLastUs = now;
  if (dt > 0.1f) dt = SAMPLE_US * 1e-6f;  // long stall (e.g. calibration)

  sensors_event_t a, g, t;
  read(a, g, t);

  gData.ax = a.acceleration.x;
  gData.ay = a.acceleration.y;
  gData.az = a.acceleration.z;
  gData.gx = g.gyro.x * RAD_TO_DEG - gBiasX;
  gData.gy = g.gyro.y * RAD_TO_DEG - gBiasY;
  gData.gz = g.gyro.z * RAD_TO_DEG - gBiasZ;
  gData.temp = t.temperature;

  float accPitch, accRoll;
  accelAngles(a, accPitch, accRoll);
  // Rotation about +Y pitches the nose down, so nose-up rate is -gy.
  gData.pitch = ALPHA * (gData.pitch - gData.gy * dt) + (1 - ALPHA) * accPitch;
  gData.roll  = ALPHA * (gData.roll + gData.gx * dt) + (1 - ALPHA) * accRoll;

  gData.yaw += gData.gz * dt;
  wrap180(gData.yaw);
}

const Data& data() { return gData; }
}  // namespace imu
