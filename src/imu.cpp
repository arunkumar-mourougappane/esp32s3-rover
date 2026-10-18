#include "imu.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

namespace {
constexpr uint32_t SAMPLE_MS = 5;      // 200 Hz
constexpr float ALPHA = 0.98f;         // gyro weight in the complementary filter
constexpr uint8_t FUEL_ADDR = 0x36;    // MAX17048
constexpr uint32_t BATTERY_PERIOD_MS = 1000;

Adafruit_MPU6050 mpu;
bool gOk = false;
bool gFuel = false;
uint32_t gLastUs = 0;
float gBiasX = 0, gBiasY = 0, gBiasZ = 0;  // deg/s

// The sampling task owns the I2C bus and the filter state (gState). It
// publishes a copy into gData under gMutex for readers on other tasks.
imu::Data gState = {};
imu::Data gData = {};
float gVbat = NAN;
SemaphoreHandle_t gMutex = nullptr;
volatile bool gCalReq = false;
volatile bool gZeroYawReq = false;

void wrap180(float& a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
}

void read(sensors_event_t& a, sensors_event_t& g, sensors_event_t& t) {
  mpu.getEvent(&a, &g, &t);
}

// MAX17048 VCELL register: 78.125 uV per LSB. Returns NaN on a bus error.
float readFuelVolts() {
  Wire.beginTransmission(FUEL_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return NAN;
  if (Wire.requestFrom(FUEL_ADDR, (uint8_t)2) != 2) return NAN;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return raw * 78.125e-6f;
}

void accelAngles(const sensors_event_t& a, float& pitch, float& roll) {
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  roll  = atan2f(ay, az) * RAD_TO_DEG;
}

void doCalibrate() {
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
  accelAngles(a, gState.pitch, gState.roll);
  gState.yaw = 0;
  gLastUs = micros();
}

void step() {
  uint32_t now = micros();
  float dt = (now - gLastUs) * 1e-6f;
  gLastUs = now;
  if (dt > 0.1f) dt = SAMPLE_MS * 1e-3f;  // long stall (e.g. calibration)

  sensors_event_t a, g, t;
  read(a, g, t);

  gState.ax = a.acceleration.x;
  gState.ay = a.acceleration.y;
  gState.az = a.acceleration.z;
  gState.gx = g.gyro.x * RAD_TO_DEG - gBiasX;
  gState.gy = g.gyro.y * RAD_TO_DEG - gBiasY;
  gState.gz = g.gyro.z * RAD_TO_DEG - gBiasZ;
  gState.temp = t.temperature;

  float accPitch, accRoll;
  accelAngles(a, accPitch, accRoll);
  // Rotation about +Y pitches the nose down, so nose-up rate is -gy.
  gState.pitch = ALPHA * (gState.pitch - gState.gy * dt) + (1 - ALPHA) * accPitch;
  gState.roll  = ALPHA * (gState.roll + gState.gx * dt) + (1 - ALPHA) * accRoll;

  gState.yaw += gState.gz * dt;
  wrap180(gState.yaw);
}

void task(void*) {
  TickType_t wake = xTaskGetTickCount();
  uint32_t lastBatMs = 0;
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_MS));
    if (gCalReq) {
      gCalReq = false;
      doCalibrate();
      wake = xTaskGetTickCount();
    }
    if (gZeroYawReq) {
      gZeroYawReq = false;
      gState.yaw = 0;
    }
    step();

    float vb = gVbat;
    uint32_t ms = millis();
    if (gFuel && ms - lastBatMs >= BATTERY_PERIOD_MS) {
      lastBatMs = ms;
      vb = readFuelVolts();
    }

    xSemaphoreTake(gMutex, portMAX_DELAY);
    gData = gState;
    gVbat = vb;
    xSemaphoreGive(gMutex);
  }
}
}  // namespace

namespace imu {
bool begin() {
  gOk = mpu.begin(0x68, &Wire) || mpu.begin(0x69, &Wire);
  if (!gOk) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Wire.beginTransmission(FUEL_ADDR);
  gFuel = Wire.endTransmission() == 0;

  doCalibrate();
  gData = gState;
  gMutex = xSemaphoreCreateMutex();
  // Core 1 shares with loop(); the higher priority keeps the sample clock
  // steady regardless of what loop() is doing.
  xTaskCreatePinnedToCore(task, "imu", 4096, nullptr, 3, nullptr, 1);
  return true;
}

bool ok() { return gOk; }
bool hasFuelGauge() { return gFuel; }

void calibrate() { if (gOk) gCalReq = true; }
void zeroYaw() { if (gOk) gZeroYawReq = true; }

Data data() {
  Data d;
  if (!gMutex) return gData;
  xSemaphoreTake(gMutex, portMAX_DELAY);
  d = gData;
  xSemaphoreGive(gMutex);
  return d;
}

float batteryVolts() {
  if (!gMutex) return NAN;
  xSemaphoreTake(gMutex, portMAX_DELAY);
  float v = gVbat;
  xSemaphoreGive(gMutex);
  return v;
}
}  // namespace imu
