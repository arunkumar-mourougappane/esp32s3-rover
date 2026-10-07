#include "motors.h"
#include "config.h"

namespace {
constexpr uint8_t PINS[4] = {PIN_ML_IN1, PIN_ML_IN2, PIN_MR_IN1, PIN_MR_IN2};
float gLeft = 0, gRight = 0;
uint16_t gDuty[4] = {0, 0, 0, 0};

float clean(float v) {
  if (isnan(v)) return 0;
  return constrain(v, -1.0f, 1.0f);
}

// side 0 = left (channels 0,1), side 1 = right (channels 2,3)
void apply(uint8_t side, float v) {
  uint16_t d = (uint16_t)lroundf(fabsf(v) * PWM_MAX);
  gDuty[side * 2]     = v > 0 ? d : 0;
  gDuty[side * 2 + 1] = v < 0 ? d : 0;
  ledcWrite(side * 2,     gDuty[side * 2]);
  ledcWrite(side * 2 + 1, gDuty[side * 2 + 1]);
}
}  // namespace

namespace motors {
void begin() {
  for (uint8_t ch = 0; ch < 4; ch++) {
    ledcSetup(ch, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(PINS[ch], ch);
    ledcWrite(ch, 0);
  }
}

void set(float left, float right) {
  gLeft = clean(left);
  gRight = clean(right);
  apply(0, gLeft);
  apply(1, gRight);
}

void stop() { set(0, 0); }
float left() { return gLeft; }
float right() { return gRight; }
uint16_t duty(uint8_t channel) { return channel < 4 ? gDuty[channel] : 0; }
}  // namespace motors
