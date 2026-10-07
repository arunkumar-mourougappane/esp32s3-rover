#pragma once
#include <Arduino.h>

namespace motors {
void begin();
// left/right in [-1, 1]; positive = forward. NaN is treated as 0.
void set(float left, float right);
void stop();
float left();
float right();
// Duty of the raw PWM channels: 0=L.IN1 1=L.IN2 2=R.IN1 3=R.IN2
uint16_t duty(uint8_t channel);
}  // namespace motors
