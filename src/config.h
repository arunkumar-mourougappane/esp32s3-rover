#pragma once
#include <Arduino.h>

// ---- Motor driver pins -------------------------------------------------
// Two PWM inputs per motor (IN1/IN2). Forward = PWM on IN1, IN2 low;
// reverse = PWM on IN2, IN1 low. Works with DRV8833, DRV8871, TB6612 (PWM
// tied high) and L298N (EN tied high). Board silkscreen name in comments.
constexpr uint8_t PIN_ML_IN1 = 5;   // D5   left  motor, forward
constexpr uint8_t PIN_ML_IN2 = 6;   // D6   left  motor, reverse
constexpr uint8_t PIN_MR_IN1 = 9;   // D9   right motor, forward
constexpr uint8_t PIN_MR_IN2 = 10;  // D10  right motor, reverse

constexpr uint32_t PWM_FREQ_HZ = 20000;  // above audible range
constexpr uint8_t  PWM_BITS    = 10;     // 20 kHz needs <= 11 bits on the S3
constexpr uint32_t PWM_MAX     = (1u << PWM_BITS) - 1;

// Motors are stopped if no drive command arrives within this window.
constexpr uint32_t DRIVE_TIMEOUT_MS = 400;

// ---- Misc board pins ---------------------------------------------------
constexpr uint8_t PIN_BOOT_BTN = 0;  // BOOT button, opens the pairing window

// ---- Network -----------------------------------------------------------
constexpr uint16_t HTTP_PORT = 80;
constexpr uint16_t WS_PORT   = 81;
constexpr uint8_t  AP_MAX_CLIENTS = 4;

// ---- Timing ------------------------------------------------------------
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;   // 20 Hz
constexpr uint32_t TFT_PERIOD_MS       = 250;  // 4 Hz
