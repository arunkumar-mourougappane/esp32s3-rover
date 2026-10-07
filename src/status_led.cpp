#include "status_led.h"
#include <Adafruit_NeoPixel.h>

namespace {
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
status_led::Mode gMode = status_led::BOOT;
status_led::Mode gTransient = status_led::BOOT;
uint32_t gTransientUntil = 0;
uint32_t gLast = 0;
uint32_t gLastColor = 0xFFFFFFFF;

struct Rgb { uint8_t r, g, b; };

// triangle wave 0..255 with the given period
uint8_t tri(uint32_t now, uint32_t periodMs) {
  uint32_t p = now % periodMs;
  uint32_t half = periodMs / 2;
  return (uint8_t)((p < half ? p : periodMs - p) * 255 / half);
}

Rgb scale(Rgb c, uint8_t k) {
  return {(uint8_t)(c.r * k / 255), (uint8_t)(c.g * k / 255), (uint8_t)(c.b * k / 255)};
}

Rgb render(status_led::Mode m, uint32_t now) {
  using namespace status_led;
  switch (m) {
    case BOOT:         return {60, 60, 60};
    case IDLE:         return scale({255, 120, 0}, 25 + tri(now, 3000) / 3);
    case CONNECTED:    return {0, 60, 0};
    case DRIVING:      return {0, 255, 40};
    case PAIRING:      return scale({0, 40, 255}, 20 + tri(now, 600) * 235 / 255);
    case PAIRING_LINK: return {0, 200, 200};
    case PAIR_OK:      return (now % 400 < 200) ? Rgb{0, 255, 0} : Rgb{0, 0, 0};
    case PAIR_FAIL:    return (now % 300 < 150) ? Rgb{255, 0, 0} : Rgb{0, 0, 0};
    case FAILSAFE:     return (now % 200 < 100) ? Rgb{255, 90, 0} : Rgb{0, 0, 0};
    case FAULT: {
      uint32_t p = now % 1500;  // two short red blinks, then pause
      bool on = p < 120 || (p > 240 && p < 360);
      return on ? Rgb{255, 0, 0} : Rgb{0, 0, 0};
    }
  }
  return {0, 0, 0};
}
}  // namespace

namespace status_led {
void begin() {
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, NEOPIXEL_POWER_ON);
  pixel.begin();
  pixel.setBrightness(40);
  Rgb c = render(BOOT, 0);
  pixel.setPixelColor(0, pixel.Color(c.r, c.g, c.b));
  pixel.show();
}

void flash(Mode transient) {
  uint32_t hold = transient == FAILSAFE ? 1500 : 2000;
  gTransient = transient;
  gTransientUntil = millis() + hold;
}

void update(bool imuFault, bool pairingOpen, bool pairingLink, bool wsClients,
            bool driving) {
  uint32_t now = millis();
  if (now - gLast < 20) return;  // 50 Hz is plenty
  gLast = now;

  bool transientActive = (int32_t)(gTransientUntil - now) > 0;
  if (imuFault)                   gMode = FAULT;
  else if (transientActive)       gMode = gTransient;
  else if (pairingLink)           gMode = PAIRING_LINK;
  else if (pairingOpen)           gMode = PAIRING;
  else if (driving)               gMode = DRIVING;
  else if (wsClients)             gMode = CONNECTED;
  else                            gMode = IDLE;

  Rgb c = render(gMode, now);
  uint32_t packed = pixel.Color(c.r, c.g, c.b);
  if (packed != gLastColor) {  // skip redundant show()
    gLastColor = packed;
    pixel.setPixelColor(0, packed);
    pixel.show();
  }
}

Mode current() { return gMode; }
}  // namespace status_led
