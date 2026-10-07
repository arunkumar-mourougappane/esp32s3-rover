#pragma once
#include <Arduino.h>

// NeoPixel status modes, highest priority first:
//   FAULT      red double-blink      IMU missing / init failure
//   FAILSAFE   orange fast blink     drive timeout tripped (1.5 s hold)
//   PAIR_FAIL  red flashes           wrong PIN / lockout (2 s hold)
//   PAIR_OK    green triple flash    credentials delivered (2 s hold)
//   PAIRING_LINK cyan solid          BLE client connected to pairing service
//   PAIRING    blue fast pulse       pairing window open, waiting
//   DRIVING    bright green          motors commanded non-zero
//   CONNECTED  dim green solid       WebSocket client attached
//   IDLE       amber slow breathe    AP up, nobody connected
//   BOOT       white                 starting up
namespace status_led {
enum Mode : uint8_t {
  BOOT, IDLE, CONNECTED, DRIVING, PAIRING, PAIRING_LINK, PAIR_OK, PAIR_FAIL,
  FAILSAFE, FAULT
};

void begin();
// Steady-state conditions; call every loop. Transient modes (PAIR_OK,
// PAIR_FAIL, FAILSAFE) are triggered with flash() and override these.
void update(bool imuFault, bool pairingOpen, bool pairingLink, bool wsClients,
            bool driving);
void flash(Mode transient);
Mode current();
}  // namespace status_led
