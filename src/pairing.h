#pragma once
#include <Arduino.h>

// BLE pairing service. While the window is open the rover advertises, shows a
// 6-digit PIN on the TFT, and releases the WiFi credentials (AES-256-GCM
// encrypted) to a client that proves it knows the PIN.
//
// GATT (service 7d1c0001-5a3e-4b8f-9a62-3c5e1f0a9b10, chars share the prefix):
//   0002 CHAL   read   16 B server nonce Ns (fresh per connection / attempt)
//   0003 AUTH   write  Nc[16] || HMAC-SHA256(PIN, "auth"||Ns||Nc)[32]
//   0004 STATUS read   [code, tries_left]  0=waiting 1=ok 2=bad PIN 3=locked
//   0005 CRED   read   iv[12] || ciphertext || tag[16], only after STATUS=ok
//                      key = HMAC-SHA256(PIN, "enc"||Ns||Nc), aad = "rover1"
//                      plaintext = {"ssid","pass","ip","http","ws"}
namespace pairing {
void begin(const char* deviceName, const char* ssid, const char* pass,
           const char* ip);
void open();            // new PIN, start advertising, 3 minute window
void close();
void loop();            // window timeout + deferred disconnects
bool isOpen();
bool linkActive();      // a BLE client is connected
const char* pin();
uint32_t secondsLeft();
// One-shot events for the UI/LED; each returns true once per occurrence.
bool takeSuccess();
bool takeFailure();
}  // namespace pairing
