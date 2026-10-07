#include "pairing.h"
#include "config.h"
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <esp_random.h>
#include <mbedtls/gcm.h>
#include <mbedtls/md.h>

namespace {
#define UUID_BASE(n) "7d1c000" #n "-5a3e-4b8f-9a62-3c5e1f0a9b10"
constexpr const char* UUID_SVC    = UUID_BASE(1);
constexpr const char* UUID_CHAL   = UUID_BASE(2);
constexpr const char* UUID_AUTH   = UUID_BASE(3);
constexpr const char* UUID_STATUS = UUID_BASE(4);
constexpr const char* UUID_CRED   = UUID_BASE(5);

constexpr uint32_t WINDOW_MS = 180000;
constexpr uint8_t MAX_TRIES = 3;
constexpr uint8_t ST_WAITING = 0, ST_OK = 1, ST_BAD = 2, ST_LOCKED = 3;

String gName, gSsid, gPass, gIp;
char gPin[7] = "000000";
bool gOpen = false;
uint32_t gOpenedAt = 0;
uint8_t gTries = 0;
uint8_t gNonce[16];
volatile bool gLink = false;
volatile uint16_t gConnHandle = 0;
volatile bool gKickRequested = false;
volatile bool gCloseRequested = false;
volatile bool gSuccess = false, gFailure = false;

NimBLEServer* gServer = nullptr;
NimBLECharacteristic *chChal, *chStatus, *chCred;

void setStatus(uint8_t code, uint8_t triesLeft) {
  uint8_t s[2] = {code, triesLeft};
  chStatus->setValue(s, 2);
}

void newNonce() {
  esp_fill_random(gNonce, sizeof(gNonce));
  chChal->setValue(gNonce, sizeof(gNonce));
}

void clearSession() {
  chCred->setValue((const uint8_t*)"", 0);
  setStatus(ST_WAITING, MAX_TRIES - gTries);
  newNonce();
}

bool hmac(const uint8_t* key, size_t keyLen, const char* label, const uint8_t* nc,
          uint8_t out[32]) {
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  bool ok = mbedtls_md_setup(&ctx, info, 1) == 0 &&
            mbedtls_md_hmac_starts(&ctx, key, keyLen) == 0 &&
            mbedtls_md_hmac_update(&ctx, (const uint8_t*)label, strlen(label)) == 0 &&
            mbedtls_md_hmac_update(&ctx, gNonce, sizeof(gNonce)) == 0 &&
            mbedtls_md_hmac_update(&ctx, nc, 16) == 0 &&
            mbedtls_md_hmac_finish(&ctx, out) == 0;
  mbedtls_md_free(&ctx);
  return ok;
}

bool ctEqual(const uint8_t* a, const uint8_t* b, size_t n) {
  uint8_t d = 0;
  for (size_t i = 0; i < n; i++) d |= a[i] ^ b[i];
  return d == 0;
}

// iv || ciphertext || tag
bool encryptCreds(const uint8_t key[32], std::string& out) {
  JsonDocument doc;
  doc["ssid"] = gSsid;
  doc["pass"] = gPass;
  doc["ip"] = gIp;
  doc["http"] = HTTP_PORT;
  doc["ws"] = WS_PORT;
  std::string plain;
  serializeJson(doc, plain);

  uint8_t iv[12], tag[16];
  esp_fill_random(iv, sizeof(iv));
  std::string ct(plain.size(), '\0');
  mbedtls_gcm_context gcm;
  mbedtls_gcm_init(&gcm);
  bool ok = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, 256) == 0 &&
            mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, plain.size(), iv,
                                      sizeof(iv), (const uint8_t*)"rover1", 6,
                                      (const uint8_t*)plain.data(),
                                      (uint8_t*)&ct[0], sizeof(tag), tag) == 0;
  mbedtls_gcm_free(&gcm);
  if (!ok) return false;
  out.assign((const char*)iv, sizeof(iv));
  out += ct;
  out.append((const char*)tag, sizeof(tag));
  return true;
}

class ServerCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo& info) override {
    gLink = true;
    gConnHandle = info.getConnHandle();
    newNonce();
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    gLink = false;
    clearSession();
    if (gOpen) NimBLEDevice::getAdvertising()->start();
  }
};

class AuthCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    NimBLEAttValue v = c->getValue();
    if (!gOpen || v.size() != 48) {
      setStatus(ST_BAD, MAX_TRIES - gTries);
      return;
    }
    const uint8_t* nc = v.data();
    const uint8_t* proof = v.data() + 16;
    uint8_t expect[32];
    if (!hmac((const uint8_t*)gPin, 6, "auth", nc, expect)) {
      setStatus(ST_BAD, MAX_TRIES - gTries);
      return;
    }
    if (ctEqual(expect, proof, 32)) {
      uint8_t key[32];
      std::string blob;
      if (hmac((const uint8_t*)gPin, 6, "enc", nc, key) && encryptCreds(key, blob)) {
        chCred->setValue((const uint8_t*)blob.data(), blob.size());
        setStatus(ST_OK, 0);
        gSuccess = true;
        gCloseRequested = true;
      } else {
        setStatus(ST_BAD, MAX_TRIES - gTries);
      }
      return;
    }
    gTries++;
    gFailure = true;
    if (gTries >= MAX_TRIES) {
      setStatus(ST_LOCKED, 0);
      gKickRequested = true;
      gCloseRequested = true;  // lockout requires a BOOT press to reopen
    } else {
      setStatus(ST_BAD, MAX_TRIES - gTries);
      newNonce();
    }
  }
};

ServerCb serverCb;
AuthCb authCb;
}  // namespace

namespace pairing {
void begin(const char* deviceName, const char* ssid, const char* pass, const char* ip) {
  gName = deviceName;
  gSsid = ssid;
  gPass = pass;
  gIp = ip;

  NimBLEDevice::init(deviceName);
  NimBLEDevice::setMTU(247);
  gServer = NimBLEDevice::createServer();
  gServer->setCallbacks(&serverCb, false);

  NimBLEService* svc = gServer->createService(UUID_SVC);
  chChal = svc->createCharacteristic(UUID_CHAL, NIMBLE_PROPERTY::READ);
  NimBLECharacteristic* chAuth =
      svc->createCharacteristic(UUID_AUTH, NIMBLE_PROPERTY::WRITE);
  chStatus = svc->createCharacteristic(UUID_STATUS, NIMBLE_PROPERTY::READ);
  chCred = svc->createCharacteristic(UUID_CRED, NIMBLE_PROPERTY::READ);
  chAuth->setCallbacks(&authCb);
  clearSession();
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_SVC);
  adv->setName(deviceName);
  adv->enableScanResponse(true);
}

void open() {
  uint32_t r = esp_random() % 1000000;
  snprintf(gPin, sizeof(gPin), "%06lu", (unsigned long)r);
  gTries = 0;
  gOpenedAt = millis();
  gOpen = true;
  gCloseRequested = false;
  clearSession();
  NimBLEDevice::getAdvertising()->start();
  Serial.printf("[pair] window open, PIN %s\n", gPin);
}

void close() {
  gOpen = false;
  NimBLEDevice::getAdvertising()->stop();
  Serial.println("[pair] window closed");
}

void loop() {
  if (gKickRequested) {
    gKickRequested = false;
    if (gLink) gServer->disconnect(gConnHandle);
  }
  if (gCloseRequested) {
    gCloseRequested = false;
    close();
  }
  if (gOpen && millis() - gOpenedAt >= WINDOW_MS) close();
}

bool isOpen() { return gOpen; }
bool linkActive() { return gLink; }
const char* pin() { return gPin; }

uint32_t secondsLeft() {
  if (!gOpen) return 0;
  uint32_t el = millis() - gOpenedAt;
  return el >= WINDOW_MS ? 0 : (WINDOW_MS - el + 999) / 1000;
}

bool takeSuccess() { bool v = gSuccess; gSuccess = false; return v; }
bool takeFailure() { bool v = gFailure; gFailure = false; return v; }
}  // namespace pairing
