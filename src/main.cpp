#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <SPI.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Wire.h>

#include "config.h"
#include "imu.h"
#include "motors.h"
#include "pairing.h"
#include "status_led.h"
#include "web_ui.h"

// Adafruit_ST7789 on the Feather ESP32-S3 TFT (pins come from the board variant)
Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

WebServer server(HTTP_PORT);
WebSocketsServer webSocket(WS_PORT);

String gSsid, gPass, gIp;
uint8_t gWsClients = 0;
uint32_t gSeq = 0;
uint32_t gLastCmdMs = 0;
bool gFailsafe = false;
bool gFuelGauge = false;  // MAX17048 at 0x36 found

// ---------------------------------------------------------------- helpers
float round1(float v) { return roundf(v * 10.0f) / 10.0f; }
float round2(float v) { return roundf(v * 100.0f) / 100.0f; }

String makePassword() {
  // no 0/O/1/l/I to keep it easy to type by hand
  static const char alphabet[] = "abcdefghjkmnpqrstuvwxyz23456789";
  String p;
  for (int i = 0; i < 12; i++) p += alphabet[esp_random() % (sizeof(alphabet) - 1)];
  return p;
}

// MAX17048 VCELL register: 78.125 uV per LSB. Returns NaN if absent.
float readBatteryVolts() {
  if (!gFuelGauge) return NAN;
  Wire.beginTransmission(0x36);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return NAN;
  if (Wire.requestFrom((uint8_t)0x36, (uint8_t)2) != 2) return NAN;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return raw * 78.125e-6f;
}

void buildTelemetry(JsonDocument& doc) {
  const imu::Data& d = imu::data();
  doc["seq"] = gSeq++;
  JsonObject i = doc["imu"].to<JsonObject>();
  i["ax"] = round2(d.ax); i["ay"] = round2(d.ay); i["az"] = round2(d.az);
  i["gx"] = round1(d.gx); i["gy"] = round1(d.gy); i["gz"] = round1(d.gz);
  JsonObject a = doc["att"].to<JsonObject>();
  a["pitch"] = round1(d.pitch); a["roll"] = round1(d.roll); a["yaw"] = round1(d.yaw);
  JsonObject m = doc["motors"].to<JsonObject>();
  m["left"] = round2(motors::left());
  m["right"] = round2(motors::right());
  JsonArray pwm = m["pwm"].to<JsonArray>();
  for (uint8_t c = 0; c < 4; c++) pwm.add(motors::duty(c));
  if (imu::ok()) doc["temp"] = round1(d.temp); else doc["temp"] = nullptr;
  float vb = readBatteryVolts();
  if (isnan(vb)) doc["vbat"] = nullptr; else doc["vbat"] = round2(vb);
  doc["failsafe"] = gFailsafe;
  doc["clients"] = gWsClients;
  doc["uptime_s"] = millis() / 1000;
  doc["heap"] = ESP.getFreeHeap();
  JsonObject p = doc["pairing"].to<JsonObject>();
  p["open"] = pairing::isOpen();
  p["secs"] = pairing::secondsLeft();
}

// ------------------------------------------------------------- websocket
void handleCommand(const char* text, size_t len) {
  if (len == 0 || len > 256) return;
  JsonDocument doc;
  if (deserializeJson(doc, text, len)) return;
  const char* cmd = doc["cmd"] | "";
  if (!strcmp(cmd, "drive")) {
    motors::set(doc["left"] | 0.0f, doc["right"] | 0.0f);
    gLastCmdMs = millis();
    gFailsafe = false;
  } else if (!strcmp(cmd, "stop")) {
    motors::stop();
    gFailsafe = false;
  } else if (!strcmp(cmd, "zero_yaw")) {
    imu::zeroYaw();
  } else if (!strcmp(cmd, "calibrate")) {
    motors::stop();
    imu::calibrate();
  }
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      gWsClients++;
      Serial.printf("[ws] #%u connected (%u total)\n", num, gWsClients);
      break;
    case WStype_DISCONNECTED:
      if (gWsClients) gWsClients--;
      if (!gWsClients) motors::stop();  // nobody left to drive
      Serial.printf("[ws] #%u disconnected (%u total)\n", num, gWsClients);
      break;
    case WStype_TEXT:
      handleCommand((const char*)payload, length);
      break;
    default:
      break;
  }
}

// -------------------------------------------------------------------- TFT
bool gTftPairingShown = false;

void tftStatic() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0, 0, 240, 18, ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLUE);
  tft.setTextSize(2);
  tft.setCursor(4, 2);
  tft.print("ESP32-S3 ROVER");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(4, 24);
  tft.printf("WiFi: %s", gSsid.c_str());
  tft.setCursor(4, 36);
  tft.printf("IP:   %s", gIp.c_str());
}

void tftUpdate() {
  bool open = pairing::isOpen();
  if (open != gTftPairingShown) {
    gTftPairingShown = open;
    tft.fillRect(0, 48, 240, 46, ST77XX_BLACK);
  }
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(150, 36);
  tft.printf("WS:%-2u", gWsClients);

  if (open) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.setTextSize(3);
    tft.setCursor(4, 50);
    tft.printf("PIN %s", pairing::pin());
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.setCursor(4, 80);
    tft.printf("Pairing open %3us %s", (unsigned)pairing::secondsLeft(),
               pairing::linkActive() ? "[linked]" : "        ");
  } else {
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.setCursor(4, 56);
    tft.print("Pairing closed.");
    tft.setCursor(4, 68);
    tft.print("Press BOOT to pair a device");
  }

  const imu::Data& d = imu::data();
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(4, 98);
  if (imu::ok())
    tft.printf("P:%+6.1f R:%+6.1f Y:%+7.1f ", d.pitch, d.roll, d.yaw);
  else
    tft.print("IMU NOT FOUND              ");
  tft.setCursor(4, 110);
  tft.printf("L:%+4.0f%% R:%+4.0f%%", motors::left() * 100, motors::right() * 100);
  float vb = readBatteryVolts();
  tft.setCursor(150, 110);
  if (isnan(vb)) tft.print("Bat --  "); else tft.printf("%.2fV  ", vb);
  tft.setCursor(4, 122);
  uint32_t s = millis() / 1000;
  tft.printf("Up %02lu:%02lu:%02lu", s / 3600, (s % 3600) / 60, s % 60);
  if (gFailsafe) {
    tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    tft.setCursor(130, 122);
    tft.print("FAILSAFE");
  } else {
    tft.fillRect(130, 122, 60, 8, ST77XX_BLACK);
  }
}

// ------------------------------------------------------------------ setup
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32-S3 Rover ===");

  pinMode(TFT_I2C_POWER, OUTPUT);   // powers the TFT and STEMMA QT port
  digitalWrite(TFT_I2C_POWER, HIGH);
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
  delay(100);

  status_led::begin();
  motors::begin();  // outputs low before anything else can glitch them

  tft.init(135, 240);
  tft.setRotation(3);

  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(0x36);
  gFuelGauge = Wire.endTransmission() == 0;
  bool imuOk = imu::begin();
  Serial.println(imuOk ? "MPU6050 ready" : "WARNING: MPU6050 not found");
  Serial.println(gFuelGauge ? "MAX17048 fuel gauge found" : "No fuel gauge");

  // WiFi AP with a per-device random password, persisted in NVS.
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char ssid[20];
  snprintf(ssid, sizeof(ssid), "Rover-%02X%02X", mac[4], mac[5]);
  Preferences prefs;
  prefs.begin("rover", false);
  gPass = prefs.getString("wpass", "");
  if (gPass.length() < 8) {
    gPass = makePassword();
    prefs.putString("wpass", gPass);
  }
  prefs.end();
  gSsid = ssid;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(gSsid.c_str(), gPass.c_str(), 6, 0, AP_MAX_CLIENTS);
  gIp = WiFi.softAPIP().toString();
  Serial.printf("AP '%s' up at %s\n", gSsid.c_str(), gIp.c_str());

  server.on("/", []() { server.send_P(200, "text/html", HTML_INDEX); });
  server.on("/json", []() {
    JsonDocument doc;
    buildTelemetry(doc);
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWsEvent);

  pairing::begin(ssid, gSsid.c_str(), gPass.c_str(), gIp.c_str());
  pairing::open();

  tftStatic();
  gTftPairingShown = true;
  if (!imuOk) status_led::flash(status_led::FAULT);
}

// ------------------------------------------------------------------- loop
void loop() {
  server.handleClient();
  webSocket.loop();
  pairing::loop();
  imu::update();

  uint32_t now = millis();

  // Drive failsafe: no command recently -> stop.
  bool moving = motors::left() != 0 || motors::right() != 0;
  if (moving && now - gLastCmdMs > DRIVE_TIMEOUT_MS) {
    motors::stop();
    gFailsafe = true;
    status_led::flash(status_led::FAILSAFE);
  }

  // BOOT button (active low): open the pairing window.
  static bool lastBtn = true;
  static uint32_t btnT = 0;
  bool btn = digitalRead(PIN_BOOT_BTN);
  if (!btn && lastBtn && now - btnT > 200) {
    btnT = now;
    pairing::open();
  }
  lastBtn = btn;

  if (pairing::takeSuccess()) status_led::flash(status_led::PAIR_OK);
  if (pairing::takeFailure()) status_led::flash(status_led::PAIR_FAIL);
  status_led::update(!imu::ok(), pairing::isOpen(), pairing::linkActive(),
                     gWsClients > 0, moving);

  static uint32_t lastTel = 0;
  if (now - lastTel >= TELEMETRY_PERIOD_MS) {
    lastTel = now;
    if (gWsClients) {
      JsonDocument doc;
      buildTelemetry(doc);
      String out;
      serializeJson(doc, out);
      webSocket.broadcastTXT(out);
    }
  }

  static uint32_t lastTft = 0;
  if (now - lastTft >= TFT_PERIOD_MS) {
    lastTft = now;
    tftUpdate();
  }
}
