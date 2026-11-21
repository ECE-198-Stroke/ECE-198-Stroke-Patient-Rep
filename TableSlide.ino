/*
 * TableSlideAccel - measures one table slide rep per trigger, using an MMA7361 on an ESP32.
 *
 * Board:    Tools > Board > esp32 > ESP32 Dev Module (ESP32-WROOM DevKit)
 * Output:   Serial Monitor at 115200 baud, and a web page (see Network below)
 * Sensor:   MMA7361 powered from VIN (5V); X -> D34, Y -> D35, Z -> D32; SL to the module's
 *           3V3 pin; GS to GND. Mount the module flat, with Y along the slide.
 *           All three are ADC1 pins, which keep working when Wi-Fi is on.
 * Button:   between D27 and GND, no resistor (uses the internal pull-up). Keep it off the sled.
 *
 * Network:  set your Wi-Fi in wifi_config.h. The address is printed on Serial at startup.
 *           Without Wi-Fi it creates an open network named "TableSlide": join it and open
 *           http://192.168.4.1. There is no login, so anyone on that network can start reps.
 *   GET  /            web page: start button, live status, results
 *   GET  /api/status  JSON status, session summary and recent reps
 *   POST /api/start   start (or restart) a rep, same as the button
 *   POST /api/reset   clear the session
 *
 * Each trigger: keep the sled still while it calibrates (about 1 s). The onboard LED (GPIO2)
 * turns on when it is ready. Slide out and back, then stop. The rep is reported and the LED
 * turns off. Triggering again during an attempt restarts it.
 *
 * Type in the Serial Monitor:
 *   s = same as a button press, e = print the session summary and reset it,
 *   r = raw data on/off, x/y/z = change the slide axis, ? = status
 */

// Arudino libraries
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>

// Dependencies
#include "AccelCore.h"
#include "web_ui.h"
#include "wifi_config.h"

static const uint8_t PIN_BUTTON = 27;
static const uint8_t PIN_READY_LED = 2;  // Blue onboard LED on most ESP32 DevKits.
static const uint8_t AXIS_PINS[] = {34, 35, 32};
static const char AXIS_NAMES[] = {'x', 'y', 'z'};

static const float V_PER_G = 0.8f;  // +/-1.5 g range. GS must be connected to GND.
static const int ADC_SAMPLES = 16;
static const uint32_t SAMPLE_PERIOD_MS = 10;
static const uint32_t ATTEMPT_TIMEOUT_MS = 30000;  // Gives up if a rep never finishes.
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static const int HISTORY_SIZE = 20;  // Reps kept for the web page.

struct SessionStats {
  uint32_t reps = 0;
  float timeSum = 0;
  float jerkSum = 0;
  float distanceSum = 0;
  uint32_t distanceCount = 0;
};

struct HistoryEntry {
  uint32_t number;
  accel::RepResult rep;
};

static accel::Tracker tracker;
static accel::Debouncer button;
static SessionStats session;
static HistoryEntry history[HISTORY_SIZE];
static int historyCount = 0;  // Newest entry is at (historyNext - 1).
static int historyNext = 0;
static uint8_t axis = 1;  // Y: real recordings showed the slide on this axis, with X only tilting.
static bool rawMode = false;
static bool waitingForRep = false;
static uint32_t readySinceMs = 0;
static String statusMessage = "Press Start or the button to begin a rep.";
static WebServer server(80);
static String networkAddress = "not connected";

// Volts-based g with no zero offset: the tracker calibrates that out for each attempt.
// analogReadMilliVolts applies the ESP32's factory ADC calibration.
static float readAxisG(uint8_t which) {
  uint32_t totalMv = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    totalMv += analogReadMilliVolts(AXIS_PINS[which]);
  }
  return (totalMv / (float)ADC_SAMPLES) / 1000.0f / V_PER_G;
}

static void setReadyLight(bool on) {
  digitalWrite(PIN_READY_LED, on ? HIGH : LOW);
}

static const char *stateName() {
  if (!tracker.active()) {
    return "idle";
  }
  return waitingForRep ? "ready" : "calibrating";
}

static void setMessage(const String &message) {
  statusMessage = message;
  Serial.print(F("# "));
  Serial.println(message);
}

static void printDistance(float distanceCm) {
  if (distanceCm >= 0) {
    Serial.println(distanceCm, 0);
  } else {
    Serial.println(F("na"));
  }
}

static void printStatus() {
  Serial.print(F("# status axis="));
  Serial.print(AXIS_NAMES[axis]);
  Serial.print(F(" attempt="));
  Serial.print(stateName());
  Serial.print(F(" reps="));
  Serial.print(session.reps);
  Serial.print(F(" web="));
  Serial.println(networkAddress);
}

static void printSessionSummary() {
  Serial.print(F("SESSION reps="));
  Serial.print(session.reps);
  Serial.print(F(" mean_rep_s="));
  Serial.print(session.reps ? session.timeSum / session.reps : 0.0f, 2);
  Serial.print(F(" mean_jerk="));
  Serial.print(session.reps ? session.jerkSum / session.reps : 0.0f, 3);
  Serial.print(F(" mean_est_dist_cm="));
  printDistance(session.distanceCount ? session.distanceSum / session.distanceCount : -1.0f);
}

static void resetSession() {
  session = SessionStats();
  historyCount = 0;
  historyNext = 0;
}

static void endAttempt(uint32_t now) {
  tracker.endSet(now);
  waitingForRep = false;
  setReadyLight(false);
}

static void startAttempt(uint32_t now, const char *source) {
  waitingForRep = false;
  setReadyLight(false);
  tracker.startSet(now);
  setMessage(String("Started from ") + source + ". Keep the sled still...");
}

static void recordRep(const accel::RepResult &rep) {
  session.reps++;
  session.timeSum += rep.durationS;
  session.jerkSum += rep.jerkRmsGS;
  if (rep.distanceCm >= 0) {
    session.distanceSum += rep.distanceCm;
    session.distanceCount++;
  }

  history[historyNext] = {session.reps, rep};
  historyNext = (historyNext + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) {
    historyCount++;
  }

  Serial.print(F("REP "));
  Serial.print(session.reps);
  Serial.print(F(" time_s="));
  Serial.print(rep.durationS, 2);
  Serial.print(F(" peak_g="));
  Serial.print(rep.peakG, 3);
  Serial.print(F(" jerk="));
  Serial.print(rep.jerkRmsGS, 3);
  Serial.print(F(" est_dist_cm="));
  printDistance(rep.distanceCm);
}

static void setAxis(uint8_t newAxis) {
  if (tracker.active()) {
    endAttempt(millis());
  }
  axis = newAxis;
  printStatus();
}

// ---- Web server ----

static String jsonNumber(float value, int decimals) {
  return String(value, decimals);
}

static String jsonOptional(float value, int decimals, bool present) {
  return present ? String(value, decimals) : String("null");
}

static String jsonEscape(const String &text) {
  String out;
  for (size_t i = 0; i < text.length(); i++) {
    const char c = text[i];
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    if ((uint8_t)c >= 0x20) {
      out += c;
    }
  }
  return out;
}

static String statusJson() {
  String json;
  json.reserve(256 + historyCount * 110);
  json += "{\"state\":\"";
  json += stateName();
  json += "\",\"message\":\"";
  json += jsonEscape(statusMessage);
  json += "\",\"axis\":\"";
  json += AXIS_NAMES[axis];
  json += "\",\"session\":{\"reps\":";
  json += session.reps;
  json += ",\"meanTime\":";
  json += jsonOptional(session.reps ? session.timeSum / session.reps : 0, 2, session.reps > 0);
  json += ",\"meanJerk\":";
  json += jsonOptional(session.reps ? session.jerkSum / session.reps : 0, 3, session.reps > 0);
  json += ",\"meanDist\":";
  json += jsonOptional(session.distanceCount ? session.distanceSum / session.distanceCount : 0, 1,
                       session.distanceCount > 0);
  json += "},\"history\":[";
  for (int i = 0; i < historyCount; i++) {
    const HistoryEntry &h = history[(historyNext - 1 - i + HISTORY_SIZE) % HISTORY_SIZE];
    if (i > 0) {
      json += ',';
    }
    json += "{\"n\":";
    json += h.number;
    json += ",\"time\":";
    json += jsonNumber(h.rep.durationS, 2);
    json += ",\"peak\":";
    json += jsonNumber(h.rep.peakG, 3);
    json += ",\"jerk\":";
    json += jsonNumber(h.rep.jerkRmsGS, 3);
    json += ",\"dist\":";
    json += jsonOptional(h.rep.distanceCm, 1, h.rep.distanceCm >= 0);
    json += '}';
  }
  json += "]}";
  return json;
}

static void sendJson(int code, const String &body) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

static void setupWebServer() {
  server.on("/", HTTP_GET, [] { server.send(200, "text/html", WEB_UI); });
  server.on("/api/status", HTTP_GET, [] { sendJson(200, statusJson()); });
  server.on("/api/start", HTTP_POST, [] {
    startAttempt(millis(), "web");
    sendJson(200, statusJson());
  });
  server.on("/api/reset", HTTP_POST, [] {
    if (tracker.active()) {
      endAttempt(millis());
    }
    resetSession();
    setMessage("Session cleared.");
    sendJson(200, statusJson());
  });
  server.onNotFound([] { sendJson(404, "{\"error\":\"not found\"}"); });
  server.begin();
}

static void startOwnNetwork() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_NAME);  // Open network: no password, as requested.
  networkAddress = String("http://") + WiFi.softAPIP().toString() + " on Wi-Fi \"" + AP_NAME + "\"";
}

static void setupNetwork() {
  if (strlen(WIFI_SSID) > 0) {
    Serial.print(F("# joining Wi-Fi \""));
    Serial.print(WIFI_SSID);
    Serial.println(F("\"..."));
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    networkAddress = String("http://") + WiFi.localIP().toString();
  } else {
    if (strlen(WIFI_SSID) > 0) {
      Serial.println(F("# could not join Wi-Fi, starting own network instead"));
    }
    startOwnNetwork();
  }

  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
  }
  setupWebServer();
  Serial.print(F("# web page: "));
  Serial.print(networkAddress);
  Serial.print(F("  (or http://"));
  Serial.print(MDNS_NAME);
  Serial.println(F(".local)"));
}

// ---- Serial ----

static void handleCommand(char command) {
  switch (command) {
    case 's':
      startAttempt(millis(), "serial");
      break;
    case 'e':
      printSessionSummary();
      resetSession();
      break;
    case 'r':
      rawMode = !rawMode;
      Serial.println(rawMode ? F("# raw on: t_ms,slide_axis_g,x_g,y_g,z_g") : F("# raw off"));
      break;
    case 'x':
      setAxis(0);
      break;
    case 'y':
      setAxis(1);
      break;
    case 'z':
      setAxis(2);
      break;
    case '?':
      printStatus();
      break;
    case '\r':
    case '\n':
    case ' ':
      break;
    default:
      Serial.println(F("# commands: s e r x y z ?"));
      break;
  }
}

static void printRawLine(uint32_t now, float g) {
  Serial.print(now);
  Serial.print(',');
  Serial.print(g, 4);
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(',');
    Serial.print(i == axis ? g : readAxisG(i), 4);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // About 0 to 3.1 V, covering the sensor's 0.45 to 2.85 V.
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_READY_LED, OUTPUT);
  setReadyLight(false);
  delay(500);
  Serial.println(F("# TableSlideAccel. Press the button, or Start on the web page."));
  setupNetwork();
  printStatus();
}

void loop() {
  server.handleClient();

  while (Serial.available() > 0) {
    handleCommand((char)Serial.read());
  }

  // Checked on every pass, so a press is not missed between sensor readings.
  if (button.update(digitalRead(PIN_BUTTON) == LOW, millis())) {
    startAttempt(millis(), "button");
  }

  static uint32_t lastSample = 0;
  const uint32_t now = millis();
  if (now - lastSample < SAMPLE_PERIOD_MS) {
    return;
  }
  lastSample = now;

  const float g = readAxisG(axis);
  if (rawMode) {
    printRawLine(now, g);
  }

  switch (tracker.update(g, now)) {
    case accel::EVENT_NONE:
      break;
    case accel::EVENT_CALIBRATED:
      waitingForRep = true;
      readySinceMs = now;
      setReadyLight(true);
      setMessage("Ready, start sliding.");
      break;
    case accel::EVENT_CALIBRATION_RETRY:
      setMessage("Moved during calibration, keep the sled still...");
      break;
    case accel::EVENT_REP:
      recordRep(tracker.lastResult());
      endAttempt(now);
      setMessage("Rep recorded. Start the next one when ready.");
      break;
  }

  if (waitingForRep && now - readySinceMs >= ATTEMPT_TIMEOUT_MS) {
    endAttempt(now);
    setMessage("Timed out with no complete rep. Start again to retry.");
  }
}
