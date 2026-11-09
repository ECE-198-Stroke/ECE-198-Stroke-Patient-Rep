/*
 * TableSlideAccel - measures one table slide rep per button press, using an MMA7361.
 *
 * Board:    Tools > Board > Arduino UNO R4 Boards > Arduino UNO R4 Minima
 * Output:   Serial Monitor at 115200 baud, over USB-C
 * Sensor:   same wiring as AccelTest_MMA7361. Mount the module flat, with Y along the slide.
 * Button:   between D2 and GND, no resistor (uses the internal pull-up). Keep it off the sled.
 *
 * Each press: keep the sled still while it calibrates (about 1 s). The L LED turns on when it
 * is ready. Slide out and back, then stop. The rep is reported and the LED turns off.
 * Pressing again during an attempt restarts it.
 *
 * Type in the Serial Monitor:
 *   s = same as a button press, e = print the session summary and reset it,
 *   r = raw data on/off, x/y/z = change the slide axis, ? = status
 */
#include "AccelCore.h"

static const uint8_t PIN_BUTTON = 2;
static const uint8_t AXIS_PINS[] = {A0, A1, A2};
static const char AXIS_NAMES[] = {'x', 'y', 'z'};

static const float ADC_REF_V = 5.0f;
static const float ADC_MAX = 16383.0f;  // 14-bit
static const float V_PER_G = 0.8f;      // +/-1.5 g range. GS must be connected to GND.
static const int ADC_SAMPLES = 16;
static const uint32_t SAMPLE_PERIOD_MS = 10;
static const uint32_t ATTEMPT_TIMEOUT_MS = 30000;  // Gives up if a rep never finishes.

struct SessionStats {
  uint32_t reps = 0;
  float timeSum = 0;
  float jerkSum = 0;
  float distanceSum = 0;
  uint32_t distanceCount = 0;
};

static accel::Tracker tracker;
static accel::Debouncer button;
static SessionStats session;
static uint8_t axis = 1;  // Y: real recordings showed the slide on this axis, with X only tilting.
static bool rawMode = false;
static bool waitingForRep = false;
static uint32_t readySinceMs = 0;

// Volts-based g with no zero offset: the tracker calibrates that out for each attempt.
static float readAxisG(uint8_t which) {
  uint32_t total = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    total += analogRead(AXIS_PINS[which]);
  }
  return (total / (float)ADC_SAMPLES) * ADC_REF_V / ADC_MAX / V_PER_G;
}

static void setReadyLight(bool on) {
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
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
  Serial.print(tracker.active() ? (waitingForRep ? F("waiting_for_rep") : F("calibrating")) : F("idle"));
  Serial.print(F(" reps="));
  Serial.println(session.reps);
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

static void endAttempt(uint32_t now) {
  tracker.endSet(now);
  waitingForRep = false;
  setReadyLight(false);
}

static void startAttempt(uint32_t now) {
  waitingForRep = false;
  setReadyLight(false);
  tracker.startSet(now);
  Serial.println(F("# keep the sled still..."));
}

static void recordRep(const accel::RepResult &rep) {
  session.reps++;
  session.timeSum += rep.durationS;
  session.jerkSum += rep.jerkRmsGS;
  if (rep.distanceCm >= 0) {
    session.distanceSum += rep.distanceCm;
    session.distanceCount++;
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

static void handleCommand(char command) {
  switch (command) {
    case 's':
      startAttempt(millis());
      break;
    case 'e':
      printSessionSummary();
      session = SessionStats();
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
  const uint32_t started = millis();
  while (!Serial && millis() - started < 3000) {
  }
  analogReadResolution(14);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  setReadyLight(false);
  Serial.println(F("# TableSlideAccel. Press the button to start a rep."));
  printStatus();
}

void loop() {
  while (Serial.available() > 0) {
    handleCommand((char)Serial.read());
  }

  // Checked on every pass, so a press is not missed between sensor readings.
  if (button.update(digitalRead(PIN_BUTTON) == LOW, millis())) {
    startAttempt(millis());
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
      Serial.println(F("# ready, start sliding"));
      break;
    case accel::EVENT_CALIBRATION_RETRY:
      Serial.println(F("# moved during calibration, keep it still..."));
      break;
    case accel::EVENT_REP:
      recordRep(tracker.lastResult());
      endAttempt(now);
      Serial.println(F("# rep done, press the button for the next one"));
      break;
  }

  if (waitingForRep && now - readySinceMs >= ATTEMPT_TIMEOUT_MS) {
    endAttempt(now);
    Serial.println(F("# timed out with no complete rep, press the button to try again"));
  }
}
