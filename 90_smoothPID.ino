// =====================================================
// LFR: Arduino Nano + L298N + 8-Sensor Array + OLED
// Pins/peripherals: matches uploaded reference layout
// Core kept: Read->Decide->Execute, Calibration,
//            Backtracking, Deadzone fix, Bias
// =====================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED ──────────────────────────────────────────────
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── Motor Pins (L298N) ──────────────────────────────────
#define ENA  3
#define IN1  2
#define IN2  4

#define ENB  11
#define IN3  6
#define IN4  7

#define STBY 13   // driver enable, HIGH at boot

// ── Buttons ───────────────────────────────────────────────
#define BTN_START 10
#define BTN_STOP  12

// ── Sensor Pins ───────────────────────────────────────────
const byte sensorPin[8] = {A7, A6, A5, A4, A3, A2, A1, A0};

// ── Config (core values unchanged) ─────────────────────────
const int BASE_SPEED        = 150;
const int TURN_SPEED        = 180;
const int SHORT_MOVE_MS     = 80;
const int POST_ROTATE_DELAY = 50;
const int HISTORY_SIZE      = 100;
const int MIN_RECOVER_ONES  = 2;
const int MOTOR_MIN         = 80;
const unsigned long OLED_INTERVAL = 150; // ms between OLED refreshes

#define LEFT_BIAS   0
#define RIGHT_BIAS  10

#define DEBUG_SERIAL true   // toggle Serial prints

// ── Move Codes ────────────────────────────────────────────
#define MV_FORWARD  0
#define MV_LEFT     1
#define MV_RIGHT    2
#define MV_ROT_L90  3
#define MV_ROT_R90  4

// ── Calibration ───────────────────────────────────────────
int sensorMin[8];
int sensorMax[8];
int sensorThresh[8];

// ── History Stack ─────────────────────────────────────────
int history[HISTORY_SIZE];
int histIdx = 0;

// ── Sensor Buffer ──────────────────────────────────────────
char sensorStr[9];

// ── Run State ─────────────────────────────────────────────
bool running      = false;
bool lastStartBtn = HIGH;
bool lastStopBtn  = HIGH;
unsigned long lastOledUpdate = 0;

// ═══════════════════════════════════════════════════════════
//                    MOTOR PRIMITIVES
// ═══════════════════════════════════════════════════════════

int applyDeadzone(int spd) {
  if (spd == 0) return 0;                  // true stop preserved
  return constrain(spd, MOTOR_MIN, 255);
}

void motorLeft(int spd, bool fwd) {
  digitalWrite(IN1, fwd ? HIGH : LOW);
  digitalWrite(IN2, fwd ? LOW  : HIGH);
  analogWrite(ENA, applyDeadzone(spd));
}

void motorRight(int spd, bool fwd) {
  digitalWrite(IN3, fwd ? HIGH : LOW);
  digitalWrite(IN4, fwd ? LOW  : HIGH);
  analogWrite(ENB, applyDeadzone(spd));
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void goForward() {
  motorLeft (BASE_SPEED + LEFT_BIAS,  true);
  motorRight(BASE_SPEED + RIGHT_BIAS, true);
}

void goBackward() {
  motorLeft (BASE_SPEED + LEFT_BIAS,  false);
  motorRight(BASE_SPEED + RIGHT_BIAS, false);
}

void turnLeft()  { motorLeft(0, true);          motorRight(TURN_SPEED, true); }
void turnRight() { motorLeft(TURN_SPEED, true); motorRight(0, true);          }

// ═══════════════════════════════════════════════════════════
//                    SENSOR FUNCTIONS
// ═══════════════════════════════════════════════════════════

void readSensors(char* s) {
  for (int i = 0; i < 8; i++)
    s[i] = (analogRead(sensorPin[i]) > sensorThresh[i]) ? '1' : '0';
  s[8] = '\0';
}

int countOnes(char* s) {
  int c = 0;
  for (int i = 0; i < 8; i++) if (s[i] == '1') c++;
  return c;
}

float weightedCenter(char* s) {
  int sum = 0, count = 0;
  for (int i = 0; i < 8; i++) {
    if (s[i] == '1') { sum += i; count++; }
  }
  return (count == 0) ? -1.0 : (float)sum / count;
}

bool leftHeavy(char* s) {
  int c = 0;
  for (int i = 0; i < 4; i++) if (s[i] == '1') c++;
  return c >= 3;
}

bool rightHeavy(char* s) {
  int c = 0;
  for (int i = 4; i < 8; i++) if (s[i] == '1') c++;
  return c >= 3;
}

bool isIntersection(char* s) {
  return (countOnes(s) >= 5) && (s[0] == '1' || s[7] == '1');
}

bool isLineLost(char* s) {
  return countOnes(s) == 0;
}

// ═══════════════════════════════════════════════════════════
//                    OLED HELPERS
// ═══════════════════════════════════════════════════════════

void showOLED(const char* line1, const char* line2) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(line1);
  display.setCursor(0, 16);
  display.println(line2);
  display.display();
}

void updateStatusOLED(const char* line1, const char* line2) {
  if (millis() - lastOledUpdate < OLED_INTERVAL) return;
  showOLED(line1, line2);
  lastOledUpdate = millis();
}

const char* moveLabel(int mv) {
  switch (mv) {
    case MV_FORWARD: return "FORWARD";
    case MV_LEFT:    return "LEFT";
    case MV_RIGHT:   return "RIGHT";
    case MV_ROT_L90: return "ROT L90";
    case MV_ROT_R90: return "ROT R90";
    default:         return "?";
  }
}

// ═══════════════════════════════════════════════════════════
//                    CALIBRATION
// ═══════════════════════════════════════════════════════════

void calibrate() {
  showOLED("CALIBRATE", "Place on WHITE");
  if (DEBUG_SERIAL) Serial.println("Place on WHITE...");
  delay(3000);
  for (int i = 0; i < 8; i++) sensorMin[i] = analogRead(sensorPin[i]);

  showOLED("CALIBRATE", "Place on BLACK");
  if (DEBUG_SERIAL) Serial.println("Place on BLACK...");
  delay(3000);
  for (int i = 0; i < 8; i++) sensorMax[i] = analogRead(sensorPin[i]);

  for (int i = 0; i < 8; i++) {
    if (sensorMin[i] > sensorMax[i]) {
      int tmp = sensorMin[i];
      sensorMin[i] = sensorMax[i];
      sensorMax[i] = tmp;
    }
    sensorThresh[i] = (sensorMin[i] + sensorMax[i]) / 2;
    if (DEBUG_SERIAL) {
      Serial.print("S"); Serial.print(i);
      Serial.print(" thresh="); Serial.println(sensorThresh[i]);
    }
  }

  showOLED("CALIBRATION", "DONE");
  delay(800);
}

// ═══════════════════════════════════════════════════════════
//                    ROTATION (sensor-based)
// ═══════════════════════════════════════════════════════════

void rotateLeft90() {
  motorLeft (TURN_SPEED, false);
  motorRight(TURN_SPEED, true);
  delay(100);
  while (true) {
    readSensors(sensorStr);
    if (countOnes(sensorStr) >= MIN_RECOVER_ONES) break;
  }
  stopMotors();
  delay(POST_ROTATE_DELAY);
}

void rotateRight90() {
  motorLeft (TURN_SPEED, true);
  motorRight(TURN_SPEED, false);
  delay(100);
  while (true) {
    readSensors(sensorStr);
    if (countOnes(sensorStr) >= MIN_RECOVER_ONES) break;
  }
  stopMotors();
  delay(POST_ROTATE_DELAY);
}

// ═══════════════════════════════════════════════════════════
//                    DECIDE / EXECUTE
// ═══════════════════════════════════════════════════════════

int decide(char* s) {
  if (isLineLost(s)) return -1;

  if (isIntersection(s)) {
    if (leftHeavy(s) && !rightHeavy(s)) return MV_ROT_L90;
    if (rightHeavy(s) && !leftHeavy(s)) return MV_ROT_R90;
    return MV_ROT_R90;                 // full cross -> default right
  }

  float center = weightedCenter(s);
  if (center < 0)   return -1;
  if (center < 2.5) return MV_LEFT;
  if (center > 4.5) return MV_RIGHT;
  return MV_FORWARD;
}

void executeMove(int mv) {
  switch (mv) {
    case MV_FORWARD: goForward(); delay(SHORT_MOVE_MS); stopMotors(); break;
    case MV_LEFT:    turnLeft();  delay(SHORT_MOVE_MS); stopMotors(); break;
    case MV_RIGHT:   turnRight(); delay(SHORT_MOVE_MS); stopMotors(); break;
    case MV_ROT_L90: rotateLeft90();  break;
    case MV_ROT_R90: rotateRight90(); break;
  }
}

// ═══════════════════════════════════════════════════════════
//                    HISTORY / BACKTRACK
// ═══════════════════════════════════════════════════════════

void pushHistory(int mv) {
  if (histIdx < HISTORY_SIZE) history[histIdx++] = mv;
}

void backtrack() {
  showOLED(sensorStr, "BACKTRACK");
  if (DEBUG_SERIAL) Serial.println("!! BACKTRACK");
  stopMotors();
  delay(100);

  while (histIdx > 0) {
    int mv = history[--histIdx];
    switch (mv) {
      case MV_FORWARD: goBackward(); delay(SHORT_MOVE_MS); stopMotors(); break;
      case MV_LEFT:    turnRight();  delay(SHORT_MOVE_MS); stopMotors(); break;
      case MV_RIGHT:   turnLeft();   delay(SHORT_MOVE_MS); stopMotors(); break;
      case MV_ROT_L90: rotateRight90(); break;  // undo left = rotate right
      case MV_ROT_R90: rotateLeft90();  break;
    }

    readSensors(sensorStr);
    if (!isLineLost(sensorStr) && countOnes(sensorStr) >= MIN_RECOVER_ONES) {
      if (DEBUG_SERIAL) Serial.println("!! Line recovered");
      return;   // preserve remaining history
    }
  }

  if (DEBUG_SERIAL) Serial.println("!! History exhausted");
  histIdx = 0;
  stopMotors();
}

// ═══════════════════════════════════════════════════════════
//                    SETUP
// ═══════════════════════════════════════════════════════════

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP,  INPUT_PULLUP);

  // A6 & A7 are analog-only on the Nano — no pinMode needed
  for (int i = 0; i < 8; i++) {
    if (sensorPin[i] != A6 && sensorPin[i] != A7) pinMode(sensorPin[i], INPUT);
  }

  Serial.begin(9600);
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  showOLED("LFR BOOT", "Calibrating...");
  calibrate();
  showOLED("READY", "Press START");
}

// ═══════════════════════════════════════════════════════════
//                    LOOP
// ═══════════════════════════════════════════════════════════

void loop() {
  // ── Start / Stop buttons ─────────────────────────────────
  bool startBtn = digitalRead(BTN_START);
  bool stopBtn  = digitalRead(BTN_STOP);

  if (startBtn == LOW && lastStartBtn == HIGH) {
    running = true;
    showOLED("RUNNING", "");
    delay(200);
  }
  if (stopBtn == LOW && lastStopBtn == HIGH) {
    running = false;
    stopMotors();
    showOLED("PAUSED", "");
    delay(200);
  }
  lastStartBtn = startBtn;
  lastStopBtn  = stopBtn;

  if (!running) return;

  // ── Step 1: Read ──────────────────────────────────────────
  readSensors(sensorStr);

  // ── Step 2: Decide (before any movement) ───────────────────
  int mv = decide(sensorStr);

  if (mv == -1) {
    backtrack();
    return;
  }

  if (DEBUG_SERIAL) {
    Serial.print("S:"); Serial.print(sensorStr);
    Serial.print(" MV:"); Serial.println(moveLabel(mv));
  }
  updateStatusOLED(sensorStr, moveLabel(mv));

  // ── Step 3: Execute ────────────────────────────────────────
  pushHistory(mv);
  executeMove(mv);
}
