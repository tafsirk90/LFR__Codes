#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED ─────────────────────────────
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── Motor Pins ───────────────────────
#define ENA 3
#define IN1 2
#define IN2 4

#define ENB 11
#define IN3 6
#define IN4 7

#define STBY 13

// ── Buttons ──────────────────────────
#define BTN_START 10
#define BTN_STOP  12

// ── Sensors ──────────────────────────
const byte sensorPin[8] = {A7, A6, A5, A4, A3, A2, A1, A0};

// ── PID CONFIG ───────────────────────
float Kp = 0.16;
float Kd = 0.34;
float lastError = 0;

// ── Speed ────────────────────────────
int BASE_SPEED        = 160;
int MAX_SPEED         = 255;
int MOTOR_MIN         = 80;
int SEARCH_SPEED      = 130;

// ── Junction handling ────────────────
int JUNCTION_TURN_SPEED = 150;
int JUNCTION_HOLD_MS    = 220;

// ── Calibration ──────────────────────
float calWhite[8];
float calBlack[8];
int   sensorThresh[8];
char  sensorStr[9];

// ── Inversion ────────────────────────
#define INVERT_HYSTERESIS   6
#define INVERT_COOLDOWN_MS  1500  // minimum ms between mode switches
#define TOLERANCE_RATIO 0.35f

bool  invertedMode     = false;
unsigned long lastFlipTime = 0;  // cooldown: prevents flipping twice too fast
int   invertConfirm    = 0;
int   normalConfirm    = 0;

// ── OLED refresh throttle ─────────────
// Drawing every loop() iteration is too slow — throttle to every 80ms
unsigned long lastOledUpdate = 0;
#define OLED_REFRESH_MS 80

// ── State ────────────────────────────
bool running         = false;
int  lastDir         = 1;
bool allBlackForward = false;

// ── Priority System ──────────────────
enum Priority { PRIORITY_CENTER, PRIORITY_LEFT, PRIORITY_RIGHT };
Priority mainPrio  = PRIORITY_CENTER;
Priority subPrio   = PRIORITY_RIGHT;
Priority leastPrio = PRIORITY_LEFT;


// ═════════ MOTOR ═════════

int applyDeadzone(int spd) {
  if (spd == 0) return 0;
  return constrain(spd, MOTOR_MIN, MAX_SPEED);
}

void motorLeft(int spd, bool fwd) {
  digitalWrite(IN1, fwd);
  digitalWrite(IN2, !fwd);
  analogWrite(ENA, applyDeadzone(spd));
}

void motorRight(int spd, bool fwd) {
  digitalWrite(IN3, fwd);
  digitalWrite(IN4, !fwd);
  analogWrite(ENB, applyDeadzone(spd));
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}


// ═════════ CALIBRATION ═════════

void averageSamples(float* out, int samples) {
  float acc[8] = {0};
  for (int s = 0; s < samples; s++) {
    for (int i = 0; i < 8; i++) acc[i] += analogRead(sensorPin[i]);
    delay(5);
  }
  for (int i = 0; i < 8; i++) out[i] = acc[i] / samples;
}

void buildThresholds() {
  float avgWhite = 0, avgBlack = 0;
  for (int i = 0; i < 8; i++) {
    sensorThresh[i] = (int)((calWhite[i] + calBlack[i]) / 2.0f);
    avgWhite += calWhite[i];
    avgBlack += calBlack[i];
  }
  avgWhite /= 8.0f;
  avgBlack /= 8.0f;

  // IR: higher ADC = more reflection = lighter surface
  // Normal (black line, white bg):   calWhite > calBlack
  // Inverted (white line, black bg): calBlack > calWhite
  // Calibration asks user to place on white first, then black —
  // so on an inverted track the user places on black bg first (low reading)
  // then white line (high reading) → calBlack > calWhite → inverted
  invertedMode = (avgBlack > avgWhite);
}

void calibrate() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Place on WHITE");
  display.println("Press START");
  display.display();
  while (digitalRead(BTN_START) == HIGH);
  delay(200);
  averageSamples(calWhite, 20);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Place on BLACK");
  display.println("Press START");
  display.display();
  while (digitalRead(BTN_START) == HIGH);
  delay(200);
  averageSamples(calBlack, 20);

  buildThresholds();
}


// ═════════ SENSOR READING ═════════

void readSensors() {
  // Normal:   line is BLACK → low ADC → raw < threshold → we want '1' for line
  //           so: raw < threshold = '1'  (line detected)
  // Inverted: line is WHITE → high ADC → raw > threshold → we want '1' for line
  //           so: raw > threshold = '1'  (line detected)
  //
  // In both cases sensorStr[i]='1' means "line detected here"
  for (int i = 0; i < 8; i++) {
    int raw = analogRead(sensorPin[i]);
    if (!invertedMode)
      sensorStr[i] = (raw < sensorThresh[i]) ? '1' : '0';  // black line = low ADC
    else
      sensorStr[i] = (raw > sensorThresh[i]) ? '1' : '0';  // white line = high ADC
  }
  sensorStr[8] = '\0';
}


// ═════════ OLED RUNNING DISPLAY ═════════
//
// Layout (128×64):
//
//  Line: NORMAL          ← mode label      (y=0)
//  ████░░░░░░░░░░░░      ← sensor bar      (y=12)
//  S0 S1 S2 S3 S4 S5 S6 S7  ← labels      (y=28)
//  Pri: Ctr>Rgt>Lft      ← priority        (y=42)
//  [■■■■░░░░░░░░░░░░]    ← position bar    (y=54)

void drawRunningOLED() {
  display.clearDisplay();

  // ── Row 1: mode label ──────────────────────────────
  display.setTextSize(1);
  display.setCursor(0, 0);
  if (invertedMode) {
    // Draw "INVERTED" with a filled rect behind it to invert display colors
    display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(2, 1);
    display.print("Line: INVERTED");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(2, 1);
    display.print("Line: NORMAL");
  }

  // ── Row 2: sensor blocks ───────────────────────────
  // 8 blocks, each 14px wide with 2px gap, starting at x=4
  // Active sensor = filled block, inactive = outline only
  int blockW = 13;
  int blockH = 12;
  int startX = 4;
  int startY = 13;

  for (int i = 0; i < 8; i++) {
    int x = startX + i * (blockW + 2);
    if (sensorStr[i] == '1') {
      display.fillRect(x, startY, blockW, blockH, SSD1306_WHITE);
    } else {
      display.drawRect(x, startY, blockW, blockH, SSD1306_WHITE);
    }
    // Sensor index label inside or below block
    display.setTextColor(sensorStr[i] == '1' ? SSD1306_BLACK : SSD1306_WHITE);
    display.setCursor(x + 3, startY + 2);
    display.print(i);
  }
  display.setTextColor(SSD1306_WHITE);

  // ── Row 3: priority ───────────────────────────────
  const char* shortNames[] = { "Ctr", "Lft", "Rgt" };
  display.setCursor(0, 28);
  display.print("P:");
  display.print(shortNames[mainPrio]);
  display.print(">");
  display.print(shortNames[subPrio]);
  display.print(">");
  display.print(shortNames[leastPrio]);

  // ── Row 4: position bar ───────────────────────────
  // Shows where the line centroid is across the sensor array
  // Bar spans full width; marker moves left/right
  float pos = -1;
  int sum = 0, count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorStr[i] == '1') { sum += i * 1000; count++; }
  }
  if (count > 0) pos = (float)sum / count;

  display.setCursor(0, 40);
  display.print("Pos:");

  // Draw track bar
  int barX = 28, barY = 40, barW = 96, barH = 8;
  display.drawRect(barX, barY, barW, barH, SSD1306_WHITE);

  if (pos >= 0) {
    // Map pos (0–7000) to bar pixel position
    int markerX = barX + (int)((pos / 7000.0f) * (barW - 4));
    markerX = constrain(markerX, barX, barX + barW - 4);
    display.fillRect(markerX, barY + 1, 4, barH - 2, SSD1306_WHITE);
  } else {
    // Lost line — show question mark in center
    display.setCursor(barX + barW/2 - 3, barY);
    display.print("?");
  }

  // ── Row 5: status ─────────────────────────────────
  display.setCursor(0, 52);
  if (pos == -1) {
    display.print("LOST LINE");
  } else {
    // Show error from center (3500)
    int err = (int)(pos - 3500);
    display.print("Err:");
    display.print(err);
  }

  display.display();
}


// ═════════ STATIC SCREENS ═════════

void showModeOnOLED() {
  const char* names[] = { "Center", "Left", "Right" };
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Mode: ");
  display.println(invertedMode ? "INVERTED" : "NORMAL");
  display.println("== Priority ==");
  display.print("1: "); display.println(names[mainPrio]);
  display.print("2: "); display.println(names[subPrio]);
  display.print("3: "); display.println(names[leastPrio]);
  display.display();
}


// ═════════ AUTO INVERT DETECTION ═════════

// ── ONLY MODIFIED PARTS SHOWN BELOW ──
// Everything else remains EXACTLY the same as your original

// ═════════ AUTO INVERT DETECTION ═════════

void updateInvertDetection() {
  int rawVals[8];
  for (int i = 0; i < 8; i++) rawVals[i] = analogRead(sensorPin[i]);

  int closerToWhite = 0, closerToBlack = 0;

  for (int i = 0; i < 8; i++) {
    float minVal = min(calWhite[i], calBlack[i]);
    float maxVal = max(calWhite[i], calBlack[i]);
    if (abs(maxVal - minVal) < 20) continue;

    float norm = (rawVals[i] - minVal) / (maxVal - minVal);
    norm = constrain(norm, 0.0f, 1.0f);

    if (norm < TOLERANCE_RATIO)             closerToWhite++;
    else if (norm > 1.0f - TOLERANCE_RATIO) closerToBlack++;
  }

  // ✅ FIX 2: require strong difference before switching
  bool strongInvert = (closerToBlack - closerToWhite) >= 4;
  bool strongNormal = (closerToWhite - closerToBlack) >= 4;

  bool shouldBeInverted = strongInvert;
  bool shouldBeNormal   = strongNormal;

  if (!invertedMode && shouldBeInverted) {
    invertConfirm++;
    normalConfirm = 0;
    if (invertConfirm >= INVERT_HYSTERESIS) {
      invertedMode  = true;
      invertConfirm = 0;
    }
  } 
  else if (invertedMode && shouldBeNormal) {
    normalConfirm++;
    invertConfirm = 0;
    if (normalConfirm >= INVERT_HYSTERESIS) {
      invertedMode  = false;
      normalConfirm = 0;
    }
  } 
  else {
    if (invertConfirm > 0) invertConfirm--;
    if (normalConfirm > 0) normalConfirm--;
  }
}


// ═════════ DETECTION ═════════

bool isJunction() {
  int count = 0;
  for (int i = 0; i < 8; i++)
    if (sensorStr[i] == '1') count++;
  return (count >= 5);
}

bool centerActive() {
  return (sensorStr[3] == '1' && sensorStr[4] == '1');
}

int sideBranch() {
  int leftCount = 0, rightCount = 0;
  for (int i = 0; i <= 2; i++) if (sensorStr[i] == '1') leftCount++;
  for (int i = 5; i <= 7; i++) if (sensorStr[i] == '1') rightCount++;
  if (rightCount >= 2) return 1;
  if (leftCount  >= 2) return -1;
  return 0;
}

bool allBlack() {
  for (int i = 0; i < 8; i++)
    if (sensorStr[i] == '0') return false;
  return true;
}

float getPosition() {
  int sum = 0, count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorStr[i] == '1') { sum += i * 1000; count++; }
  }
  if (count == 0) return -1;
  return (float)sum / count;
}


// ═════════ PRIORITY SYSTEM ═════════

void cyclePriority() {
  mainPrio  = (Priority)((mainPrio + 1) % 3);
  subPrio   = (Priority)((mainPrio + 1) % 3);
  leastPrio = (Priority)((mainPrio + 2) % 3);
  showModeOnOLED();
  delay(300);
}

bool tryDirection(Priority p) {
  if (p == PRIORITY_CENTER && centerActive()) return true;
  if (p == PRIORITY_RIGHT  && sideBranch() == 1) {
    motorLeft(JUNCTION_TURN_SPEED, true);
    motorRight(JUNCTION_TURN_SPEED, false);
    delay(JUNCTION_HOLD_MS);
    lastDir = 1;
    return true;
  }
  if (p == PRIORITY_LEFT && sideBranch() == -1) {
    motorLeft(JUNCTION_TURN_SPEED, false);
    motorRight(JUNCTION_TURN_SPEED, true);
    delay(JUNCTION_HOLD_MS);
    lastDir = -1;
    return true;
  }
  return false;
}


// ═════════ SETUP ═════════

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP,  INPUT_PULLUP);

  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { for (;;); }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibrating...");
  display.display();

  calibrate();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("READY");
  display.println("Short: Start");
  display.println("Long:  Priority");
  display.display();
  delay(1500);

  showModeOnOLED();
}


// ═════════ LOOP ═════════

void loop() {

  // ── Buttons ──────────────────────────────────────────────
  if (digitalRead(BTN_START) == LOW) {
    unsigned long pressTime = millis();
    while (digitalRead(BTN_START) == LOW);

    if (!running) {
      if (millis() - pressTime > 600) {
        cyclePriority();
      } else {
        running = true;
        delay(200);
      }
    }
  }

  if (digitalRead(BTN_STOP) == LOW) {
    running       = false;
    lastFlipTime  = 0;     // clear cooldown for next run
    invertConfirm = 0;
    normalConfirm = 0;
    stopMotors();
    showModeOnOLED();
  }

  if (!running) return;

  // ── Read sensors FIRST so detection uses current frame data ─
  readSensors();

  // ── Auto invert detection (uses fresh sensorStr) ──────────
  updateInvertDetection();

  // ── OLED update (throttled) ───────────────────────────────
  if (millis() - lastOledUpdate >= OLED_REFRESH_MS) {
    drawRunningOLED();
    lastOledUpdate = millis();
  }

  // ── STOP BAR ─────────────────────────────────────────────
  if (allBlack()) {
    if (!allBlackForward) {
      motorLeft(BASE_SPEED, true);
      motorRight(BASE_SPEED, true);
      delay(150);
      allBlackForward = true;
    }
    readSensors();
    if (allBlack()) {
      stopMotors();
      running = false;
      showModeOnOLED();
      return;
    }
  } else {
    allBlackForward = false;
  }

  // ── PRIORITY JUNCTION HANDLER ─────────────────────────────
  if (isJunction() || centerActive()) {
    bool handled = tryDirection(mainPrio);
    if (!handled) handled = tryDirection(subPrio);
    if (!handled) tryDirection(leastPrio);

    if (!centerActive() || isJunction()) return;
  }

  // ── LOST LINE RECOVERY ────────────────────────────────────
  float pos = getPosition();

  if (pos == -1) {
    if (lastDir > 0) {
      motorLeft(SEARCH_SPEED, true);
      motorRight(SEARCH_SPEED, false);
    } else {
      motorLeft(SEARCH_SPEED, false);
      motorRight(SEARCH_SPEED, true);
    }
    return;
  }

  // ── PID ───────────────────────────────────────────────────
  float error      = pos - 3500;
  if (error > 600)       lastDir = 1;
  else if (error < -600) lastDir = -1;

  float derivative = error - lastError;
  float correction = Kp * error + Kd * derivative;
  lastError = error;

  int leftSpeed  = constrain((int)(BASE_SPEED + correction), 0, 255);
  int rightSpeed = constrain((int)(BASE_SPEED - correction), 0, 255);

  motorLeft(leftSpeed,  true);
  motorRight(rightSpeed, true);
}
