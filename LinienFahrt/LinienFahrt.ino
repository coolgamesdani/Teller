#include <math.h>

void drive(float vx, float vy, float omega, int speed);

const int MOTOR_PINS[4][2] = { {21, 19}, {5, 17}, {25, 26}, {27, 14} };
const int MaxSpeed = 255;
const int motorMinPWM[4] = { 220, 198, 207, 185 };
const int motorMaxPWM[4] = { 255, 255, 255, 255 };
const int motorInvert[4] = { -1, 1, -1, -1 };

void setMotor(int i, float value, int maxSpeed) {
  value = constrain(value, -1.0f, 1.0f) * motorInvert[i];
  int a = MOTOR_PINS[i][0], b = MOTOR_PINS[i][1];
  if (fabsf(value) < 0.001f) {
    analogWrite(a, 255);
    analogWrite(b, 255);
    return;
  }
  int maxPWM = min(motorMaxPWM[i], maxSpeed);
  int pwm = motorMinPWM[i] + (int)(fabsf(value) * (maxPWM - motorMinPWM[i]));
  pwm = constrain(pwm, 0, 255);
  analogWrite(a, value > 0 ? pwm : 0);
  analogWrite(b, value > 0 ? 0 : pwm);
}

void drive(float vx, float vy, float omega, int speed) {
  float v[4] = { -vx - vy - omega, vx - vy - omega, -vx + vy - omega, vx + vy - omega };
  float maxMag = 1.0f;
  for (int i = 0; i < 4; i++) maxMag = max(maxMag, fabsf(v[i]));
  for (int i = 0; i < 4; i++) setMotor(i, v[i] / maxMag, speed);
}

void stopMotors() {
  drive(0, 0, 0, MaxSpeed);
}

const int lightPins[3] = { 35, 34, 39 };
const int LIGHT_WHITE[3] = { 467, 1135, 680 };
const int LIGHT_BLACK[3] = { 207,  307, 167 };
int lightRaw[3];

float readDarkness(int i) {
  lightRaw[i] = analogRead(lightPins[i]);
  float d = (float)(LIGHT_WHITE[i] - lightRaw[i]) / (float)(LIGHT_WHITE[i] - LIGHT_BLACK[i]);
  return constrain(d, 0.0f, 1.0f);
}

float LINE_SPEED = 0.80f;
float LINE_TURN_GAIN = 0.80f;
float LINE_TURN_DAMP = 0.02f;
float LINE_CURVE_SLOW = 0.20f;
float LINE_PIVOT_TURN = 0.40f;
float LINE_CORNER_DARK = 0.85f;
unsigned long LINE_PIVOT_OFF = 160;
const unsigned long LINE_PIVOT_ON = 40;
const float LINE_SEARCH_SPEED = 0.10f;
const float LINE_DARK = 0.5f;
const float LINE_CORNER_CLEAR = 0.30f;
const float LINE_STOP_DARK = 0.9f;
const float LINE_SIDE_ERROR = 0.3f;
const unsigned long LINE_CORNER_RISE = 120;
const unsigned long LINE_CORNER_COOLDOWN = 500;
const unsigned long LINE_CORNER_PAUSE = 120;
const unsigned long LINE_CORNER_MAX = 3000;
const unsigned long LINE_STOP_TIME = 150;
const unsigned long LINE_SIDE_MEMORY = 1000;
const unsigned long LINE_LOST_TIMEOUT = 1500;
const unsigned long LINE_FINISH_WINDOW = 200;

enum LinePhase { LINE_FOLLOW, LINE_CORNER_BRAKE, LINE_CORNER_TURN, LINE_CORNER_SETTLE, LINE_GIVEN_UP, LINE_FINISHED };
LinePhase linePhase = LINE_FOLLOW;
unsigned long linePhaseSince = 0;
float lineCornerDir = 0.0f;
bool lineCornerSeenDark = false;
unsigned long lineLostSince = 0, lineDarkSince = 0, lineLastTime = 0, lineSideTime = 0, lineCornerDone = 0;
unsigned long lineBrightTime[2] = { 0, 0 };
unsigned long lineNotDarkTime[2] = { 0, 0 };
bool lineAtFinish = false;
float lineLastError = 0.0f;

void setLinePhase(int phase) {
  linePhase = (LinePhase)phase;
  linePhaseSince = millis();
}

void clearLineMemory() {
  lineLostSince = lineDarkSince = lineLastTime = lineSideTime = 0;
  lineBrightTime[0] = lineBrightTime[1] = 0;
  lineLastError = 0.0f;
}

void resetLineFollower() {
  clearLineMemory();
  lineCornerDone = 0;
  setLinePhase(LINE_FOLLOW);
}

void lineDebug(const char* zustand, float dL, float dM, float dR, float error) {
  Serial.printf("Linie: %-11s L=%d M=%d R=%d   roh %4d/%4d/%4d  e=%+.2f\n",
                zustand, dL < LINE_DARK, dM < LINE_DARK, dR < LINE_DARK,
                lightRaw[0], lightRaw[1], lightRaw[2], error);
}

void pivotPulse(unsigned long elapsed, float dir) {
  if (elapsed % (LINE_PIVOT_ON + LINE_PIVOT_OFF) < LINE_PIVOT_ON) drive(0, 0, dir * LINE_PIVOT_TURN, MaxSpeed);
  else stopMotors();
}

bool handleCorner(float dL, float dM, float dR) {
  if (linePhase == LINE_FOLLOW) return false;

  unsigned long elapsed = millis() - linePhaseSince;
  const char* name = lineCornerDir > 0 ? "rechtsecke" : "linksecke";
  float dSide = lineCornerDir > 0 ? dR : dL;
  float dOther = lineCornerDir > 0 ? dL : dR;

  if (linePhase == LINE_CORNER_BRAKE) {
    stopMotors();
    if (lineAtFinish) {
      setLinePhase(LINE_FINISHED);
    } else if (elapsed >= LINE_CORNER_PAUSE) {
      lineCornerSeenDark = dSide >= LINE_DARK;
      setLinePhase(LINE_CORNER_TURN);
    }
  } else if (linePhase == LINE_CORNER_TURN) {
    if (dSide >= LINE_DARK) lineCornerSeenDark = true;
    bool done = lineCornerSeenDark && dSide <= LINE_CORNER_CLEAR;
    bool overshot = dOther >= LINE_DARK && dSide < LINE_DARK;
    if (done || overshot) {
      stopMotors();
      setLinePhase(LINE_CORNER_SETTLE);
    } else if (elapsed > LINE_CORNER_MAX) {
      stopMotors();
      setLinePhase(LINE_GIVEN_UP);
    } else {
      pivotPulse(elapsed, lineCornerDir);
    }
  } else if (linePhase == LINE_CORNER_SETTLE) {
    stopMotors();
    if (elapsed >= LINE_CORNER_PAUSE) {
      clearLineMemory();
      lineCornerDone = millis();
      setLinePhase(LINE_FOLLOW);
    }
  } else {
    stopMotors();
    name = linePhase == LINE_FINISHED ? "ziel" : "verloren";
  }

  lineDebug(name, dL, dM, dR, lineCornerDir);
  return true;
}

void followLine() {
  float dL = readDarkness(0), dM = readDarkness(1), dR = readDarkness(2);
  unsigned long now = millis();
  if (dL < LINE_CORNER_DARK) lineNotDarkTime[0] = now;
  if (dR < LINE_CORNER_DARK) lineNotDarkTime[1] = now;
  lineAtFinish = dL >= LINE_CORNER_DARK && dR >= LINE_CORNER_DARK
              && now - lineNotDarkTime[0] <= LINE_FINISH_WINDOW && now - lineNotDarkTime[1] <= LINE_FINISH_WINDOW;

  if (handleCorner(dL, dM, dR)) return;
  bool sideRecent = lineSideTime != 0 && now - lineSideTime < LINE_SIDE_MEMORY;

  if (min(dL, min(dM, dR)) >= LINE_STOP_DARK && !sideRecent) {
    if (lineDarkSince == 0) lineDarkSince = now;
    if (now - lineDarkSince > LINE_STOP_TIME) {
      stopMotors();
      lineDebug("stopp", dL, dM, dR, 0.0f);
      return;
    }
  } else {
    lineDarkSince = 0;
  }

  if (dL <= LINE_CORNER_CLEAR) lineBrightTime[0] = now;
  if (dR <= LINE_CORNER_CLEAR) lineBrightTime[1] = now;
  bool jumpL = dL >= LINE_CORNER_DARK && lineBrightTime[0] != 0 && now - lineBrightTime[0] <= LINE_CORNER_RISE;
  bool jumpR = dR >= LINE_CORNER_DARK && lineBrightTime[1] != 0 && now - lineBrightTime[1] <= LINE_CORNER_RISE;
  bool cooldown = lineCornerDone != 0 && now - lineCornerDone < LINE_CORNER_COOLDOWN;

  if (lineAtFinish && !cooldown) {
    stopMotors();
    setLinePhase(LINE_FINISHED);
    lineDebug("ziel", dL, dM, dR, 0.0f);
    return;
  }

  float corner = 0.0f;
  if (!cooldown) {
    if (jumpR) corner = 1.0f;
    else if (jumpL) corner = -1.0f;
  }
  if (corner != 0.0f) {
    lineCornerDir = corner;
    stopMotors();
    setLinePhase(LINE_CORNER_BRAKE);
    lineDebug(corner > 0 ? "rechtsecke" : "linksecke", dL, dM, dR, corner);
    return;
  }

  if (max(dL, max(dM, dR)) < LINE_DARK) {
    if (lineLostSince == 0) lineLostSince = now;
    if (now - lineLostSince > LINE_LOST_TIMEOUT) {
      stopMotors();
      lineDebug("verloren", dL, dM, dR, lineLastError);
      return;
    }
    if (fabsf(lineLastError) <= 0.05f) drive(LINE_SEARCH_SPEED, 0, 0, MaxSpeed);
    else pivotPulse(now - lineLostSince, lineLastError > 0 ? 1.0f : -1.0f);
    lineDebug("suchen", dL, dM, dR, lineLastError);
    return;
  }
  lineLostSince = 0;

  float error = dR - dL;
  float rate = 0.0f;
  if (lineLastTime != 0 && now > lineLastTime) {
    rate = (error - lineLastError) * 1000.0f / (float)(now - lineLastTime);
  }
  lineLastTime = now;
  lineLastError = error;
  if (fabsf(error) >= LINE_SIDE_ERROR) {
    lineSideTime = now;
  }

  float vx = LINE_SPEED * (1.0f - LINE_CURVE_SLOW * fabsf(error));
  float omega = LINE_TURN_GAIN * error + LINE_TURN_DAMP * rate;
  drive(vx, 0, omega, MaxSpeed);
  lineDebug(error > 0.1f ? "rechts" : (error < -0.1f ? "links" : "geradeaus"), dL, dM, dR, error);
}

const unsigned long KEEPALIVE_TIMEOUT = 1000;
const unsigned long RUN_MAX = 30000;
const unsigned long FIXED_MAX = 3000;
bool running = false, fixedRun = false;
float fixedVx = 0.0f;
unsigned long runStart = 0, lastKeepalive = 0;

void endRun(const char* grund) {
  running = fixedRun = false;
  stopMotors();
  resetLineFollower();
  Serial.printf("# stopp: %s\n", grund);
}

void setParameter(char c) {
  char z[3] = { 0, 0, 0 };
  if (Serial.readBytes(z, 2) == 2) {
    int n = atoi(z);
    switch (c) {
      case 'V': LINE_SPEED = n / 100.0f; break;
      case 'C': LINE_CURVE_SLOW = n / 100.0f; break;
      case 'T': LINE_TURN_GAIN = n / 100.0f; break;
      case 'H': LINE_PIVOT_TURN = n / 100.0f; break;
      case 'E': LINE_CORNER_DARK = n / 100.0f; break;
      case 'D': LINE_TURN_DAMP = n / 1000.0f; break;
      case 'P': LINE_PIVOT_OFF = n * 10; break;
    }
  }
  Serial.printf("# speed %.2f  curveslow %.2f  gain %.2f  damp %.3f  pivot %.2f  ecke %.2f  drehpause %lu ms\n",
                LINE_SPEED, LINE_CURVE_SLOW, LINE_TURN_GAIN, LINE_TURN_DAMP, LINE_PIVOT_TURN,
                LINE_CORNER_DARK, LINE_PIVOT_OFF);
}

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i][0], OUTPUT);
    pinMode(MOTOR_PINS[i][1], OUTPUT);
  }
  for (int i = 0; i < 3; i++) pinMode(lightPins[i], INPUT);
  stopMotors();
  digitalWrite(2, HIGH);
  Serial.println("# Linienfahrt-Test bereit (g = start, k = Lebenszeichen, p = Rohwerte)");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'k') {
      lastKeepalive = millis();
    } else if (c == 'g') {
      resetLineFollower();
      running = true;
      runStart = lastKeepalive = millis();
      Serial.println("# start");
    } else if (c == 'p') {
      float dL = readDarkness(0), dM = readDarkness(1), dR = readDarkness(2);
      Serial.printf("# stand: dunkel %.2f/%.2f/%.2f   roh %4d/%4d/%4d\n",
                    dL, dM, dR, lightRaw[0], lightRaw[1], lightRaw[2]);
    } else if (strchr("VCTHDEP", c)) {
      setParameter(c);
    } else if (c >= '0' && c <= '9') {
      running = false;
      fixedRun = true;
      fixedVx = (c == '0') ? 1.0f : (c - '0') / 10.0f;
      runStart = lastKeepalive = millis();
      Serial.printf("# geradeaus %.1f\n", fixedVx);
    } else if (c != '\n' && c != '\r') {
      if (running || fixedRun) endRun("Kommando");
    }
  }

  if (running || fixedRun) {
    if (millis() - lastKeepalive > KEEPALIVE_TIMEOUT) {
      endRun("kein Lebenszeichen");
    } else if (millis() - runStart > (running ? RUN_MAX : FIXED_MAX)) {
      endRun("Zeitlimit");
    } else if (running) {
      Serial.printf("%lu ", millis() - runStart);
      followLine();
    } else {
      drive(fixedVx, 0, 0, MaxSpeed);
      for (int i = 0; i < 3; i++) readDarkness(i);
      Serial.printf("F %d %d %d\n", lightRaw[0], lightRaw[1], lightRaw[2]);
    }
  }

  delay(20);
}
