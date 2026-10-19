// ==========================================================
// Geradeaus-Test zum Abgleich der Motoren (nur USB-Serial, kein Bluetooth).
// Der Teller faehrt ungeregelt geradeaus ueber eine gerade schwarze Linie und
// gibt alle 5 ms die Rohwerte der Lichtsensoren aus. fahrt.py startet die
// Fahrt und wertet aus, wie die Linie unter den Sensoren zur Seite wandert.
//
// Kommando (eine Zeile):  g <vx 0-100> <dauer ms> <max1> <max2> <max3> <max4>
// Die Fahrt endet nach <dauer>, am Linienende (alle Sensoren weiss) oder
// sobald irgendein Zeichen ueber USB kommt.
//
// Motoren und Sensoren wie in Teller.ino, damit die Werte uebertragbar sind.
// ==========================================================
#include <math.h>

const int MOTOR_PINS[4][2] = { {21, 19}, {5, 17}, {25, 26}, {27, 14} };
const int motorMinPWM[4] = { 220, 198, 207, 185 };
const int motorInvert[4] = { -1, 1, -1, -1 };
int motorMaxPWM[4] = { 255, 255, 255, 255 };

const int lightPins[3] = { 35, 34, 39 };
const int LIGHT_WHITE[3] = { 466, 653, 225 };
const int LIGHT_BLACK[3] = {  85,  53,   8 };

const unsigned long SAMPLE_INTERVAL = 5;   // ms
const unsigned long WHITE_STOP_TIME = 150; // so lange alles weiss -> Linie zu Ende
const unsigned long MAX_DURATION = 4000;

void setMotor(int i, float value) {
  value = constrain(value, -1.0f, 1.0f) * motorInvert[i];
  int a = MOTOR_PINS[i][0], b = MOTOR_PINS[i][1];
  if (fabsf(value) < 0.001f) {
    analogWrite(a, 255);
    analogWrite(b, 255);
    return;
  }
  int pwm = motorMinPWM[i] + (int)(fabsf(value) * (motorMaxPWM[i] - motorMinPWM[i]));
  pwm = constrain(pwm, 0, 255);
  analogWrite(a, value > 0 ? pwm : 0);
  analogWrite(b, value > 0 ? 0 : pwm);
}

// Nur vorwaerts: gleiche Vorzeichen wie drive(vx, 0, 0) in Teller.ino.
void driveForward(float vx) {
  setMotor(0, -vx);
  setMotor(1, vx);
  setMotor(2, -vx);
  setMotor(3, vx);
}

bool isDark(int i, int raw) {
  return raw < (LIGHT_WHITE[i] + LIGHT_BLACK[i]) / 2;
}

void run(float vx, unsigned long duration) {
  Serial.printf("# start vx=%.2f dauer=%lu max=%d/%d/%d/%d\n", vx, duration,
                motorMaxPWM[0], motorMaxPWM[1], motorMaxPWM[2], motorMaxPWM[3]);
  unsigned long start = millis(), lastSample = 0, lastDark = start;
  const char *grund = "zeit";
  driveForward(vx);
  while (millis() - start < duration) {
    if (Serial.available()) { grund = "abbruch"; break; }
    unsigned long now = millis();
    if (now - lastSample < SAMPLE_INTERVAL) continue;
    lastSample = now;
    int raw[3];
    bool dark = false;
    for (int i = 0; i < 3; i++) {
      raw[i] = analogRead(lightPins[i]);
      dark = dark || isDark(i, raw[i]);
    }
    Serial.printf("%lu %d %d %d\n", now - start, raw[0], raw[1], raw[2]);
    if (dark) lastDark = now;
    if (now - lastDark > WHITE_STOP_TIME) { grund = "weiss"; break; }
  }
  driveForward(0);
  while (Serial.available()) Serial.read();
  Serial.printf("# ende %s\n", grund);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i][0], OUTPUT);
    pinMode(MOTOR_PINS[i][1], OUTPUT);
  }
  driveForward(0);
  for (int i = 0; i < 3; i++) pinMode(lightPins[i], INPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.println("# Geradeaus-Test bereit");
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  int vx, m[4];
  unsigned long duration;
  if (sscanf(line.c_str(), "g %d %lu %d %d %d %d", &vx, &duration, &m[0], &m[1], &m[2], &m[3]) != 6) {
    if (line.startsWith("r")) { // nur Sensoren lesen, nicht fahren
      Serial.printf("# roh %d %d %d\n", analogRead(lightPins[0]), analogRead(lightPins[1]), analogRead(lightPins[2]));
    }
    return;
  }
  for (int i = 0; i < 4; i++) motorMaxPWM[i] = constrain(m[i], motorMinPWM[i], 255);
  run(constrain(vx, 0, 100) / 100.0f, min(duration, MAX_DURATION));
}
