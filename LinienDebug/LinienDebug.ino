// ==========================================================
// Debug-Sketch fuer die Linienverfolgung (nur USB-Serial, keine Motoren,
// kein Bluetooth). Zeigt die ganze Kette, die der Linienfolger durchlaeuft:
//
//   Rohwert -> Schwelle -> hell/dunkel -> Zustand -> vx/omega -> PWM
//
// Teller ueber die Linie halten oder schieben und mitlesen. Die Logik ist
// identisch zu Teller.ino; aendert sich dort eine Konstante, hier mitziehen.
//
// Kommandos (115200):
//   p   Kalibriertabelle einmalig ausgeben
//   l   Dauerausgabe an/aus
//   h   diese Hilfe
// ==========================================================
#include <math.h>

// ---------- Lichtsensoren (wie Teller.ino) ----------
const int lichtL = 35;
const int lichtM = 34;
const int lichtR = 39;
const int lightPins[3] = { lichtL, lichtM, lichtR };
const char* lightName[3] = { "L", "M", "R" };

// Feste Kalibrierung aus Kalibrierung/messwerte.csv, siehe Teller.ino.
const int LIGHT_WHITE[3] = { 467, 1135, 680 };
const int LIGHT_BLACK[3] = { 207,  307, 167 };
const int lightThreshold[3] = {
  (LIGHT_WHITE[0] + LIGHT_BLACK[0]) / 2, // L: 337
  (LIGHT_WHITE[1] + LIGHT_BLACK[1]) / 2, // M: 721
  (LIGHT_WHITE[2] + LIGHT_BLACK[2]) / 2, // R: 423
};

// ---------- Motorkalibrierung (wie Teller.ino) ----------
const int MaxSpeed = 255;
int motorMinPWM[4] = { 220, 198, 207, 185 };
int motorMaxPWM[4] = { 255, 240, 245, 235 };
const int motorInvert[4] = { -1, 1, -1, -1 };

// ---------- Linienfolger-Konstanten (wie Teller.ino) ----------
const float LINE_SPEED        = 0.45f;
const float LINE_CORNER_SPEED = 0.15f;
const float LINE_TURN_SOFT    = 0.25f;
const float LINE_TURN_HARD    = 0.55f;

const unsigned long PRINT_INTERVAL = 200; // ms
bool streaming = true;
unsigned long lastPrint = 0;
int printedRows = 0;

struct Reading {
  int  raw;
  bool hell;
};

Reading readLight(int index) {
  Reading r;
  r.raw = analogRead(lightPins[index]);
  r.hell = r.raw > lightThreshold[index];
  return r;
}

// Entscheidungsbaum aus followLine(), aber ohne zu fahren.
const char* lineState(bool lL, bool lM, bool lR, float* vx, float* omega) {
  *vx = LINE_SPEED;
  *omega = 0.0f;

  if (!lL && !lM && !lR) { *vx = 0.0f; return "stopp"; }
  if (lL && lM && lR)    { *vx = LINE_CORNER_SPEED; return "verloren"; }

  if (!lM && lL && lR)  return "geradeaus";
  if (!lR && lM && lL)  { *omega =  LINE_TURN_SOFT; return "rechtskurve"; }
  if (!lL && lM && lR)  { *omega = -LINE_TURN_SOFT; return "linkskurve"; }
  if (!lM && !lR && lL) { *vx = LINE_CORNER_SPEED; *omega =  LINE_TURN_HARD; return "rechtsecke"; }
  if (!lM && !lL && lR) { *vx = LINE_CORNER_SPEED; *omega = -LINE_TURN_HARD; return "linksecke"; }
  return "kreuzung";
}

// Welcher PWM-Wert landet bei dieser Vorgabe am Motor? (Mischmatrix aus drive())
int pwmFor(int index, float vx, float vy, float omega) {
  float v[4];
  v[0] = -vx - vy - omega;
  v[1] =  vx - vy - omega;
  v[2] = -vx + vy - omega;
  v[3] =  vx + vy - omega;

  float maxMag = 0.0f;
  for (int i = 0; i < 4; i++) if (fabs(v[i]) > maxMag) maxMag = fabs(v[i]);
  if (maxMag > 1.0f) for (int i = 0; i < 4; i++) v[i] /= maxMag;

  float magnitude = fabs(v[index] * motorInvert[index]);
  if (magnitude < 0.001f) return 0;
  int minPWM = motorMinPWM[index];
  int maxPWM = min(motorMaxPWM[index], MaxSpeed);
  return constrain(minPWM + (int)(magnitude * (maxPWM - minPWM)), 0, 255);
}

void printCalibration() {
  Serial.println();
  Serial.println("Sensor   roh  schwarz<=  schwelle  weiss>=");
  for (int i = 0; i < 3; i++) {
    Reading r = readLight(i);
    Serial.printf("  %s    %5d      %5d     %5d    %5d\n",
                  lightName[i], r.raw, LIGHT_BLACK[i], lightThreshold[i], LIGHT_WHITE[i]);
  }
  Serial.println();
  printedRows = 0;
}

void printHeader() {
  Serial.println();
  Serial.println("   L     M     R  | hell  |  Zustand     |   vx   omega |  PWM M1  M2  M3  M4");
  Serial.println("-----------------------------------------------------------------------------");
}

void printRow() {
  Reading rL = readLight(0);
  Reading rM = readLight(1);
  Reading rR = readLight(2);

  float vx, omega;
  const char* state = lineState(rL.hell, rM.hell, rR.hell, &vx, &omega);

  if (printedRows % 15 == 0) printHeader();
  printedRows++;

  Serial.printf("%5d %5d %5d  | %c %c %c | %-12s | %4.2f  %+5.2f |      %3d %3d %3d %3d\n",
                rL.raw, rM.raw, rR.raw,
                rL.hell ? 'h' : 'd', rM.hell ? 'h' : 'd', rR.hell ? 'h' : 'd',
                state, vx, omega,
                pwmFor(0, vx, 0, omega), pwmFor(1, vx, 0, omega),
                pwmFor(2, vx, 0, omega), pwmFor(3, vx, 0, omega));
}

void printHelp() {
  Serial.println();
  Serial.println("p = Kalibriertabelle | l = Dauerausgabe an/aus | h = Hilfe");
  Serial.println("h = hell (keine Linie), d = dunkel (Linie unter dem Sensor)");
  printedRows = 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(lichtL, INPUT);
  pinMode(lichtM, INPUT);
  pinMode(lichtR, INPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  Serial.println();
  Serial.println("=== Linien-Debug (keine Motoren, kein Bluetooth) ===");
  printCalibration();
  printHelp();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'p': printCalibration(); break;
      case 'l':
        streaming = !streaming;
        Serial.printf("\nDauerausgabe %s\n", streaming ? "an" : "aus");
        printedRows = 0;
        break;
      case '\n': case '\r': break;
      default: printHelp(); break;
    }
  }

  if (streaming && millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();
    printRow();
  }
}
