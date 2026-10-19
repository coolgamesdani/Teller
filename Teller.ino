#include <BluetoothSerial.h>
#include <math.h>
#include <LittleFS.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"

BluetoothSerial SerialBT;
uint8_t masterAddress[6] = {0x70, 0x4B, 0xCA, 0x22, 0x10, 0xF2};  //Controler
//uint8_t masterAddress[6] = {0x20, 0x9B, 0xA9, 0x73, 0x9B, 0x42};    //Teller

enum MessageType : uint8_t {
  MODE_PIN = 1,
  MODE_TEXT   = 2,
  MODE_JOYSTICK = 3,
  MODE_DRIVEMODE = 4,
  MODE_SCAN = 5,      // Teller -> Controller: ein Stueck Lidar-Rundumbild
  MODE_LIFT = 6,      // Controller -> Teller: Hebebuehne hoch/runter/stopp,
                      // Teller -> Controller: Zustand des Hebemotors
  MODE_BATTERY = 7    // Teller -> Controller: Akkuspannung in mV (uint16)
};

// Fahrmodus: manuell per Joystick oder autonom der Linie folgen
enum DriveMode : uint8_t {
  DRIVE_MANUAL = 0,
  DRIVE_LINE   = 1
};

DriveMode driveMode = DRIVE_MANUAL;

// Motoren
int M1 = 21;
int m1 = 19;
int M2 = 5;
int m2 = 17;
int M3 = 25;
int m3 = 26;
int M4 = 27;
int m4 = 14;
// ==================== MOTORKALIBRIERUNG ====================
const int MaxSpeed = 255;
// Linienfolger: abgestimmt mit diesen Anfahrschwellen (am Boden unter Last
// gemessen, Motortest Stufe 0.80) und 255 auf allen Motoren - nicht aendern,
// ohne den Linienfolger neu abzustimmen.
const int lineMinPWM[4] = {
  220,  // M1
  198,  // M2
  207,  // M3
  185   // M4
};
// Manuelles Fahren: abgeglichen mit dem Lidar-Fahrtest (Fahrtest/, 14.10.2026).
// Mit den Anfahrschwellen oben liefen die rechten Motoren bei Teilgas deutlich
// langsamer, der Teller drehte sich vorwaerts/rueckwaerts um 50-75 Grad ein
// (aufnahme2); bei Vollgas waren M2/M3 rund 40 % staerker als M1/M4.
// 1. Schritt (aufnahme3): Eindrehen noch 17/34 Grad, Vollgas ausgeglichen.
// 2. Schritt: aus beiden Aufnahmen pro Motor die Wirkung einer PWM-Aenderung
// geschaetzt und die Werte so gesetzt, dass alle vier bei 60 % und bei Vollgas
// gleich schnell laufen (linear dazwischen).
int motorMinPWM[4] = {
  214,  // M1 (vorne links)
  219,  // M2 (vorne rechts)
  212,  // M3 (hinten links)
  214   // M4 (hinten rechts)
};
int motorMaxPWM[4] = {
  255,  // M1 (vorne links)
  235,  // M2 (vorne rechts)
  239,  // M3 (hinten links)
  245   // M4 (hinten rechts)
};
// Anfahrschwelle, wenn das Rad rueckwaerts rollt. Mit den Werten oben drehte
// der Teller vorwaerts kaum noch (+4 Grad), rueckwaerts aber in allen drei
// Testrunden nach links (+74/+34/+40 Grad): rueckwaerts schieben die linken
// Raeder staerker. Darum hier links weniger, rechts mehr (je ~9 Stufen, aus der
// Wirkung frueherer Aenderungen geschaetzt: ~2,4 Grad pro PWM-Stufe Unterschied).
int motorMinPWMRev[4] = {
  205,  // M1 (vorne links)
  228,  // M2 (vorne rechts)
  203,  // M3 (hinten links)
  223   // M4 (hinten rechts)
};
// Vorzeichen des Werts aus drive(), bei dem das Rad vorwaerts rollt
// (alle Raeder rollen vorwaerts bei vx > 0).
const int wheelForward[4] = { -1, 1, -1, 1 };
// Einbaurichtung: -1 fuer Motoren, die anders herum angeschlossen sind als
// drive() erwartet. Im Motortest ermittelt: bei vorwaerts lief nur M2
// (vorne rechts) richtig, M1, M3 und M4 liefen rueckwaerts.
const int motorInvert[4] = {
  -1,   // M1
  -1,   // M2 (am 15.10. umgepolt)
  -1,   // M3
  -1    // M4
};

// ==================== Fahren ================================
void stopMotors(){
  drive(0, 0, 0, MaxSpeed);
}

void setMotor(int pin1, int pin2, float value,
              int motorIndex, int maxSpeed) {
  bool rollsForward = value * wheelForward[motorIndex] > 0;
  value = constrain(value, -1.0f, 1.0f) * motorInvert[motorIndex];
  if (fabs(value) < 0.001f) {
    analogWrite(pin1, 255);
    analogWrite(pin2, 255);
    return;
  }
  float magnitude = fabs(value);
  int minPWM = driveMode == DRIVE_LINE ? lineMinPWM[motorIndex]
             : rollsForward ? motorMinPWM[motorIndex] : motorMinPWMRev[motorIndex];
  // Der Linienfolger ist mit voller Leistung auf allen Motoren abgestimmt.
  int maxPWM = min(driveMode == DRIVE_LINE ? 255 : motorMaxPWM[motorIndex], maxSpeed);
  int pwm = minPWM + (int)(magnitude * (maxPWM - minPWM));
  pwm = constrain(pwm, 0, 255);
  if (value > 0) {
    analogWrite(pin1, pwm);
    analogWrite(pin2, 0);
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, pwm);
  }
}

float lastVx = 0, lastVy = 0, lastOmega = 0; // fuer den Fahrtenschreiber

void drive(float vx, float vy, float omega, int speed) {
  lastVx = vx; lastVy = vy; lastOmega = omega;
  float v1 = -vx - vy - omega;
  float v2 =  vx - vy - omega;
  float v3 = -vx + vy - omega;
  float v4 =  vx + vy - omega;

  float maxMag = fabs(v1);
  if (fabs(v2) > maxMag) maxMag = fabs(v2);
  if (fabs(v3) > maxMag) maxMag = fabs(v3);
  if (fabs(v4) > maxMag) maxMag = fabs(v4);
  if (maxMag > 1.0f) {
    v1 /= maxMag; v2 /= maxMag; v3 /= maxMag; v4 /= maxMag;
  }

  setMotor(M1, m1, v1, 0, speed);
  setMotor(M2, m2, v2, 1, speed);
  setMotor(M3, m3, v3, 2, speed);
  setMotor(M4, m4, v4, 3, speed);
}

// ==================== LICHTSENSOREN ====================
// Dunkle Linie auf hellem Untergrund: kleiner Messwert = Linie, grosser Wert = hell.
const int lichtL = 35;
const int lichtM = 34;
const int lichtR = 39;
const int lightPins[3] = { lichtL, lichtM, lichtR };

// Feste Kalibrierung, gemessen mit Kalibrierung/Kalibrierung.ino (messwerte.csv),
// zuletzt am 14.10.2026 nach dem Einbau der neuen LEDs.
// Mittelwerte der Rohwerte von analogRead():
//
//          weiss              schwarz
//          Licht an / aus     Licht an / aus
//     L     671 /  503          68 /   0
//     M     825 /  698          38 /   0
//     R     396 /  238           1 /   0
//
// LIGHT_WHITE = dunkelstes gemessenes Weiss, LIGHT_BLACK = hellstes gemessenes
// Schwarz, jeweils aus dem unguenstigeren der beiden Lichtverhaeltnisse.
// Dazwischen wird linear gerechnet, ausserhalb abgeschnitten - das passt so mit
// und ohne Umgebungslicht. Die Sensoren sind sehr unterschiedlich empfindlich,
// darum hat jeder seine eigenen Grenzen.
// Genommen ist das 1-%- bzw. 99-%-Quantil statt des kleinsten/groessten
// Einzelwerts: etwa alle 1-2 s sackt ein einzelner Messwert an allen drei
// Sensoren gleichzeitig um ~20 % ab (vermutlich die Versorgung), das soll die
// Grenzen nicht verschieben.
// Neu kalibriert am 16.10.2026 mit neuem rechtem Sensor: Weiss = 1-%-Quantil
// auf Weiss, Schwarz = 99-%-Quantil auf Schwarz. An D39 (R) kommen auf Schwarz
// noch vereinzelte Ausreisser (max. ~130), deutlich weniger als vorher.
// Auf der Strecke liefert der Boden deutlich weniger (Testfahrt 16.10.,
// LinienFahrt/fahrten/): L ~250-320, M ~430-470, R ~250-320. Weiss darum auf
// den dunkelsten Boden der Strecke gesetzt, sonst gilt der Boden als halb dunkel.
const int LIGHT_WHITE[3] = { 240, 420, 230 };
const int LIGHT_BLACK[3] = {   0,  32,  48 };
int lightRaw[3]; // letzter Rohwert, nur fuer die Statusausgabe

// Wie dunkel sieht der Sensor? 0.0 = weiss, 1.0 = schwarz, dazwischen linear.
// Ausserhalb der Kalibriergrenzen wird abgeschnitten, darum stimmt das mit und
// ohne Umgebungslicht.
float readDarkness(int index) {
  lightRaw[index] = analogRead(lightPins[index]);
  float d = (float)(LIGHT_WHITE[index] - lightRaw[index])
          / (float)(LIGHT_WHITE[index] - LIGHT_BLACK[index]);
  return constrain(d, 0.0f, 1.0f);
}

// ==================== LINIENFOLGER ====================
// Schwarze Linie auf hellem Boden, etwa so breit wie der Abstand zwischen L und R.
// Gelenkt wird proportional zu (dR - dL). An einer Ecke haelt der Teller an und
// dreht auf der Stelle, am Querbalken (Ziel) bleibt er stehen.
// Abgestimmt mit LinienFahrt/LinienFahrt.ino. omega > 0 dreht nach rechts.
float LINE_SPEED        = 0.65f; // Grundgeschwindigkeit auf der Linie (16.10.: von 0.80 gesenkt)
float LINE_TURN_GAIN    = 0.45f; // Drehrate, wenn die Linie ganz auf einer Seite liegt
float LINE_TURN_DAMP    = 0.03f; // Daempfung gegen Pendeln (s)
// Die Sensoren springen in einem Schritt von Weiss auf Schwarz; ungeglaettet
// wurde daraus ein voller Lenkausschlag hin und gleich wieder zurueck.
float LINE_RATE_SMOOTH  = 0.25f; // Anteil des neuen Werts pro Schritt
// Mecanum: zusaetzlich seitlich zur Linie hin rutschen. Das bringt die Sensoren
// zurueck, ohne den Teller zu verdrehen, und schwingt darum weniger.
float LINE_STRAFE_GAIN  = 0.15f; // 0.35 schaukelte sich in der S-Kurve auf
float LINE_CURVE_SLOW   = 0.40f; // Anteil, um den in der Kurve gebremst wird
float LINE_PIVOT_TURN   = 0.30f; // Drehrate beim Drehen auf der Stelle (16.10.: von 0.40 gesenkt)
const float LINE_SEARCH_SPEED = 0.10f;
const float LINE_DARK         = 0.5f;  // ab hier gilt ein Sensor als "auf der Linie"
const float LINE_CORNER_DARK  = 0.85f; // Aussensensor ganz auf Schwarz -> Ecke oder Ziel
const float LINE_CORNER_CLEAR = 0.30f; // Aussensensor wieder auf Weiss
const float LINE_STOP_DARK    = 0.9f;
const float LINE_SIDE_ERROR   = 0.3f;
// Eigene Ecken-Erkennung (anhalten und auf der Stelle drehen). Auf der Strecke
// (Gerade, S-Kurve, eine Rechtsecke, Gerade, T) hat sie in der S-Kurve ~5 Mal
// pro Durchgang faelschlich angesprochen. Aus: Die Kurve wird normal gelenkt,
// an der Ecke dreht die Suche zur Seite, auf der die Linie zuletzt war.
const bool LINE_USE_CORNERS = false;
// Linie verloren: zuerst zur zuletzt gesehenen Seite drehen, nach
// LINE_SEARCH_FLIP_MS zur anderen Seite schwenken. Schraeges Ueberfahren und
// die Rechtsecke sehen fuer die drei Sensoren zu aehnlich aus, um die Seite
// sicher zu erraten (Testfahrten 16.10.) - so wird beides gefunden.
const unsigned long LINE_SEARCH_FLIP_MS = 350;
// Beim Drehen sucht er, bis die Linie wieder da ist (hoechstens etwa eine
// ganze Umdrehung). Die 1-s-Grenze gilt nur, wenn er geradeaus sucht.
const unsigned long LINE_SEARCH_MAX_MS = 3500;
float lineSearchDir = 0.0f;

const unsigned long LINE_PIVOT_ON        = 40;   // Drehen in kurzen Schueben: ms an ...
const unsigned long LINE_PIVOT_OFF       = 120;  // ... und ms Pause
const unsigned long LINE_CORNER_RISE     = 120;  // so schnell muss der Aussensensor schwarz werden
const unsigned long LINE_CORNER_COOLDOWN = 500;  // nach einer Ecke keine neue Ecke / kein Ziel
const unsigned long LINE_BRAKE_TIME      = 60;   // Bremsen vor dem Drehen (steht laut Messung nach ~80 ms)
const unsigned long LINE_SETTLE_TIME     = 40;   // kurzer Halt nach dem Drehen
const unsigned long LINE_CORNER_MAX      = 3000;
const unsigned long LINE_STOP_TIME       = 150;
const unsigned long LINE_SIDE_MEMORY     = 1000;
const unsigned long LINE_LOST_TIMEOUT    = 1000; // so lange nur Weiss -> von der Linie abgekommen, anhalten
const unsigned long LINE_FINISH_WINDOW   = 200;  // beide Aussensensoren fast gleichzeitig schwarz -> Ziel
const unsigned long LINE_DEBUG_INTERVAL  = 250;  // ms zwischen Statusausgaben

// Wende an jeder durchgezogenen Querlinie: anhalten, um 180 Grad nach rechts
// drehen, rueckwaerts fahren bis die Sensoren auf der Querlinie stehen,
// UTURN_WAIT_MS warten und dann der Linie zurueck folgen. Nach dem Warten wird
// der Zaehler zurueckgesetzt, am naechsten Querbalken parkt er also wieder ein.
// Den Drehwinkel misst der Lidar (lidarTrack..., siehe LIDAR). Gegen Ende der
// Drehung kommt der mittlere Sensor wieder ueber die Fahrlinie, auf der der
// Teller gekommen ist - das ist der genaue Anschlag.
// Gedreht wird in kurzen Schueben (an/aus), damit er langsam dreht und nicht
// ueberdreht - mit weniger Gas allein bleiben die Motoren stehen.
const float UTURN_TURN           = 0.35f;  // Drehrate waehrend eines Schubs
const unsigned long UTURN_PULSE_ON  = 50;  // ms an ...
const unsigned long UTURN_PULSE_OFF = 110; // ... und ms Pause
const float UTURN_DEG            = 180.0f;
const unsigned long UTURN_ALIGN_MAX_MS   = 1500; // Feinausrichtung nach dem Nachdrehen
// Erst ab 160 Grad: ca. 20 cm vor dem T ist eine Ecke, deren Linie der mittlere
// Sensor beim Drehen schon bei ~135 Grad ueberstreicht (Testfahrt 16.10.).
const float UTURN_LINE_FROM_DEG  = 145.0f; // 160 war zu spaet: mittig lag er schon bei 158
const float UTURN_LIDAR_MAX_DEG  = 240.0f; // Notbremse, falls er die Linie nicht findet // ab hier zaehlt der mittlere Sensor auf Schwarz als "fertig"
const unsigned long UTURN_BRAKE_MS        = 200;
const unsigned long UTURN_SETTLE_MS       = 250;
const unsigned long UTURN_NOLIDAR_MIN_MS  = 700;  // ohne Lidar: so lange mindestens drehen, dann nach Linie
const unsigned long UTURN_MAX_MS          = 15000;
const float PARK_SPEED           = 0.55f;  // rueckwaerts (0.25/0.40: einzelne Raeder blieben unter Last stehen)
const float PARK_STRAFE          = 0.15f;  // seitlich zur Fahrlinie hin korrigieren
const float PARK_TURN            = 0.15f;
const unsigned long PARK_MAX_MS           = 5000;
const unsigned long UTURN_WAIT_MS         = 3000;
const unsigned long LEAVE_MAX_MS          = 2000; // von der Querlinie herunterfahren
const float LEAVE_SPEED          = 0.40f;
// Kommt der Teller schraeg ans T, erreicht ein Aussensensor die Linie vor dem
// anderen: beide muessen innerhalb dieser Zeit Schwarz gesehen haben.
const unsigned long PARK_BOTH_WINDOW_MS   = 600;
unsigned long parkDarkTime[2] = { 0, 0 };

enum LinePhase { LINE_FOLLOW, LINE_CORNER_BRAKE, LINE_CORNER_TURN, LINE_CORNER_SETTLE, LINE_GIVEN_UP, LINE_FINISHED,
                 LINE_UTURN_BRAKE, LINE_UTURN, LINE_UTURN_SETTLE, LINE_PARK, LINE_PARK_WAIT, LINE_LEAVE,
                 LINE_UTURN_ALIGN };
float uturnAlignDir = -1.0f;
bool lineUturnDone = false;   // die erste Querlinie ist schon gewendet
float lineUturnStopDeg = 0;   // Lidar-Winkel beim Abschalten, fuer die Auswertung
LinePhase linePhase = LINE_FOLLOW;
unsigned long linePhaseSince = 0;
float lineCornerDir = 0.0f;
bool lineCornerSeenDark = false;
unsigned long lineLostSince = 0, lineDarkSince = 0, lineLastTime = 0, lineSideTime = 0, lineCornerDone = 0;
unsigned long lineBrightTime[2] = { 0, 0 };
unsigned long lineNotDarkTime[2] = { 0, 0 };
unsigned long lastLineDebug = 0;
bool lineAtFinish = false;
float lineLastError = 0.0f;
float lineRate = 0.0f;          // geglaettete Aenderung des Fehlers (1/s)

void setLinePhase(int phase) {
  linePhase = (LinePhase)phase;
  linePhaseSince = millis();
  if (linePhase == LINE_FINISHED || linePhase == LINE_GIVEN_UP) logSave();
}

void clearLineMemory() {
  lineLostSince = lineDarkSince = lineLastTime = lineSideTime = 0;
  lineBrightTime[0] = lineBrightTime[1] = 0;
  lineLastError = 0.0f;
  lineRate = 0.0f;
}

void resetLineFollower() {
  clearLineMemory();
  lineCornerDone = 0;
  lineUturnDone = false;
  setLinePhase(LINE_FOLLOW);
  logStart();
}

// Querlinie erreicht: beim ersten Mal wenden, danach ist es das Ziel.
void lineCrossingReached() {
  stopMotors();
  if (lineUturnDone) {
    setLinePhase(LINE_FINISHED);
  } else {
    lineUturnDone = true;
    setLinePhase(LINE_UTURN_BRAKE);
  }
}

// Wende, Einparken, Warten und Herunterfahren von der Querlinie.
const char* handleUturn(float dL, float dM, float dR) {
  unsigned long elapsed = millis() - linePhaseSince;
  if (linePhase == LINE_UTURN_BRAKE) {
    stopMotors();
    if (elapsed >= UTURN_BRAKE_MS) {
      lidarTrackStart();
      setLinePhase(LINE_UTURN);
    }
    return "wende-halt";
  }
  if (linePhase == LINE_UTURN) {
    float deg = fabsf(lidarTrackDeg());
    bool lidarOk = lidarTrackAlive();
    // Lidar allein stoppt nur noch als Notbremse weit hinter 180 Grad; fertig
    // ist die Wende, wenn er wieder mittig auf der schwarzen Linie steht.
    bool byLidar = lidarOk && deg >= UTURN_LIDAR_MAX_DEG;
    // Lichtsensoren: Drehung fertig, wenn der mittlere Sensor auf der Fahrlinie
    // liegt und beide aeusseren daneben auf Weiss - dann steht er mittig auf
    // der Linie. Liegt auch ein Aussensensor auf Schwarz, ist es eher die Ecke.
    bool centered = dM >= LINE_DARK && dL < LINE_DARK && dR < LINE_DARK;
    bool lineCue = centered &&
                   (lidarOk ? deg >= UTURN_LINE_FROM_DEG : elapsed >= UTURN_NOLIDAR_MIN_MS);
    if (byLidar || lineCue) {
      stopMotors();
      lineUturnStopDeg = deg;
      Serial.printf("Wende: aus nach %lu ms bei %.0f Grad (%s)\n", elapsed, deg,
                    lineCue ? "Linie" : "Lidar");
      setLinePhase(LINE_UTURN_SETTLE);
    } else if (elapsed > UTURN_MAX_MS) {
      stopMotors();
      setLinePhase(LINE_GIVEN_UP);
    } else if (elapsed % (UTURN_PULSE_ON + UTURN_PULSE_OFF) < UTURN_PULSE_ON) {
      drive(0, 0, UTURN_TURN, MaxSpeed);
    } else {
      stopMotors();
    }
    return "wenden";
  }
  if (linePhase == LINE_UTURN_SETTLE) {
    stopMotors();
    if (elapsed >= UTURN_SETTLE_MS) {
      Serial.printf("Wende: steht bei %.0f Grad\n", fabsf(lidarTrackDeg()));
      uturnAlignDir = -1.0f; // meist hat er nach rechts ueberdreht
      setLinePhase(LINE_UTURN_ALIGN);
    }
    return "wende-steht";
  }
  if (linePhase == LINE_UTURN_ALIGN) {
    // Er dreht nach dem Abschalten noch 10-20 Grad nach. In kleinen Schueben
    // zurueckdrehen, bis die Linie genau unter dem mittleren Sensor liegt.
    bool centered = dM >= LINE_DARK && dL < LINE_DARK && dR < LINE_DARK;
    if (centered || elapsed > UTURN_ALIGN_MAX_MS) {
      stopMotors();
      setLinePhase(LINE_PARK);
    } else {
      if (dL >= LINE_DARK && dR < LINE_DARK) uturnAlignDir = -1.0f;
      else if (dR >= LINE_DARK && dL < LINE_DARK) uturnAlignDir = 1.0f;
      pivotPulse(elapsed, uturnAlignDir);
    }
    return "ausrichten";
  }
  if (linePhase == LINE_PARK) {
    // Rueckwaerts, bis beide Aussensensoren auf der Querlinie sind. Die Sensoren
    // sitzen vorne, also am hinteren Ende der Fahrt: seitlich schieben und
    // leicht drehen bringt sie zur Fahrlinie zurueck.
    unsigned long now = millis();
    if (elapsed < 30) parkDarkTime[0] = parkDarkTime[1] = 0; // neue Einparkfahrt
    if (dL >= LINE_CORNER_DARK) parkDarkTime[0] = now;
    if (dR >= LINE_CORNER_DARK) parkDarkTime[1] = now;
    bool bothSeen = parkDarkTime[0] && parkDarkTime[1]
                 && now - parkDarkTime[0] <= PARK_BOTH_WINDOW_MS && now - parkDarkTime[1] <= PARK_BOTH_WINDOW_MS;
    if (bothSeen) {
      stopMotors();
      setLinePhase(LINE_PARK_WAIT);
    } else if (elapsed > PARK_MAX_MS) {
      stopMotors();               // T nicht gefunden: trotzdem warten und weiterfahren
      Serial.println("Einparken: kein Querbalken gefunden");
      setLinePhase(LINE_PARK_WAIT);
    } else {
      float error = dR - dL;
      drive(-PARK_SPEED, PARK_STRAFE * error, PARK_TURN * error, MaxSpeed);
    }
    return "einparken";
  }
  if (linePhase == LINE_PARK_WAIT) {
    stopMotors();
    if (elapsed >= UTURN_WAIT_MS) {
      lineUturnDone = false; // naechster Querbalken: wieder wenden und einparken
      setLinePhase(LINE_LEAVE);
    }
    return "warten";
  }
  // LINE_LEAVE: geradeaus ueber die Querlinie, bis nur der mittlere Sensor auf
  // der Fahrlinie liegt, dann normal folgen. Er rollt beim Einparken etwas
  // ueber das T hinaus, steht also anfangs dahinter auf Weiss.
  bool onStem = dM >= LINE_DARK && dL < LINE_CORNER_CLEAR && dR < LINE_CORNER_CLEAR;
  if (onStem || elapsed > LEAVE_MAX_MS) {
    clearLineMemory();
    lineCornerDone = millis(); // Abklingzeit: keine Ecke direkt danach
    setLinePhase(LINE_FOLLOW);
  } else {
    drive(LEAVE_SPEED, 0, 0, MaxSpeed);
  }
  return "losfahren";
}

void lineDebug(const char* zustand, float dL, float dM, float dR, float error) {
  if (millis() - lastLineDebug < LINE_DEBUG_INTERVAL) return;
  lastLineDebug = millis();
  Serial.printf("Linie: %-11s L=%d M=%d R=%d   roh %4d/%4d/%4d  e=%+.2f\n",
                zustand, dL < LINE_DARK, dM < LINE_DARK, dR < LINE_DARK,
                lightRaw[0], lightRaw[1], lightRaw[2], error);
}

void pivotPulse(unsigned long elapsed, float dir) {
  if (elapsed % (LINE_PIVOT_ON + LINE_PIVOT_OFF) < LINE_PIVOT_ON) drive(0, 0, dir * LINE_PIVOT_TURN, MaxSpeed);
  else stopMotors();
}

// Ecke, Ziel und "verloren": liefert true, solange nicht normal gefolgt wird.
bool handleCorner(float dL, float dM, float dR) {
  if (linePhase == LINE_FOLLOW) return false;
  if (linePhase >= LINE_UTURN_BRAKE) {
    lineDebug(handleUturn(dL, dM, dR), dL, dM, dR, lidarTrackDeg());
    return true;
  }

  unsigned long elapsed = millis() - linePhaseSince;
  const char* name = lineCornerDir > 0 ? "rechtsecke" : "linksecke";
  float dSide = lineCornerDir > 0 ? dR : dL;
  float dOther = lineCornerDir > 0 ? dL : dR;

  if (linePhase == LINE_CORNER_BRAKE) {
    stopMotors();
    if (lineAtFinish) {
      lineCrossingReached();
    } else if (elapsed >= LINE_BRAKE_TIME) {
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
    if (elapsed >= LINE_SETTLE_TIME) {
      clearLineMemory();
      lineCornerDone = millis();
      setLinePhase(LINE_FOLLOW);
    }
  } else { // Ziel erreicht oder aufgegeben: stehen bleiben, bis der Modus neu gestartet wird
    stopMotors();
    name = linePhase == LINE_FINISHED ? "ziel" : "verloren";
  }

  lineDebug(name, dL, dM, dR, lineCornerDir);
  return true;
}

void followLine() {
  float dL = readDarkness(0), dM = readDarkness(1), dR = readDarkness(2);
  unsigned long now = millis();
  logRecord();
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
    lineCrossingReached();
    lineDebug(linePhase == LINE_FINISHED ? "ziel" : "querlinie", dL, dM, dR, 0.0f);
    return;
  }

  float corner = 0.0f;
  if (!cooldown && LINE_USE_CORNERS) {
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

  if (max(dL, max(dM, dR)) < LINE_DARK) { // alles hell -> Linie verloren
    if (lineLostSince == 0) {
      lineLostSince = now;
      lineSearchDir = fabsf(lineLastError) <= 0.05f ? 0.0f : (lineLastError > 0 ? 1.0f : -1.0f);
    }
    unsigned long limit = lineSearchDir == 0.0f ? LINE_LOST_TIMEOUT : LINE_SEARCH_MAX_MS;
    if (now - lineLostSince > limit) {
      stopMotors();
      setLinePhase(LINE_GIVEN_UP); // bleibt stehen, bis der Modus neu gestartet wird
      lineDebug("verloren", dL, dM, dR, lineLastError);
      return;
    }
    unsigned long lost = now - lineLostSince;
    if (lineSearchDir == 0.0f) drive(LINE_SEARCH_SPEED, 0, 0, MaxSpeed);
    else pivotPulse(lost, lost < LINE_SEARCH_FLIP_MS ? lineSearchDir : -lineSearchDir);
    lineDebug("suchen", dL, dM, dR, lineLastError);
    return;
  }
  lineLostSince = 0;

  float error = dR - dL; // > 0: Linie liegt rechts, < 0: links
  if (lineLastTime != 0 && now > lineLastTime) {
    float raw = (error - lineLastError) * 1000.0f / (float)(now - lineLastTime);
    lineRate += LINE_RATE_SMOOTH * (raw - lineRate);
  }
  float rate = lineRate;
  lineLastTime = now;
  lineLastError = error;
  if (fabsf(error) >= LINE_SIDE_ERROR) lineSideTime = now;

  float vx = LINE_SPEED * (1.0f - LINE_CURVE_SLOW * fabsf(error));
  float omega = LINE_TURN_GAIN * error + LINE_TURN_DAMP * rate;
  drive(vx, LINE_STRAFE_GAIN * error, omega, MaxSpeed);
  lineDebug(error > 0.1f ? "rechts" : (error < -0.1f ? "links" : "geradeaus"), dL, dM, dR, error);
}

// ==================== HEBEBUEHNE ====================
// Zwei Spindelmotoren heben die Buehne, jeder an einem eigenen L9110S-Kanal
// (ein Kanal schafft dauerhaft nur ~0,8 A):
//   Motor 1: U5 Kanal B, Platinenpins 5/6 = D16/D4
//   Motor 2: U2 Kanal A, Platinenpins 1/2 = D23/D22
// D12 (U5 Platinenpin 1) ist ein Boot-Pin: liegt er beim Reset auf HIGH, laesst
// sich nichts hochladen. Darum am Modul in U5 die beiden Eingaenge von Kanal A
// (auf Platinenpin 1/2) umbiegen, damit sie keinen Kontakt haben.
// Die Endschalter unten liegen in Reihe mit dem jeweiligen Motor und
// unterbrechen ihn selbst (mit Diode parallel, damit die Buehne von unten
// wieder hochfaehrt) - darum gibt es hier keine Endschalter-Eingaenge. Beim
// Runterfahren gleichen sich beide Seiten so von selbst wieder an.
const int LIFT_MOTORS = 2;
const int LIFT_A[LIFT_MOTORS] = { 16, 23 };
const int LIFT_B[LIFT_MOTORS] = {  4, 22 };
const int LIFT_DIR[LIFT_MOTORS] = { -1, -1 }; // -1 fuer einen Motor, der bei "hoch" runterfaehrt (beide getestet)
// Laeuft eine Seite schneller, dort den Wert senken, damit die Buehne gerade bleibt.
// Motor 1 (D16/D4) lief staerker als Motor 2, darum etwas gedrosselt.
const int LIFT_PWM_UP[LIFT_MOTORS]   = { 230, 255 };
const int LIFT_PWM_DOWN[LIFT_MOTORS] = { 230, 255 };
// Der Controller schickt den Befehl laufend; bleibt er aus, haelt die Buehne an.
const unsigned long LIFT_TIMEOUT = 300; // ms

// Hoehe ueber die Fahrzeit: Die Endschalter liegen in Reihe zum Motor, der
// Teller sieht also nicht, wann die Buehne unten ist. Darum faehrt sie beim
// Start LIFT_HOME_MS lang runter (die Endschalter stoppen jeden Motor unten
// selbst) und gilt dann als Hoehe 0. Danach wird mitgezaehlt: hoch +Zeit,
// runter -Zeit. Bei LIFT_MAX_UP_MS ist Schluss, dann geht nur noch runter.
// LIFT_MAX_UP_MS einstellen: von ganz unten mit voller Fahrt bis zur
// gewuenschten Hoehe die Zeit stoppen.
const unsigned long LIFT_MAX_UP_MS = 1000;
const unsigned long LIFT_HOME_MS = LIFT_MAX_UP_MS * 3 / 2 + 1000; // reicht sicher von ganz oben
// Runter zaehlt hoechstens so schnell wie hoch (mit der Last ist es eher schneller):
// so wird die Hoehe eher zu hoch geschaetzt und die Obergrenze nie ueberfahren.
const float LIFT_DOWN_RATE = 1.0f;

int liftCommand = 0;            // 1 = hoch, -1 = runter, 0 = stopp
int liftMoving = 0;             // was die Motoren gerade wirklich tun
unsigned long lastLiftReceived = 0;
float liftPosMs = 0;            // geschaetzte Hoehe in ms Hochfahrt
unsigned long lastLiftUpdate = 0;

void setLiftMotor(int dir) {
  liftMoving = dir;
  for (int i = 0; i < LIFT_MOTORS; i++) {
    int d = dir * LIFT_DIR[i];
    int pwm = dir > 0 ? LIFT_PWM_UP[i] : LIFT_PWM_DOWN[i];
    if (d > 0) {
      analogWrite(LIFT_A[i], pwm);
      analogWrite(LIFT_B[i], 0);
    } else if (d < 0) {
      analogWrite(LIFT_A[i], 0);
      analogWrite(LIFT_B[i], pwm);
    } else {
      analogWrite(LIFT_A[i], 0); // L9110S: beide LOW = Motor aus
      analogWrite(LIFT_B[i], 0);
    }
  }
}

void liftBegin() {
  for (int i = 0; i < LIFT_MOTORS; i++) {
    pinMode(LIFT_A[i], OUTPUT);
    pinMode(LIFT_B[i], OUTPUT);
  }
  setLiftMotor(0);

  // Nullfahrt
  Serial.printf("Hebebuehne: Nullfahrt %lu ms\n", LIFT_HOME_MS);
  setLiftMotor(-1);
  delay(LIFT_HOME_MS);
  setLiftMotor(0);
  liftPosMs = 0;
  lastLiftUpdate = millis();
}

uint8_t liftPercent() {
  return (uint8_t)constrain(liftPosMs * 100.0f / LIFT_MAX_UP_MS, 0.0f, 100.0f);
}

void sendLiftStatus() {
  uint8_t buf[4] = { 3, MODE_LIFT, (uint8_t)(int8_t)liftMoving, liftPercent() };
  SerialBT.write(buf, sizeof(buf));
}

// Laeuft jede Runde: Timeout und Obergrenze wirken sofort, nicht erst beim
// naechsten Befehl.
void updateLift() {
  unsigned long now = millis();
  unsigned long dt = now - lastLiftUpdate;
  lastLiftUpdate = now;
  if (liftMoving > 0) liftPosMs += dt;
  if (liftMoving < 0) liftPosMs = max(0.0f, liftPosMs - dt * LIFT_DOWN_RATE);

  if (liftCommand != 0 && now - lastLiftReceived > LIFT_TIMEOUT) liftCommand = 0;
  int dir = liftCommand;
  if (dir > 0 && liftPosMs >= LIFT_MAX_UP_MS) dir = 0;
  if (dir != liftMoving) {
    setLiftMotor(dir);
    Serial.printf("Hebebuehne: %s, Hoehe %d %%\n",
                  dir > 0 ? "hoch" : dir < 0 ? "runter" : "stopp", liftPercent());
  }
}

void handleLiftMessage(int8_t command) {
  liftCommand = constrain(command, -1, 1);
  lastLiftReceived = millis();
  updateLift();
  sendLiftStatus(); // Antwort auf jeden Befehl, der Controller zeigt sie an
}

// ==================== AKKUSPANNUNG ====================
// Spannungsteiler 18k/10k von VBAT auf D33 (ADC1, geht auch mit Bluetooth):
// am Pin liegt das 10/28-fache der Akkuspannung (8,4 V -> 3,0 V).
const int BATT_PIN = 33;
const float BATT_DIVIDER = 28.0f / 10.0f;
const float BATT_CAL = 1.0f;    // mit dem Multimeter abgleichen: Soll / angezeigt
const unsigned long BATT_SAMPLE_INTERVAL = 100;
const unsigned long BATT_SEND_INTERVAL = 1000;
float battMv = 0;               // geglaettet, 0 = noch nicht gemessen
unsigned long lastBattSample = 0, lastBattSend = 0;

void updateBattery() {
  unsigned long now = millis();
  if (now - lastBattSample >= BATT_SAMPLE_INTERVAL) {
    lastBattSample = now;
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) sum += analogReadMilliVolts(BATT_PIN);
    float mv = sum / 8.0f * BATT_DIVIDER * BATT_CAL;
    // Glaetten, damit Anfahrstroeme die Anzeige nicht springen lassen
    battMv = battMv == 0 ? mv : battMv + 0.1f * (mv - battMv);
  }
  if (now - lastBattSend >= BATT_SEND_INTERVAL && battMv > 0) {
    lastBattSend = now;
    uint16_t mv = (uint16_t)battMv;
    uint8_t buf[4] = { 3, MODE_BATTERY, (uint8_t)(mv & 0xFF), (uint8_t)(mv >> 8) };
    SerialBT.write(buf, sizeof(buf));
  }
}

// ==================== JOYSTICK-EMPFANG ====================
const unsigned long JOYSTICK_TIMEOUT = 500; // ms
unsigned long lastJoystickReceived = 0;

// ==================== LIDAR (LDROBOT LD06 / LD19) ====================
// Verkabelung: Lidar Tx -> GPIO36 (VP, nur Eingang), P5V -> 5V, GND -> GND,
// PWM offen (der Lidar regelt seine Drehzahl selbst).
// Der Lidar liefert ~23 kB/s - zu viel fuer die Bluetooth-Strecke. Darum wird
// hier auf 2-Grad-Faecher verdichtet und das Rundumbild in vier Stuecken an
// den Controller geschickt (~4 kB/s), der es auf dem Display zeichnet.
const int LIDAR_RX = 36;
const uint32_t LIDAR_BAUD = 230400;
const int LIDAR_PACKET_LEN = 47;
const int LIDAR_POINTS_PER_PACKET = 12;
const float LIDAR_BIN_DEG = 2.0f;
const int LIDAR_BINS = 180;          // 360 / LIDAR_BIN_DEG
const int LIDAR_BINS_PER_MSG = 45;   // ein Viertel des Rundumbilds pro Nachricht
// Einbaulage: Winkel, den der Lidar meldet, wenn er genau nach vorne schaut.
// Der Lidar ist um 90 Grad nach rechts gedreht eingebaut und zaehlt im
// Uhrzeigersinn, "vorne" liegt fuer ihn also bei 270 Grad.
const float LIDAR_MOUNT_DEG = 270.0f;
const unsigned long LIDAR_STALE_MS = 500;    // aelterer Faecher gilt als "kein Echo"
const unsigned long LIDAR_SAME_PASS_MS = 20; // Punkte desselben Umlaufs im selben Faecher
const unsigned long LIDAR_DATA_TIMEOUT = 1000;

uint8_t lidarCrcTable[256];
uint8_t lidarPkt[LIDAR_PACKET_LEN];
int lidarPktLen = 0;
uint16_t lidarMm[LIDAR_BINS];
uint8_t lidarIntensity[LIDAR_BINS];
unsigned long lidarStamp[LIDAR_BINS];
unsigned long lidarLastPacket = 0;
int lidarNextChunk = 0;

void lidarBegin() {
  for (int i = 0; i < 256; i++) {
    uint8_t c = i;
    for (int k = 0; k < 8; k++) c = (c & 0x80) ? (c << 1) ^ 0x4D : c << 1;
    lidarCrcTable[i] = c;
  }
  // Der Standardpuffer (256 Byte) liefe waehrend eines Schleifendurchlaufs ueber.
  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, -1);
}

uint16_t lidarU16(int off) { return lidarPkt[off] | (lidarPkt[off + 1] << 8); }

// ---------- Drehwinkel aus dem Lidar (fuer die Wende) ----------
// Jeder Umlauf (10 Hz) landet in 1-Grad-Faechern. Zwei aufeinanderfolgende
// Umlaeufe werden gegeneinander verschoben, bis sie am besten passen; die
// Verschiebung ist die Drehung dazwischen. Aufsummiert ergibt das den
// Drehwinkel seit lidarTrackStart(). Der Lidar zaehlt im Uhrzeigersinn, bei
// einer Rechtsdrehung des Tellers wird der Winkel also positiv.
const int TRACK_BINS = 360;
const int TRACK_MAX_SHIFT = 45;        // mehr als 45 Grad pro Umlauf dreht er nicht
const int TRACK_CLIP_MM = 300;         // groessere Abweichungen zaehlen nicht mehr
const int TRACK_MIN_OVERLAP = 60;      // so viele Faecher muessen in beiden Umlaeufen belegt sein
const unsigned long TRACK_ALIVE_MS = 400;
uint16_t trackCur[TRACK_BINS], trackPrev[TRACK_BINS];
float trackLastStart = -1;
bool trackHavePrev = false;
float trackDeg = 0;
unsigned long trackLastUpdate = 0;
bool trackActive = false;

void lidarTrackStart() {
  trackHavePrev = false;
  trackDeg = 0;
  trackLastUpdate = millis();
  trackActive = true;
}

float lidarTrackDeg() { return trackDeg; }
bool lidarTrackAlive() { return trackActive && millis() - trackLastUpdate < TRACK_ALIVE_MS; }

// Mittlere Abweichung, wenn der neue Umlauf um shift Grad versetzt zum alten liegt.
float trackCost(int shift, int &overlap) {
  long sum = 0;
  overlap = 0;
  for (int i = 0; i < TRACK_BINS; i++) {
    uint16_t a = trackCur[i], b = trackPrev[(i + shift + TRACK_BINS) % TRACK_BINS];
    if (a == 0 || b == 0) continue;
    sum += min(abs((int)a - (int)b), TRACK_CLIP_MM);
    overlap++;
  }
  return overlap ? (float)sum / overlap : 1e9f;
}

void lidarTrackRevolution() {
  if (trackActive && trackHavePrev) {
    float cost[2 * TRACK_MAX_SHIFT + 1];
    int best = 0, overlap;
    for (int s = -TRACK_MAX_SHIFT; s <= TRACK_MAX_SHIFT; s++) {
      cost[s + TRACK_MAX_SHIFT] = trackCost(s, overlap);
      if (overlap < TRACK_MIN_OVERLAP) cost[s + TRACK_MAX_SHIFT] = 1e9f;
      if (cost[s + TRACK_MAX_SHIFT] < cost[best + TRACK_MAX_SHIFT]) best = s;
    }
    if (cost[best + TRACK_MAX_SHIFT] < 1e8f) {
      float shift = best;
      if (best > -TRACK_MAX_SHIFT && best < TRACK_MAX_SHIFT) { // Parabel fuer Bruchteile
        float c0 = cost[best + TRACK_MAX_SHIFT - 1], c1 = cost[best + TRACK_MAX_SHIFT], c2 = cost[best + TRACK_MAX_SHIFT + 1];
        float den = c0 - 2 * c1 + c2;
        if (den > 0 && c0 < 1e8f && c2 < 1e8f) shift += 0.5f * (c0 - c2) / den;
      }
      trackDeg += shift;
      trackLastUpdate = millis();
    }
  }
  memcpy(trackPrev, trackCur, sizeof(trackCur));
  memset(trackCur, 0, sizeof(trackCur));
  trackHavePrev = true;
}

void lidarHandlePacket(unsigned long now) {
  lidarLastPacket = now;
  float start = lidarU16(4) / 100.0f;
  float end = lidarU16(42) / 100.0f;
  if (start < trackLastStart - 180) lidarTrackRevolution(); // neuer Umlauf
  trackLastStart = start;
  if (end < start) end += 360; // Paket geht ueber 0 Grad
  float step = (end - start) / (LIDAR_POINTS_PER_PACKET - 1);
  for (int i = 0; i < LIDAR_POINTS_PER_PACKET; i++) {
    uint16_t d = lidarU16(6 + 3 * i);
    if (d == 0) continue;
    int tb = (int)lroundf(start + step * i) % TRACK_BINS;
    if (trackCur[tb] == 0 || d < trackCur[tb]) trackCur[tb] = d;
    float angle = fmodf(start + step * i - LIDAR_MOUNT_DEG + 720.0f, 360.0f);
    int b = (int)lroundf(angle / LIDAR_BIN_DEG) % LIDAR_BINS;
    // In einen Faecher fallen mehrere Punkte pro Umlauf: der naechste zaehlt.
    bool samePass = lidarMm[b] != 0 && now - lidarStamp[b] < LIDAR_SAME_PASS_MS;
    if (samePass && d >= lidarMm[b]) continue;
    lidarMm[b] = d;
    lidarIntensity[b] = lidarPkt[8 + 3 * i];
    lidarStamp[b] = now;
  }
}

void readLidar() {
  unsigned long now = millis();
  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    if (lidarPktLen == 0 && byte != 0x54) continue;
    if (lidarPktLen == 1 && byte != 0x2C) {
      // Doch kein Paketanfang; dieses Byte koennte der naechste sein.
      lidarPktLen = byte == 0x54 ? 1 : 0;
      continue;
    }
    lidarPkt[lidarPktLen++] = byte;
    if (lidarPktLen < LIDAR_PACKET_LEN) continue;
    lidarPktLen = 0;
    uint8_t crc = 0;
    for (int i = 0; i < LIDAR_PACKET_LEN - 1; i++) crc = lidarCrcTable[crc ^ lidarPkt[i]];
    if (crc == lidarPkt[LIDAR_PACKET_LEN - 1]) lidarHandlePacket(now);
  }
}

// Schickt pro Aufruf ein Viertel des Rundumbilds. Pro Faecher 16 Bit:
// Bit 0-11 Entfernung in 4-mm-Schritten (0 = kein Echo), Bit 12-15 Intensitaet.
// Gesendet wird nur, solange der Controller im manuellen Modus Joystickwerte
// schickt - also genau dann, wenn dort die Lidar-Ansicht laeuft.
void sendLidarChunk() {
  unsigned long now = millis();
  if (driveMode != DRIVE_MANUAL || lastJoystickReceived == 0) return;
  if (lidarLastPacket == 0 || now - lidarLastPacket > LIDAR_DATA_TIMEOUT) return;

  uint8_t buf[3 + 2 * LIDAR_BINS_PER_MSG];
  int first = lidarNextChunk * LIDAR_BINS_PER_MSG;
  buf[0] = sizeof(buf) - 1; // length = MODE(1) + erster Faecher(1) + Daten
  buf[1] = MODE_SCAN;
  buf[2] = first;
  for (int i = 0; i < LIDAR_BINS_PER_MSG; i++) {
    int b = first + i;
    uint16_t word = 0;
    if (lidarMm[b] != 0 && now - lidarStamp[b] < LIDAR_STALE_MS) {
      word = constrain(lidarMm[b] / 4, 1, 4095) | ((lidarIntensity[b] >> 4) << 12);
    }
    buf[3 + 2 * i] = word & 0xFF;
    buf[4 + 2 * i] = word >> 8;
  }
  SerialBT.write(buf, sizeof(buf));
  lidarNextChunk = (lidarNextChunk + 1) % (LIDAR_BINS / LIDAR_BINS_PER_MSG);
}

void checkConnection() {
  if (!SerialBT.connected()) {
    Serial.println("Connection lost, trying to reconnect...");
    digitalWrite(2, LOW);
    stopMotors();
    while (!SerialBT.connect(masterAddress)) {
      handleSerialCommands(); // Aufzeichnung auslesen geht auch ohne Controller
      delay(1000);
      Serial.println("Waiting for ESP32_Master to be connected...");
    }
    Serial.println("Connected to ESP32_Master");
    digitalWrite(2, HIGH);
  }
}

// ==================== FAHRTENSCHREIBER ====================
// Jede Linienfahrt wird mit 50 Hz im Speicher aufgezeichnet (max. ~80 s) und
// kann danach per USB mit "d" ausgelesen werden (LinienFahrt/auswerten.py).
struct LogEntry {
  uint32_t ms;
  uint16_t raw[3];
  int16_t trackDeg10;           // Lidar-Drehwinkel in 0,1 Grad
  int8_t vx, vy, omega;         // Sollwerte x100
  uint8_t phase;
};
const int LOG_MAX = 4000;
LogEntry logBuf[LOG_MAX];
int logCount = 0;
unsigned long logStartMs = 0;
unsigned long lastLogMs = 0;
bool logSaved = true;
bool logLive = false;           // jede Messung sofort ueber USB ausgeben
bool logFsOk = false;
const char *LOG_FILE = "/fahrt.bin";

// Am Ende einer Fahrt in den Flash, damit die Aufzeichnung ein Ausschalten
// uebersteht. Dauert ~0,2 s, darum nur, wenn der Teller ohnehin steht.
void logSave() {
  if (logSaved || !logFsOk) return;
  logSaved = true;
  File f = LittleFS.open(LOG_FILE, "w");
  if (!f) return;
  f.write((uint8_t *)logBuf, logCount * sizeof(LogEntry));
  f.close();
  Serial.printf("# Aufzeichnung gespeichert (%d)\n", logCount);
}

void logBegin() {
  logFsOk = LittleFS.begin(true);
  if (!logFsOk) return;
  File f = LittleFS.open(LOG_FILE, "r");
  if (!f) return;
  logCount = min((int)(f.size() / sizeof(LogEntry)), LOG_MAX); // letzte Fahrt
  f.read((uint8_t *)logBuf, logCount * sizeof(LogEntry));
  f.close();
}

void logStart() {
  logSaved = false;
  logCount = 0;
  logStartMs = lastLogMs = millis();
  trackActive = false;
}

void logRecord() {
  unsigned long now = millis();
  if (logCount >= LOG_MAX || now - lastLogMs < 20) return;
  lastLogMs = now;
  LogEntry &e = logBuf[logCount++];
  e.ms = now - logStartMs;
  for (int i = 0; i < 3; i++) e.raw[i] = lightRaw[i];
  e.trackDeg10 = (int16_t)constrain(trackDeg * 10, -32000.0f, 32000.0f);
  e.vx = (int8_t)lroundf(constrain(lastVx, -1.0f, 1.0f) * 100);
  e.vy = (int8_t)lroundf(constrain(lastVy, -1.0f, 1.0f) * 100);
  e.omega = (int8_t)lroundf(constrain(lastOmega, -1.0f, 1.0f) * 100);
  e.phase = linePhase;
  if (logLive)
    Serial.printf("@%lu,%d,%d,%d,%d,%.1f,%d,%d,%d\n", (unsigned long)e.ms, e.phase,
                  e.raw[0], e.raw[1], e.raw[2], e.trackDeg10 / 10.0f, e.vx, e.vy, e.omega);
}

void logDump() {
  Serial.printf("# log %d\n", logCount);
  Serial.println("ms,phase,L,M,R,dreh,vx,vy,om");
  for (int i = 0; i < logCount; i++) {
    LogEntry &e = logBuf[i];
    Serial.printf("%lu,%d,%d,%d,%d,%.1f,%d,%d,%d\n", (unsigned long)e.ms, e.phase,
                  e.raw[0], e.raw[1], e.raw[2], e.trackDeg10 / 10.0f, e.vx, e.vy, e.omega);
  }
  Serial.println("# ende");
}

// USB-Befehle fuer Testfahrten ohne Controller-Menue:
//   g = Linienfahrt starten (dann alle 250 ms "k" als Lebenszeichen schicken)
//   k = Lebenszeichen, s = anhalten, d = Aufzeichnung ausgeben
//   v = Live-Ausgabe jeder Messung an/aus ("@ms,phase,L,M,R,dreh,vx,vy,om")
//   p NAME WERT<Enter> = Linienparameter aendern (bis zum Neustart), p<Enter> = alle zeigen
void setDriveMode(int mode);
extern unsigned long lastLineHeartbeat;

struct Tunable { const char *name; float *value; };
const Tunable TUNABLES[] = {
  { "speed", &LINE_SPEED }, { "turn", &LINE_TURN_GAIN }, { "damp", &LINE_TURN_DAMP },
  { "smooth", &LINE_RATE_SMOOTH }, { "strafe", &LINE_STRAFE_GAIN },
  { "slow", &LINE_CURVE_SLOW }, { "pivot", &LINE_PIVOT_TURN },
};
String serialLine;
bool serialInLine = false;

void handleParamLine(const String &line) {
  char name[16];
  float value;
  int n = sscanf(line.c_str(), "p %15s %f", name, &value);
  for (const Tunable &t : TUNABLES) {
    if (n == 2 && strcmp(name, t.name) == 0) *t.value = value;
  }
  Serial.print("# param");
  for (const Tunable &t : TUNABLES) Serial.printf(" %s=%.3f", t.name, *t.value);
  Serial.println();
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (serialInLine) {
      if (c == '\n') {
        serialInLine = false;
        handleParamLine(serialLine);
      } else if (serialLine.length() < 40) {
        serialLine += c;
      }
    } else if (c == 'p') {
      serialInLine = true;
      serialLine = "p";
    }
    else if (c == 'g') setDriveMode(DRIVE_LINE);
    else if (c == 'k') lastLineHeartbeat = millis();
    else if (c == 's') setDriveMode(DRIVE_MANUAL);
    else if (c == 'd') logDump();
    else if (c == 'v') logLive = !logLive;
  }
}

// Der Controller wiederholt DRIVE_LINE laufend als Lebenszeichen. Bleibt es aus
// (Controller aus, Funkabriss), haelt der Teller an.
const unsigned long LINE_HEARTBEAT_TIMEOUT = 800; // ms
unsigned long lastLineHeartbeat = 0;

void setDriveMode(int mode) {
  lastLineHeartbeat = millis();
  DriveMode newMode = mode == DRIVE_LINE ? DRIVE_LINE : DRIVE_MANUAL;
  if (newMode == driveMode) return;

  stopMotors(); // sofort anhalten, egal in welche Richtung gewechselt wird
  driveMode = newMode;
  lastJoystickReceived = 0;

  if (driveMode == DRIVE_LINE) {
    resetLineFollower();
    Serial.println("Fahrmodus: Linienfolger");
  } else {
    logSave();
    Serial.println("Fahrmodus: manuell");
  }
}

void processIncoming() {
  while (SerialBT.available()) {
    uint8_t length = SerialBT.read();

    uint8_t buffer[128];
    uint8_t received = SerialBT.readBytes(buffer, length);
    if (received != length) {
      Serial.println("Timeout / Message too short");
      return;
    }

    switch (buffer[0]) {
      case MODE_PIN:
        pinMode(buffer[1], OUTPUT);
        digitalWrite(buffer[1], buffer[2]);
        Serial.printf("Pin %d auf %d gesetzt\n", buffer[1], buffer[2]);
        break;

      case MODE_TEXT: {
        char text[128];
        memcpy(text, buffer + 1, length - 1);
        text[length - 1] = '\0';
        Serial.print("Text vom Master: ");
        Serial.println(text);
        break;
      }

      case MODE_JOYSTICK: {
        int8_t vx    = (int8_t)buffer[1];
        int8_t vy    = (int8_t)buffer[2];
        int8_t omega = (int8_t)buffer[3];

        if (driveMode != DRIVE_MANUAL) break; // im Linienmodus nicht reinfahren

        Serial.printf("Joystick: vx=%d, vy=%d, omega=%d\n", vx, vy, omega); Serial.println(" ");
        drive(vx / 100.0f, vy / 100.0f, omega / 100.0f, MaxSpeed);
        lastJoystickReceived = millis();
        break;
      }

      case MODE_DRIVEMODE:
        setDriveMode(buffer[1]);
        break;

      case MODE_LIFT:
        handleLiftMessage((int8_t)buffer[1]);
        break;

      default:
        Serial.println("Unknown Mode");
        break;
    }
  }
}

void checkJoystickTimeout() {
  if (lastJoystickReceived != 0 && millis() - lastJoystickReceived > JOYSTICK_TIMEOUT) {
    stopMotors();
    lastJoystickReceived = 0;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Reset reason: %d\n", (int)esp_reset_reason());
  pinMode(2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(lichtL, INPUT);
  pinMode(lichtM, INPUT);
  pinMode(lichtR, INPUT);
  lidarBegin();
  logBegin();
  liftBegin();
  // Bluetooth
  SerialBT.begin("ESP32_Slave", true);
  Serial.println("ESP32 Slave gestartet.");
  esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9);
  while (!SerialBT.connect(masterAddress)) {
    handleSerialCommands(); // Aufzeichnung auslesen geht auch ohne Controller
    Serial.println("Waiting for ESP32_Master to connect...");
    delay(1000);
  }
  Serial.println("Connected to ESP32_Master");
  digitalWrite(2, HIGH);
}

void loop() {
  handleSerialCommands();
  checkConnection();
  processIncoming();
  readLidar();
  updateLift();
  updateBattery();

  if (driveMode == DRIVE_LINE) {
    if (millis() - lastLineHeartbeat > LINE_HEARTBEAT_TIMEOUT) {
      Serial.println("Linienfolger: kein Signal vom Controller");
      setDriveMode(DRIVE_MANUAL);
    } else {
      followLine();
    }
  } else {
    checkJoystickTimeout();
    sendLidarChunk();
  }

  delay(20);
}
