#include <BluetoothSerial.h>
#include <math.h>
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
  MODE_SCAN = 5       // Teller -> Controller: ein Stueck Lidar-Rundumbild
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
// Anfahrschwelle pro Motor, am Boden unter Last gemessen (Motortest, Stufe 0.80):
// darunter dreht sich unter Last nichts. Die alten Werte {85, 35, 60, -10} waren
// mit freilaufenden Raedern ermittelt, dadurch lag der halbe Stellbereich im
// Stillstand. Vorgabe 0 entspricht jetzt "faengt gerade an zu fahren".
int motorMinPWM[4] = {
  220,  // M1
  198,  // M2
  207,  // M3
  185   // M4
};
int motorMaxPWM[4] = {
  255,  // M1
  240,  // M2
  245,  // M3
  235   // M4
};
// Einbaurichtung: -1 fuer Motoren, die anders herum angeschlossen sind als
// drive() erwartet. Im Motortest ermittelt: bei vorwaerts lief nur M2
// (vorne rechts) richtig, M1, M3 und M4 liefen rueckwaerts.
const int motorInvert[4] = {
  -1,   // M1
   1,   // M2
  -1,   // M3
  -1    // M4
};

// ==================== Fahren ================================
void stopMotors(){
  drive(0, 0, 0, MaxSpeed);
}

void setMotor(int pin1, int pin2, float value,
              int motorIndex, int maxSpeed) {
  value = constrain(value, -1.0f, 1.0f) * motorInvert[motorIndex];
  if (fabs(value) < 0.001f) {
    analogWrite(pin1, 255);
    analogWrite(pin2, 255);
    return;
  }
  float magnitude = fabs(value);
  int minPWM = motorMinPWM[motorIndex];
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

void drive(float vx, float vy, float omega, int speed) {
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

// Feste Kalibrierung, gemessen mit Kalibrierung/messen.py (messwerte.csv).
// Mittelwerte der Rohwerte von analogRead():
//
//          weiss              schwarz
//          Licht an / aus     Licht an / aus
//     L     820 /  477         198 /   0
//     M    1531 / 1144         300 /  86
//     R    1101 /  690         159 /   0
//
// LIGHT_WHITE = dunkelstes gemessenes Weiss, LIGHT_BLACK = hellstes gemessenes
// Schwarz, jeweils der unguenstigere Einzelwert aus beiden Lichtverhaeltnissen.
// Dazwischen wird linear gerechnet, ausserhalb abgeschnitten - das passt so mit
// und ohne Umgebungslicht. Die Sensoren sind sehr unterschiedlich empfindlich,
// darum hat jeder seine eigenen Grenzen.
const int LIGHT_WHITE[3] = { 467, 1135, 680 };
const int LIGHT_BLACK[3] = { 207,  307, 167 };
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
const float LINE_SPEED        = 0.80f; // Grundgeschwindigkeit auf der Linie
const float LINE_TURN_GAIN    = 0.80f; // Drehrate, wenn die Linie ganz auf einer Seite liegt
const float LINE_TURN_DAMP    = 0.02f; // Daempfung gegen Pendeln (s)
const float LINE_CURVE_SLOW   = 0.20f; // Anteil, um den in der Kurve gebremst wird
const float LINE_PIVOT_TURN   = 0.40f; // Drehrate beim Drehen auf der Stelle
const float LINE_SEARCH_SPEED = 0.10f;
const float LINE_DARK         = 0.5f;  // ab hier gilt ein Sensor als "auf der Linie"
const float LINE_CORNER_DARK  = 0.85f; // Aussensensor ganz auf Schwarz -> Ecke oder Ziel
const float LINE_CORNER_CLEAR = 0.30f; // Aussensensor wieder auf Weiss
const float LINE_STOP_DARK    = 0.9f;
const float LINE_SIDE_ERROR   = 0.3f;

const unsigned long LINE_PIVOT_ON        = 40;   // Drehen in kurzen Schueben: ms an ...
const unsigned long LINE_PIVOT_OFF       = 120;  // ... und ms Pause
const unsigned long LINE_CORNER_RISE     = 120;  // so schnell muss der Aussensensor schwarz werden
const unsigned long LINE_CORNER_COOLDOWN = 500;  // nach einer Ecke keine neue Ecke / kein Ziel
const unsigned long LINE_BRAKE_TIME      = 60;   // Bremsen vor dem Drehen (steht laut Messung nach ~80 ms)
const unsigned long LINE_SETTLE_TIME     = 40;   // kurzer Halt nach dem Drehen
const unsigned long LINE_CORNER_MAX      = 3000;
const unsigned long LINE_STOP_TIME       = 150;
const unsigned long LINE_SIDE_MEMORY     = 1000;
const unsigned long LINE_LOST_TIMEOUT    = 1500; // so lange wird die verlorene Linie gesucht
const unsigned long LINE_FINISH_WINDOW   = 200;  // beide Aussensensoren fast gleichzeitig schwarz -> Ziel
const unsigned long LINE_DEBUG_INTERVAL  = 250;  // ms zwischen Statusausgaben

enum LinePhase { LINE_FOLLOW, LINE_CORNER_BRAKE, LINE_CORNER_TURN, LINE_CORNER_SETTLE, LINE_GIVEN_UP, LINE_FINISHED };
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

  unsigned long elapsed = millis() - linePhaseSince;
  const char* name = lineCornerDir > 0 ? "rechtsecke" : "linksecke";
  float dSide = lineCornerDir > 0 ? dR : dL;
  float dOther = lineCornerDir > 0 ? dL : dR;

  if (linePhase == LINE_CORNER_BRAKE) {
    stopMotors();
    if (lineAtFinish) {
      setLinePhase(LINE_FINISHED);
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

  if (max(dL, max(dM, dR)) < LINE_DARK) { // alles hell -> Linie verloren
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

  float error = dR - dL; // > 0: Linie liegt rechts, < 0: links
  float rate = 0.0f;
  if (lineLastTime != 0 && now > lineLastTime) {
    rate = (error - lineLastError) * 1000.0f / (float)(now - lineLastTime);
  }
  lineLastTime = now;
  lineLastError = error;
  if (fabsf(error) >= LINE_SIDE_ERROR) lineSideTime = now;

  float vx = LINE_SPEED * (1.0f - LINE_CURVE_SLOW * fabsf(error));
  float omega = LINE_TURN_GAIN * error + LINE_TURN_DAMP * rate;
  drive(vx, 0, omega, MaxSpeed);
  lineDebug(error > 0.1f ? "rechts" : (error < -0.1f ? "links" : "geradeaus"), dL, dM, dR, error);
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
const float LIDAR_MOUNT_DEG = 0.0f;
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

void lidarHandlePacket(unsigned long now) {
  lidarLastPacket = now;
  float start = lidarU16(4) / 100.0f;
  float end = lidarU16(42) / 100.0f;
  if (end < start) end += 360; // Paket geht ueber 0 Grad
  float step = (end - start) / (LIDAR_POINTS_PER_PACKET - 1);
  for (int i = 0; i < LIDAR_POINTS_PER_PACKET; i++) {
    uint16_t d = lidarU16(6 + 3 * i);
    if (d == 0) continue;
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
      delay(1000);
      Serial.println("Waiting for ESP32_Master to be connected...");
    }
    Serial.println("Connected to ESP32_Master");
    digitalWrite(2, HIGH);
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
  // Bluetooth
  SerialBT.begin("ESP32_Slave", true);
  Serial.println("ESP32 Slave gestartet.");
  esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9);
  while (!SerialBT.connect(masterAddress)) {
    Serial.println("Waiting for ESP32_Master to connect...");
    delay(1000);
  }
  Serial.println("Connected to ESP32_Master");
  digitalWrite(2, HIGH);
}

void loop() {
  checkConnection();
  processIncoming();
  readLidar();

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
