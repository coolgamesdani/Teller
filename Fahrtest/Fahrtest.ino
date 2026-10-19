// ==========================================================
// Fahrtest mit Lidar (nur USB-Serial, kein Bluetooth).
// Der Teller faehrt auf Befehl vom PC kurze Stuecke in eine Richtung und
// reicht dabei jedes gueltige Lidar-Paket (47 Byte, CRC geprueft) unveraendert
// per USB weiter. fahrtest.py zeichnet alles auf und berechnet aus den Scans
// vor und nach jeder Bewegung, wie weit und wohin der Teller wirklich gefahren ist.
//
// USB mit 921600 Baud. Zwischen den Lidar-Paketen kommen Textzeilen, die mit
// '#' beginnen und mit '\n' enden:
//   # bereit                     nach dem Start
//   # f vx vy omega ms <millis>  Fahrt beginnt
//   # e <millis>                 Fahrt zu Ende, Motoren stehen
//   # s <millis>                 Abbruch durch 's'
//   # kein lidar                 seit 1 s kein gueltiges Paket
//
// Kommandos vom PC (eine Zeile):
//   f <vx> <vy> <omega> <ms>     Werte -100..100 wie vom Joystick, ms <= 1500
//   s                            sofort anhalten
//
// Motoren, Abgleich und Lidar-Pin wie in Teller.ino.
// ==========================================================
#include <math.h>

const int MOTOR_PINS[4][2] = { {21, 19}, {5, 17}, {25, 26}, {27, 14} };
// Wie motorMinPWM/motorMaxPWM (manuelles Fahren) in Teller.ino.
const int motorMinPWM[4] = { 214, 219, 212, 214 };
const int motorMaxPWM[4] = { 255, 235, 239, 245 };
const int motorMinPWMRev[4] = { 205, 228, 203, 223 }; // Rad rollt rueckwaerts
const int wheelForward[4] = { -1, 1, -1, 1 };         // Vorzeichen fuer "Rad rollt vorwaerts"
const int motorInvert[4] = { -1, 1, -1, -1 };

const int LIDAR_RX = 36;
const uint32_t LIDAR_BAUD = 230400;
const int LIDAR_PACKET_LEN = 47;

const unsigned long MAX_DRIVE_MS = 1500;

uint8_t crcTable[256];
uint8_t pkt[LIDAR_PACKET_LEN];
int pktLen = 0;
unsigned long lastPacket = 0, lastNoLidarMsg = 0;

bool driving = false;
unsigned long driveStart = 0, driveMs = 0;

char cmd[48];
int cmdLen = 0;

void setMotor(int i, float value) {
  bool rollsForward = value * wheelForward[i] > 0;
  value = constrain(value, -1.0f, 1.0f) * motorInvert[i];
  int a = MOTOR_PINS[i][0], b = MOTOR_PINS[i][1];
  if (fabsf(value) < 0.001f) {
    analogWrite(a, 255);
    analogWrite(b, 255);
    return;
  }
  int minPWM = rollsForward ? motorMinPWM[i] : motorMinPWMRev[i];
  int pwm = minPWM + (int)(fabsf(value) * (motorMaxPWM[i] - minPWM));
  pwm = constrain(pwm, 0, 255);
  analogWrite(a, value > 0 ? pwm : 0);
  analogWrite(b, value > 0 ? 0 : pwm);
}

// Gleiche Mischung wie drive() in Teller.ino.
void drive(float vx, float vy, float omega) {
  float v[4] = { -vx - vy - omega, vx - vy - omega, -vx + vy - omega, vx + vy - omega };
  float maxMag = 1.0f;
  for (int i = 0; i < 4; i++) maxMag = max(maxMag, fabsf(v[i]));
  for (int i = 0; i < 4; i++) setMotor(i, v[i] / maxMag);
}

void readLidar() {
  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    if (pktLen == 0 && byte != 0x54) continue;
    if (pktLen == 1 && byte != 0x2C) {
      pktLen = byte == 0x54 ? 1 : 0;
      continue;
    }
    pkt[pktLen++] = byte;
    if (pktLen < LIDAR_PACKET_LEN) continue;
    pktLen = 0;
    uint8_t crc = 0;
    for (int i = 0; i < LIDAR_PACKET_LEN - 1; i++) crc = crcTable[crc ^ pkt[i]];
    if (crc != pkt[LIDAR_PACKET_LEN - 1]) continue;
    Serial.write(pkt, LIDAR_PACKET_LEN);
    lastPacket = millis();
  }
}

void handleCommand() {
  int vx, vy, om;
  unsigned long ms;
  if (cmd[0] == 's') {
    drive(0, 0, 0);
    driving = false;
    Serial.printf("# s %lu\n", millis());
  } else if (sscanf(cmd, "f %d %d %d %lu", &vx, &vy, &om, &ms) == 4) {
    vx = constrain(vx, -100, 100);
    vy = constrain(vy, -100, 100);
    om = constrain(om, -100, 100);
    driveMs = min(ms, MAX_DRIVE_MS);
    Serial.printf("# f %d %d %d %lu %lu\n", vx, vy, om, driveMs, millis());
    driveStart = millis();
    driving = true;
    drive(vx / 100.0f, vy / 100.0f, om / 100.0f);
  }
}

void readCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmd[cmdLen] = '\0';
        handleCommand();
      }
      cmdLen = 0;
    } else if (cmdLen < (int)sizeof(cmd) - 1) {
      cmd[cmdLen++] = c;
    }
  }
}

void setup() {
  Serial.setTxBufferSize(8192);
  Serial.begin(921600);
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i][0], OUTPUT);
    pinMode(MOTOR_PINS[i][1], OUTPUT);
  }
  drive(0, 0, 0);
  for (int i = 0; i < 256; i++) {
    uint8_t c = i;
    for (int k = 0; k < 8; k++) c = (c & 0x80) ? (c << 1) ^ 0x4D : c << 1;
    crcTable[i] = c;
  }
  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, -1);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.print("# bereit\n");
}

void loop() {
  readLidar();
  readCommands();
  unsigned long now = millis();
  if (driving && now - driveStart >= driveMs) {
    drive(0, 0, 0);
    driving = false;
    Serial.printf("# e %lu\n", now);
  }
  if (now - lastPacket > 1000 && now - lastNoLidarMsg > 1000) {
    lastNoLidarMsg = now;
    Serial.print("# kein lidar\n");
  }
}
