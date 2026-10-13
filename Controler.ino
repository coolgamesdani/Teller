#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Keypad.h>

#include <algorithm>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"

BluetoothSerial SerialBT;

enum MessageType : uint8_t {
  MODE_PIN = 1,
  MODE_TEXT   = 2,
  MODE_JOYSTICK = 3,
  MODE_DRIVEMODE = 4,
  MODE_SCAN = 5       // Teller -> Controller: ein Stueck Lidar-Rundumbild
};

// Fahrmodus des Tellers (gleiche Werte wie in Teller.ino)
enum DriveMode : uint8_t {
  DRIVE_MANUAL = 0,
  DRIVE_LINE   = 1
};
// --- Serielle Kommandos ---
enum SerialCommand {
  CMD_UNKNOWN,
  CMD_BUTTON,
  CMD_TEXT,
  CMD_STATUS,
  CMD_MODE,
  CMD_HELP
};

// ==================== DISPLAY ====================
#define TFT_SCLK 13
#define TFT_MOSI 12
#define TFT_RST  14
#define TFT_DC   27
#define TFT_CS   26
#define TFT_LED  25

// Hardware-SPI (HSPI) auf den vorhandenen Pins. Mit dem Software-SPI von vorher
// braeuchte ein Lidar-Bild ~0,3 s, in denen kein Joystickwert gesendet wuerde.
SPIClass tftSPI(HSPI);
Adafruit_ST7735 tft = Adafruit_ST7735(&tftSPI, TFT_CS, TFT_DC, TFT_RST);

// ==================== KEYPAD (Steuerkreuz) ====================
const byte rows = 2;
const byte collumns = 2;
char keys[rows][collumns] = {
  {'1','2'},
  {'4','3'}
};

byte pinsRows[rows]       = {16, 17};
byte pinsCollumns[collumns] = {18, 5};

Keypad keyboard = Keypad(makeKeymap(keys), pinsRows, pinsCollumns, rows, collumns);

// ==================== MENÜ ====================
const char* modes[] = { "Manuell", "Automatisch", "Projekte" };
const int numModes = sizeof(modes) / sizeof(modes[0]);
const int MODE_MANUELL_INDEX = 0;
const int MODE_AUTOMATIK_INDEX = 1; // Linienfolger

int selectedMode = 0;
int lastSelectedMode = -1;

enum AppState { STATE_MENU, STATE_ACTIVE };
AppState appState = STATE_MENU;
int activeModeIndex = -1;

const int lineHeight = 20;
const int firstLineY = 10;

// ==================== BILDSCHIRMSCHONER ====================
const unsigned long SCREENSAVER_TIMEOUT = 1000000000;
unsigned long lastInputTime = 0;       
bool screensaverActive = false;

// --- Vorwärtsdeklarationen (werden weiter unten definiert) ---
void sendTextMessage(const String &text);
void sendButtonMessage(uint8_t pin, uint8_t status);
void sendJoystickMessage(int8_t vx, int8_t vy, int8_t omega);
void sendDriveModeMessage(uint8_t mode);
bool isManualModeActive();
bool isLineModeActive();

void drawMenuLine(int i) {
  tft.fillRect(0, firstLineY + i * lineHeight, tft.width(), lineHeight - 4, ST77XX_BLACK);

  tft.setCursor(5, firstLineY + i * lineHeight);
  tft.setTextSize(1);
  if (i == selectedMode) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("> ");
  } else {
    tft.setTextColor(ST77XX_WHITE);
    tft.print("  ");
  }
  tft.println(modes[i]);
}

void drawMenu() {
  tft.fillScreen(ST77XX_BLACK);
  for (int i = 0; i < numModes; i++) {
    drawMenuLine(i);
  }
  lastSelectedMode = selectedMode;
}

void updateMenuSelection() {
  if (selectedMode == lastSelectedMode) return;

  drawMenuLine(lastSelectedMode);
  drawMenuLine(selectedMode);

  lastSelectedMode = selectedMode;
}

void drawActiveMode(int index) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_GREEN);

  const char* name = modes[index];
  int textWidth = strlen(name) * 12;  
  int textHeight = 16;                

  int x = (tft.width() - textWidth) / 2;
  int y = (tft.height() - textHeight) / 2;
  if (x < 0) x = 0;
  if (y < 0) y = 0;

  tft.setCursor(x, y);
  tft.println(name);
}

// ==================== LIDAR-ANSICHT ====================
// Im manuellen Modus schickt der Teller sein Lidar-Rundumbild (siehe Teller.ino).
// Nach LIDAR_VIEW_DELAY ersetzt es den Modusnamen auf dem Display: links das
// Radarbild (oben = vorne), rechts Zoom, Ringabstand und das naechste Hindernis.
const int LIDAR_BINS = 180;         // wie in Teller.ino
const float LIDAR_BIN_DEG = 2.0f;
const unsigned long LIDAR_VIEW_DELAY = 1000;    // so lange bleibt "Manuell" stehen
const unsigned long LIDAR_FRAME_INTERVAL = 100; // der Lidar dreht mit 10 Hz
const unsigned long LIDAR_DATA_TIMEOUT = 1000;

const float LIDAR_MIN_RANGE_M = 0.5f;
const float LIDAR_MAX_RANGE_M = 12.0f;
const int LIDAR_RINGS = 4;
// Der Zoom fasst diesen Anteil der Punkte, damit ein paar weite Echos (offene
// Tuer, Fenster) nicht das ganze Bild verkleinern.
const float LIDAR_AUTO_COVER = 0.9f;
const float LIDAR_AUTO_MARGIN = 1.15f;
// Schnell herauszoomen, damit nichts ueber den Rand laeuft, langsam hinein,
// damit das Bild nicht pumpt.
const float LIDAR_AUTO_OUT_RATE = 0.5f;
const float LIDAR_AUTO_IN_RATE = 0.08f;

const int LIDAR_W = 160, LIDAR_H = 128;
const int LIDAR_CX = 64, LIDAR_CY = 64, LIDAR_R = 61;
const int LIDAR_TEXT_X = 130;

constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
const uint16_t LIDAR_BG = rgb565(3, 10, 6);
const uint16_t LIDAR_SCOPE = rgb565(5, 20, 11);
const uint16_t LIDAR_GRID = rgb565(18, 64, 36);
const uint16_t LIDAR_GRID_BRIGHT = rgb565(34, 110, 62);
const uint16_t LIDAR_TEXT = rgb565(110, 230, 150);
const uint16_t LIDAR_NEAREST = rgb565(255, 190, 70);
const uint16_t LIDAR_ERROR = rgb565(255, 90, 80);

// Jedes Bild wird im Speicher gezeichnet und am Stueck uebertragen: kein Flackern.
GFXcanvas16 lidarCanvas(LIDAR_W, LIDAR_H);

uint16_t lidarMm[LIDAR_BINS];       // 0 = kein Echo
uint8_t lidarIntensity[LIDAR_BINS]; // 0..15
uint16_t lidarSorted[LIDAR_BINS];   // Arbeitsspeicher fuer lidarAutoRange()
float lidarRangeM = 4.0f;
unsigned long lidarViewSince = 0;
unsigned long lidarLastScan = 0;
unsigned long lidarLastFrame = 0;

void startLidarView() {
  memset(lidarMm, 0, sizeof(lidarMm));
  lidarRangeM = 4.0f;
  lidarViewSince = millis();
  lidarLastScan = 0;
  lidarLastFrame = 0;
}

// Nutzlast von MODE_SCAN: erster Faecher, dann pro Faecher 16 Bit (Bit 0-11
// Entfernung in 4-mm-Schritten, Bit 12-15 Intensitaet).
void handleScanMessage(const uint8_t *data, int len) {
  int first = data[0];
  for (int i = 0; 2 * i + 2 < len && first + i < LIDAR_BINS; i++) {
    uint16_t word = data[1 + 2 * i] | (data[2 + 2 * i] << 8);
    lidarMm[first + i] = (word & 0x0FFF) * 4;
    lidarIntensity[first + i] = word >> 12;
  }
  lidarLastScan = millis();
}

void lidarToScreen(float angleDeg, float distM, int16_t &x, int16_t &y) {
  float a = radians(angleDeg);
  float r = distM / lidarRangeM * LIDAR_R;
  x = LIDAR_CX + lroundf(r * sinf(a));
  y = LIDAR_CY - lroundf(r * cosf(a));
}

void lidarAutoRange() {
  int n = 0;
  for (int b = 0; b < LIDAR_BINS; b++) {
    if (lidarMm[b]) lidarSorted[n++] = lidarMm[b];
  }
  if (n == 0) return; // keine Daten: Zoom behalten
  int k = (n - 1) * LIDAR_AUTO_COVER;
  std::nth_element(lidarSorted, lidarSorted + k, lidarSorted + n);
  float target = constrain(lidarSorted[k] / 1000.0f * LIDAR_AUTO_MARGIN, LIDAR_MIN_RANGE_M, LIDAR_MAX_RANGE_M);
  // Im Logarithmus glaetten, damit sich der Zoom in jedem Massstab gleich anfuehlt.
  float rate = target > lidarRangeM ? LIDAR_AUTO_OUT_RATE : LIDAR_AUTO_IN_RATE;
  lidarRangeM *= powf(target / lidarRangeM, rate);
}

void drawLidarScope() {
  lidarCanvas.fillScreen(LIDAR_BG);
  lidarCanvas.fillCircle(LIDAR_CX, LIDAR_CY, LIDAR_R, LIDAR_SCOPE);
  for (int deg = 0; deg < 360; deg += 30) {
    int16_t x, y;
    lidarToScreen(deg, lidarRangeM, x, y);
    lidarCanvas.drawLine(LIDAR_CX, LIDAR_CY, x, y, LIDAR_GRID);
  }
  for (int i = 1; i <= LIDAR_RINGS; i++) {
    lidarCanvas.drawCircle(LIDAR_CX, LIDAR_CY, LIDAR_R * i / LIDAR_RINGS, i == LIDAR_RINGS ? LIDAR_GRID_BRIGHT : LIDAR_GRID);
  }
  // Der Teller, mit einem Strich nach vorne.
  lidarCanvas.fillCircle(LIDAR_CX, LIDAR_CY, 2, LIDAR_GRID_BRIGHT);
  lidarCanvas.drawFastVLine(LIDAR_CX, LIDAR_CY - 5, 5, LIDAR_TEXT);
}

void drawLidarPoints() {
  int maxMm = lidarRangeM * 1000;
  bool havePrev = false;
  int16_t px = 0, py = 0, firstX = 0, firstY = 0;
  int pd = 0;
  // Ein Faecher mehr als vorhanden, damit sich der Kreis bei 0 Grad schliesst.
  for (int i = 0; i <= LIDAR_BINS; i++) {
    int b = i % LIDAR_BINS;
    int d = lidarMm[b];
    if (d == 0 || d > maxMm) {
      havePrev = false;
      continue;
    }
    int16_t x, y;
    lidarToScreen(b * LIDAR_BIN_DEG, d / 1000.0f, x, y);
    // Nachbarpunkte auf derselben Flaeche verbinden.
    if (havePrev && abs(pd - d) < 150 + d / 8) {
      lidarCanvas.drawLine(px, py, x, y, LIDAR_GRID_BRIGHT);
    }
    float k = 0.45f + 0.55f * lidarIntensity[b] / 15;
    lidarCanvas.drawPixel(x, y, rgb565(140 * k, 255 * k, 170 * k));
    havePrev = true;
    px = x;
    py = y;
    pd = d;
  }
}

// Liefert den Faecher mit dem naechsten Echo, oder -1.
int drawLidarNearest() {
  int best = -1;
  for (int b = 0; b < LIDAR_BINS; b++) {
    if (lidarMm[b] && (best < 0 || lidarMm[b] < lidarMm[best])) best = b;
  }
  if (best >= 0 && lidarMm[best] <= lidarRangeM * 1000) {
    int16_t x, y;
    lidarToScreen(best * LIDAR_BIN_DEG, lidarMm[best] / 1000.0f, x, y);
    lidarCanvas.drawCircle(x, y, 4, LIDAR_NEAREST);
  }
  return best;
}

void lidarLabel(int y, uint16_t color, const char *text) {
  lidarCanvas.setTextColor(color);
  lidarCanvas.setCursor(LIDAR_TEXT_X, y);
  lidarCanvas.print(text);
}

void drawLidarStatus(int nearest) {
  char line[8];
  lidarCanvas.setTextSize(1);
  lidarLabel(2, LIDAR_GRID_BRIGHT, "Zoom");
  snprintf(line, sizeof(line), "%.1fm", lidarRangeM);
  lidarLabel(12, LIDAR_TEXT, line);
  lidarLabel(30, LIDAR_GRID_BRIGHT, "Ring");
  snprintf(line, sizeof(line), "%.2f", lidarRangeM / LIDAR_RINGS);
  lidarLabel(40, LIDAR_TEXT, line);
  if (nearest >= 0) {
    lidarLabel(58, LIDAR_GRID_BRIGHT, "Nah");
    snprintf(line, sizeof(line), "%.2f", lidarMm[nearest] / 1000.0f);
    lidarLabel(68, LIDAR_NEAREST, line);
    // Winkel im Uhrzeigersinn, 0 = vorne
    snprintf(line, sizeof(line), "@%d", (int)(nearest * LIDAR_BIN_DEG));
    lidarLabel(78, LIDAR_NEAREST, line);
  }
}

void handleLidarView() {
  if (!isManualModeActive() || screensaverActive) return;
  unsigned long now = millis();
  if (now - lidarViewSince < LIDAR_VIEW_DELAY) return;
  if (now - lidarLastFrame < LIDAR_FRAME_INTERVAL) return;
  lidarLastFrame = now;

  bool noData = lidarLastScan == 0 || now - lidarLastScan > LIDAR_DATA_TIMEOUT;
  if (noData) memset(lidarMm, 0, sizeof(lidarMm));
  lidarAutoRange();
  drawLidarScope();
  drawLidarPoints();
  drawLidarStatus(drawLidarNearest());
  if (noData) {
    lidarCanvas.setTextColor(LIDAR_ERROR);
    lidarCanvas.setCursor(LIDAR_CX - 30, LIDAR_CY + 12);
    lidarCanvas.print("kein Lidar");
  }
  tft.drawRGBBitmap(0, 0, lidarCanvas.getBuffer(), LIDAR_W, LIDAR_H);
}

void activateMode(int index) {
  appState = STATE_ACTIVE;
  activeModeIndex = index;
  drawActiveMode(index);
  if (index == MODE_MANUELL_INDEX) startLidarView();

  sendTextMessage(String(modes[index]));
  sendDriveModeMessage(index == MODE_AUTOMATIK_INDEX ? DRIVE_LINE : DRIVE_MANUAL);
}

bool isManualModeActive() {
  return appState == STATE_ACTIVE && activeModeIndex == MODE_MANUELL_INDEX;
}

bool isLineModeActive() {
  return appState == STATE_ACTIVE && activeModeIndex == MODE_AUTOMATIK_INDEX;
}

void exitMode() {
  if (isManualModeActive()) {
    sendJoystickMessage(0, 0, 0); 
  }
  if (isLineModeActive()) {
    sendDriveModeMessage(DRIVE_MANUAL); // Linienfolger anhalten
  }

  appState = STATE_MENU;
  activeModeIndex = -1;
  lastSelectedMode = -1; 
  drawMenu();
  Serial.println("Zurueck zum Menü");
}
void activateScreensaver() {
  tft.fillScreen(ST77XX_BLACK);
  digitalWrite(TFT_LED, LOW);
  // Platzhalter: hier könnte statt des schwarzen Screens ein Bild angezeigt werden,
  // tft.drawRGBBitmap(x, y, bitmapDaten, breite, hoehe);
  // oder tft.drawBitmap(...) für ein 1-Bit-Bild aus dem Flash-Speicher.

  screensaverActive = true;
}
void wakeUpScreen() {
  screensaverActive = false;
  digitalWrite(TFT_LED, HIGH);
  lastSelectedMode = -1;
  drawMenu();
}
void checkScreensaver() {
  if (!screensaverActive && (millis() - lastInputTime >= SCREENSAVER_TIMEOUT)) {
    activateScreensaver();
  }
}
void handleKeypad() {
  char key = keyboard.getKey();
  if (!key) return;
  lastInputTime = millis();
  if (screensaverActive) {
    wakeUpScreen();
    return;
  }

  if (appState == STATE_MENU) {
    switch (key) {
      case '2': // Hoch
        selectedMode = (selectedMode - 1 + numModes) % numModes;
        updateMenuSelection();
        break;

      case '4': // Runter
        selectedMode = (selectedMode + 1) % numModes;
        updateMenuSelection();
        break;

      case '3': // Rechts
        activateMode(selectedMode);
        break;

      case '1': //Links
        break;
    }
  } else { // STATE_ACTIVE
    switch (key) {
      case '1':
        exitMode();
        break;
      default:
        break;
    }
  }
}

// --- Senden ---
void sendButtonMessage(uint8_t pin, uint8_t status) {
  uint8_t buf[4];
  buf[0] = 3; // length = MODE(1) + pin(1) + status(1)
  buf[1] = MODE_PIN;
  buf[2] = pin;
  buf[3] = status;
  SerialBT.write(buf, sizeof(buf));
}

// Zuletzt gesendeter Fahrmodus. Solange der Linienfolger laeuft, wird DRIVE_LINE
// als Lebenszeichen wiederholt - bleibt es aus, haelt der Teller von selbst an.
const unsigned long LINE_HEARTBEAT_INTERVAL = 200; // ms
uint8_t tellerDriveMode = DRIVE_MANUAL;
unsigned long lastDriveModeSend = 0;

void writeDriveMode(uint8_t mode) {
  uint8_t buf[3];
  buf[0] = 2; // length = MODE(1) + fahrmodus(1)
  buf[1] = MODE_DRIVEMODE;
  buf[2] = mode;
  SerialBT.write(buf, sizeof(buf));
  tellerDriveMode = mode;
  lastDriveModeSend = millis();
}

void sendDriveModeMessage(uint8_t mode) {
  writeDriveMode(mode);
  Serial.printf("Sende Fahrmodus: %d\n", mode);
}

void handleLineHeartbeat() {
  if (tellerDriveMode != DRIVE_LINE) return;
  if (millis() - lastDriveModeSend < LINE_HEARTBEAT_INTERVAL) return;
  writeDriveMode(DRIVE_LINE);
}

void sendTextMessage(const String &text) {
  uint8_t len = text.length();
  if (len > 126) len = 126; // buffer[128] - 1 (length) - 1 (mode)

  uint8_t buf[128];
  buf[0] = len + 1;
  buf[1] = MODE_TEXT;
  memcpy(buf + 2, text.c_str(), len);

  SerialBT.write(buf, len + 2);
}

// --- Schultertasten ---
const int TURN_RIGHT_PIN = 21;
const int TURN_LEFT_PIN  = 19;

// ==================== JOYSTICK (Fahren + Drehen) ====================
const int JOY_X_PIN = 32; // vy
const int JOY_Y_PIN = 35; // vx

const int JOY_CENTER   = 1760; 
const int JOY_DEADZONE = 200;  

// Achsrichtung des Joysticks: -1, wenn die Achse spiegelverkehrt liefert.
// Am Fahrzeug ermittelt - der Motortest hat bestaetigt, dass vx>0 vorwaerts
// fahrt und omega>0 rechts dreht, die Vorzeichen kamen also von hier.
const int JOY_VX_SIGN = -1; // vor/zurueck
const int JOY_VY_SIGN = -1; // links/rechts

const int8_t TURN_SPEED = 40; 
const unsigned long JOYSTICK_SEND_INTERVAL = 50;
unsigned long lastJoystickSend = 0;

int8_t readJoystickAxis(int pin) {
  int raw = analogRead(pin); // 0..4095
  //Serial.println(raw);
  int centered = raw - JOY_CENTER;
  if (abs(centered) < JOY_DEADZONE) return 0;

  float normalized = (float)centered / JOY_CENTER;
  if (normalized > 1.0f) normalized = 1.0f;
  if (normalized < -1.0f) normalized = -1.0f;
  return (int8_t)(normalized * 100);
}

void sendJoystickMessage(int8_t vx, int8_t vy, int8_t omega) {
  uint8_t buf[5];
  buf[0] = 4; // length = MODE(1) + vx(1) + vy(1) + omega(1)
  buf[1] = MODE_JOYSTICK;
  buf[2] = (uint8_t)vx;
  buf[3] = (uint8_t)vy;
  buf[4] = (uint8_t)omega;
  SerialBT.write(buf, sizeof(buf));
}

void handleJoystick() {
  if (!isManualModeActive()) return;

  if (millis() - lastJoystickSend < JOYSTICK_SEND_INTERVAL) return;
  lastJoystickSend = millis();

  int8_t vx = JOY_VX_SIGN * readJoystickAxis(JOY_Y_PIN); // vor/zurueck
  int8_t vy = JOY_VY_SIGN * readJoystickAxis(JOY_X_PIN); // links/rechts

  // Tasten haengen mit INPUT_PULLUP an GND: gedrueckt = LOW.
  bool turnRight = digitalRead(TURN_RIGHT_PIN) == LOW;
  bool turnLeft  = digitalRead(TURN_LEFT_PIN)  == LOW;
  int8_t omega = 0;
  if (turnRight && !turnLeft) omega =  TURN_SPEED;
  if (turnLeft  && !turnRight) omega = -TURN_SPEED;

  if (vx != 0 || vy != 0 || omega != 0) {
    lastInputTime = millis();
  }

  sendJoystickMessage(vx, vy, omega);
}

// --- Verbindung: nur passiv warten, NICHT selbst connecten ---
void checkConnection() {
  if (!SerialBT.connected()) {
    Serial.println("Connection lost, waiting for Slave to reconnect...");
    digitalWrite(2, LOW);
    while (!SerialBT.connected()) {
      delay(1000);
      Serial.println("Waiting for ESP32_Slave to connect...");
    }
    Serial.println("Connected to ESP32_Slave");
    digitalWrite(2, HIGH);
  }
}

// --- Empfang von Nachrichten vom Slave ---
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
      case MODE_TEXT: {
        char text[128];
        memcpy(text, buffer + 1, length - 1);
        text[length - 1] = '\0';
        Serial.print("Received from Slave: ");
        Serial.println(text);
        break;
      }
      case MODE_SCAN:
        handleScanMessage(buffer + 1, length - 1);
        break;
      default:
        Serial.println("Unknown Mode from Slave");
        break;
    }
  }
}

SerialCommand parseCommand(const String &word) {
  if (word == "pin") return CMD_BUTTON;
  if (word == "text")   return CMD_TEXT;
  if (word == "status") return CMD_STATUS;
  if (word == "mode")   return CMD_MODE;
  if (word == "help")   return CMD_HELP;
  return CMD_UNKNOWN;
}

int splitParams(const String &input, String parts[], int maxParts) {
  int count = 0;
  int start = 0;
  while (count < maxParts) {
    int spaceIndex = input.indexOf(' ', start);
    if (spaceIndex == -1) {
      parts[count++] = input.substring(start);
      break;
    }
    parts[count++] = input.substring(start, spaceIndex);
    start = spaceIndex + 1;
  }
  return count;
}

void printHelp() {
  Serial.println("Verfuegbare Kommandos:");
  Serial.println("  pin <pin> <0|1>   - Pin am Slave setzen");
  Serial.println("  text <nachricht>     - Text an Slave senden");
  Serial.println("  status               - Verbindungsstatus anzeigen");
  Serial.println("  mode <0|1>           - Fahrmodus: 0=manuell, 1=Linienfolger");
  Serial.println("  help                 - diese Liste anzeigen");
}

void handleSerialInput() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  int firstSpace = input.indexOf(' ');
  String cmdWord = (firstSpace == -1) ? input : input.substring(0, firstSpace);
  String rest    = (firstSpace == -1) ? ""    : input.substring(firstSpace + 1);
  cmdWord.toLowerCase();

  switch (parseCommand(cmdWord)) {
    case CMD_BUTTON: {
      String parts[2];
      int n = splitParams(rest, parts, 2);
      if (n < 2) {
        Serial.println("Nutzung: pin <pin> <0|1>");
        break;
      }
      int pin    = parts[0].toInt();
      int status = parts[1].toInt();
      sendButtonMessage(pin, status);
      Serial.printf("Sende Pin: pin=%d status=%d\n", pin, status);
      break;
    }

    case CMD_TEXT:
      if (rest.length() == 0) {
        Serial.println("Nutzung: text <nachricht>");
        break;
      }
      sendTextMessage(rest);
      Serial.println("Sende Text: " + rest);
      break;

    case CMD_STATUS:
      Serial.println(SerialBT.connected() ? "Verbunden mit Slave" : "Nicht verbunden");
      break;

    case CMD_MODE: {
      if (rest.length() == 0) {
        Serial.println("Nutzung: mode <0|1>");
        break;
      }
      int mode = rest.toInt();
      if (mode != DRIVE_MANUAL && mode != DRIVE_LINE) {
        Serial.println("Nutzung: mode <0|1>");
        break;
      }
      sendDriveModeMessage((uint8_t)mode);
      break;
    }

    case CMD_HELP:
      printHelp();
      break;

    default:
      Serial.println("Unbekanntes Kommando. 'help' fuer Uebersicht.");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  pinMode(TURN_RIGHT_PIN, INPUT_PULLUP);
  pinMode(TURN_LEFT_PIN, INPUT_PULLUP);

  // --- Display ---
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);
  tftSPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);
  tft.initR(INITR_BLACKTAB);
  tft.setSPISpeed(24000000);
  tft.setRotation(3);
  drawMenu();
  lastInputTime = millis(); // Screensaver-Timer starten

  SerialBT.begin("ESP32_Master");
  Serial.println("ESP32 Master gestartet, warte auf Slave...");

  esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9);

  while (!SerialBT.connected()) {
    Serial.println("Waiting for ESP32_Slave to connect...");
    delay(1000);
  }
  Serial.println("Connected to ESP32_Slave");
  digitalWrite(2, HIGH);

  printHelp();
}

void loop() {
  checkConnection();
  processIncoming();
  handleJoystick();
  handleKeypad();
  handleLidarView();
  handleLineHeartbeat();
  checkScreensaver();
  handleSerialInput();
  delay(5);
}