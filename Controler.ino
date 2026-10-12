#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Keypad.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"

BluetoothSerial SerialBT;

enum MessageType : uint8_t {
  MODE_PIN = 1,
  MODE_TEXT   = 2,
  MODE_JOYSTICK = 3
};
// --- Serielle Kommandos ---
enum SerialCommand {
  CMD_UNKNOWN,
  CMD_BUTTON,
  CMD_TEXT,
  CMD_STATUS,
  CMD_HELP
};

// ==================== DISPLAY ====================
#define TFT_SCLK 13
#define TFT_MOSI 12
#define TFT_RST  14
#define TFT_DC   27
#define TFT_CS   26
#define TFT_LED  25

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

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

int selectedMode = 0;
int lastSelectedMode = -1;

enum AppState { STATE_MENU, STATE_ACTIVE };
AppState appState = STATE_MENU;
int activeModeIndex = -1;

const int lineHeight = 20;
const int firstLineY = 10;

// ==================== BILDSCHIRMSCHONER ====================
const unsigned long SCREENSAVER_TIMEOUT = 10000; 
unsigned long lastInputTime = 0;       
bool screensaverActive = false;

// --- Vorwärtsdeklarationen (werden weiter unten definiert) ---
void sendTextMessage(const String &text);
void sendButtonMessage(uint8_t pin, uint8_t status);
void sendJoystickMessage(int8_t vx, int8_t vy, int8_t omega);
bool isManualModeActive();

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

void activateMode(int index) {
  appState = STATE_ACTIVE;
  activeModeIndex = index;
  drawActiveMode(index);

  sendTextMessage(String(modes[index]));
}

bool isManualModeActive() {
  return appState == STATE_ACTIVE && activeModeIndex == MODE_MANUELL_INDEX;
}

void exitMode() {
  if (isManualModeActive()) {
    sendJoystickMessage(0, 0, 0); 
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

  int8_t vx = readJoystickAxis(JOY_Y_PIN); // vor/zurueck
  int8_t vy = readJoystickAxis(JOY_X_PIN); // links/rechts

  bool turnRight = digitalRead(TURN_RIGHT_PIN) == HIGH;
  bool turnLeft  = digitalRead(TURN_LEFT_PIN)  == HIGH;
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
  if (SerialBT.available()) {
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
  checkScreensaver();
  handleSerialInput();
  delay(5);
}