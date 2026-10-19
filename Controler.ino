#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Keypad.h>
#include <Preferences.h>

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
  MODE_SCAN = 5,      // Teller -> Controller: ein Stueck Lidar-Rundumbild
  MODE_LIFT = 6,      // Controller -> Teller: Hebebuehne hoch/runter/stopp,
                      // Teller -> Controller: Zustand des Hebemotors
  MODE_BATTERY = 7    // Teller -> Controller: Akkuspannung in mV (uint16)
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
  CMD_JOY,
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

// Pfeile der eingebauten Display-Schrift fuer die Tasten 1-4 in Bildschirmtexten
#define ARROW_UP    "\x18" // Taste 2
#define ARROW_DOWN  "\x19" // Taste 4
#define ARROW_RIGHT "\x1A" // Taste 3
#define ARROW_LEFT  "\x1B" // Taste 1

// --- Schultertasten ---
const int TURN_RIGHT_PIN = 21;
const int TURN_LEFT_PIN  = 19;

// ==================== MENÜ ====================
const char* modes[] = { "Manuell", "Automatisch", "Hebebuehne", "Programme" };
const int numModes = sizeof(modes) / sizeof(modes[0]);
const int MODE_MANUELL_INDEX = 0;
const int MODE_AUTOMATIK_INDEX = 1; // Linienfolger
const int MODE_LIFT_INDEX = 2;
const int MODE_PROGRAMS_INDEX = 3;

// Unterpunkte von "Programme"
const char* programs[] = { "Tetris", "Mini-Doom" };
const int numPrograms = sizeof(programs) / sizeof(programs[0]);
const int PROGRAM_TETRIS = 0;
const int PROGRAM_DOOM = 1;
int selectedProgram = 0;
int runningProgram = -1; // -1 = Programmliste wird angezeigt

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
void startDoom();

// ==================== AKKUANZEIGE ====================
// Oben rechts auf jedem Bild: Akkusymbol mit Fuellstand und Spannung des Tellers.
const int BATT_X = 124, BATT_W = 36, BATT_H = 9;
const float BATT_EMPTY_V = 6.0f;  // 7,2V-NiMH: leer
const float BATT_FULL_V  = 8.4f;  // frisch geladen
const float BATT_WARN_V  = 7.0f;  // darunter gelb
const float BATT_LOW_V   = 6.5f;  // darunter rot: laden
const unsigned long BATT_TIMEOUT = 3000;
float battVolt = 0;
unsigned long lastBattMsg = 0;
const unsigned long BATT_BLINK_MS = 500; // ohne Verbindung blinkt die Anzeige grau
int shownBattTenths = -2;         // zuletzt gezeichneter Wert, -1 = keine Daten
bool shownBattBlink = true;

int batteryTenths() {
  if (lastBattMsg == 0 || millis() - lastBattMsg > BATT_TIMEOUT) return -1;
  return (int)(battVolt * 10 + 0.5f);
}

bool batteryBlinkOn() {
  return (millis() / BATT_BLINK_MS) % 2 == 0;
}

void drawBattery(Adafruit_GFX &g, uint16_t bg) {
  int tenths = batteryTenths();
  shownBattTenths = tenths;
  shownBattBlink = batteryBlinkOn();
  g.fillRect(BATT_X, 0, BATT_W, BATT_H, bg);
  if (tenths < 0 && !shownBattBlink) return; // Blinkphase aus

  float v = tenths / 10.0f;
  uint16_t color = tenths < 0 ? 0x7BEF                  // grau
                 : v < BATT_LOW_V ? ST77XX_RED
                 : v < BATT_WARN_V ? ST77XX_YELLOW : ST77XX_GREEN;
  // Symbol 10x7 mit Pluspol
  g.drawRect(BATT_X, 1, 10, 7, color);
  g.fillRect(BATT_X + 10, 3, 1, 3, color);
  if (tenths >= 0) {
    float level = constrain((v - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V), 0.0f, 1.0f);
    int w = (int)(level * 8 + 0.5f);
    if (w > 0) g.fillRect(BATT_X + 1, 2, w, 5, color);
  }

  char text[6];
  if (tenths < 0) snprintf(text, sizeof(text), "--V");
  else snprintf(text, sizeof(text), "%d.%dV", tenths / 10, tenths % 10);
  g.setTextSize(1);
  g.setTextColor(color);
  g.setCursor(BATT_X + 12, 1);
  g.print(text);
}

void handleBatteryMessage(const uint8_t *data, int len) {
  if (len < 2) return;
  battVolt = (data[0] | (data[1] << 8)) / 1000.0f;
  lastBattMsg = millis();
}

bool lidarViewShowing(); // die Lidar-Ansicht zeichnet die Anzeige selbst ins Bild

// Nur neu zeichnen, wenn sich der angezeigte Wert aendert.
void handleBatteryDisplay() {
  if (screensaverActive || lidarViewShowing()) return;
  int tenths = batteryTenths();
  if (tenths != shownBattTenths || (tenths < 0 && batteryBlinkOn() != shownBattBlink))
    drawBattery(tft, ST77XX_BLACK);
}

// Eine Zeile einer Auswahlliste, die erste Zeile steht bei y0.
void drawListLine(const char *const items[], int i, int selected, int y0) {
  tft.fillRect(0, y0 + i * lineHeight, tft.width(), lineHeight - 4, ST77XX_BLACK);

  tft.setCursor(5, y0 + i * lineHeight);
  tft.setTextSize(1);
  if (i == selected) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("> ");
  } else {
    tft.setTextColor(ST77XX_WHITE);
    tft.print("  ");
  }
  tft.println(items[i]);
}

void drawMenuLine(int i) {
  drawListLine(modes, i, selectedMode, firstLineY);
}

void drawMenu() {
  tft.fillScreen(ST77XX_BLACK);
  for (int i = 0; i < numModes; i++) {
    drawMenuLine(i);
  }
  lastSelectedMode = selectedMode;
  drawBattery(tft, ST77XX_BLACK);
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
  drawBattery(tft, ST77XX_BLACK);
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
// Zoom von Hand in 10-cm-Schritten (hoch = hinein, runter = heraus). Wird die
// Taste gehalten, wiederholt sich der Schritt. Von Hand geht es naeher heran
// als mit dem automatischen Zoom.
const int LIDAR_ZOOM_STEP_DM = 1;
const float LIDAR_ZOOM_MIN_RANGE_M = 0.1f;
const unsigned long LIDAR_ZOOM_HOLD_TIME = 400; // ms bis zur ersten Wiederholung
const unsigned long LIDAR_ZOOM_REPEAT = 60;     // ms zwischen den Wiederholungen

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
bool lidarAutoZoom = true; // bis zum ersten Zoom von Hand, Taste rechts schaltet zurueck
unsigned long lidarViewSince = 0;
unsigned long lidarLastScan = 0;
unsigned long lidarLastFrame = 0;

void startLidarView() {
  memset(lidarMm, 0, sizeof(lidarMm));
  lidarRangeM = 4.0f;
  lidarAutoZoom = true;
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

// steps > 0 zoomt heraus, steps < 0 hinein. Der Wert landet immer auf vollen 10 cm.
void lidarZoom(int steps) {
  lidarAutoZoom = false;
  float dm = roundf(lidarRangeM * 10.0f) + steps * LIDAR_ZOOM_STEP_DM;
  lidarRangeM = constrain(dm / 10.0f, LIDAR_ZOOM_MIN_RANGE_M, LIDAR_MAX_RANGE_M);
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
  lidarLabel(16, LIDAR_GRID_BRIGHT, lidarAutoZoom ? "Auto" : "Zoom");
  snprintf(line, sizeof(line), "%.1fm", lidarRangeM);
  lidarLabel(26, LIDAR_TEXT, line);
  lidarLabel(44, LIDAR_GRID_BRIGHT, "Ring");
  snprintf(line, sizeof(line), "%.2f", lidarRangeM / LIDAR_RINGS);
  lidarLabel(54, LIDAR_TEXT, line);
  if (nearest >= 0) {
    lidarLabel(72, LIDAR_GRID_BRIGHT, "Nah");
    snprintf(line, sizeof(line), "%.2f", lidarMm[nearest] / 1000.0f);
    lidarLabel(82, LIDAR_NEAREST, line);
    // Winkel im Uhrzeigersinn, 0 = vorne
    snprintf(line, sizeof(line), "@%d", (int)(nearest * LIDAR_BIN_DEG));
    lidarLabel(92, LIDAR_NEAREST, line);
  }
  drawBattery(lidarCanvas, LIDAR_BG);
}

bool lidarViewShowing() {
  return isManualModeActive() && millis() - lidarViewSince >= LIDAR_VIEW_DELAY;
}

void handleLidarView() {
  if (!lidarViewShowing() || screensaverActive) return;
  unsigned long now = millis();
  if (now - lidarLastFrame < LIDAR_FRAME_INTERVAL) return;
  lidarLastFrame = now;

  bool noData = lidarLastScan == 0 || now - lidarLastScan > LIDAR_DATA_TIMEOUT;
  if (noData) memset(lidarMm, 0, sizeof(lidarMm));
  if (lidarAutoZoom) lidarAutoRange();
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

// ==================== HEBEBUEHNE ====================
// Taste 2 (hoch) bzw. 4 (runter) gedrueckt halten, loslassen = stopp. Der
// Befehl geht laufend an den Teller, der ohne ihn nach 300 ms anhaelt. Der
// Endschalter unten liegt in Reihe mit dem Motor. Der Teller meldet seinen Zustand zurueck.
const unsigned long LIFT_SEND_INTERVAL = 100; // ms
const unsigned long LIFT_STATUS_TIMEOUT = 500;
unsigned long lastLiftSend = 0;
int8_t liftWanted = 0;
int8_t liftState = 0;         // vom Teller gemeldet
int liftPercent = 0;          // vom Teller gemeldet, 100 = Obergrenze
int lastLiftPercentShown = -1;
unsigned long lastLiftStatus = 0;
int lastLiftScreen = -1;      // zuletzt gezeichneter Zustand

bool isLiftModeActive() {
  return appState == STATE_ACTIVE && activeModeIndex == MODE_LIFT_INDEX;
}

void sendLiftMessage(int8_t command) {
  uint8_t buf[3] = { 2, MODE_LIFT, (uint8_t)command };
  SerialBT.write(buf, sizeof(buf));
}

void handleLiftStatusMessage(const uint8_t *data, int len) {
  if (len < 1) return;
  liftState = (int8_t)data[0];
  if (len >= 2) liftPercent = data[1];
  lastLiftStatus = millis();
}

// Gedrueckt oder gehalten zaehlt, losgelassen nicht.
bool keyIsDown(char c) {
  for (int i = 0; i < LIST_MAX; i++) {
    if (keyboard.key[i].kchar == c &&
        (keyboard.key[i].kstate == PRESSED || keyboard.key[i].kstate == HOLD)) return true;
  }
  return false;
}

void drawLiftScreen(int screen) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(20, 16);
  tft.print("Hebebuehne");

  tft.setTextSize(3);
  tft.setCursor(30, 50);
  if (screen < 0) {
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(14, 56);
    tft.print("kein Teller");
  } else if (liftState > 0) {
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("HOCH");
  } else if (liftState < 0) {
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("RUNTER");
  } else {
    tft.setTextColor(ST77XX_WHITE);
    tft.print("Stopp");
  }

  lastLiftPercentShown = -1;

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(4, 100);
  tft.print(ARROW_UP " hoch / " ARROW_DOWN " runter (halten)");
  tft.setCursor(4, 114);
  tft.print(ARROW_LEFT " zurueck");
  drawBattery(tft, ST77XX_BLACK);
}

// Nur die Hoehenzeile, damit das Bild beim Fahren nicht flackert.
void drawLiftHeight(bool online) {
  int shown = online ? liftPercent : -2;
  if (shown == lastLiftPercentShown) return;
  lastLiftPercentShown = shown;
  tft.fillRect(0, 80, tft.width(), 10, ST77XX_BLACK);
  if (!online) return;
  tft.setTextSize(1);
  tft.setCursor(30, 81);
  if (liftPercent >= 100) {
    tft.setTextColor(ST77XX_RED);
    tft.print("Hoehe 100% - oben");
  } else {
    tft.setTextColor(ST77XX_CYAN);
    tft.printf("Hoehe %d%%", liftPercent);
  }
}

void handleLift() {
  if (!isLiftModeActive()) return;
  unsigned long now = millis();

  bool up = keyIsDown('2'), down = keyIsDown('4');
  liftWanted = up && !down ? 1 : (down && !up ? -1 : 0);
  if (liftWanted != 0) lastInputTime = now;

  if (now - lastLiftSend >= LIFT_SEND_INTERVAL) {
    lastLiftSend = now;
    sendLiftMessage(liftWanted);
  }

  if (screensaverActive) return;
  bool online = lastLiftStatus != 0 && now - lastLiftStatus < LIFT_STATUS_TIMEOUT;
  int screen = online ? liftState + 1 : -1;
  if (screen != lastLiftScreen) {
    lastLiftScreen = screen;
    drawLiftScreen(screen);
  }
  drawLiftHeight(online);
}


// ==================== PROGRAMME ====================
const int PROGRAM_LIST_Y = 30;

bool isProgramsActive() {
  return appState == STATE_ACTIVE && activeModeIndex == MODE_PROGRAMS_INDEX;
}

bool isTetrisActive() {
  return isProgramsActive() && runningProgram == PROGRAM_TETRIS;
}

bool isDoomActive() {
  return isProgramsActive() && runningProgram == PROGRAM_DOOM;
}

void drawProgramMenu() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(5, firstLineY);
  tft.print("Programme");
  for (int i = 0; i < numPrograms; i++) {
    drawListLine(programs, i, selectedProgram, PROGRAM_LIST_Y);
  }
  tft.setTextColor(0x7BEF);
  tft.setCursor(5, 114);
  tft.print(ARROW_RIGHT " starten  " ARROW_LEFT " zurueck");
  drawBattery(tft, ST77XX_BLACK);
}

// Tasten fuer die Spiele: Steuerkreuz '1'..'4' mit Wiederholung beim Halten
// und die Schultertasten (0 = rechts, 1 = links).
const unsigned long PROGRAM_BUTTON_DEBOUNCE = 30;
bool progKeyPrev[4];
unsigned long progKeyPressedAt[4], progKeyLastRepeat[4];
bool progButtonDown[2];
unsigned long progButtonChanged[2];

// Liefert true beim Druecken und, wenn repeatEvery > 0, beim Halten wiederholt.
bool programKeyEvent(int i, unsigned long now, unsigned long repeatAfter, unsigned long repeatEvery) {
  bool down = keyIsDown('1' + i);
  bool pressed = down && !progKeyPrev[i];
  progKeyPrev[i] = down;
  if (pressed) {
    progKeyPressedAt[i] = progKeyLastRepeat[i] = now;
    return true;
  }
  if (!down || repeatEvery == 0) return false;
  if (now - progKeyPressedAt[i] < repeatAfter || now - progKeyLastRepeat[i] < repeatEvery) return false;
  progKeyLastRepeat[i] = now;
  return true;
}

// Schultertaste: true beim Druecken, nach jedem Wechsel kurz entprellen.
bool programButtonPressed(int i, int pin, unsigned long now) {
  bool down = digitalRead(pin) == LOW;
  if (down == progButtonDown[i] || now - progButtonChanged[i] < PROGRAM_BUTTON_DEBOUNCE) return false;
  progButtonDown[i] = down;
  progButtonChanged[i] = now;
  return down;
}

// Was jetzt schon gedrueckt ist (z.B. die Starttaste), zaehlt nicht.
void resetProgramInputs() {
  for (int i = 0; i < 4; i++) progKeyPrev[i] = keyIsDown('1' + i);
  progButtonDown[0] = digitalRead(TURN_RIGHT_PIN) == LOW;
  progButtonDown[1] = digitalRead(TURN_LEFT_PIN) == LOW;
}

// ==================== TETRIS ====================
// Steuerkreuz: 1/3 schieben (halten = wiederholen), 2 drehen, 4 schneller
// fallen (halten). Schultertaste rechts laesst den Stein ganz fallen, links
// haelt das Spiel an. Gezeichnet wird wie die Lidar-Ansicht in lidarCanvas.
const int TET_COLS = 10, TET_ROWS = 20, TET_CELL = 6;
const int TET_X = 50, TET_Y = 4;            // linke obere Ecke des Spielfelds
const int TET_PANEL_X = 114;                // rechte Spalte: Punkte usw.
const unsigned long TET_DAS = 170;          // ms bis zur ersten Wiederholung beim Schieben
const unsigned long TET_ARR = 50;           // ms zwischen den Wiederholungen
const unsigned long TET_SOFT_DROP = 40;     // ms pro Reihe mit Taste 4
const unsigned long TET_CLEAR_MS = 250;     // so lange blinken volle Reihen

// 7 Steine in je 4 Drehlagen als 4x4-Raster, Bit 15 = oben links.
const uint16_t TET_SHAPES[7][4] = {
  { 0x0F00, 0x2222, 0x00F0, 0x4444 }, // I
  { 0x8E00, 0x6440, 0x0E20, 0x44C0 }, // J
  { 0x2E00, 0x4460, 0x0E80, 0xC440 }, // L
  { 0x6600, 0x6600, 0x6600, 0x6600 }, // O
  { 0x6C00, 0x4620, 0x06C0, 0x8C40 }, // S
  { 0x4E00, 0x4640, 0x0E40, 0x4C40 }, // T
  { 0xC600, 0x2640, 0x0C60, 0x4C80 }  // Z
};
const uint16_t TET_COLORS[8] = {
  rgb565(14, 14, 24),                                       // leer
  rgb565(0, 220, 230), rgb565(40, 90, 255), rgb565(255, 140, 0),
  rgb565(240, 220, 0), rgb565(0, 210, 60), rgb565(170, 60, 230), rgb565(240, 40, 40)
};
const uint16_t TET_FRAME = rgb565(90, 90, 120);
const uint16_t TET_LABEL = rgb565(110, 230, 150);
const uint16_t TET_HINT = rgb565(120, 120, 140);

enum TetState { TET_PLAY, TET_CLEAR, TET_PAUSE, TET_OVER };
TetState tetState = TET_PLAY;
uint8_t tetBoard[TET_ROWS][TET_COLS]; // 0 = leer, sonst Stein + 1
int tetPiece, tetRot, tetPx, tetPy, tetNext;
uint8_t tetBag[7];
int tetBagPos = 7;
uint32_t tetScore = 0, tetBest = 0;
int tetLines = 0, tetLevel = 0;
uint32_t tetClearRows = 0;            // Bitmaske der vollen Reihen
unsigned long tetLastFall = 0, tetClearUntil = 0;
bool tetDirty = true;
Preferences tetPrefs;                 // Bestwert bleibt nach dem Ausschalten


bool tetCell(uint16_t shape, int r, int c) {
  return shape & (0x8000 >> (r * 4 + c));
}

bool tetFits(int piece, int rot, int x, int y) {
  uint16_t shape = TET_SHAPES[piece][rot];
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      if (!tetCell(shape, r, c)) continue;
      int bx = x + c, by = y + r;
      if (bx < 0 || bx >= TET_COLS || by >= TET_ROWS) return false;
      if (by >= 0 && tetBoard[by][bx]) return false;
    }
  }
  return true;
}

// Alle 7 Steine gemischt nacheinander, damit keiner lange ausbleibt.
int tetNextFromBag() {
  if (tetBagPos >= 7) {
    for (int i = 0; i < 7; i++) tetBag[i] = i;
    for (int i = 6; i > 0; i--) std::swap(tetBag[i], tetBag[esp_random() % (i + 1)]);
    tetBagPos = 0;
  }
  return tetBag[tetBagPos++];
}

void tetGameOver() {
  tetState = TET_OVER;
  if (tetScore > tetBest) {
    tetBest = tetScore;
    tetPrefs.begin("tetris", false);
    tetPrefs.putUInt("best", tetBest);
    tetPrefs.end();
  }
}

void tetSpawn() {
  tetPiece = tetNext;
  tetNext = tetNextFromBag();
  tetRot = 0;
  tetPx = 3;
  tetPy = 0;
  tetLastFall = millis();
  if (!tetFits(tetPiece, tetRot, tetPx, tetPy)) tetGameOver();
}

unsigned long tetFallInterval() {
  return max(60, 800 - 70 * tetLevel);
}

bool tetMove(int dx, int dy) {
  if (!tetFits(tetPiece, tetRot, tetPx + dx, tetPy + dy)) return false;
  tetPx += dx;
  tetPy += dy;
  tetDirty = true;
  return true;
}

// Drehen; passt der Stein nicht, wird er etwas zur Seite oder nach oben geschoben.
void tetRotate() {
  const int8_t kicks[][2] = { {0, 0}, {-1, 0}, {1, 0}, {-2, 0}, {2, 0}, {0, -1} };
  int rot = (tetRot + 1) % 4;
  for (auto &k : kicks) {
    if (tetFits(tetPiece, rot, tetPx + k[0], tetPy + k[1])) {
      tetRot = rot;
      tetPx += k[0];
      tetPy += k[1];
      tetDirty = true;
      return;
    }
  }
}

void tetLock() {
  uint16_t shape = TET_SHAPES[tetPiece][tetRot];
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      if (!tetCell(shape, r, c)) continue;
      if (tetPy + r < 0) { // ragt oben heraus
        tetGameOver();
        return;
      }
      tetBoard[tetPy + r][tetPx + c] = tetPiece + 1;
    }
  }
  tetClearRows = 0;
  for (int y = 0; y < TET_ROWS; y++) {
    bool full = true;
    for (int x = 0; x < TET_COLS && full; x++) full = tetBoard[y][x];
    if (full) tetClearRows |= 1UL << y;
  }
  tetDirty = true;
  if (tetClearRows) {
    tetState = TET_CLEAR;
    tetClearUntil = millis() + TET_CLEAR_MS;
  } else {
    tetSpawn();
  }
}

void tetFinishClear() {
  const uint16_t points[5] = { 0, 40, 100, 300, 1200 };
  int n = 0;
  for (int y = 0; y < TET_ROWS; y++) {
    if (!(tetClearRows & (1UL << y))) continue;
    n++;
    memmove(tetBoard[1], tetBoard[0], y * TET_COLS); // alles darueber eine Reihe runter
    memset(tetBoard[0], 0, TET_COLS);
  }
  tetScore += points[n] * (tetLevel + 1);
  tetLines += n;
  tetLevel = tetLines / 10;
  tetClearRows = 0;
  tetState = TET_PLAY;
  tetSpawn();
  tetDirty = true;
}



void tetNewGame() {
  memset(tetBoard, 0, sizeof(tetBoard));
  tetScore = 0;
  tetLines = 0;
  tetLevel = 0;
  tetBagPos = 7;
  tetClearRows = 0;
  tetState = TET_PLAY;
  tetNext = tetNextFromBag();
  tetSpawn();
  resetProgramInputs();
  tetDirty = true;
}

void startTetris() {
  runningProgram = PROGRAM_TETRIS;
  tetPrefs.begin("tetris", true);
  tetBest = tetPrefs.getUInt("best", 0);
  tetPrefs.end();
  tetNewGame();
}

void quitProgram() {
  runningProgram = -1;
  drawProgramMenu();
}

void tetDrawCell(int x, int y, uint16_t color) {
  lidarCanvas.fillRect(TET_X + x * TET_CELL, TET_Y + y * TET_CELL, TET_CELL - 1, TET_CELL - 1, color);
}

void tetText(int x, int y, uint16_t color, const char *text, int size = 1) {
  lidarCanvas.setTextSize(size);
  lidarCanvas.setTextColor(color);
  lidarCanvas.setCursor(x, y);
  lidarCanvas.print(text);
}

void tetDrawValue(int y, const char *label, uint32_t value) {
  char line[10];
  tetText(TET_PANEL_X, y, TET_LABEL, label);
  snprintf(line, sizeof(line), "%lu", (unsigned long)value);
  tetText(TET_PANEL_X, y + 10, ST77XX_WHITE, line);
}

void tetDrawBox(int y, int h) {
  lidarCanvas.fillRect(TET_X - 2, y, TET_COLS * TET_CELL + 3, h, ST77XX_BLACK);
  lidarCanvas.drawRect(TET_X - 2, y, TET_COLS * TET_CELL + 3, h, TET_FRAME);
}

void tetDraw() {
  GFXcanvas16 &g = lidarCanvas;
  g.fillScreen(ST77XX_BLACK);
  g.drawRect(TET_X - 2, TET_Y - 2, TET_COLS * TET_CELL + 3, TET_ROWS * TET_CELL + 3, TET_FRAME);

  // liegende Steine, volle Reihen blinken weiss
  bool flash = (millis() / 60) % 2 == 0;
  for (int y = 0; y < TET_ROWS; y++) {
    bool clearing = tetClearRows & (1UL << y);
    for (int x = 0; x < TET_COLS; x++) {
      uint16_t color = TET_COLORS[tetBoard[y][x]];
      if (clearing && flash) color = ST77XX_WHITE;
      tetDrawCell(x, y, color);
    }
  }

  if (tetState == TET_PLAY || tetState == TET_PAUSE) {
    uint16_t shape = TET_SHAPES[tetPiece][tetRot];
    uint16_t color = TET_COLORS[tetPiece + 1];
    // Schatten: dort landet der Stein
    int ghostY = tetPy;
    while (tetFits(tetPiece, tetRot, tetPx, ghostY + 1)) ghostY++;
    for (int r = 0; r < 4; r++) {
      for (int c = 0; c < 4; c++) {
        if (!tetCell(shape, r, c)) continue;
        if (ghostY + r >= 0)
          g.drawRect(TET_X + (tetPx + c) * TET_CELL, TET_Y + (ghostY + r) * TET_CELL, TET_CELL - 1, TET_CELL - 1, color);
        if (tetPy + r >= 0) tetDrawCell(tetPx + c, tetPy + r, color);
      }
    }
  }

  // links: naechster Stein und Tasten
  tetText(6, 4, TET_LABEL, "Next");
  uint16_t next = TET_SHAPES[tetNext][0];
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++)
      if (tetCell(next, r, c))
        g.fillRect(8 + c * TET_CELL, 16 + r * TET_CELL, TET_CELL - 1, TET_CELL - 1, TET_COLORS[tetNext + 1]);
  const char *hints[] = { ARROW_LEFT ARROW_RIGHT " Seite", ARROW_UP " drehen", ARROW_DOWN " unten", "R fall", "L Pause" };
  for (int i = 0; i < 5; i++) tetText(2, 58 + i * 11, TET_HINT, hints[i]);

  // rechts: Punkte
  tetDrawValue(14, "Punkte", tetScore);
  tetDrawValue(40, "Reihen", tetLines);
  tetDrawValue(66, "Level", tetLevel + 1);
  tetDrawValue(92, "Best", max(tetBest, tetScore));

  if (tetState == TET_PAUSE) {
    tetDrawBox(40, 50);
    tetText(TET_X, 46, ST77XX_YELLOW, "PAUSE", 2);
    tetText(TET_X + 3, 68, ST77XX_WHITE, ARROW_RIGHT " weiter");
    tetText(TET_X + 3, 78, ST77XX_WHITE, ARROW_LEFT " Menue");
  } else if (tetState == TET_OVER) {
    tetDrawBox(30, 70);
    tetText(TET_X + 6, 36, ST77XX_RED, "GAME", 2);
    tetText(TET_X + 6, 54, ST77XX_RED, "OVER", 2);
    tetText(TET_X + 3, 78, ST77XX_WHITE, ARROW_RIGHT " neu");
    tetText(TET_X + 3, 88, ST77XX_WHITE, ARROW_LEFT " Menue");
  }

  drawBattery(g, ST77XX_BLACK);
  tft.drawRGBBitmap(0, 0, g.getBuffer(), LIDAR_W, LIDAR_H);
}

void handleTetris() {
  if (!isTetrisActive()) return;
  unsigned long now = millis();

  bool left   = programKeyEvent(0, now, TET_DAS, TET_ARR);
  bool rotate = programKeyEvent(1, now, 0, 0);
  bool right  = programKeyEvent(2, now, TET_DAS, TET_ARR);
  bool down   = programKeyEvent(3, now, 0, TET_SOFT_DROP);
  bool drop   = programButtonPressed(0, TURN_RIGHT_PIN, now);
  bool pause  = programButtonPressed(1, TURN_LEFT_PIN, now);
  if (left || rotate || right || down || drop || pause) lastInputTime = now;

  switch (tetState) {
    case TET_PLAY:
      if (pause) {
        tetState = TET_PAUSE;
        tetDirty = true;
        break;
      }
      if (left) tetMove(-1, 0);
      if (right) tetMove(1, 0);
      if (rotate) tetRotate();
      if (drop) {
        int n = 0;
        while (tetMove(0, 1)) n++;
        tetScore += 2 * n;
        tetLock();
        break;
      }
      if (down) {
        if (tetMove(0, 1)) {
          tetScore++;
          tetLastFall = now;
        } else {
          tetLock();
        }
        break;
      }
      if (now - tetLastFall >= tetFallInterval()) {
        tetLastFall = now;
        if (!tetMove(0, 1)) tetLock();
      }
      break;

    case TET_CLEAR:
      if ((long)(now - tetClearUntil) >= 0) tetFinishClear();
      else tetDirty = true; // blinken
      break;

    case TET_PAUSE:
      if (left) { quitProgram(); return; }
      if (right || pause) {
        tetState = TET_PLAY;
        tetLastFall = now;
        tetDirty = true;
      }
      break;

    case TET_OVER:
      if (left) { quitProgram(); return; }
      if (right) tetNewGame();
      break;
  }

  if (tetDirty && !screensaverActive) {
    tetDirty = false;
    tetDraw();
  }
}

// ==================== STARTANIMATION ====================
// Das Rabutz-Robotics-Logo wie auf der Platine: Kreis mit gespiegeltem und
// normalem R Ruecken an Ruecken. Es waechst erst bildschirmfuellend, schrumpft
// dann auf seine Endgroesse und der Name erscheint darunter. Gezeichnet wird
// in lidarCanvas, damit nichts flackert.
const uint16_t LOGO_BG = rgb565(4, 8, 28);       // dunkelblau wie die Platine
const uint16_t LOGO_FG = rgb565(235, 232, 200);  // Bestueckungsdruck
const unsigned long LOGO_GROW_MS = 900;
const unsigned long LOGO_SHRINK_MS = 600;
const unsigned long LOGO_HOLD_MS = 1200;
const float LOGO_MAX_R = 62, LOGO_END_R = 34;
const int LOGO_CX = 80, LOGO_START_CY = 64, LOGO_END_CY = 50;

// Linie mit Strichstaerke t (Pixel)
void logoLine(float x0, float y0, float x1, float y1, int t) {
  int h = t / 2;
  for (int dx = -h; dx <= t - 1 - h; dx++)
    for (int dy = -h; dy <= t - 1 - h; dy++)
      lidarCanvas.drawLine(x0 + dx, y0 + dy, x1 + dx, y1 + dy, LOGO_FG);
}

// Ein R in Einheiten des Kreisradius; side = 1 normal, -1 gespiegelt.
// Masse vom Platinenlogo abgenommen.
void logoR(float cx, float cy, float r, int side, int t) {
  auto X = [&](float u) { return cx + side * u * r; };
  auto Y = [&](float v) { return cy + v * r; };
  const float stem = 0.06f, top = -0.44f, mid = 0.14f, bottom = 0.60f;
  const float bowlX = 0.42f, bowlR = (mid - top) / 2, bowlY = (mid + top) / 2;
  logoLine(X(stem), Y(top), X(stem), Y(bottom), t);   // Strich
  logoLine(X(stem), Y(top), X(bowlX), Y(top), t);     // Bauch oben
  logoLine(X(stem), Y(mid), X(bowlX), Y(mid), t);     // Bauch unten
  const int SEG = 12;                                 // Bauch rund
  for (int i = 0; i < SEG; i++) {
    float a0 = -PI / 2 + PI * i / SEG, a1 = -PI / 2 + PI * (i + 1) / SEG;
    logoLine(X(bowlX + bowlR * cosf(a0)), Y(bowlY + bowlR * sinf(a0)),
             X(bowlX + bowlR * cosf(a1)), Y(bowlY + bowlR * sinf(a1)), t);
  }
  logoLine(X(0.25f), Y(mid), X(0.70f), Y(bottom), t); // Bein
}

void drawLogo(float cx, float cy, float r) {
  int t = max(1, (int)(r / 14 + 0.5f));
  for (int i = 0; i < t; i++) lidarCanvas.drawCircle(cx, cy, r - i, LOGO_FG);
  logoR(cx, cy, r, -1, t);
  logoR(cx, cy, r, 1, t);
}

float easeOut(float x) { return 1 - (1 - x) * (1 - x); }
float easeInOut(float x) { return x < 0.5f ? 2 * x * x : 1 - 2 * (1 - x) * (1 - x); }

void logoFrame(float cy, float r, bool withName) {
  lidarCanvas.fillScreen(LOGO_BG);
  if (r >= 2) drawLogo(LOGO_CX, cy, r);
  if (withName) {
    lidarCanvas.setTextSize(1);
    lidarCanvas.setTextColor(LOGO_FG);
    const char *name = "Rabutz Robotics";
    lidarCanvas.setCursor(LOGO_CX - strlen(name) * 3, 100);
    lidarCanvas.print(name);
  }
  tft.drawRGBBitmap(0, 0, lidarCanvas.getBuffer(), LIDAR_W, LIDAR_H);
}

void playStartAnimation() {
  unsigned long start = millis();
  for (;;) {                                   // wachsen
    float x = (millis() - start) / (float)LOGO_GROW_MS;
    if (x >= 1) break;
    logoFrame(LOGO_START_CY, LOGO_MAX_R * easeOut(x), false);
  }
  start = millis();
  for (;;) {                                   // schrumpfen und nach oben
    float x = (millis() - start) / (float)LOGO_SHRINK_MS;
    if (x >= 1) break;
    float e = easeInOut(x);
    logoFrame(LOGO_START_CY + (LOGO_END_CY - LOGO_START_CY) * e,
              LOGO_MAX_R + (LOGO_END_R - LOGO_MAX_R) * e, false);
  }
  logoFrame(LOGO_END_CY, LOGO_END_R, true);
  delay(LOGO_HOLD_MS);
}

void activateMode(int index) {
  appState = STATE_ACTIVE;
  activeModeIndex = index;
  if (index == MODE_PROGRAMS_INDEX) {
    runningProgram = -1;
    drawProgramMenu();
  } else {
    drawActiveMode(index);
  }
  if (index == MODE_MANUELL_INDEX) startLidarView();
  if (index == MODE_LIFT_INDEX) {
    lastLiftStatus = 0;
    lastLiftScreen = -2; // sofort neu zeichnen
  }

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
  if (isLiftModeActive()) {
    sendLiftMessage(0);
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
// Hoch/Runter gehalten: Zoom der Lidar-Ansicht wiederholen.
unsigned long lastZoomRepeat = 0;

void handleZoomHold() {
  if (!isManualModeActive() || keyboard.getState() != HOLD) return;
  char held = keyboard.key[0].kchar;
  if (held != '2' && held != '4') return;
  if (millis() - lastZoomRepeat < LIDAR_ZOOM_REPEAT) return;
  lastZoomRepeat = millis();
  lastInputTime = millis();
  lidarZoom(held == '2' ? -1 : 1);
}

void handleKeypad() {
  char key = keyboard.getKey();
  if (!key) {
    handleZoomHold();
    return;
  }
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
  } else if (isProgramsActive()) {
    if (runningProgram >= 0) return; // das Spiel liest die Tasten selbst
    switch (key) {
      case '2': // Hoch
      case '4': // Runter
        drawListLine(programs, selectedProgram, -1, PROGRAM_LIST_Y);
        selectedProgram = (selectedProgram + (key == '2' ? -1 : 1) + numPrograms) % numPrograms;
        drawListLine(programs, selectedProgram, selectedProgram, PROGRAM_LIST_Y);
        break;
      case '3': // Rechts: starten
        if (selectedProgram == PROGRAM_TETRIS) startTetris();
        if (selectedProgram == PROGRAM_DOOM) startDoom();
        break;
      case '1': // Links: zurueck zum Hauptmenue
        exitMode();
        break;
    }
  } else { // STATE_ACTIVE
    switch (key) {
      case '1':
        exitMode();
        break;
      case '2': // Hoch: Lidar-Bild vergroessern
        if (isManualModeActive()) lidarZoom(-1);
        break;
      case '4': // Runter: Lidar-Bild verkleinern
        if (isManualModeActive()) lidarZoom(1);
        break;
      case '3': // Rechts: zurueck zum automatischen Zoom
        if (isManualModeActive()) lidarAutoZoom = true;
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

// ==================== MINI-DOOM ====================
// 3D-Ballerspiel im Stil von Wolfenstein/Doom: Raycaster mit Texturen,
// Monstern, Medikits und Munition. Joystick vor/zurueck = laufen,
// links/rechts = drehen (dazu auch Taste 1/3), Schultertaste rechts oder
// Taste 2 = schiessen (halten = Dauerfeuer), Taste 4 = Pause.
// Tueren gehen auf, wenn man davor steht. Ziel ist der gruene Ausgang.
// Gezeichnet wird wie die Lidar-Ansicht in lidarCanvas.
const int DOOM_W = 160, DOOM_VIEW_H = 112;  // darunter 16 Pixel Anzeige
const int DOOM_MAP = 16, DOOM_TEX = 16;
const float DOOM_PLANE = 0.66f;                       // Sichtfeld ~66 Grad
const float DOOM_PROJ = (DOOM_W / 2) / DOOM_PLANE;    // Pixel pro Kachel in 1 Kachel Abstand
const float DOOM_MOVE_SPEED = 2.6f;    // Kacheln pro Sekunde
const float DOOM_TURN_SPEED = 2.4f;    // rad/s
const float DOOM_RADIUS = 0.22f;       // so dick ist der Spieler
const float DOOM_FOG = 13.0f;          // ab hier ist alles dunkel
const float DOOM_MONSTER_SPEED = 1.3f;
const float DOOM_MONSTER_SIGHT = 10.0f;
const float DOOM_MONSTER_RANGE = 8.0f; // so weit schiessen die Monster
const int DOOM_MONSTER_HP = 3;
const unsigned long DOOM_FRAME_MS = 30;
const unsigned long DOOM_FIRE_MS = 380;
const unsigned long DOOM_ATTACK_WINDUP = 350; // so lange leuchten die Klauen vor dem Treffer
const unsigned long DOOM_MSG_MS = 1200;
// Laut drive() in Teller.ino ist vy > 0 nach rechts. Dreht der Joystick
// verkehrt herum, hier -1 eintragen.
const int DOOM_TURN_SIGN = 1;

// '#' Ziegel, 'T' Technik, 'S' Stein, 'D' Tuer, 'E' Ausgang,
// 'P' Start, 'm' Monster, 'h' Medikit, 'a' Munition
const char *const DOOM_LEVELS[][DOOM_MAP] = {
  {
    "################",
    "#P...#....a....#",
    "#....#..m......#",
    "#....D....SS...#",
    "#....#....SS.m.#",
    "##D###.........#",
    "#....###DTTTTTT#",
    "#.h..#.......a.#",
    "#..m.D...m.....#",
    "#....#....m..h.#",
    "#SSSS#TTTDTTTTT#",
    "#a.............#",
    "#..m...SS...m..#",
    "#......SS......#",
    "#h.........m..E#",
    "################",
  },
  {
    "################",
    "#P.#......#...a#",
    "#..D..m...D.m..#",
    "#..#......#....#",
    "####..TT..###D##",
    "#h.D..TT.....m.#",
    "#..#.m......m..#",
    "#..#..TT..TT...#",
    "#a.#..TT..TT.h.#",
    "##D###########D#",
    "#....m....#....#",
    "#.SS...SS.D..m.#",
    "#.SS.m.SS.#....#",
    "#.........#a.m.#",
    "#..h......#...E#",
    "################",
  },
};
const int DOOM_NUM_LEVELS = sizeof(DOOM_LEVELS) / sizeof(DOOM_LEVELS[0]);

enum DoomTile : uint8_t { TILE_EMPTY, TILE_BRICK, TILE_TECH, TILE_STONE, TILE_DOOR, TILE_EXIT, TILE_COUNT };

// Sprites 16x16, '.' = durchsichtig. Farben siehe doomSpriteColor().
const char *const DOOM_SPRITE_MONSTER[16] = {
  ".....dbbbbd.....",
  "....dbbbbbbd....",
  "....brrbbrrb....",
  "....bbbbbbbb....",
  ".....bwwwwb.....",
  "......dbbd......",
  "...dbbbbbbbbd...",
  "..dbbbbbbbbbbd..",
  "..bb.bbddbb.bb..",
  "..bb.bbbbbb.bb..",
  "..cc.dbbbbd.cc..",
  ".....bbbbbb.....",
  ".....bb..bb.....",
  ".....bb..bb.....",
  ".....db..bd.....",
  "....ccc..ccc....",
};
const char *const DOOM_SPRITE_CORPSE[16] = {
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "......k...k.....",
  "...kkdbbbbdkk...",
  "..kbbrbbdbbbbkk.",
  ".kkkkbbbbbbkkkk.",
};
const char *const DOOM_SPRITE_HEALTH[16] = {
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "....GwwwwwwG....",
  "....wwwrrwww....",
  "....wwwrrwww....",
  "....wrrrrrrw....",
  "....wwwrrwww....",
  "....wwwrrwww....",
  "....GGGGGGGG....",
};
const char *const DOOM_SPRITE_AMMO[16] = {
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  "................",
  ".....y.y.y.y....",
  ".....r.r.r.r....",
  "....dddddddddd..",
  "....dyyyyyyyyd..",
  "....dyddddddyd..",
  "....dddddddddd..",
};

enum DoomThingType : uint8_t { THING_MONSTER, THING_HEALTH, THING_AMMO };
struct DoomThing {
  float x, y;
  DoomThingType type;
  bool active;       // Medikit/Munition noch da
  bool awake;        // Monster hat den Spieler gesehen
  int8_t hp;         // Monster: <= 0 = tot
  unsigned long hurtUntil, attackAt, nextAttack;
};
const int DOOM_MAX_THINGS = 32;
DoomThing doomThings[DOOM_MAX_THINGS];
int doomNumThings = 0;

enum DoomState { DOOM_PLAY, DOOM_PAUSE, DOOM_DEAD, DOOM_LEVEL_DONE, DOOM_WON };
DoomState doomState = DOOM_PLAY;
uint8_t doomMap[DOOM_MAP][DOOM_MAP];
uint16_t doomTex[TILE_COUNT][DOOM_TEX * DOOM_TEX];
bool doomTexReady = false;
uint16_t doomRowColor[DOOM_VIEW_H];  // Decke und Boden
float doomZ[DOOM_W];                 // Wandabstand je Spalte, verdeckt Sprites
float doomX, doomY, doomAngle, doomBob;
int doomHealth, doomAmmo, doomKills, doomMonsters, doomLevel;
unsigned long doomLastFrame, doomLastFire, doomHurtUntil, doomMsgUntil;
const char *doomMsg = "";

uint16_t doomShade(uint16_t c, int k) { // k = 0..256
  int r = ((c >> 11) & 31) * k >> 8, g = ((c >> 5) & 63) * k >> 8, b = (c & 31) * k >> 8;
  return (r << 11) | (g << 5) | b;
}

int doomFogLevel(float dist) {
  return constrain((int)(256 * (1 - dist / DOOM_FOG)), 30, 256);
}

// Kleines Pseudo-Rauschen fuer die Texturen
uint8_t doomNoise(int x, int y, int seed) {
  uint32_t h = x * 374761393u + y * 668265263u + seed * 2147483647u;
  h = (h ^ (h >> 13)) * 1274126177u;
  return (h ^ (h >> 16)) & 0xFF;
}

void doomMakeTextures() {
  for (int y = 0; y < DOOM_TEX; y++) {
    for (int x = 0; x < DOOM_TEX; x++) {
      int n = doomNoise(x, y, 1) % 40;
      int i = y * DOOM_TEX + x;
      // Ziegel: versetzte Steine mit Fugen
      bool mortar = y % 4 == 3 || (x + (y / 4 % 2) * 4) % 8 == 7;
      doomTex[TILE_BRICK][i] = mortar ? rgb565(70 + n / 2, 60 + n / 2, 55) : rgb565(150 + n, 55 + n / 2, 35);
      // Technik: Metallplatten mit Nieten und gruener Leuchtleiste
      bool edge = x == 0 || y == 0 || x == 15 || y == 15 || y == 7;
      bool rivet = (x == 2 || x == 13) && (y == 2 || y == 12);
      if (y >= 9 && y <= 10 && x >= 4 && x <= 11) doomTex[TILE_TECH][i] = rgb565(60, 230, 90);
      else if (edge) doomTex[TILE_TECH][i] = rgb565(55, 60, 75);
      else if (rivet) doomTex[TILE_TECH][i] = rgb565(200, 200, 210);
      else doomTex[TILE_TECH][i] = rgb565(100 + n / 2, 110 + n / 2, 130 + n / 2);
      // Stein: grobe graue Bloecke
      int block = doomNoise(x / 5, (y + (x / 5) * 2) / 6, 7) % 50;
      bool gap = x % 5 == 4 || (y + (x / 5) * 2) % 6 == 5;
      doomTex[TILE_STONE][i] = gap ? rgb565(45, 45, 45) : rgb565(95 + block + n / 3, 95 + block + n / 3, 90 + block);
      // Tuer: braunes Metall mit Griff und gelb-schwarzer Kante
      if (y >= 14) doomTex[TILE_DOOR][i] = ((x + y) / 2 % 2) ? rgb565(230, 200, 0) : rgb565(20, 20, 20);
      else if (x == 7 || x == 8) doomTex[TILE_DOOR][i] = rgb565(40, 30, 20);
      else if ((x == 5 || x == 10) && y >= 6 && y <= 8) doomTex[TILE_DOOR][i] = rgb565(220, 220, 220);
      else doomTex[TILE_DOOR][i] = rgb565(120 + n / 2, 85 + n / 3, 50);
      // Ausgang: leuchtend gruener Schalter mit hellem Rahmen
      bool frame = x <= 1 || x >= 14 || y <= 1 || y >= 14;
      bool arrow = abs(x - 8) <= (y - 3) && y >= 4 && y <= 9;
      bool stem = x >= 6 && x <= 9 && y >= 9 && y <= 12;
      doomTex[TILE_EXIT][i] = frame ? rgb565(200, 255, 200)
                            : (arrow || stem) ? rgb565(255, 255, 255) : rgb565(20, 170 + n, 50);
    }
  }
  // Decke dunkel blaugrau, Boden braun, beide zum Horizont hin dunkler
  for (int y = 0; y < DOOM_VIEW_H / 2; y++) {
    int k = 256 - y * 200 / (DOOM_VIEW_H / 2);
    doomRowColor[y] = doomShade(rgb565(80, 85, 100), k);
    doomRowColor[DOOM_VIEW_H - 1 - y] = doomShade(rgb565(110, 85, 60), k);
  }
  doomTexReady = true;
}

bool doomIsWall(float x, float y) {
  int mx = (int)x, my = (int)y;
  if (x < 0 || y < 0 || mx >= DOOM_MAP || my >= DOOM_MAP) return true;
  return doomMap[my][mx] != TILE_EMPTY;
}

bool doomFree(float x, float y, float r) {
  return !doomIsWall(x - r, y - r) && !doomIsWall(x + r, y - r) &&
         !doomIsWall(x - r, y + r) && !doomIsWall(x + r, y + r);
}

bool doomLineOfSight(float x0, float y0) {
  float dx = doomX - x0, dy = doomY - y0;
  int n = (int)(sqrtf(dx * dx + dy * dy) / 0.2f) + 1;
  for (int i = 1; i < n; i++) {
    if (doomIsWall(x0 + dx * i / n, y0 + dy * i / n)) return false;
  }
  return true;
}

void doomMessage(const char *text) {
  doomMsg = text;
  doomMsgUntil = millis() + DOOM_MSG_MS;
}

void doomLoadLevel(int level) {
  doomLevel = level;
  doomNumThings = 0;
  doomKills = 0;
  doomMonsters = 0;
  for (int y = 0; y < DOOM_MAP; y++) {
    for (int x = 0; x < DOOM_MAP; x++) {
      char c = DOOM_LEVELS[level][y][x];
      uint8_t tile = TILE_EMPTY;
      switch (c) {
        case '#': tile = TILE_BRICK; break;
        case 'T': tile = TILE_TECH; break;
        case 'S': tile = TILE_STONE; break;
        case 'D': tile = TILE_DOOR; break;
        case 'E': tile = TILE_EXIT; break;
        case 'P': doomX = x + 0.5f; doomY = y + 0.5f; break;
        case 'm': case 'h': case 'a':
          if (doomNumThings < DOOM_MAX_THINGS) {
            DoomThing &t = doomThings[doomNumThings++];
            t = DoomThing();
            t.x = x + 0.5f;
            t.y = y + 0.5f;
            t.active = true;
            t.type = c == 'm' ? THING_MONSTER : c == 'h' ? THING_HEALTH : THING_AMMO;
            if (t.type == THING_MONSTER) {
              t.hp = DOOM_MONSTER_HP;
              doomMonsters++;
            }
          }
          break;
      }
      doomMap[y][x] = tile;
    }
  }
  // In die Richtung schauen, in der am meisten Platz ist.
  float best = -1;
  for (int i = 0; i < 4; i++) {
    float a = i * HALF_PI, d = 0;
    while (d < DOOM_MAP && !doomIsWall(doomX + cosf(a) * d, doomY + sinf(a) * d)) d += 0.25f;
    if (d > best) {
      best = d;
      doomAngle = a;
    }
  }
  doomState = DOOM_PLAY;
  doomHurtUntil = 0;
  doomLastFire = 0;
  doomLastFrame = millis();
  static char msg[12];
  snprintf(msg, sizeof(msg), "Level %d", level + 1);
  doomMessage(level == 0 ? "Finde den Ausgang!" : msg);
}

void doomNewGame() {
  doomHealth = 100;
  doomAmmo = 30;
  doomLoadLevel(0);
}

void startDoom() {
  runningProgram = PROGRAM_DOOM;
  if (!doomTexReady) doomMakeTextures();
  doomNewGame();
  resetProgramInputs();
}

// ---------- Zeichnen ----------

void doomRenderWalls(uint16_t *buf, float dirX, float dirY, float planeX, float planeY) {
  for (int x = 0; x < DOOM_W; x++) {
    float cam = 2.0f * x / DOOM_W - 1;
    float rdx = dirX + planeX * cam, rdy = dirY + planeY * cam;
    int mx = (int)doomX, my = (int)doomY;
    float ddx = rdx == 0 ? 1e9f : fabsf(1 / rdx);
    float ddy = rdy == 0 ? 1e9f : fabsf(1 / rdy);
    int stepX = rdx < 0 ? -1 : 1, stepY = rdy < 0 ? -1 : 1;
    float sdx = (rdx < 0 ? doomX - mx : mx + 1 - doomX) * ddx;
    float sdy = (rdy < 0 ? doomY - my : my + 1 - doomY) * ddy;
    int side = 0, tile = TILE_BRICK;
    for (int i = 0; i < 2 * DOOM_MAP + 2; i++) {
      if (sdx < sdy) { sdx += ddx; mx += stepX; side = 0; }
      else           { sdy += ddy; my += stepY; side = 1; }
      if (mx < 0 || my < 0 || mx >= DOOM_MAP || my >= DOOM_MAP) { tile = TILE_BRICK; break; }
      tile = doomMap[my][mx];
      if (tile != TILE_EMPTY) break;
    }
    float dist = side == 0 ? sdx - ddx : sdy - ddy;
    if (dist < 0.05f) dist = 0.05f;
    doomZ[x] = dist;

    float wallX = side == 0 ? doomY + dist * rdy : doomX + dist * rdx;
    wallX -= floorf(wallX);
    int tx = (int)(wallX * DOOM_TEX);
    if ((side == 0 && rdx < 0) || (side == 1 && rdy > 0)) tx = DOOM_TEX - 1 - tx;

    // Die 16 Texel dieser Spalte einmal abdunkeln, dann nur noch nachschlagen.
    int k = doomFogLevel(dist);
    if (side == 1) k = k * 3 / 4;
    uint16_t column[DOOM_TEX];
    for (int t = 0; t < DOOM_TEX; t++) column[t] = doomShade(doomTex[tile][t * DOOM_TEX + tx], k);

    int h = (int)(DOOM_PROJ / dist);
    int top = DOOM_VIEW_H / 2 - h / 2;
    int y0 = max(0, top), y1 = min(DOOM_VIEW_H, top + h);
    float step = (float)DOOM_TEX / h, ty = (y0 - top) * step;
    for (int y = 0; y < y0; y++) buf[y * DOOM_W + x] = doomRowColor[y];
    for (int y = y0; y < y1; y++, ty += step) buf[y * DOOM_W + x] = column[min((int)ty, DOOM_TEX - 1)];
    for (int y = y1; y < DOOM_VIEW_H; y++) buf[y * DOOM_W + x] = doomRowColor[y];
  }
}

uint16_t doomSpriteColor(char c, bool attacking) {
  switch (c) {
    case 'b': return rgb565(150, 80, 40);   // Haut
    case 'd': return rgb565(90, 45, 20);    // Schatten
    case 'r': return rgb565(255, 40, 20);   // Augen, Kreuz, Patronen
    case 'w': return rgb565(235, 235, 220); // Zaehne, Medikit
    case 'c': return attacking ? rgb565(255, 220, 60) : rgb565(60, 30, 10); // Klauen
    case 'k': return rgb565(140, 0, 0);     // Blut
    case 'y': return rgb565(230, 190, 40);  // Munitionskiste
    case 'G': return rgb565(130, 130, 130);
    default:  return 0;
  }
}

// Raum vor der Kamera: tx seitlich (Ebene), ty Tiefe
void doomToCamera(float x, float y, float dirX, float dirY, float planeX, float planeY, float &tx, float &ty) {
  float invDet = 1.0f / (planeX * dirY - dirX * planeY);
  float sx = x - doomX, sy = y - doomY;
  tx = invDet * (dirY * sx - dirX * sy);
  ty = invDet * (-planeY * sx + planeX * sy);
}

void doomRenderThings(uint16_t *buf, float dirX, float dirY, float planeX, float planeY) {
  // Von hinten nach vorne zeichnen.
  int order[DOOM_MAX_THINGS], n = 0;
  float depth[DOOM_MAX_THINGS];
  for (int i = 0; i < doomNumThings; i++) {
    const DoomThing &t = doomThings[i];
    if (!t.active) continue;
    float dx = t.x - doomX, dy = t.y - doomY;
    depth[i] = dx * dx + dy * dy;
    int j = n++;
    while (j > 0 && depth[order[j - 1]] < depth[i]) {
      order[j] = order[j - 1];
      j--;
    }
    order[j] = i;
  }

  unsigned long now = millis();
  for (int o = 0; o < n; o++) {
    const DoomThing &t = doomThings[order[o]];
    float tx, ty;
    doomToCamera(t.x, t.y, dirX, dirY, planeX, planeY, tx, ty);
    if (ty < 0.15f) continue;
    int size = (int)(DOOM_PROJ / ty);
    int left = (int)(DOOM_W / 2 * (1 + tx / ty)) - size / 2;
    int top = DOOM_VIEW_H / 2 - size / 2;

    const char *const *bmp = t.type == THING_HEALTH ? DOOM_SPRITE_HEALTH
                           : t.type == THING_AMMO ? DOOM_SPRITE_AMMO
                           : t.hp > 0 ? DOOM_SPRITE_MONSTER : DOOM_SPRITE_CORPSE;
    bool hurt = now < t.hurtUntil;
    bool attacking = t.attackAt != 0;
    int k = doomFogLevel(ty);

    int x0 = max(0, left), x1 = min(DOOM_W, left + size);
    int y0 = max(0, top), y1 = min(DOOM_VIEW_H, top + size);
    for (int x = x0; x < x1; x++) {
      if (ty >= doomZ[x]) continue; // hinter der Wand
      int sx = (x - left) * 16 / size;
      for (int y = y0; y < y1; y++) {
        char c = bmp[(y - top) * 16 / size][sx];
        if (c == '.') continue;
        buf[y * DOOM_W + x] = hurt ? rgb565(255, 255, 255) : doomShade(doomSpriteColor(c, attacking), k);
      }
    }
  }
}

void doomDrawWeapon(unsigned long now) {
  GFXcanvas16 &g = lidarCanvas;
  bool firing = now - doomLastFire < 90;
  int bx = (int)(sinf(doomBob) * 4), by = (int)(fabsf(cosf(doomBob)) * 3) + (firing ? 5 : 0);
  int cx = DOOM_W / 2 + bx, y = 80 + by;
  if (firing) {
    g.fillCircle(cx, y - 4, 9, rgb565(255, 160, 30));
    g.fillCircle(cx, y - 4, 5, rgb565(255, 250, 180));
  }
  // Schrotflinte: zwei Laeufe, Pumpe und Haende
  g.fillRect(cx - 6, y, 12, DOOM_VIEW_H - y, rgb565(55, 55, 60));
  g.drawFastVLine(cx - 1, y, DOOM_VIEW_H - y, rgb565(20, 20, 22));
  g.drawFastVLine(cx - 5, y, DOOM_VIEW_H - y, rgb565(120, 120, 130));
  g.fillRect(cx - 9, y + 14, 18, 10, rgb565(110, 70, 35));
  g.drawFastHLine(cx - 9, y + 14, 18, rgb565(150, 100, 55));
  g.fillRoundRect(cx - 18, y + 20, 12, DOOM_VIEW_H - y - 20, 3, rgb565(200, 140, 100));
  g.fillRoundRect(cx + 6, y + 22, 12, DOOM_VIEW_H - y - 22, 3, rgb565(185, 125, 90));
}

void doomText(int x, int y, uint16_t color, const char *text, int size = 1) {
  lidarCanvas.setTextSize(size);
  lidarCanvas.setTextColor(color);
  lidarCanvas.setCursor(x, y);
  lidarCanvas.print(text);
}

void doomTextCentered(int y, uint16_t color, const char *text, int size = 1) {
  doomText(DOOM_W / 2 - strlen(text) * 3 * size, y, color, text, size);
}

void doomDrawHud() {
  GFXcanvas16 &g = lidarCanvas;
  char line[16];
  g.fillRect(0, DOOM_VIEW_H, DOOM_W, 128 - DOOM_VIEW_H, rgb565(40, 40, 45));
  g.drawFastHLine(0, DOOM_VIEW_H, DOOM_W, rgb565(90, 90, 100));
  snprintf(line, sizeof(line), "\x03%d", max(doomHealth, 0));
  doomText(3, DOOM_VIEW_H + 5, doomHealth > 30 ? rgb565(255, 90, 80) : ST77XX_RED, line);
  snprintf(line, sizeof(line), "Mun %d", doomAmmo);
  doomText(40, DOOM_VIEW_H + 5, doomAmmo > 0 ? rgb565(240, 200, 60) : rgb565(140, 140, 140), line);
  snprintf(line, sizeof(line), "%d/%d", doomKills, doomMonsters);
  doomText(96, DOOM_VIEW_H + 5, rgb565(150, 230, 150), line);
  snprintf(line, sizeof(line), "L%d", doomLevel + 1);
  doomText(142, DOOM_VIEW_H + 5, ST77XX_WHITE, line);
}

void doomDrawOverlay(const char *title, uint16_t color, const char *line1, const char *line2) {
  GFXcanvas16 &g = lidarCanvas;
  g.fillRect(20, 30, 120, 56, ST77XX_BLACK);
  g.drawRect(20, 30, 120, 56, color);
  doomTextCentered(38, color, title, 2);
  doomTextCentered(60, ST77XX_WHITE, line1);
  doomTextCentered(72, ST77XX_WHITE, line2);
}

void doomDraw(unsigned long now) {
  uint16_t *buf = lidarCanvas.getBuffer();
  float dirX = cosf(doomAngle), dirY = sinf(doomAngle);
  float planeX = -dirY * DOOM_PLANE, planeY = dirX * DOOM_PLANE;
  doomRenderWalls(buf, dirX, dirY, planeX, planeY);
  doomRenderThings(buf, dirX, dirY, planeX, planeY);
  if (doomState != DOOM_DEAD) doomDrawWeapon(now);
  // Fadenkreuz
  lidarCanvas.drawPixel(DOOM_W / 2, DOOM_VIEW_H / 2, rgb565(255, 255, 255));

  if (now < doomHurtUntil || doomState == DOOM_DEAD) {
    for (int i = 0; i < 3; i++) lidarCanvas.drawRect(i, i, DOOM_W - 2 * i, DOOM_VIEW_H - 2 * i, ST77XX_RED);
  }
  if (now < doomMsgUntil && doomState == DOOM_PLAY) doomText(3, 3, ST77XX_WHITE, doomMsg);
  doomDrawHud();

  char line[24];
  switch (doomState) {
    case DOOM_PAUSE:
      doomDrawOverlay("PAUSE", ST77XX_YELLOW, ARROW_RIGHT " weiter", ARROW_LEFT " Menue");
      break;
    case DOOM_DEAD:
      doomDrawOverlay("TOT", ST77XX_RED, ARROW_RIGHT " nochmal", ARROW_LEFT " Menue");
      break;
    case DOOM_LEVEL_DONE:
      snprintf(line, sizeof(line), "Monster %d/%d", doomKills, doomMonsters);
      doomDrawOverlay("GESCHAFFT", ST77XX_GREEN, line, ARROW_RIGHT " weiter");
      break;
    case DOOM_WON:
      doomDrawOverlay("GEWONNEN!", ST77XX_GREEN, ARROW_RIGHT " neu", ARROW_LEFT " Menue");
      break;
    default:
      break;
  }
  drawBattery(lidarCanvas, ST77XX_BLACK);
  tft.drawRGBBitmap(0, 0, buf, LIDAR_W, LIDAR_H);
}

// ---------- Spiellogik ----------

void doomFire(unsigned long now) {
  if (now - doomLastFire < DOOM_FIRE_MS) return;
  if (doomAmmo <= 0) {
    doomLastFire = now;
    doomMessage("Keine Munition!");
    return;
  }
  doomAmmo--;
  doomLastFire = now;
  // Getroffen wird das naechste lebende Monster in der Bildmitte vor der Wand.
  float dirX = cosf(doomAngle), dirY = sinf(doomAngle);
  float planeX = -dirY * DOOM_PLANE, planeY = dirX * DOOM_PLANE;
  DoomThing *target = nullptr;
  float targetDepth = doomZ[DOOM_W / 2];
  for (int i = 0; i < doomNumThings; i++) {
    DoomThing &t = doomThings[i];
    if (t.type != THING_MONSTER || t.hp <= 0) continue;
    float tx, ty;
    doomToCamera(t.x, t.y, dirX, dirY, planeX, planeY, tx, ty);
    if (ty < 0.15f || ty >= targetDepth) continue;
    float screenX = DOOM_W / 2 * (1 + tx / ty);
    if (fabsf(screenX - DOOM_W / 2) < DOOM_PROJ / ty * 0.35f) {
      target = &t;
      targetDepth = ty;
    }
  }
  if (!target) return;
  target->hp -= 1 + random(2);
  target->hurtUntil = now + 120;
  if (!target->awake) {
    target->awake = true;
    target->nextAttack = now + 600;
  }
  if (target->hp <= 0) {
    target->attackAt = 0;
    doomKills++;
  }
}

void doomUpdatePlayer(float dt, unsigned long now) {
  float forward = JOY_VX_SIGN * readJoystickAxis(JOY_Y_PIN) / 100.0f;
  float turn = DOOM_TURN_SIGN * JOY_VY_SIGN * readJoystickAxis(JOY_X_PIN) / 100.0f;
  if (keyIsDown('1')) turn -= 1;
  if (keyIsDown('3')) turn += 1;
  turn = constrain(turn, -1.0f, 1.0f);
  if (forward != 0 || turn != 0) lastInputTime = now;

  doomAngle += turn * DOOM_TURN_SPEED * dt;
  float dirX = cosf(doomAngle), dirY = sinf(doomAngle);
  float mx = dirX * forward * DOOM_MOVE_SPEED * dt;
  float my = dirY * forward * DOOM_MOVE_SPEED * dt;
  if (doomFree(doomX + mx, doomY, DOOM_RADIUS)) doomX += mx;
  if (doomFree(doomX, doomY + my, DOOM_RADIUS)) doomY += my;
  if (mx != 0 || my != 0) doomBob += dt * 9;

  // Tueren und Ausgang in den Nachbarkacheln
  int px = (int)doomX, py = (int)doomY;
  for (int y = py - 1; y <= py + 1; y++) {
    for (int x = px - 1; x <= px + 1; x++) {
      if (x < 0 || y < 0 || x >= DOOM_MAP || y >= DOOM_MAP) continue;
      float dx = x + 0.5f - doomX, dy = y + 0.5f - doomY;
      float d = sqrtf(dx * dx + dy * dy);
      if (doomMap[y][x] == TILE_DOOR && d < 1.3f) {
        doomMap[y][x] = TILE_EMPTY;
        doomMessage("Tuer offen");
      }
      if (doomMap[y][x] == TILE_EXIT && d < 0.85f) {
        doomState = doomLevel + 1 < DOOM_NUM_LEVELS ? DOOM_LEVEL_DONE : DOOM_WON;
      }
    }
  }

  // Sachen aufsammeln
  for (int i = 0; i < doomNumThings; i++) {
    DoomThing &t = doomThings[i];
    if (!t.active || t.type == THING_MONSTER) continue;
    float dx = t.x - doomX, dy = t.y - doomY;
    if (dx * dx + dy * dy > 0.5f * 0.5f) continue;
    if (t.type == THING_HEALTH && doomHealth < 100) {
      doomHealth = min(100, doomHealth + 25);
      t.active = false;
      doomMessage("+25 Gesundheit");
    } else if (t.type == THING_AMMO) {
      doomAmmo = min(99, doomAmmo + 12);
      t.active = false;
      doomMessage("+12 Munition");
    }
  }

  if (keyIsDown('2') || digitalRead(TURN_RIGHT_PIN) == LOW) doomFire(now);
}

void doomUpdateMonsters(float dt, unsigned long now) {
  for (int i = 0; i < doomNumThings; i++) {
    DoomThing &t = doomThings[i];
    if (t.type != THING_MONSTER || t.hp <= 0) continue;
    float dx = doomX - t.x, dy = doomY - t.y;
    float dist = sqrtf(dx * dx + dy * dy);
    bool sees = dist < DOOM_MONSTER_SIGHT && doomLineOfSight(t.x, t.y);
    if (sees && !t.awake) {
      t.awake = true;
      t.nextAttack = now + 800; // kurz Zeit zum Reagieren
    }
    if (!t.awake) continue;

    if (t.attackAt) { // holt gerade aus
      if (now < t.attackAt) continue;
      t.attackAt = 0;
      float chance = max(0.25f, 0.85f - dist * 0.08f);
      if (sees && random(1000) < chance * 1000) {
        doomHealth -= 4 + random(9);
        doomHurtUntil = now + 200;
        if (doomHealth <= 0) doomState = DOOM_DEAD;
      }
      continue;
    }
    if (sees && dist < DOOM_MONSTER_RANGE && now >= t.nextAttack) {
      t.attackAt = now + DOOM_ATTACK_WINDUP;
      t.nextAttack = now + 1300 + random(1200);
      continue;
    }
    if (dist > 1.1f) {
      float step = DOOM_MONSTER_SPEED * dt / dist;
      float nx = t.x + dx * step, ny = t.y + dy * step;
      if (doomFree(nx, t.y, 0.25f)) t.x = nx;
      if (doomFree(t.x, ny, 0.25f)) t.y = ny;
    }
  }
}

void handleDoom() {
  if (!isDoomActive()) return;
  unsigned long now = millis();
  if (now - doomLastFrame < DOOM_FRAME_MS) return;
  float dt = min(0.1f, (now - doomLastFrame) / 1000.0f);
  doomLastFrame = now;

  bool left  = programKeyEvent(0, now, 0, 0);
  bool right = programKeyEvent(2, now, 0, 0);
  bool pause = programKeyEvent(3, now, 0, 0);
  programKeyEvent(1, now, 0, 0);
  if (left || right || pause) lastInputTime = now;

  switch (doomState) {
    case DOOM_PLAY:
      if (pause) {
        doomState = DOOM_PAUSE;
        break;
      }
      doomUpdatePlayer(dt, now);
      if (doomState == DOOM_PLAY) doomUpdateMonsters(dt, now);
      break;
    case DOOM_PAUSE:
      if (left) { quitProgram(); return; }
      if (right || pause) doomState = DOOM_PLAY;
      break;
    case DOOM_DEAD:
      if (left) { quitProgram(); return; }
      if (right) doomNewGame();
      break;
    case DOOM_LEVEL_DONE:
      if (right) doomLoadLevel(doomLevel + 1);
      break;
    case DOOM_WON:
      if (left) { quitProgram(); return; }
      if (right) doomNewGame();
      break;
  }

  if (!screensaverActive) doomDraw(now);
}

// --- Verbindung: nur passiv warten, NICHT selbst connecten ---
// Blockiert nicht: Menue und Tasten laufen auch ohne Verbindung weiter. Die LED
// an Pin 2 zeigt, ob der Teller verbunden ist.
bool slaveConnected = false;

void checkConnection() {
  bool connected = SerialBT.connected();
  if (connected == slaveConnected) return;
  slaveConnected = connected;
  digitalWrite(2, connected ? HIGH : LOW);
  Serial.println(connected ? "Connected to ESP32_Slave"
                           : "Connection lost, waiting for Slave to reconnect...");
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
      case MODE_LIFT:
        handleLiftStatusMessage(buffer + 1, length - 1);
        break;
      case MODE_BATTERY:
        handleBatteryMessage(buffer + 1, length - 1);
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
  if (word == "joy")    return CMD_JOY;
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
  Serial.println("  joy                  - Joystick-Rohwerte und Schultertasten anzeigen");
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

    case CMD_JOY:
      // Zur Fehlersuche: in Ruhe muessen beide Rohwerte innerhalb der Totzone liegen.
      Serial.printf("Joystick roh: vor/zurueck=%d links/rechts=%d (Mitte %d +-%d) -> vx=%d vy=%d, Tasten rechts=%d links=%d\n",
                    analogRead(JOY_Y_PIN), analogRead(JOY_X_PIN), JOY_CENTER, JOY_DEADZONE,
                    JOY_VX_SIGN * readJoystickAxis(JOY_Y_PIN), JOY_VY_SIGN * readJoystickAxis(JOY_X_PIN),
                    digitalRead(TURN_RIGHT_PIN) == LOW, digitalRead(TURN_LEFT_PIN) == LOW);
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
  tftSPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);
  tft.initR(INITR_BLACKTAB);
  tft.setSPISpeed(24000000);
  tft.setRotation(3);
  keyboard.setHoldTime(LIDAR_ZOOM_HOLD_TIME);
  playStartAnimation();
  drawMenu();
  lastInputTime = millis(); // Screensaver-Timer starten

  SerialBT.begin("ESP32_Master");
  Serial.println("ESP32 Master gestartet, warte auf Slave...");

  esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9);

  printHelp();
}

void loop() {
  checkConnection();
  processIncoming();
  handleJoystick();
  handleKeypad();
  handleLidarView();
  handleLift();
  handleTetris();
  handleDoom();
  handleBatteryDisplay();
  handleLineHeartbeat();
  checkScreensaver();
  handleSerialInput();
  delay(5);
}