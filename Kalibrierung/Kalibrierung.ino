// ==========================================================
// Kalibriermessung der Lichtsensoren (nur USB-Serial, kein Bluetooth).
// Gibt alle 7 ms die drei Rohwerte "L M R" aus, messen.py wertet sie aus.
// Motoren werden auf Stillstand gehalten, der Teller faehrt nicht.
//
// Pins und analogRead() wie in Teller.ino, damit die Werte uebertragbar sind.
// ==========================================================

const int lichtL = 35;
const int lichtM = 34;
const int lichtR = 39;

const int motorPin1[4] = { 21, 5, 25, 27 };
const int motorPin2[4] = { 19, 17, 26, 14 };

// 7 ms statt 5 oder 10: kein Vielfaches der 100-Hz-Lampenflimmerperiode,
// sonst wuerde immer dieselbe Phase getroffen.
const unsigned long SAMPLE_INTERVAL = 7; // ms
unsigned long lastSample = 0;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(motorPin1[i], OUTPUT);
    pinMode(motorPin2[i], OUTPUT);
    analogWrite(motorPin1[i], 255);
    analogWrite(motorPin2[i], 255);
  }
  pinMode(lichtL, INPUT);
  pinMode(lichtM, INPUT);
  pinMode(lichtR, INPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.println("# Kalibriermessung: L M R");
}

void loop() {
  if (millis() - lastSample < SAMPLE_INTERVAL) return;
  lastSample = millis();
  Serial.printf("%d %d %d\n", analogRead(lichtL), analogRead(lichtM), analogRead(lichtR));
}
