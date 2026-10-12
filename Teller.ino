#include <BluetoothSerial.h>
#include <math.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"

BluetoothSerial SerialBT;
uint8_t masterAddress[6] = {0x70, 0x4B, 0xCA, 0x22, 0x10, 0xF2};  //Controler
//uint8_t masterAddress[6] = {0xB0, 0xCB, 0xD8, 0xC1, 0x4C, 0x02};    //Teller

enum MessageType : uint8_t {
  MODE_PIN = 1,
  MODE_TEXT   = 2,
  MODE_JOYSTICK = 3
};

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
int motorMinPWM[4] = {
  85,   // M1
  35,   // M2
  60,   // M3
  -10    // M4
};
int motorMaxPWM[4] = {
  255,  // M1
  240,  // M2
  245,  // M3
  235   // M4
};

// ==================== Fahren ================================
void stopMotors(){
  drive(0, 0, 0, MaxSpeed);
}

void setMotor(int pin1, int pin2, float value,
              int motorIndex, int maxSpeed) {
  value = constrain(value, -1.0f, 1.0f);
  if (fabs(value) < 0.001f) {
    analogWrite(pin1, 255);
    analogWrite(pin2, 255);
    return;
  }
  float magnitude = fabs(value);
  int minPWM = motorMinPWM[motorIndex];
  int maxPWM = min(motorMaxPWM[motorIndex], maxSpeed);
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

// ==================== JOYSTICK-EMPFANG ====================
const unsigned long JOYSTICK_TIMEOUT = 500; // ms
unsigned long lastJoystickReceived = 0;

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

        Serial.printf("Joystick: vx=%d, vy=%d, omega=%d\n", vx, vy, omega); Serial.println(" ");
        drive(vx / 100.0f, vy / 100.0f, omega / 100.0f, MaxSpeed);
        lastJoystickReceived = millis();
        break;
      }

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
  pinMode(2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
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
  checkJoystickTimeout();
  delay(20);
}
