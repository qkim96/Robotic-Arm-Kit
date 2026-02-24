/************************************************************************************************

 * Organization  : LognCoding | LognSteam
 * Project       : Robotic Arm Part 2 - Arm
 * Created       : 2025-12-31
 * Author        : Kyu Rae Kim

 * MCU           : ESP32
 * Module        : WROOM-32
 * Dev Board     : Lolin D32

 * Description:
 *   This firmware implements real-time control of a 3-DoF robotic arm using two joysticks.
 *   Joystick inputs are received wirelessly via ESP-NOW and mapped to control the arm's joints.
 *   The analog signal of each joystick axis (X1, Y1, X2, Y2) determines
 *   the corresponding servo's speed for smooth and precise movement.

************************************************************************************************/


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <esp_now.h>

#define ENABLE_PIN 23

// Pulse width for each servo
#define SERVOMIN0 70   // min/max must be tested & calibrated
#define SERVOMAX0 420  // for each servo to be mapped to desired angle range
#define SERVOMIN1 180  // 70-540 => roughly 0-180 degress
#define SERVOMAX1 540
#define SERVOMIN2 70
#define SERVOMAX2 540
#define SERVOMIN3 70
#define SERVOMAX3 540

// Servo max speed (5-15 recommended)
int maxSpeed = 10;

int servoMins[4] = {SERVOMIN0, SERVOMIN1, SERVOMIN2, SERVOMIN3};
int servoMaxs[4] = {SERVOMAX0, SERVOMAX1, SERVOMAX2, SERVOMAX3};

// Servo: channel 12-15 -> base, elbow, claw, wrist
float initServoPos[4] = {95.0, 105.0, 180.0, 110.0};  // change angles to desired initial positions
float servoAngles[4] = {95.0, 105.0, 180.0, 110.0};   // initially set servoAngles to initServoPos
float servoSpeeds[4] = {0.0, 0.0, 0.0, 0.0};

typedef struct {
  int joystick_x1;
  int joystick_y1;
  int joystick_x2;
  int joystick_y2;
} joystick_data;

joystick_data rxData;

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40, Wire);


// ----------------------------------------------------------------------------------------------


// Function for controlling corresponding servo by mapping angle to pulse width
void moveServo(int servoNum, float angle) {
  int ch = servoNum + 12;
  int pulse = map(angle, 0, 180, servoMins[servoNum], servoMaxs[servoNum]);
  pca.setPWM(ch, 0, pulse);
}

// Function to initialize servos to initial positions
void servoInit() {
  for (int i = 0; i < 4; i++) {
    moveServo(i, initServoPos[i]);
    delay(50);
  }
}

// Function to map joystick analog inputs to each servo's speed
void setServoSpeed() {
  int joystickInputs[4] =
  {
    rxData.joystick_x1,
    rxData.joystick_y1,
    rxData.joystick_x2,
    rxData.joystick_y2
  };

  // 1800-1920 when idle (joystick signal)
  // range: 0-4095
  for (int i = 0; i < 4; i++) {
    if ((joystickInputs[i] > 1750) & (joystickInputs[i] < 1970)) {
      servoSpeeds[i] = 0;
      continue;
    }
    if (joystickInputs[i] <= 1750) {
      servoSpeeds[i] = map(joystickInputs[i], 1850, 0, 0, -maxSpeed);
    }
    else if (joystickInputs[i] >= 1970) {
      servoSpeeds[i] = map(joystickInputs[i], 1970, 4095, 0, maxSpeed);
    }
  }
}

// Function to store joystick inputs when received via ESP-NOW
void onDataReceived(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&rxData, incomingData, sizeof(rxData));

  Serial.print("[RECEIVED]\nX1: ");
  Serial.print(rxData.joystick_x1);
  Serial.print(", Y1: ");
  Serial.println(rxData.joystick_y1);
  Serial.print("X2: ");
  Serial.print(rxData.joystick_x2);
  Serial.print(", Y2: ");
  Serial.println(rxData.joystick_y2);
  Serial.println("----------------------------");
}


// ----------------------------------------------------------------------------------------------


void setup() {
  Serial.begin(115200);

  pinMode(ENABLE_PIN, OUTPUT);

  // Prevent servo inrush current
  digitalWrite(ENABLE_PIN, HIGH);  // OE pin -> active low (disabled)
  
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(60);
  
  servoInit();
  delay(100);

  digitalWrite(ENABLE_PIN, LOW);   // OE pin -> active low (enabled)

  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataReceived));
}

void loop() {
  setServoSpeed();
  
  for (int i = 0; i < 4; i++) {
    servoAngles[i] -= servoSpeeds[i];
    if (servoAngles[i] > 180) { servoAngles[i] = 180; }
    if (servoAngles[i] < 0) { servoAngles[i] = 0; }
    moveServo(i, servoAngles[i]);
  }
  
  delay(30);
}
