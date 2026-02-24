/************************************************************************************************

 * Organization  : LognCoding | LognSteam
 * Project       : Robotic Arm Part 1 - Controller
 * Created       : 2025-12-31
 * Author        : Kyu Rae Kim

 * MCU           : ESP32
 * Module        : WROOM-32
 * Dev Board     : Lolin D32

 * Description:
 *   This firmware implements real-time control of a 3-DoF robotic arm using two joysticks.
 *   Joystick analog inputs are transmitted wirelessly via ESP-NOW to control the arm's joints.
 *   With the battery permanently connected inside the controller case, the ESP is configured
 *   to be set to deep sleep mode when there are no joystick inputs and to be woken up upon any
 *   joystick push in order to conserve power while allowing for immediate control when needed.

************************************************************************************************/


#include <WiFi.h>
#include <esp_now.h>

#define JOYSTICK_X1_PIN 32
#define JOYSTICK_Y1_PIN 33
#define JOYSTICK_X2_PIN 39
#define JOYSTICK_Y2_PIN 34
#define JOYSTICK_SW1_PIN GPIO_NUM_2
#define JOYSTICK_SW2_PIN GPIO_NUM_0

// Deep sleep mode timeout in ms (set to 3 min)
#define INACTIVE_TIMEOUT 180000

unsigned long lastInputTime = 0;

typedef struct {
  int joystick_x1;
  int joystick_y1;
  int joystick_x2;
  int joystick_y2;
} joystick_data;

joystick_data txData;

// Must check & change peer's MAC address
/*
**CODE TO CHECK YOUR ESP MAC ADDRESS**

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
  delay(1000);
}
 */
uint8_t peerAddress[] = { 0x80, 0xF3, 0xDA, 0x90, 0x9C, 0xDC };
esp_now_peer_info_t peerInfo;


// ----------------------------------------------------------------------------------------------


// Function to update the most recent time when there was a joystick input
void updateActiveTime(joystick_data inputData) {
  int joystickInputs[4] =
  {
    inputData.joystick_x1,
    inputData.joystick_y1,
    inputData.joystick_x2,
    inputData.joystick_y2
  };

  for (int i = 0; i < 4; i++) {
    if ((joystickInputs[i] < 1850) | (joystickInputs[i] > 1970)) {
      lastInputTime = millis();
    }
  }
}

// Function to put ESP to deep sleep mode
void goToSleep() {
  if (millis() - lastInputTime > INACTIVE_TIMEOUT) {
    Serial.println("Going to sleep.....");
    esp_deep_sleep_start();
  }
}

// Function to check whether the data was successfully sent via ESP-NOW
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success\n" : "Delivery Failed\n");
}


// ----------------------------------------------------------------------------------------------


void setup() {
  Serial.begin(115200);

  pinMode(JOYSTICK_SW1_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK_SW2_PIN, INPUT_PULLUP);

  // Wake up on either joystick push
  esp_sleep_enable_ext0_wakeup(JOYSTICK_SW1_PIN, LOW);
  esp_sleep_enable_ext1_wakeup(1ULL << JOYSTICK_SW2_PIN, ESP_EXT1_WAKEUP_ALL_LOW);
  
  lastInputTime = millis();
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  int joystick_x1 = analogRead(JOYSTICK_X1_PIN);
  int joystick_y1 = analogRead(JOYSTICK_Y1_PIN);
  int joystick_x2 = analogRead(JOYSTICK_X2_PIN);
  int joystick_y2 = analogRead(JOYSTICK_Y2_PIN);

  txData.joystick_x1 = joystick_x1;
  txData.joystick_y1 = joystick_y1;
  txData.joystick_x2 = joystick_x2;
  txData.joystick_y2 = joystick_y2;

  esp_err_t result = esp_now_send(peerAddress, (uint8_t*) &txData, sizeof(txData));

  if (result == ESP_OK) {
    Serial.print("[SENT]\nX1: ");
    Serial.print(joystick_x1);
    Serial.print(", Y1: ");
    Serial.println(joystick_y1);
    Serial.print("X2: ");
    Serial.print(joystick_x2);
    Serial.print(", Y2: ");
    Serial.println(joystick_y2);
    Serial.println("----------------------------");
  }
  else {
    Serial.println("Error sending the data");
  }

  updateActiveTime(txData);
  goToSleep();
  
  delay(50);
}
