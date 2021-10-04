/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

int ena = 32;
int in1 = 26;
int in2 = 25;

int enb = 33;
int in3 = 14;
int in4 = 27;

int enl = 21;
int inla = 23;
int inlb = 22;

const int resolution = 9;
const int freq = 1000;
const int pwmChannel_A = 0;
const int pwmChannel_B = 1;
const int pwmChannel_L = 2;


void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //    Serial.println("Error initializing ESP-NOW");
    return;
  }

  ledcSetup(pwmChannel_A, freq, resolution);
  ledcSetup(pwmChannel_B, freq, resolution);
  ledcSetup(pwmChannel_L, freq, resolution);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(inla, OUTPUT);
  pinMode(inlb, OUTPUT);

  ledcAttachPin(ena, pwmChannel_A);
  ledcAttachPin(enb, pwmChannel_B);
  ledcAttachPin(enl, pwmChannel_L);

  Serial.println("init done, grantRecveiverESP32");
  //Use the printed mac in the controller
  Serial.println(WiFi.macAddress());
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

// Structure example to receive data
// Must match the sender structure
typedef struct move_message {
  int a_L;
  int a_R;
  int b_L;
  int b_R;
  int rise;
  int lower;
} move_message;

// Create a struct_message
move_message moveData;

int lastMessage = 0;
int failSafeDelay = 150;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  lastMessage = millis();
  memcpy(&moveData, incomingData, sizeof(moveData));

  Serial.println(moveData.a_L);
  Serial.println(moveData.a_R);
  Serial.println(moveData.b_L);
  Serial.println(moveData.b_R);
  Serial.println(moveData.rise);
  Serial.println(moveData.lower);
  Serial.println("          ");


  //move
  if (moveData.a_L > moveData.a_R) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel_A, moveData.a_L * 4);
  } else if (moveData.a_L < moveData.a_R) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel_A, moveData.a_R * 4);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  if (moveData.b_L > moveData.b_R) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    ledcWrite(pwmChannel_B, moveData.b_L * 4);
  } else if (moveData.b_L < moveData.b_R) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    ledcWrite(pwmChannel_B, moveData.b_R * 4);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  
  if (moveData.rise > moveData.lower) {
    digitalWrite(inla, HIGH);
    digitalWrite(inlb, LOW);
    ledcWrite(pwmChannel_L, moveData.rise * 4);
  } else if (moveData.rise < moveData.lower) {
    digitalWrite(inla, LOW);
    digitalWrite(inlb, HIGH);
    ledcWrite(pwmChannel_L, moveData.lower * 4);
  } else {
    digitalWrite(inla, LOW);
    digitalWrite(inlb, LOW);
  }
}



void loop() {
  if (millis() - lastMessage > failSafeDelay) {
    //timeout stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    digitalWrite(inla, LOW);
    digitalWrite(inlb, LOW);
  }
}
