#include <esp_now.h>
#include <WiFi.h>

//ES32 controller buttons
//rev left
int button1 = 13;
//rev right
int button2 = 14;
//fwd left
int button3 = 15;
//fwd right
int button4 = 16;


//two robots, toggle switch position on controller boot determines which one will be controlled.
int choice = 27;

int lower = 17;
int rise = 4;

//bias left/right (try to compensate for any drive train efficiency differences)
int pot1 = 32;
//power
int pot2 = 33;

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t grant[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t boringbot[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


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

move_message moveData;


//Receiver set to 8 bit PWM resolution, 0...255

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //  Serial.print("\r\nLast Packet Send Status:\t");
  //  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

uint8_t broadcastAddress[6];

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  //0 - 511
  analogReadResolution(9);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  pinMode(choice, INPUT_PULLUP);
  pinMode(lower, INPUT_PULLUP);
  pinMode(rise, INPUT_PULLUP);


  //process the choice switch state for which robot to control
  if (digitalRead(choice) == LOW) {
    for (int i = 0 ; i < 6; i++) {
      broadcastAddress[i] = grant[i];
    }
  } else {
    for (int i = 0 ; i < 6; i++) {
      broadcastAddress[i] = boringbot[i];
    }
  }

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println("Controller");
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //    Serial.println("Failed to add peer");
    return;
  }
}

int a_L_fact = 0;
int a_R_fact = 0;
int b_L_fact = 0;
int b_R_fact = 0;
int rise_fact = 0;
int lower_fact = 0;

//control with 10k potentiometer
int magnitude = 64;
float bias = 0.5;

void loop() {
  magnitude = analogRead(pot1) / 2;
  bias = analogRead(pot2) / 511.0;

  bool send = false;


  //I should have put this into a nice neat function
  if (digitalRead(button3) == LOW) {
    a_L_fact = 1;
    send = true;
  } else {
    if (a_L_fact == 1) {
      send = true;
    }
    a_L_fact = 0;
  }

  if (digitalRead(button1) == LOW) {
    a_R_fact = 1;
    send = true;
  } else {
    if (a_R_fact == 1) {
      send = true;
    }
    a_R_fact = 0;
  }

  if (digitalRead(button4) == LOW) {
    b_L_fact = 1;
    send = true;
  } else {
    if (b_L_fact == 1) {
      send = true;
    }
    b_L_fact = 0;
  }

  if (digitalRead(button2) == LOW) {
    b_R_fact = 1;
    send = true;
  } else {
    if (b_R_fact == 1) {
      send = true;
    }
    b_R_fact = 0;
  }
  
  if (digitalRead(rise) == LOW) {
    rise_fact = 1;
    send = true;
  } else {
    if (rise_fact == 1) {
      send = true;
    }
    rise_fact = 0;
  }

    if (digitalRead(lower) == LOW) {
    lower_fact = 1;
    send = true;
  } else {
    if (lower_fact == 1) {
      send = true;
    }
    lower_fact = 0;
  }

  // Send message via ESP-NOW
  moveData.a_L = a_L_fact * magnitude * 0.5;
  moveData.a_R = a_R_fact * magnitude * 0.5;
  moveData.b_L = b_L_fact * magnitude * 0.5;
  moveData.b_R = b_R_fact * magnitude * 0.5;
  moveData.rise = rise_fact * magnitude;
  moveData.lower = lower_fact * magnitude;

  if (send) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &moveData, sizeof(moveData));
    Serial.print(moveData.a_L);
    Serial.print(" ");
    Serial.print(moveData.a_R);
    Serial.print(" ");
    Serial.print(moveData.b_L);
    Serial.print(" ");
    Serial.print(moveData.b_R);
    Serial.print(" ");
    Serial.print(moveData.rise);
    Serial.print(" ");
    Serial.print(moveData.lower);
    Serial.print(" ");
    Serial.println(digitalRead(choice));

    for (int i = 0; i < 6; i++) {
      Serial.print(broadcastAddress[i]);
      Serial.print(" ");
    }
    Serial.println("");
  }

  delay(20);
}
