#include <esp_now.h>
#include <WiFi.h>
#include "time.h"
#define SERIAL_SIZE_RX  500

//Constant
uint8_t broadcastAddress[] = {0x78, 0x21, 0x84, 0x8E, 0x1A, 0x7C};
bool receivedDataFromESP2 = false;
const int size_message = 249;

// Structure to send data ESP
typedef struct comunity_esp {
  byte payload[size_message];
  byte len;
} comunity_esp;
comunity_esp comu_esp;

// ESPNOW protocol
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&comu_esp, incomingData, sizeof(comu_esp));
    for (int i = 0; i < comu_esp.len; i++){
      Serial.print(comu_esp.payload[i]);
      Serial.print(" ");
    }
    Serial.println("");
    
}

void setup() {
  Serial.setRxBufferSize(SERIAL_SIZE_RX);
  Serial.begin(230400);
  WiFi.mode(WIFI_STA);
  
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop(){
}
