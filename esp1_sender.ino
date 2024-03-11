#include <esp_now.h>
#include <WiFi.h>
#include "time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#define SERIAL_SIZE_RX  500

// Constant
const int size_serial = 500;
const int size_message = 249;
uint8_t broadcastAddress[] = {0x0C, 0xB8, 0x15, 0xF3, 0x33, 0x30};
int min_size_message = 2;

// Structure to send data ESP
typedef struct comunity_esp {
  byte payload[size_message];
  byte len;
} comunity_esp;
comunity_esp comu_esp;

// Structure to comunity task
typedef struct comunity_task {
  byte payload[size_serial];
  bool dataReadyToSend = false;
  SemaphoreHandle_t dataSemaphore;
  int len;
} comunity_task;
comunity_task comu_task;

// ESPNOW protocol
esp_now_peer_info_t peerInfo;

// Callback ESPNOW protocol
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
}

// Task to read data from Serial and write
void readSerialTask(void *parameter) {
  while (true) {
    // timeout in 1 sec to send ESP
    
    int count_byte_available = Serial.available();
    if (count_byte_available > min_size_message) {
      Serial.readBytes(comu_task.payload, count_byte_available);
      comu_task.len = count_byte_available;
      
      xSemaphoreGive(comu_task.dataSemaphore);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    comu_task.dataReadyToSend = true;
  // Check Serial every 50ms
  }
}

// Task to process and send data
void sendDataTask(void *parameter) {
  while (true) {
    if (xSemaphoreTake(comu_task.dataSemaphore, portMAX_DELAY) == pdTRUE) { // Wait for data
      if (comu_task.dataReadyToSend) { // Check if data is ready to send
        if (comu_task.len > size_message){
          for (int i = 0; i < size_serial; i += size_message) {
          for (int j = 0; j < size_message; j++) {
            comu_esp.payload[j] = comu_task.payload[i + j];
          }     
          // Get actual length  when send to ESP
          if (comu_task.len >= size_message) {
            comu_esp.len = size_message;
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &comu_esp, sizeof(comu_esp));
            comu_task.len = comu_task.len - size_message;
          } 
          else if(comu_task.len > 0 && comu_task.len < size_message) {
            comu_esp.len = (byte)comu_task.len;
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &comu_esp, sizeof(comu_esp));
            comu_task.len = 0;
          }
          else if(comu_task.len ==0){
            continue;
          }
          }
        }
        else if(comu_task.len <= size_message){
          for (int i = 0; i < comu_task.len; i++){
            comu_esp.payload[i] = comu_task.payload[i];
          }
          comu_esp.len = (byte)comu_task.len;
          esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &comu_esp, sizeof(comu_esp));
        }
        
        comu_task.dataReadyToSend = false;
      }
    }
  }
}

void setup() {
  Serial.setRxBufferSize(SERIAL_SIZE_RX);
  Serial.begin(230400);
  WiFi.mode(WIFI_STA);
//  Serial.setTimeout(50);


  comu_task.dataSemaphore = xSemaphoreCreateBinary(); // Create semaphore

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // Create tasks
  xTaskCreatePinnedToCore(
    readSerialTask,   // Function to implement the task
    "ReadSerialTask", // Name of the task
    4096,             // Stack size in words
    NULL,             // Task input parameter
    1,                // Priority of the task
    NULL,             // Task handle
    1);               // Core to run the task (core 1)

  xTaskCreatePinnedToCore(
    sendDataTask,   // Function to implement the task
    "SendDataTask", // Name of the task
    4096,           // Stack size in words
    NULL,           // Task input parameter
    1,              // Priority of the task
    NULL,           // Task handle
    0);             // Core to run the task (core 0)
}

void loop() {
  delay(10000);
}
