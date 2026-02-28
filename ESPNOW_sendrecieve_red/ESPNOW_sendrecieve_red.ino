//RED BOARD ESP 32 WROOM 32E

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE OTHER BOARD'S MAC ADDRESS
uint8_t peerAddress[] = {0x31, 0x94, 0x54, 0x39, 0x1C, 0x28};

typedef struct struct_message {
  char a[32];
  int b;
} struct_message;

struct_message myData;      // Data to send
struct_message incomingData; // Data received

// Callback when data is sent
void OnDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Callback when data is received
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));
  Serial.print("Received Character: ");
  Serial.println(incomingData.a); // Prints the char/string received
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) return;

  // Register BOTH callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register the peer (the other ESP32)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  // Send data every 5 seconds
  strcpy(myData.a, "William!");
  myData.b = 100;
  esp_now_send(peerAddress, (uint8_t *) &myData, sizeof(myData));
  
  delay(5000);
}

