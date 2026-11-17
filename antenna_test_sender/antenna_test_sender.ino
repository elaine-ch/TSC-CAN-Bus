// ANTENNA TEST - SENDER (the board without the antenna)
// Sends a numbered "ping" every PING_MS to the receiver and listens for its
// replies. Reports:
//   acked  = MAC-level delivery success (sender -> receiver link)
//   pongs  = replies that made it back (round trip)
//   RSSI@me    = signal strength of receiver's replies here (receiver TX side)
//   RSSI@them  = signal strength the receiver measured of my pings (receiver RX side)

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ---- PUT THE RECEIVER'S MAC HERE (it prints it on boot) ----
uint8_t receiverMac[] = {0x3C, 0x8A, 0x1F, 0xD3, 0x85, 0x48};

#define CHANNEL        1      // must match receiver
#define PING_MS        50     // 20 packets/second
#define REPORT_MS      2000
#define TX_POWER_QDBM  -1     // -1 = default (max). Else 8..84 in 0.25 dBm steps (e.g. 8 = 2 dBm)

typedef struct { uint32_t seq; } ping_t;
typedef struct { uint32_t seq; int8_t rssi; } pong_t;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
uint32_t seq = 0;
uint32_t wSent = 0, wAcked = 0, wPongs = 0;
int32_t  wRssiMe = 0, wRssiThem = 0;

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  portENTER_CRITICAL(&mux);
  if (status == ESP_NOW_SEND_SUCCESS) wAcked++;
  portEXIT_CRITICAL(&mux);
}

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len != sizeof(pong_t)) return;
  pong_t p;
  memcpy(&p, data, sizeof(p));
  portENTER_CRITICAL(&mux);
  wPongs++;
  wRssiMe += info->rx_ctrl->rssi;
  wRssiThem += p.rssi;
  portEXIT_CRITICAL(&mux);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  if (TX_POWER_QDBM > 0) esp_wifi_set_max_tx_power(TX_POWER_QDBM);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMac, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println();
  Serial.println("=== ANTENNA TEST SENDER ===");
  Serial.print("My MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  static uint32_t lastPing = 0, lastReport = 0;

  if (millis() - lastPing >= PING_MS) {
    lastPing = millis();
    ping_t p = { seq++ };
    if (esp_now_send(receiverMac, (uint8_t *)&p, sizeof(p)) == ESP_OK) {
      portENTER_CRITICAL(&mux); wSent++; portEXIT_CRITICAL(&mux);
    }
  }

  if (millis() - lastReport >= REPORT_MS) {
    lastReport = millis();
    uint32_t sent, acked, pongs; int32_t rMe, rThem;
    portENTER_CRITICAL(&mux);
    sent = wSent; acked = wAcked; pongs = wPongs; rMe = wRssiMe; rThem = wRssiThem;
    wSent = wAcked = wPongs = 0; wRssiMe = wRssiThem = 0;
    portEXIT_CRITICAL(&mux);

    Serial.printf("[%5lus] sent %3lu  acked %5.1f%%  pongs %5.1f%%", millis() / 1000, sent,
                  sent ? 100.0f * acked / sent : 0.0f, sent ? 100.0f * pongs / sent : 0.0f);
    if (pongs) Serial.printf("  RSSI@me %4ld  RSSI@them %4ld dBm\n", rMe / (int32_t)pongs, rThem / (int32_t)pongs);
    else       Serial.println("  (no replies)");
  }
}
