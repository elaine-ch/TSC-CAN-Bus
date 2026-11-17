// ANTENNA TEST - RECEIVER (the board with the antenna)
// Listens for numbered "ping" packets from antenna_test_sender, measures
// packet loss + RSSI, and replies to every ping so the sender can measure
// this board's transmit side too.
//
// On boot it prints its MAC address -> put that into antenna_test_sender.

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define CHANNEL        1      // must match sender
#define REPORT_MS      2000   // print a summary this often
#define TX_POWER_QDBM  -1     // -1 = default (max). Else 8..84 in 0.25 dBm steps (e.g. 8 = 2 dBm)

typedef struct { uint32_t seq; } ping_t;
typedef struct { uint32_t seq; int8_t rssi; } pong_t;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// stats for the current report window (written in callback, read in loop)
bool     started = false;
uint32_t wFirst = 0, wLast = 0, wCount = 0;
int32_t  wRssiSum = 0;
int8_t   wMin = 127, wMax = -128;
uint32_t totalRx = 0, totalExpected = 0;

// pending reply (sending from inside the recv callback is discouraged)
volatile bool replyPending = false;
pong_t  reply;
uint8_t senderMac[6];

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len != sizeof(ping_t)) return;
  ping_t p;
  memcpy(&p, data, sizeof(p));
  int8_t rssi = info->rx_ctrl->rssi;

  portENTER_CRITICAL(&mux);
  if (!started || p.seq + 1 < wFirst) {   // first packet, or sender rebooted
    started = true;
    wFirst = p.seq; wCount = 0; wRssiSum = 0; wMin = 127; wMax = -128;
  }
  wCount++;
  wLast = p.seq;
  wRssiSum += rssi;
  if (rssi < wMin) wMin = rssi;
  if (rssi > wMax) wMax = rssi;

  reply.seq = p.seq;
  reply.rssi = rssi;
  memcpy(senderMac, info->src_addr, 6);
  replyPending = true;
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
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println();
  Serial.println("=== ANTENNA TEST RECEIVER ===");
  Serial.print("My MAC (put this in the sender): ");
  Serial.println(WiFi.macAddress());
  Serial.println("Waiting for pings...");
}

void loop() {
  // send reply outside the callback
  if (replyPending) {
    pong_t r; uint8_t mac[6];
    portENTER_CRITICAL(&mux);
    r = reply; memcpy(mac, senderMac, 6); replyPending = false;
    portEXIT_CRITICAL(&mux);

    if (!esp_now_is_peer_exist(mac)) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, mac, 6);
      peer.channel = CHANNEL;
      peer.encrypt = false;
      esp_now_add_peer(&peer);
    }
    esp_now_send(mac, (uint8_t *)&r, sizeof(r));
  }

  static uint32_t lastReport = 0;
  if (millis() - lastReport >= REPORT_MS) {
    lastReport = millis();

    uint32_t count, expected; int32_t sum; int8_t mn, mx;
    portENTER_CRITICAL(&mux);
    count = wCount; sum = wRssiSum; mn = wMin; mx = wMax;
    expected = (started && wLast + 1 >= wFirst) ? (wLast - wFirst + 1) : 0;
    if (count == 0) expected = 0;
    // start next window right after the last seq we saw
    wFirst = wLast + 1; wCount = 0; wRssiSum = 0; wMin = 127; wMax = -128;
    portEXIT_CRITICAL(&mux);

    Serial.printf("[%5lus] ", millis() / 1000);
    if (count == 0) {
      Serial.println("NO PACKETS");
      return;
    }
    totalRx += count; totalExpected += expected;
    float loss = 100.0f * (expected - count) / expected;
    float totalLoss = 100.0f * (totalExpected - totalRx) / totalExpected;
    Serial.printf("rx %3lu/%3lu  loss %5.1f%%  RSSI avg %4d  min %4d  max %4d dBm   | total loss %5.1f%%\n",
                  count, expected, loss, (int)(sum / (int32_t)count), mn, mx, totalLoss);
  }
}
