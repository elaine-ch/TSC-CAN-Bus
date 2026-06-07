/*
  Orion BMS 2 one-way wireless CAN monitor - laptop side

  Data path:
    Orion BMS -> BMS-side ESP32 -> ESP-NOW -> this ESP32-S3 -> Serial Monitor

  This sketch receives raw CAN-frame packets over ESP-NOW and prints them. It
  does not initialize the laptop-side MCP2515, transmit CAN frames, or send
  anything back to the BMS-side node.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <esp_mac.h>
#include <mcp2515.h>  // Supplies struct can_frame flag/mask definitions.

// Use the ESP32-S3 native USB JTAG/serial port (/dev/cu.usbmodem... on macOS).
#if defined(CONFIG_IDF_TARGET_ESP32S3) && !ARDUINO_USB_CDC_ON_BOOT
#error "Set Arduino IDE: Tools > USB CDC On Boot > Enabled, then compile and upload again."
#endif

// ========================= USER CONFIGURATION =========================

// Must match the BMS-side sketch.
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 1;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t SERIAL_ATTACH_TIMEOUT_MS = 5000;
constexpr uint32_t MAC_PRINT_INTERVAL_MS = 10000;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 2000;

// Printing every frame is useful for inspection but can become the bottleneck
// on a busy CAN bus. Set false to print only periodic counters.
constexpr bool PRINT_EVERY_CAN_FRAME = true;

// Fixed-size callback queue; no dynamic allocation is used.
constexpr size_t RX_QUEUE_SIZE = 128;
constexpr size_t MAX_PACKETS_PER_LOOP = 32;

// ======================= END USER CONFIGURATION =======================

constexpr uint8_t PACKET_MAGIC = 0xA7;
constexpr uint8_t PACKET_VERSION = 3;
constexpr uint8_t SOURCE_NODE_BMS = 1;
constexpr canid_t FORWARDED_CAN_FLAGS = CAN_EFF_FLAG | CAN_RTR_FLAG;

struct __attribute__((packed)) WirelessCanPacket {
  uint8_t magic;
  uint8_t version;
  uint8_t sourceNode;
  uint8_t dlc;
  uint32_t sequence;
  uint32_t senderMillis;
  uint32_t canIdWithFlags;
  uint8_t data[8];
};

static_assert(sizeof(WirelessCanPacket) <= ESP_NOW_MAX_DATA_LEN,
              "Wireless CAN packet is too large for ESP-NOW");

WirelessCanPacket rxQueue[RX_QUEUE_SIZE];
volatile size_t rxHead = 0;
volatile size_t rxTail = 0;
portMUX_TYPE rxQueueMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t espNowPacketsReceived = 0;
volatile uint32_t badSizePackets = 0;
volatile uint32_t badHeaderPackets = 0;
volatile uint32_t badFramePackets = 0;
volatile uint32_t queueOverflowPackets = 0;
uint32_t framesPrinted = 0;
uint32_t sequenceDrops = 0;
uint32_t sequenceOutOfOrder = 0;
uint32_t lastSequence = 0;
bool haveLastSequence = false;
uint32_t lastDebugPrintMs = 0;
uint32_t lastMacPrintMs = 0;

void haltWithMessage(const char *message) {
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

void printMacAddress(const uint8_t *mac) {
  for (uint8_t i = 0; i < 6; ++i) {
    if (i != 0) {
      Serial.print(':');
    }
    if (mac[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(mac[i], HEX);
  }
}

void printLocalMacAddress() {
  uint8_t stationMac[6];
  if (esp_read_mac(stationMac, ESP_MAC_WIFI_STA) != ESP_OK) {
    Serial.println("ERROR: Could not read laptop-side Wi-Fi station MAC.");
    return;
  }

  Serial.print("Laptop-side Wi-Fi MAC: ");
  printMacAddress(stationMac);
  Serial.println();
  Serial.println("Use this address as LAPTOP_SIDE_MAC in the BMS-side sketch.");
}

bool packetHasValidCanFrame(const WirelessCanPacket &packet) {
  const canid_t canId = packet.canIdWithFlags;

  if (packet.dlc > CAN_MAX_DLC || (canId & CAN_ERR_FLAG) != 0) {
    return false;
  }

  if ((canId & ~(CAN_EFF_MASK | FORWARDED_CAN_FLAGS)) != 0) {
    return false;
  }

  if ((canId & CAN_EFF_FLAG) == 0 &&
      (canId & CAN_EFF_MASK) > CAN_SFF_MASK) {
    return false;
  }

  return true;
}

void enqueuePacket(const uint8_t *data, int length) {
  ++espNowPacketsReceived;

  if (length != static_cast<int>(sizeof(WirelessCanPacket))) {
    ++badSizePackets;
    return;
  }

  WirelessCanPacket packet;
  memcpy(&packet, data, sizeof(packet));

  if (packet.magic != PACKET_MAGIC ||
      packet.version != PACKET_VERSION ||
      packet.sourceNode != SOURCE_NODE_BMS) {
    ++badHeaderPackets;
    return;
  }

  if (!packetHasValidCanFrame(packet)) {
    ++badFramePackets;
    return;
  }

  portENTER_CRITICAL(&rxQueueMux);
  const size_t nextHead = (rxHead + 1) % RX_QUEUE_SIZE;
  if (nextHead == rxTail) {
    ++queueOverflowPackets;
  } else {
    rxQueue[rxHead] = packet;
    rxHead = nextHead;
  }
  portEXIT_CRITICAL(&rxQueueMux);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onEspNowReceive(const esp_now_recv_info_t *info,
                     const uint8_t *data,
                     int length) {
  (void)info;
  enqueuePacket(data, length);
}
#else
void onEspNowReceive(const uint8_t *macAddress,
                     const uint8_t *data,
                     int length) {
  (void)macAddress;
  enqueuePacket(data, length);
}
#endif

bool dequeuePacket(WirelessCanPacket &packet) {
  bool available = false;

  portENTER_CRITICAL(&rxQueueMux);
  if (rxTail != rxHead) {
    packet = rxQueue[rxTail];
    rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
    available = true;
  }
  portEXIT_CRITICAL(&rxQueueMux);

  return available;
}

void trackSequence(uint32_t sequence) {
  if (!haveLastSequence) {
    haveLastSequence = true;
    lastSequence = sequence;
    return;
  }

  const uint32_t expected = lastSequence + 1;
  const int32_t difference = static_cast<int32_t>(sequence - expected);

  if (difference == 0) {
    lastSequence = sequence;
  } else if (difference > 0) {
    sequenceDrops += static_cast<uint32_t>(difference);
    lastSequence = sequence;
  } else {
    ++sequenceOutOfOrder;
  }
}

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void printCanPacket(const WirelessCanPacket &packet) {
  const bool extended = (packet.canIdWithFlags & CAN_EFF_FLAG) != 0;
  const bool remote = (packet.canIdWithFlags & CAN_RTR_FLAG) != 0;
  const canid_t identifier =
      packet.canIdWithFlags & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);

  Serial.print("RX ms=");
  Serial.print(millis());
  Serial.print(" source_ms=");
  Serial.print(packet.senderMillis);
  Serial.print(" seq=");
  Serial.print(packet.sequence);
  Serial.print(" ID=");

  const uint8_t idDigits = extended ? 8 : 3;
  for (int8_t shift = (idDigits - 1) * 4; shift >= 0; shift -= 4) {
    Serial.print(static_cast<uint8_t>((identifier >> shift) & 0x0F), HEX);
  }

  Serial.print(extended ? " EXT" : " STD");
  if (remote) {
    Serial.print(" RTR");
  }
  Serial.print(" DLC=");
  Serial.print(packet.dlc);
  Serial.print(" DATA=");

  if (remote) {
    Serial.print("<remote request>");
  } else if (packet.dlc == 0) {
    Serial.print("<empty>");
  } else {
    for (uint8_t i = 0; i < packet.dlc; ++i) {
      if (i != 0) {
        Serial.print(' ');
      }
      printHexByte(packet.data[i]);
    }
  }

  Serial.println();
  ++framesPrinted;
}

void initializeEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    haltWithMessage("ERROR: Could not set ESP-NOW Wi-Fi channel.");
  }

  printLocalMacAddress();

  if (esp_now_init() != ESP_OK) {
    haltWithMessage("ERROR: ESP-NOW initialization failed.");
  }

  if (esp_now_register_recv_cb(onEspNowReceive) != ESP_OK) {
    haltWithMessage("ERROR: ESP-NOW receive callback registration failed.");
  }
}

void printStatistics() {
  Serial.print("[Laptop] ESP-NOW RX=");
  Serial.print(espNowPacketsReceived);
  Serial.print(" printed=");
  Serial.print(framesPrinted);
  Serial.print(" sequence-drops=");
  Serial.print(sequenceDrops);
  Serial.print(" out-of-order=");
  Serial.print(sequenceOutOfOrder);
  Serial.print(" queue-overflow=");
  Serial.print(queueOverflowPackets);
  Serial.print(" bad(size/header/frame)=");
  Serial.print(badSizePackets);
  Serial.print('/');
  Serial.print(badHeaderPackets);
  Serial.print('/');
  Serial.println(badFramePackets);
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  const uint32_t serialWaitStarted = millis();
  while (!Serial && millis() - serialWaitStarted < SERIAL_ATTACH_TIMEOUT_MS) {
    delay(10);
  }

  Serial.println();
  Serial.println("Orion BMS 2 one-way wireless CAN monitor - laptop side");
  initializeEspNow();
  Serial.println("Monitor ready. Waiting for raw Orion CAN frames.");
}

void loop() {
  WirelessCanPacket packet;
  size_t processed = 0;

  while (processed < MAX_PACKETS_PER_LOOP && dequeuePacket(packet)) {
    trackSequence(packet.sequence);
    if (PRINT_EVERY_CAN_FRAME) {
      printCanPacket(packet);
    }
    ++processed;
  }

  const uint32_t now = millis();
  if (now - lastDebugPrintMs >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMs = now;
    printStatistics();
  }

  if (now - lastMacPrintMs >= MAC_PRINT_INTERVAL_MS) {
    lastMacPrintMs = now;
    printLocalMacAddress();
  }

  delay(0);
}
