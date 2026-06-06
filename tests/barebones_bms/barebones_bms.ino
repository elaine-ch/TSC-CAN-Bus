/*
  Orion BMS 2 read-only wireless CAN bridge - BMS side

  Data path:
    Orion BMS CAN -> MCP2515 -> ESP32-WROOM-32UE -> ESP-NOW

  This sketch forwards complete CAN 2.0 frames. It does not decode Orion data
  and it never receives or forwards CAN traffic toward the BMS.

  IMPORTANT:
  - This is a message-level bridge, not a physical wireless CAN cable.
  - CAN arbitration, ACKs, retries, and error handling remain local to each
    wired CAN segment.
  - Bench-test with non-critical monitoring only.
  - Do not use this bridge for firmware updates, profile programming,
    charger/load control, or any safety-critical function.

  Required library:
    AutoWP MCP2515: https://github.com/autowp/arduino-mcp2515
*/

#include <SPI.h>
#include <mcp2515.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>

// ========================= USER CONFIGURATION =========================

// These are common VSPI pins for a classic ESP32-WROOM board. Verify the
// pinout of the particular ESP32-WROOM-32UE carrier board being used.
constexpr int SPI_SCK_PIN = 18;
constexpr int SPI_MISO_PIN = 19;
constexpr int SPI_MOSI_PIN = 23;
constexpr int MCP2515_CS_PIN = 5;

// Optional MCP2515 INT output. This first version polls readMessage(), so the
// pin may be left disconnected and set to -1.
constexpr int MCP2515_INT_PIN = -1;

// Match the Orion BMS CAN interface profile. Orion BMS 2 supports:
//   CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS
constexpr CAN_SPEED CAN_BITRATE = CAN_500KBPS;

// Many HW-184 boards are sold with either an 8 MHz or 16 MHz crystal.
// Read the marking on the metal crystal and select MCP_8MHZ or MCP_16MHZ.
// A wrong setting prevents reliable CAN communication.
constexpr CAN_CLOCK MCP2515_CLOCK = MCP_8MHZ;

// Both ESP-NOW nodes must use the same 2.4 GHz Wi-Fi channel.
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 1;

// Upload the laptop-side sketch first. Copy the MAC address printed in its
// Serial Monitor and replace this example address.
const uint8_t LAPTOP_SIDE_PEER_MAC[6] = {
  0x48, 0x27, 0xE2, 0x82, 0x97, 0x44
};

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 2000;

// ======================= END USER CONFIGURATION =======================

MCP2515 mcp2515(MCP2515_CS_PIN);

constexpr uint8_t PACKET_MAGIC = 0xA7;
constexpr uint8_t PACKET_VERSION = 1;
constexpr uint8_t FLAG_EXTENDED = 0x01;
constexpr uint8_t FLAG_RTR = 0x02;

struct __attribute__((packed)) WirelessCanPacket {
  uint8_t magic;
  uint8_t version;
  uint8_t flags;
  uint8_t dlc;
  uint32_t sequence;
  uint32_t sourceMillis;
  uint32_t canId;
  uint8_t data[8];
};

static_assert(sizeof(WirelessCanPacket) <= ESP_NOW_MAX_DATA_LEN,
              "Wireless CAN packet is too large for ESP-NOW");

uint32_t nextSequence = 0;
uint32_t canFramesReceived = 0;
uint32_t espNowSendQueued = 0;
uint32_t espNowImmediateFailures = 0;
volatile uint32_t espNowDeliverySuccesses = 0;
volatile uint32_t espNowDeliveryFailures = 0;
uint32_t unsupportedErrorFrames = 0;
uint32_t lastDebugPrintMs = 0;

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

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void onEspNowSent(const esp_now_send_info_t *info,
                  esp_now_send_status_t status) {
  (void)info;
#else
void onEspNowSent(const uint8_t *macAddress,
                  esp_now_send_status_t status) {
  (void)macAddress;
#endif
  if (status == ESP_NOW_SEND_SUCCESS) {
    ++espNowDeliverySuccesses;
  } else {
    ++espNowDeliveryFailures;
  }
}

void initializeCan() {
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, MCP2515_CS_PIN);
  pinMode(MCP2515_CS_PIN, OUTPUT);

  if (MCP2515_INT_PIN >= 0) {
    pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  }

  mcp2515.reset();
  if (mcp2515.setBitrate(CAN_BITRATE, MCP2515_CLOCK) != MCP2515::ERROR_OK) {
    haltWithMessage("ERROR: MCP2515 bitrate setup failed. Check wiring and crystal selection.");
  }

  // Normal mode participates in CAN ACK handling. Listen-only mode would avoid
  // ACKing frames, but that can be undesirable when this is the other active
  // node on a bench bus. This bridge never calls sendMessage() on the BMS side.
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    haltWithMessage("ERROR: MCP2515 could not enter normal mode.");
  }

  Serial.println("MCP2515 initialized in normal mode.");
}

void initializeEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    haltWithMessage("ERROR: Could not set ESP-NOW Wi-Fi channel.");
  }

  Serial.print("BMS-side Wi-Fi MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    haltWithMessage("ERROR: ESP-NOW initialization failed.");
  }

  if (esp_now_register_send_cb(onEspNowSent) != ESP_OK) {
    haltWithMessage("ERROR: ESP-NOW send callback registration failed.");
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, LAPTOP_SIDE_PEER_MAC, sizeof(LAPTOP_SIDE_PEER_MAC));
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    haltWithMessage("ERROR: Could not add laptop-side ESP-NOW peer.");
  }

  Serial.print("ESP-NOW peer: ");
  printMacAddress(LAPTOP_SIDE_PEER_MAC);
  Serial.println();
}

void forwardCanFrame(const struct can_frame &frame) {
  // MCP2515 error frames are local controller diagnostics, not bus data frames.
  // Do not reproduce them on the remote CAN segment.
  if ((frame.can_id & CAN_ERR_FLAG) != 0) {
    ++unsupportedErrorFrames;
    return;
  }

  WirelessCanPacket packet = {};
  packet.magic = PACKET_MAGIC;
  packet.version = PACKET_VERSION;
  packet.flags = 0;
  packet.dlc = min<uint8_t>(frame.can_dlc, 8);
  packet.sequence = nextSequence++;
  packet.sourceMillis = millis();

  if ((frame.can_id & CAN_EFF_FLAG) != 0) {
    packet.flags |= FLAG_EXTENDED;
    packet.canId = frame.can_id & CAN_EFF_MASK;
  } else {
    packet.canId = frame.can_id & CAN_SFF_MASK;
  }

  if ((frame.can_id & CAN_RTR_FLAG) != 0) {
    packet.flags |= FLAG_RTR;
  } else {
    memcpy(packet.data, frame.data, packet.dlc);
  }

  const esp_err_t result =
      esp_now_send(LAPTOP_SIDE_PEER_MAC,
                   reinterpret_cast<const uint8_t *>(&packet),
                   sizeof(packet));

  if (result == ESP_OK) {
    ++espNowSendQueued;
  } else {
    ++espNowImmediateFailures;
  }
}

void printStatistics() {
  Serial.print("[BMS side] CAN RX=");
  Serial.print(canFramesReceived);
  Serial.print(" | ESP-NOW queued=");
  Serial.print(espNowSendQueued);
  Serial.print(" | delivered=");
  Serial.print(espNowDeliverySuccesses);
  Serial.print(" | immediate fail=");
  Serial.print(espNowImmediateFailures);
  Serial.print(" | delivery fail=");
  Serial.print(espNowDeliveryFailures);
  Serial.print(" | skipped error frames=");
  Serial.print(unsupportedErrorFrames);
  Serial.print(" | next sequence=");
  Serial.println(nextSequence);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("Orion BMS 2 read-only wireless CAN bridge - BMS side");
  initializeCan();
  initializeEspNow();
  Serial.println("Bridge ready. Waiting for CAN frames.");
}

void loop() {
  struct can_frame frame;

  // Drain all currently buffered MCP2515 frames without adding a blocking delay.
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    ++canFramesReceived;
    forwardCanFrame(frame);
  }

  const uint32_t now = millis();
  if (now - lastDebugPrintMs >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMs = now;
    printStatistics();
  }

  // Yield to the Wi-Fi task while keeping CAN polling responsive.
  delay(0);
}

/*
  BMS-side wiring:

  ESP32                 HW-184 MCP2515
  SPI_SCK_PIN    <----> SCK
  SPI_MISO_PIN   <----> SO / MISO
  SPI_MOSI_PIN   <----> SI / MOSI
  MCP2515_CS_PIN <----> CS
  optional INT   <----> INT
  GND            <----> GND

  HW-184 CANH/CANL connect to the selected Orion BMS CANH/CANL interface.
  Use twisted-pair CAN wiring. Unless galvanic isolation is provided, the
  ESP32/HW-184 low-voltage ground must share the BMS CAN-side reference ground.

  WARNING: Many HW-184 modules are designed for 5 V power and may expose 5 V
  logic on SPI pins. ESP32 GPIO is 3.3 V only and is not 5 V tolerant. Verify
  the exact module schematic and transceiver/controller parts. Use suitable
  level shifting or a known 3.3 V-safe CAN module when required.

  CAN termination must be correct. A physical CAN bus needs exactly two 120 ohm
  terminators, one at each end. A standard Orion BMS 2 normally has termination
  on CAN1 and no termination on CAN2, but verify the unit's ordered options.
  With all bus power removed, CANH-to-CANL should measure about 60 ohms when
  exactly two 120 ohm terminators are installed.
*/
