/*
  Orion BMS 2 one-way wireless CAN monitor - BMS side

  Data path:
    Orion BMS CAN -> HW-184 MCP2515 -> ESP32-WROOM-32UE
    -> ESP-NOW -> laptop-side ESP32-S3 -> Serial Monitor

  This sketch only reads the Orion CAN segment and sends complete CAN frames
  wirelessly. It never accepts wireless commands and never transmits CAN frames.

  This is a message-level monitor, not a physical wireless CAN cable. CAN ACK
  and error handling remain local to the Orion CAN segment. Do not use this
  monitor as part of charger/load control or any safety-critical function.

  Required library:
    AutoWP MCP2515: https://github.com/autowp/arduino-mcp2515
*/

#include <SPI.h>
#include <mcp2515.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <esp_mac.h>

// ========================= USER CONFIGURATION =========================

constexpr int SPI_SCK_PIN = 18;
constexpr int SPI_MISO_PIN = 19;
constexpr int SPI_MOSI_PIN = 23;
constexpr int MCP2515_CS_PIN = 5;

// This version polls readMessage(); INT may be left disconnected.
constexpr int MCP2515_INT_PIN = -1;

// Match the configured Orion CAN interface:
//   CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS
constexpr CAN_SPEED CAN_BITRATE = CAN_500KBPS;

// HW-184 modules may have an 8 MHz or 16 MHz crystal. Read the marking and
// select MCP_8MHZ or MCP_16MHZ. The wrong value prevents reliable CAN.
constexpr CAN_CLOCK MCP2515_CLOCK = MCP_8MHZ;

// Must match the laptop-side sketch.
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 1;

// Laptop-side ESP32-S3 station MAC.
const uint8_t LAPTOP_SIDE_MAC[6] = {
  0x48, 0x27, 0xE2, 0x82, 0x97, 0x44
};

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 2000;
constexpr uint32_t MAC_PRINT_INTERVAL_MS = 10000;
constexpr size_t MAX_CAN_FRAMES_PER_LOOP = 32;

// Raw per-frame logging confirms exactly what the MCP2515 receives. Serial
// output can become a throughput bottleneck on a busy bus; set false after
// validating CAN reception.
constexpr bool PRINT_EVERY_RECEIVED_CAN_FRAME = true;

// ======================= END USER CONFIGURATION =======================

MCP2515 mcp2515(MCP2515_CS_PIN);

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

uint32_t nextSequence = 0;
uint32_t canFramesReceived = 0;
uint32_t canFramesForwarded = 0;
uint32_t invalidCanFrames = 0;
uint32_t errorFramesSkipped = 0;
uint32_t espNowImmediateFailures = 0;
volatile uint32_t espNowDeliverySuccesses = 0;
volatile uint32_t espNowDeliveryFailures = 0;
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
    Serial.println("ERROR: Could not read BMS-side Wi-Fi station MAC.");
    return;
  }

  Serial.print("BMS-side Wi-Fi MAC: ");
  printMacAddress(stationMac);
  Serial.println();
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
    haltWithMessage("ERROR: MCP2515 bitrate setup failed. Check SPI wiring and crystal selection.");
  }

  // Normal mode allows this node to ACK frames on a two-node bench bus. This
  // sketch never calls mcp2515.sendMessage().
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    haltWithMessage("ERROR: MCP2515 could not enter normal mode.");
  }

  Serial.println("MCP2515 initialized in normal mode (receive only in firmware).");
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

  if (esp_now_register_send_cb(onEspNowSent) != ESP_OK) {
    haltWithMessage("ERROR: ESP-NOW send callback registration failed.");
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, LAPTOP_SIDE_MAC, sizeof(LAPTOP_SIDE_MAC));
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    haltWithMessage("ERROR: Could not add laptop-side ESP-NOW peer.");
  }

  Serial.print("Laptop-side peer: ");
  printMacAddress(LAPTOP_SIDE_MAC);
  Serial.println();
}

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void printCanIdentifier(canid_t identifier, bool extended) {
  const uint8_t digits = extended ? 8 : 3;
  for (int8_t shift = (digits - 1) * 4; shift >= 0; shift -= 4) {
    Serial.print(static_cast<uint8_t>((identifier >> shift) & 0x0F), HEX);
  }
}

const char *knownFrameName(canid_t identifier, bool extended) {
  if (extended) {
    return nullptr;
  }

  switch (identifier) {
    case 0x6B0:
      return "Pack current | Inst voltage | SOC | Relay state | Checksum";
    case 0x6B1:
      return "Pack DCL | High temp | Low temp | Reserved | Checksum";
    case 0x6B2:
      return "DTC flags 1/2 | Pack Current | Pack SOC | Checksum";
    default:
      return nullptr;
  }
}

void printReceivedCanFrame(const struct can_frame &frame) {
  const bool extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  const bool remote = (frame.can_id & CAN_RTR_FLAG) != 0;
  const canid_t identifier =
      frame.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);

  Serial.print("[CAN RX] ms=");
  Serial.print(millis());
  Serial.print(" ID=0x");
  printCanIdentifier(identifier, extended);
  Serial.print(extended ? " EXT" : " STD");
  if (remote) {
    Serial.print(" RTR");
  }
  Serial.print(" DLC=");
  Serial.print(frame.can_dlc);
  Serial.print(" DATA=");

  if (remote) {
    Serial.print("<remote request>");
  } else if (frame.can_dlc == 0) {
    Serial.print("<empty>");
  } else {
    for (uint8_t i = 0; i < frame.can_dlc; ++i) {
      if (i != 0) {
        Serial.print(' ');
      }
      printHexByte(frame.data[i]);
    }
  }

  const char *name = knownFrameName(identifier, extended);
  if (name != nullptr) {
    Serial.print(" | ");
    Serial.print(name);
  }
  Serial.println();

  // Print explicit byte boundaries for the three configured Orion messages.
  if (!remote && frame.can_dlc == 8 && !extended) {
    if (identifier == 0x6B0) {
      Serial.print("  6B0 fields: current=");
      printHexByte(frame.data[0]);
      Serial.print(' ');
      printHexByte(frame.data[1]);
      Serial.print(" voltage=");
      printHexByte(frame.data[2]);
      Serial.print(' ');
      printHexByte(frame.data[3]);
      Serial.print(" SOC=");
      printHexByte(frame.data[4]);
      Serial.print(" relay=");
      printHexByte(frame.data[5]);
      Serial.print(' ');
      printHexByte(frame.data[6]);
      Serial.print(" checksum=");
      printHexByte(frame.data[7]);
      Serial.println();
    } else if (identifier == 0x6B1) {
      Serial.print("  6B1 fields: DCL=");
      for (uint8_t i = 0; i < 4; ++i) {
        if (i != 0) {
          Serial.print(' ');
        }
        printHexByte(frame.data[i]);
      }
      Serial.print(" high-temp=");
      printHexByte(frame.data[4]);
      Serial.print(" low-temp=");
      printHexByte(frame.data[5]);
      Serial.print(" byte6=");
      printHexByte(frame.data[6]);
      Serial.print(" checksum=");
      printHexByte(frame.data[7]);
      Serial.println();
    } else if (identifier == 0x6B2) {
      Serial.print("  6B2 fields: DTC1=");
      printHexByte(frame.data[0]);
      Serial.print(" DTC2=");
      printHexByte(frame.data[1]);
      Serial.print(" ADC1=");
      printHexByte(frame.data[2]);
      Serial.print(" ADC2=");
      printHexByte(frame.data[3]);
      Serial.print(" bytes4-6=");
      for (uint8_t i = 4; i <= 6; ++i) {
        if (i != 4) {
          Serial.print(' ');
        }
        printHexByte(frame.data[i]);
      }
      Serial.print(" checksum=");
      printHexByte(frame.data[7]);
      Serial.println();
    }
  }
}

void forwardCanFrame(const struct can_frame &frame) {
  ++canFramesReceived;

  if ((frame.can_id & CAN_ERR_FLAG) != 0) {
    ++errorFramesSkipped;
    return;
  }

  if (frame.can_dlc > CAN_MAX_DLC) {
    ++invalidCanFrames;
    return;
  }

  if (PRINT_EVERY_RECEIVED_CAN_FRAME) {
    printReceivedCanFrame(frame);
  }

  WirelessCanPacket packet = {};
  packet.magic = PACKET_MAGIC;
  packet.version = PACKET_VERSION;
  packet.sourceNode = SOURCE_NODE_BMS;
  packet.dlc = frame.can_dlc;
  packet.sequence = nextSequence++;
  packet.senderMillis = millis();
  packet.canIdWithFlags =
      frame.can_id & (CAN_EFF_MASK | FORWARDED_CAN_FLAGS);

  if ((frame.can_id & CAN_RTR_FLAG) == 0) {
    memcpy(packet.data, frame.data, frame.can_dlc);
  }

  const esp_err_t result =
      esp_now_send(LAPTOP_SIDE_MAC,
                   reinterpret_cast<const uint8_t *>(&packet),
                   sizeof(packet));

  if (result == ESP_OK) {
    ++canFramesForwarded;
  } else {
    ++espNowImmediateFailures;
  }
}

void printStatistics() {
  Serial.print("[BMS] CAN RX=");
  Serial.print(canFramesReceived);
  Serial.print(" forwarded=");
  Serial.print(canFramesForwarded);
  Serial.print(" delivered=");
  Serial.print(espNowDeliverySuccesses);
  Serial.print(" send-fail=");
  Serial.print(espNowImmediateFailures + espNowDeliveryFailures);
  Serial.print(" invalid=");
  Serial.print(invalidCanFrames);
  Serial.print(" error-frame-skip=");
  Serial.print(errorFramesSkipped);
  Serial.print(" next-sequence=");
  Serial.println(nextSequence);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("Orion BMS 2 one-way wireless CAN monitor - BMS side");
  initializeEspNow();
  initializeCan();
  Serial.println("Monitor ready. Forwarding Orion CAN frames to laptop side.");
}

void loop() {
  struct can_frame frame;
  size_t processed = 0;

  while (processed < MAX_CAN_FRAMES_PER_LOOP &&
         mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    forwardCanFrame(frame);
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
  Unless isolated, share the required low-voltage CAN reference ground.

  Many HW-184 boards are 5 V modules. ESP32 GPIO is 3.3 V only and is not
  5 V tolerant. Verify SPI voltage levels and add level shifting if required.

  CAN requires exactly two 120 ohm terminators at the physical ends. With all
  power removed, CANH-to-CANL should measure approximately 60 ohms.
*/
