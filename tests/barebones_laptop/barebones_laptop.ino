/*
  Orion BMS 2 read-only wireless CAN bridge - laptop/CANdapter side

  Data path:
    ESP-NOW -> ESP32-S3-WROOM-1 -> MCP2515 -> CANdapter -> laptop

  This sketch only reproduces CAN frames received over ESP-NOW. It never reads
  or forwards CANdapter traffic back to the BMS.

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
#include <esp_mac.h>

// This sketch is intended to use the ESP32-S3 native USB JTAG/serial port
// (/dev/cu.usbmodem... on macOS). Without this Arduino IDE option, Serial is
// routed to UART0 instead (normally the CP2102 /dev/cu.usbserial... port).
#if defined(CONFIG_IDF_TARGET_ESP32S3) && !ARDUINO_USB_CDC_ON_BOOT
#error "Set Arduino IDE: Tools > USB CDC On Boot > Enabled, then compile and upload again."
#endif

// ========================= USER CONFIGURATION =========================

// Common custom SPI choices for an ESP32-S3 DevKit-style carrier. ESP32-S3
// modules do not impose one universal Arduino SPI pinout, so verify the exact
// carrier board and change these values before wiring.
constexpr int SPI_SCK_PIN = 12;
constexpr int SPI_MISO_PIN = 13;
constexpr int SPI_MOSI_PIN = 11;
constexpr int MCP2515_CS_PIN = 10;

// Optional MCP2515 INT output. This version does not need it because this side
// only transmits CAN frames. Leave disconnected and set to -1 if unused.
constexpr int MCP2515_INT_PIN = -1;

// This must match the CAN bitrate selected by the Orion Utility/CANdapter and
// the Orion CAN interface being mirrored:
//   CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS
constexpr CAN_SPEED CAN_BITRATE = CAN_500KBPS;

// Many HW-184 boards are sold with either an 8 MHz or 16 MHz crystal.
// Read the marking on the metal crystal and select MCP_8MHZ or MCP_16MHZ.
// A wrong setting prevents reliable CAN communication.
constexpr CAN_CLOCK MCP2515_CLOCK = MCP_8MHZ;

// Must match ESPNOW_WIFI_CHANNEL in the BMS-side sketch.
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 1;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 2000;
constexpr uint32_t SERIAL_ATTACH_TIMEOUT_MS = 5000;
constexpr uint32_t MAC_PRINT_INTERVAL_MS = 10000;

// Incoming packets are copied here by the Wi-Fi callback and transmitted from
// loop(). Increase carefully if bursts overflow the queue.
constexpr size_t RX_QUEUE_SIZE = 64;

// ======================= END USER CONFIGURATION =======================

MCP2515 mcp2515(MCP2515_CS_PIN);

constexpr uint8_t PACKET_MAGIC = 0xA7;
constexpr uint8_t PACKET_VERSION = 1;
constexpr uint8_t FLAG_EXTENDED = 0x01;
constexpr uint8_t FLAG_RTR = 0x02;
constexpr uint8_t ALLOWED_FLAGS = FLAG_EXTENDED | FLAG_RTR;

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

WirelessCanPacket rxQueue[RX_QUEUE_SIZE];
volatile size_t rxQueueHead = 0;
volatile size_t rxQueueTail = 0;
portMUX_TYPE rxQueueMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t espNowPacketsReceived = 0;
volatile uint32_t invalidSizePackets = 0;
volatile uint32_t invalidHeaderPackets = 0;
volatile uint32_t invalidFramePackets = 0;
volatile uint32_t queueOverflowPackets = 0;

uint32_t canFramesTransmitted = 0;
uint32_t mcp2515TransmitFailures = 0;
uint32_t wirelessSequenceDrops = 0;
uint32_t wirelessOutOfOrder = 0;
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

bool packetHasValidCanFields(const WirelessCanPacket &packet) {
  if (packet.dlc > 8 || (packet.flags & ~ALLOWED_FLAGS) != 0) {
    return false;
  }

  if ((packet.flags & FLAG_EXTENDED) != 0) {
    return packet.canId <= CAN_EFF_MASK;
  }

  return packet.canId <= CAN_SFF_MASK;
}

void enqueuePacket(const uint8_t *data, int length) {
  ++espNowPacketsReceived;

  if (length != static_cast<int>(sizeof(WirelessCanPacket))) {
    ++invalidSizePackets;
    return;
  }

  WirelessCanPacket packet;
  memcpy(&packet, data, sizeof(packet));

  if (packet.magic != PACKET_MAGIC || packet.version != PACKET_VERSION) {
    ++invalidHeaderPackets;
    return;
  }

  if (!packetHasValidCanFields(packet)) {
    ++invalidFramePackets;
    return;
  }

  portENTER_CRITICAL(&rxQueueMux);
  const size_t nextHead = (rxQueueHead + 1) % RX_QUEUE_SIZE;
  if (nextHead == rxQueueTail) {
    ++queueOverflowPackets;
  } else {
    rxQueue[rxQueueHead] = packet;
    rxQueueHead = nextHead;
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
  if (rxQueueTail != rxQueueHead) {
    packet = rxQueue[rxQueueTail];
    rxQueueTail = (rxQueueTail + 1) % RX_QUEUE_SIZE;
    available = true;
  }
  portEXIT_CRITICAL(&rxQueueMux);

  return available;
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

  // Normal mode is required to transmit frames and ACK CANdapter traffic on
  // this local segment. This sketch intentionally never reads that traffic.
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

  if (esp_now_init() != ESP_OK) {
    haltWithMessage("ERROR: ESP-NOW initialization failed.");
  }

  if (esp_now_register_recv_cb(onEspNowReceive) != ESP_OK) {
    haltWithMessage("ERROR: ESP-NOW receive callback registration failed.");
  }
}

void printLaptopMacAddress() {
  uint8_t stationMac[6];

  if (esp_read_mac(stationMac, ESP_MAC_WIFI_STA) != ESP_OK) {
    Serial.println("ERROR: Could not read the ESP32-S3 Wi-Fi station MAC.");
    return;
  }

  Serial.print("Laptop-side Wi-Fi MAC: ");
  for (uint8_t i = 0; i < sizeof(stationMac); ++i) {
    if (i != 0) {
      Serial.print(':');
    }
    if (stationMac[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(stationMac[i], HEX);
  }
  Serial.println();
  Serial.println("Copy this MAC into LAPTOP_SIDE_PEER_MAC in the BMS-side sketch.");
}

bool sequenceIsUsable(uint32_t sequence) {
  if (!haveLastSequence) {
    haveLastSequence = true;
    lastSequence = sequence;
    return true;
  }

  const uint32_t expected = lastSequence + 1;
  const int32_t difference = static_cast<int32_t>(sequence - expected);

  if (difference == 0) {
    lastSequence = sequence;
    return true;
  }

  if (difference > 0) {
    wirelessSequenceDrops += static_cast<uint32_t>(difference);
    lastSequence = sequence;
    return true;
  }

  // Duplicate or late packet. Do not reproduce an old frame on the CAN bus.
  ++wirelessOutOfOrder;
  return false;
}

void transmitPacketToCan(const WirelessCanPacket &packet) {
  if (!sequenceIsUsable(packet.sequence)) {
    return;
  }

  struct can_frame frame = {};
  frame.can_dlc = packet.dlc;

  if ((packet.flags & FLAG_EXTENDED) != 0) {
    frame.can_id = (packet.canId & CAN_EFF_MASK) | CAN_EFF_FLAG;
  } else {
    frame.can_id = packet.canId & CAN_SFF_MASK;
  }

  if ((packet.flags & FLAG_RTR) != 0) {
    frame.can_id |= CAN_RTR_FLAG;
  } else {
    memcpy(frame.data, packet.data, frame.can_dlc);
  }

  if (mcp2515.sendMessage(&frame) == MCP2515::ERROR_OK) {
    ++canFramesTransmitted;
  } else {
    ++mcp2515TransmitFailures;
  }
}

void printStatistics() {
  Serial.print("[Laptop side] ESP-NOW RX=");
  Serial.print(espNowPacketsReceived);
  Serial.print(" | CAN TX=");
  Serial.print(canFramesTransmitted);
  Serial.print(" | CAN TX fail=");
  Serial.print(mcp2515TransmitFailures);
  Serial.print(" | sequence drops=");
  Serial.print(wirelessSequenceDrops);
  Serial.print(" | out-of-order=");
  Serial.print(wirelessOutOfOrder);
  Serial.print(" | queue overflow=");
  Serial.print(queueOverflowPackets);
  Serial.print(" | invalid size/header/frame=");
  Serial.print(invalidSizePackets);
  Serial.print('/');
  Serial.print(invalidHeaderPackets);
  Serial.print('/');
  Serial.println(invalidFramePackets);
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Native USB CDC serial on an ESP32-S3 may take a few seconds to enumerate.
  // Do not wait forever because the bridge must still run without a monitor.
  const uint32_t serialWaitStarted = millis();
  while (!Serial && millis() - serialWaitStarted < SERIAL_ATTACH_TIMEOUT_MS) {
    delay(10);
  }

  Serial.println();
  Serial.println("Orion BMS 2 read-only wireless CAN bridge - laptop side");

  // Initialize Wi-Fi and print the MAC before touching the MCP2515. That way a
  // missing or miswired CAN module cannot prevent the MAC from being reported.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  printLaptopMacAddress();

  initializeEspNow();
  initializeCan();
  Serial.println("Bridge ready. Waiting for ESP-NOW CAN packets.");
}

void loop() {
  WirelessCanPacket packet;

  // Keep SPI/CAN work out of the ESP-NOW callback's Wi-Fi task.
  while (dequeuePacket(packet)) {
    transmitPacketToCan(packet);
  }

  const uint32_t now = millis();
  if (now - lastDebugPrintMs >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMs = now;
    printStatistics();
  }

  // Repeat the address so opening Serial Monitor after boot does not miss it.
  if (now - lastMacPrintMs >= MAC_PRINT_INTERVAL_MS) {
    lastMacPrintMs = now;
    printLaptopMacAddress();
  }

  delay(0);
}

/*
  Laptop-side wiring:

  ESP32-S3              HW-184 MCP2515
  SPI_SCK_PIN    <----> SCK
  SPI_MISO_PIN   <----> SO / MISO
  SPI_MOSI_PIN   <----> SI / MOSI
  MCP2515_CS_PIN <----> CS
  optional INT   <----> INT
  GND            <----> GND

  HW-184 CANH/CANL connect to the CANdapter-side local CAN bus. The
  CANdapter and MCP2515 are the two active CAN nodes on this segment.

  WARNING: Many HW-184 modules are designed for 5 V power and may expose 5 V
  logic on SPI pins. ESP32-S3 GPIO is 3.3 V only and is not 5 V tolerant.
  Verify the exact module schematic and transceiver/controller parts. Use
  suitable level shifting or a known 3.3 V-safe CAN module when required.

  The local CAN segment needs exactly two 120 ohm terminators, one at each
  physical end. Do not assume either HW-184 or CANdapter termination is fitted;
  inspect the boards/manual and measure about 60 ohms between CANH and CANL
  with all bus power removed.

  The CANdapter manual notes that CAN transmission requires at least two CAN
  nodes. Here, the MCP2515 bridge node supplies the second node and ACKs frames.
*/
