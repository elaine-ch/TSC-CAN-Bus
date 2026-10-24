/*
  Orion BMS 2 one-way wireless CAN monitor - BMS side

  Data path:
    Orion BMS CAN -> HW-184 MCP2515 -> ESP32-WROOM-32UE
    -> ESP-NOW -> laptop-side ESP32-S3 -> Serial Monitor

  TFT display:
    ESP32-WROOM-32UE -> 4.0" ST7796 TFT

  The serial monitor continues to print all existing information.
  The TFT displays the latest CAN frame and continuously updated statistics.

  Required libraries:
    AutoWP MCP2515
    TFT_eSPI
*/

#include <SPI.h>
#include <mcp2515.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <esp_mac.h>
#include <TFT_eSPI.h>

// ========================= USER CONFIGURATION =========================

// ---------- MCP2515 ----------

constexpr int SPI_SCK_PIN = 18;
constexpr int SPI_MISO_PIN = 19;
constexpr int SPI_MOSI_PIN = 23;
constexpr int MCP2515_CS_PIN = 5;

// This version polls readMessage(); INT may be left disconnected.
constexpr int MCP2515_INT_PIN = -1;

// Match the configured Orion CAN interface.
constexpr CAN_SPEED CAN_BITRATE = CAN_500KBPS;

// HW-184 modules may have an 8 MHz or 16 MHz crystal.
constexpr CAN_CLOCK MCP2515_CLOCK = MCP_8MHZ;

// ---------- ESP-NOW ----------

constexpr uint8_t ESPNOW_WIFI_CHANNEL = 1;

// Laptop-side ESP32-S3 station MAC.
const uint8_t LAPTOP_SIDE_MAC[6] = {
  0x48, 0x27, 0xE2, 0x82, 0x97, 0x44
};

// ---------- Serial ----------

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 2000;
constexpr uint32_t MAC_PRINT_INTERVAL_MS = 10000;

// ---------- CAN ----------

constexpr size_t MAX_CAN_FRAMES_PER_LOOP = 32;

// Raw per-frame logging confirms exactly what the MCP2515 receives.
constexpr bool PRINT_EVERY_RECEIVED_CAN_FRAME = true;

// ---------- TFT ----------

constexpr int TFT_BL_PIN = 16;

constexpr uint32_t DISPLAY_STAT_UPDATE_INTERVAL_MS = 500;

// ======================= END USER CONFIGURATION =======================

MCP2515 mcp2515(MCP2515_CS_PIN);
TFT_eSPI tft = TFT_eSPI();

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
uint32_t lastDisplayUpdateMs = 0;

// Stores information about the most recently received CAN frame.
canid_t lastCanIdentifier = 0;
uint8_t lastCanDlc = 0;
uint8_t lastCanData[8] = {};
bool lastCanExtended = false;
bool lastCanRemote = false;
bool haveLastCanFrame = false;

// =====================================================================
// DISPLAY FUNCTIONS
// =====================================================================

void displayPrintFixed(const char *text,
                       int x,
                       int y,
                       int textSize,
                       uint16_t color,
                       int width) {
  // Clear the region first so values of different lengths don't overlap.
  tft.fillRect(x, y, width, textSize * 8 + 4, TFT_BLACK);

  tft.setTextColor(color, TFT_BLACK);
  tft.setTextSize(textSize);
  tft.setCursor(x, y);
  tft.print(text);
}

void displayPrintValue(const char *label,
                       uint32_t value,
                       int x,
                       int y,
                       int textSize,
                       uint16_t color,
                       int width) {
  tft.fillRect(x, y, width, textSize * 8 + 4, TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(textSize);
  tft.setCursor(x, y);
  tft.print(label);

  tft.setTextColor(color, TFT_BLACK);
  tft.print(value);
}

void displayPrintHex32(const char *label,
                       uint32_t value,
                       int x,
                       int y,
                       int textSize,
                       uint16_t color,
                       int width) {
  tft.fillRect(x, y, width, textSize * 8 + 4, TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(textSize);
  tft.setCursor(x, y);
  tft.print(label);

  tft.setTextColor(color, TFT_BLACK);
  tft.print("0x");

  if (value < 0x1000) {
    tft.print("000");
  } else if (value < 0x10000) {
    tft.print("00");
  } else if (value < 0x100000) {
    tft.print("0");
  }

  tft.print(value, HEX);
}

void displayPrintCanData(int x, int y) {
  tft.fillRect(x, y, 440, 30, TFT_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(x, y);

  if (lastCanRemote) {
    tft.print("<remote request>");
    return;
  }

  if (lastCanDlc == 0) {
    tft.print("<empty>");
    return;
  }

  for (uint8_t i = 0; i < lastCanDlc; ++i) {
    if (i != 0) {
      tft.print(' ');
    }

    if (lastCanData[i] < 0x10) {
      tft.print('0');
    }

    tft.print(lastCanData[i], HEX);
  }
}

void updateDisplayCanFrame(const struct can_frame &frame) {
  lastCanExtended = (frame.can_id & CAN_EFF_FLAG) != 0;
  lastCanRemote = (frame.can_id & CAN_RTR_FLAG) != 0;

  lastCanIdentifier =
      frame.can_id &
      (lastCanExtended ? CAN_EFF_MASK : CAN_SFF_MASK);

  lastCanDlc = frame.can_dlc;

  if (!lastCanRemote) {
    memcpy(lastCanData, frame.data, frame.can_dlc);
  }

  haveLastCanFrame = true;

  // Header
  tft.fillRect(0, 0, 480, 35, TFT_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 5);
  tft.print("Orion BMS CAN Monitor");

  // CAN ID
  displayPrintHex32(
      "ID: ",
      lastCanIdentifier,
      10,
      45,
      2,
      TFT_CYAN,
      220
  );

  // Format
  tft.fillRect(250, 45, 220, 22, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(250, 45);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  if (lastCanExtended) {
    tft.print("EXT");
  } else {
    tft.print("STD");
  }

  if (lastCanRemote) {
    tft.print(" RTR");
  }

  // DLC
  displayPrintValue(
      "DLC: ",
      lastCanDlc,
      10,
      80,
      2,
      TFT_YELLOW,
      150
  );

  // Data
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 115);
  tft.print("DATA:");

  displayPrintCanData(10, 140);

  // Known Orion frame information
  tft.fillRect(10, 175, 460, 40, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 175);

  if (!lastCanExtended && lastCanIdentifier == 0x6B0) {
    tft.print("6B0: Current | Voltage | SOC | Relay");
  } else if (!lastCanExtended && lastCanIdentifier == 0x6B1) {
    tft.print("6B1: DCL | High Temp | Low Temp");
  } else if (!lastCanExtended && lastCanIdentifier == 0x6B2) {
    tft.print("6B2: DTC | Current | SOC");
  } else {
    tft.print("Unknown CAN frame");
  }
}

void updateDisplayStatistics() {
  // Statistics section
  tft.fillRect(0, 220, 480, 100, TFT_BLACK);

  tft.setTextSize(2);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 225);
  tft.print("CAN RX: ");

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print(canFramesReceived);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(180, 225);
  tft.print("Forwarded: ");

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print(canFramesForwarded);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 250);
  tft.print("ESP-NOW OK: ");

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print(espNowDeliverySuccesses);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(250, 250);
  tft.print("Failed: ");

  tft.setTextColor(
      (espNowImmediateFailures + espNowDeliveryFailures) > 0
          ? TFT_RED
          : TFT_GREEN,
      TFT_BLACK
  );

  tft.print(espNowImmediateFailures + espNowDeliveryFailures);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 275);
  tft.print("Invalid: ");

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.print(invalidCanFrames);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(180, 275);
  tft.print("Errors skipped: ");

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.print(errorFramesSkipped);

  // Sequence number
  tft.fillRect(10, 300, 460, 20, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 300);
  tft.print("Next sequence: ");

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print(nextSequence);
}

void initializeDisplay() {
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, HIGH);

  tft.init();

  // Landscape orientation: 480 x 320
  tft.setRotation(1);

  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(3);
  tft.setCursor(10, 10);
  tft.println("Orion BMS");

  tft.setTextSize(2);
  tft.setCursor(10, 50);
  tft.println("Initializing...");

  tft.setCursor(10, 80);
  tft.println("CAN + ESP-NOW");

  delay(500);

  tft.fillScreen(TFT_BLACK);

  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 10);
  tft.println("Orion BMS CAN Monitor");

  tft.setTextSize(2);
  tft.setCursor(10, 45);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("Waiting for CAN frame...");

  updateDisplayStatistics();
}

// =====================================================================
// GENERAL FUNCTIONS
// =====================================================================

void haltWithMessage(const char *message) {
  Serial.println(message);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 20);
  tft.println("ERROR:");
  tft.setCursor(10, 50);
  tft.println(message);

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
  // MCP2515 uses its own SPI pins.
  SPI.begin(
      SPI_SCK_PIN,
      SPI_MISO_PIN,
      SPI_MOSI_PIN,
      MCP2515_CS_PIN
  );

  pinMode(MCP2515_CS_PIN, OUTPUT);

  if (MCP2515_INT_PIN >= 0) {
    pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  }

  mcp2515.reset();

  if (mcp2515.setBitrate(
          CAN_BITRATE,
          MCP2515_CLOCK
      ) != MCP2515::ERROR_OK) {

    haltWithMessage(
        "ERROR: MCP2515 bitrate setup failed. "
        "Check SPI wiring and crystal selection."
    );
  }

  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    haltWithMessage(
        "ERROR: MCP2515 could not enter normal mode."
    );
  }

  Serial.println(
      "MCP2515 initialized in normal mode "
      "(receive only in firmware)."
  );
}

void initializeEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_wifi_set_channel(
          ESPNOW_WIFI_CHANNEL,
          WIFI_SECOND_CHAN_NONE
      ) != ESP_OK) {

    haltWithMessage(
        "ERROR: Could not set ESP-NOW Wi-Fi channel."
    );
  }

  printLocalMacAddress();

  if (esp_now_init() != ESP_OK) {
    haltWithMessage(
        "ERROR: ESP-NOW initialization failed."
    );
  }

  if (esp_now_register_send_cb(onEspNowSent) != ESP_OK) {
    haltWithMessage(
        "ERROR: ESP-NOW send callback registration failed."
    );
  }

  esp_now_peer_info_t peerInfo = {};

  memcpy(
      peerInfo.peer_addr,
      LAPTOP_SIDE_MAC,
      sizeof(LAPTOP_SIDE_MAC)
  );

  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    haltWithMessage(
        "ERROR: Could not add laptop-side ESP-NOW peer."
    );
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

  for (
      int8_t shift = (digits - 1) * 4;
      shift >= 0;
      shift -= 4
  ) {
    Serial.print(
        static_cast<uint8_t>(
            (identifier >> shift) & 0x0F
        ),
        HEX
    );
  }
}

const char *knownFrameName(
    canid_t identifier,
    bool extended
) {
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

// =====================================================================
// CAN FRAME PRINTING + DISPLAY UPDATE
// =====================================================================

void printReceivedCanFrame(const struct can_frame &frame) {
  const bool extended =
      (frame.can_id & CAN_EFF_FLAG) != 0;

  const bool remote =
      (frame.can_id & CAN_RTR_FLAG) != 0;

  const canid_t identifier =
      frame.can_id &
      (extended ? CAN_EFF_MASK : CAN_SFF_MASK);

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

  const char *name =
      knownFrameName(identifier, extended);

  if (name != nullptr) {
    Serial.print(" | ");
    Serial.print(name);
  }

  Serial.println();

  // Print explicit byte boundaries for configured Orion messages.
  if (!remote &&
      frame.can_dlc == 8 &&
      !extended) {

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

  // Update TFT with the same received CAN frame information.
  updateDisplayCanFrame(frame);
}

// =====================================================================
// ESP-NOW FORWARDING
// =====================================================================

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
  } else {
    // Even if serial CAN logging is disabled, keep the TFT updated.
    updateDisplayCanFrame(frame);
  }

  WirelessCanPacket packet = {};

  packet.magic = PACKET_MAGIC;
  packet.version = PACKET_VERSION;
  packet.sourceNode = SOURCE_NODE_BMS;
  packet.dlc = frame.can_dlc;
  packet.sequence = nextSequence++;
  packet.senderMillis = millis();

  packet.canIdWithFlags =
      frame.can_id &
      (CAN_EFF_MASK | FORWARDED_CAN_FLAGS);

  if ((frame.can_id & CAN_RTR_FLAG) == 0) {
    memcpy(
        packet.data,
        frame.data,
        frame.can_dlc
    );
  }

  const esp_err_t result =
      esp_now_send(
          LAPTOP_SIDE_MAC,
          reinterpret_cast<const uint8_t *>(&packet),
          sizeof(packet)
      );

  if (result == ESP_OK) {
    ++canFramesForwarded;
  } else {
    ++espNowImmediateFailures;
  }
}

// =====================================================================
// STATISTICS
// =====================================================================

void printStatistics() {
  Serial.print("[BMS] CAN RX=");
  Serial.print(canFramesReceived);

  Serial.print(" forwarded=");
  Serial.print(canFramesForwarded);

  Serial.print(" delivered=");
  Serial.print(espNowDeliverySuccesses);

  Serial.print(" send-fail=");
  Serial.print(
      espNowImmediateFailures +
      espNowDeliveryFailures
  );

  Serial.print(" invalid=");
  Serial.print(invalidCanFrames);

  Serial.print(" error-frame-skip=");
  Serial.print(errorFramesSkipped);

  Serial.print(" next-sequence=");
  Serial.println(nextSequence);
}

void updateDisplay() {
  const uint32_t now = millis();

  if (now - lastDisplayUpdateMs <
      DISPLAY_STAT_UPDATE_INTERVAL_MS) {
    return;
  }

  lastDisplayUpdateMs = now;

  updateDisplayStatistics();
}

// =====================================================================
// SETUP / LOOP
// =====================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println(
      "Orion BMS 2 one-way wireless CAN monitor - BMS side"
  );

  // Initialize display first so errors can be shown on-screen.
  initializeDisplay();

  initializeEspNow();
  initializeCan();

  Serial.println(
      "Monitor ready. Forwarding Orion CAN frames to laptop side."
  );

  tft.fillRect(10, 45, 460, 25, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, 45);
  tft.println("Monitor ready - waiting for CAN...");
}

void loop() {
  struct can_frame frame;

  size_t processed = 0;

  while (
      processed < MAX_CAN_FRAMES_PER_LOOP &&
      mcp2515.readMessage(&frame) ==
          MCP2515::ERROR_OK
  ) {
    forwardCanFrame(frame);
    ++processed;
  }

  const uint32_t now = millis();

  if (now - lastDebugPrintMs >=
      DEBUG_PRINT_INTERVAL_MS) {

    lastDebugPrintMs = now;

    printStatistics();
  }

  if (now - lastMacPrintMs >=
      MAC_PRINT_INTERVAL_MS) {

    lastMacPrintMs = now;

    printLocalMacAddress();
  }

  // Statistics on the TFT update in place.
  updateDisplay();

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

  TFT:

  ESP32                 TFT
  3.3V             <--> VCC
  GND              <--> GND
  GPIO15           <--> CS
  GPIO4            <--> RESET
  GPIO2            <--> DC/RS
  GPIO13           <--> SDI/MOSI
  GPIO14           <--> SCK
  GPIO16           <--> LED
  GPIO12           <--> SDO/MISO

  Touch pins:
  T_CLK, T_CS, T_DIN, T_DO, T_IRQ
  are left disconnected.

  MCP2515 and TFT use separate SPI buses/pin sets, so there is no
  SPI-device conflict in this configuration.

  HW-184 CANH/CANL connect to the selected Orion BMS CANH/CANL interface.
  Unless isolated, share the required low-voltage CAN reference ground.

  CAN requires exactly two 120 ohm terminators at the physical ends.
  With all power removed, CANH-to-CANL should measure approximately 60 ohms.

  Many HW-184 boards are 5 V modules. ESP32 GPIO is 3.3 V only and is not
  5 V tolerant. Verify SPI voltage levels and add level shifting if required.
*/
