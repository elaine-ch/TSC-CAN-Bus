#include <SPI.h>
#include "mcp2515_can.h"

// SPI PINS FOR ESP32 (WROOM 32E)
const int SCK_PIN  = 18;
const int MISO_PIN = 19;
const int MOSI_PIN = 23;
const int CAN_CS_PIN = 5;  // Connected to D9 on the Seeed Shield

mcp2515_can CAN(CAN_CS_PIN);

unsigned long MY_CAN_ID = 3; // Assigned as the Gateway Node
int BOARD_INDEX = 3;

void setup() {
    // Increased baud rate to handle the high volume of solar car telemetry
    Serial.begin(115200);
    
    // 1. Initialize SPI for ESP32
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CAN_CS_PIN);

    // 2. Initialize CAN Bus at 500kbps
    // Note: If you get "Fail", change MCP_16MHz to MCP_8MHz depending on your shield's crystal
    while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_16MHz)) {
        Serial.println("CAN Init Fail. Check wiring between ESP32 and Seeed Shield!");
        delay(1000);
    }
    
    Serial.println("TSC GATEWAY ONLINE. Listening on CAN Bus...");
}

void loop() {
    // RECEIVE 
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
        unsigned char len = 0;
        unsigned char buf[8];
        CAN.readMsgBuf(&len, buf);
        
        unsigned long incomingId = CAN.getCanId();
        
        // Rebuild the float bytes sent by the Arduino Unos
        float receivedValue;
        memcpy(&receivedValue, buf, 4); 

        // OUTPUT FOR INFLUXDB / LAPTOP
        // Format: node_id,value (This makes it easy for a Python script to parse)
        Serial.print("DATA,");
        Serial.print(incomingId);
        Serial.print(",");
        Serial.println(receivedValue);
    }
}