#include <SPI.h>
#include "mcp2515_can.h"

const int SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN);

void setup() {
    Serial.begin(115200);

    // Initialize the CAN board at 500kbps (must match the transmitter!)
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        delay(100);
    }
    Serial.println("Receiver Ready!");
}

void loop() {
    unsigned char len = 0;
    unsigned char buf[8]; // Buffer to hold incoming bytes

    // Check if a message has arrived in the MCP2515's buffer
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
        
        // Read the message (gets the ID, length, and the actual bytes)
        CAN.readMsgBuf(&len, buf);
        unsigned long canId = CAN.getCanId();

        // Only process if the ID matches our Temperature Sensor (0x123)
        if (canId == 0x123) {
            float receivedTempF;
            
            // "Unpack" the 4 bytes back into a float
            memcpy(&receivedTempF, buf, sizeof(receivedTempF));

            Serial.print("Received Temp: ");
            Serial.print(receivedTempF);
            Serial.println(" °F");
        }
    }
}