#include <SPI.h>
#include "mcp2515_can.h"

// PIN ASSIGNMENTS
const int PIN_ID_BIT0 = 7;      
const int PIN_ID_BIT1 = 8;      
const int CAN_CS_PIN  = 9;      
const int TEMP_SENSOR_PIN = A0; 

mcp2515_can CAN(CAN_CS_PIN);

unsigned long MY_CAN_ID;
int BOARD_INDEX;

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_ID_BIT0, INPUT_PULLUP);
    pinMode(PIN_ID_BIT1, INPUT_PULLUP);

    int bit0 = !digitalRead(PIN_ID_BIT0); 
    int bit1 = !digitalRead(PIN_ID_BIT1);
    
    BOARD_INDEX = (bit1 << 1) | bit0; 
    MY_CAN_ID = (unsigned long)BOARD_INDEX; 

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN Init Fail. Check SPI wiring!");
        delay(1000);
    }
    
    Serial.print("TSC NODE ONLINE. Identified as ID: ");
    Serial.println(BOARD_INDEX);
}

void loop() {
    // TRANSMIT
    static unsigned long lastSend = 0;
    if (millis() - lastSend > 1000) {
        // 1. Get Celsius from the sensor
        float voltage = analogRead(TEMP_SENSOR_PIN) * (5.0 / 1023.0);
        float tempC = (100.0 * voltage) - 50.0;
        
        // 2. Convert to Fahrenheit 
        float tempF = (tempC * 9.0 / 5.0) + 32.0;
        
        // 3. Send the float bytes over CAN
        CAN.sendMsgBuf(MY_CAN_ID, 0, 4, (unsigned char*)&tempF);
        
        Serial.print(">>> [TX] Node ");
        Serial.print(BOARD_INDEX);
        Serial.print(" sent: ");
        Serial.print(tempF);
        Serial.println(" F");
        lastSend = millis();
    }

    // RECEIVE 
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
        unsigned char len = 0;
        unsigned char buf[8];
        CAN.readMsgBuf(&len, buf);
        
        unsigned long incomingId = CAN.getCanId();
        
        if (incomingId != MY_CAN_ID) {
            float remoteTempF;
            memcpy(&remoteTempF, buf, 4); // Rebuild the float bytes

            Serial.print("<<< [RX] from Node ");
            Serial.print(incomingId);
            Serial.print(": ");
            Serial.print(remoteTempF);
            Serial.println(" F");
        }
    }
}