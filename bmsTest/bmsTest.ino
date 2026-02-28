#include <SPI.h>
#include "mcp2515_can.h"

// PIN ASSIGNMENTS
const int PIN_ID_BIT0 = 7;      
const int PIN_ID_BIT1 = 8;      
const int CAN_CS_PIN  = 9;      
const int TEMP_SENSOR_PIN = A0; 

mcp2515_can CAN(CAN_CS_PIN);

// IDENTIFICATION
unsigned long MY_CAN_ID;
int BOARD_INDEX;

// BMS IDs (From your Orion BMS config)
const unsigned long BMS_ID_A = 0x6B0; // Pack Current, Inst. Voltage, SOC
const unsigned long BMS_ID_B = 0x6B1; // Pack DCL, High Temp, Low Temp

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_ID_BIT0, INPUT_PULLUP);
    pinMode(PIN_ID_BIT1, INPUT_PULLUP);

    // Calculate this node's ID based on hardware pins
    int bit0 = !digitalRead(PIN_ID_BIT0); 
    int bit1 = !digitalRead(PIN_ID_BIT1);
    BOARD_INDEX = (bit1 << 1) | bit0; 
    MY_CAN_ID = (unsigned long)BOARD_INDEX; 

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN Init Fail. Check SPI wiring!");
        delay(1000);
    }
    
    Serial.print("NODE ONLINE. ID: ");
    Serial.println(BOARD_INDEX);
}

void loop() {
    // --- SECTION 1: TRANSMIT ---
    static unsigned long lastSend = 0;
    if (millis() - lastSend > 1000) {
        float voltage = analogRead(TEMP_SENSOR_PIN) * (5.0 / 1023.0);
        float tempF = (( (100.0 * voltage) - 50.0 ) * 9.0 / 5.0) + 32.0;
        
        // No Tag needed. We just send the 4-byte float.
        CAN.sendMsgBuf(MY_CAN_ID, 0, 4, (unsigned char*)&tempF);
        
        Serial.print(">>> [TX] Sent Temp: ");
        Serial.println(tempF);
        lastSend = millis();
    }

    // --- SECTION 2: RECEIVE ---
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
        unsigned char len = 0;
        unsigned char buf[8];
        CAN.readMsgBuf(&len, buf);
        unsigned long incomingId = CAN.getCanId();
        
        if (incomingId == MY_CAN_ID) return; // Skip our own messages

        // 1. Process Orion BMS Messages
        if (incomingId == BMS_ID_A) {
            decodeBmsA(buf);
        } 
        else if (incomingId == BMS_ID_B) {
            decodeBmsA(buf);
        }
        // 2. Process other sensor nodes (Assumed to be Temp Nodes)
        else {
            float remoteTemp;
            memcpy(&remoteTemp, buf, 4); 
            Serial.print("<<< [RX] Node ");
            Serial.print(incomingId);
            Serial.print(" Temp: ");
            Serial.print(remoteTemp);
            Serial.println(" F");
        }
    }
}

// Decoding BMS 0x6B0 based on your image (Big Endian)
void decodeBmsA(byte *buf) {
    // Current: Bytes 0-1. We shift Byte 0 up and add Byte 1.
    int rawCurrent = (buf[0] << 8) | buf[1];
    
    // Voltage: Bytes 2-3
    int rawVoltage = (buf[2] << 8) | buf[3];
    
    // SOC: Byte 4
    int soc = buf[4];

    Serial.print("<<< [BMS 0x6B0] ");
    Serial.print("Current: "); Serial.print(rawCurrent);
    Serial.print(" | Volts: "); Serial.print(rawVoltage);
    Serial.print(" | SOC: "); Serial.print(soc);
    Serial.println("%");
}

// Decoding 0x6B1: Pack DCL, High Temp, Low Temp
void decodeBmsB(byte *buf) {
    // Byte 0-1: Pack DCL (Discharge Current Limit)
    int dcl = (buf[0] << 8) | buf[1];
    
    // Byte 4: High Temperature
    int highTemp = (int8_t)buf[4]; // Cast to signed if your BMS allows negative temps
    
    // Byte 5: Low Temperature
    int lowTemp = (int8_t)buf[5];

    Serial.print("<<< [BMS B] DCL: "); Serial.print(dcl);
    Serial.print("A | High T: "); Serial.print(highTemp);
    Serial.print("C | Low T: "); Serial.print(lowTemp);
    Serial.println("C");
}