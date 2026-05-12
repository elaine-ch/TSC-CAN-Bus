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

    int bit0 = digitalRead(PIN_ID_BIT0); 
    int bit1 = digitalRead(PIN_ID_BIT1);
    
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
    //CODE FOR NODE 0 (BMS)
    if(BOARD_INDEX == 0){
      //RECEIVE
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

      //TRANSMIT
      static unsigned long lastSend = 0;
      if (millis() - lastSend > 1000) {
          
          // 3. Send the float bytes over CAN
          CAN.sendMsgBuf(MY_CAN_ID, 0, 4, (unsigned char*)&tempF);
          
          Serial.print(">>> [TX] Node ");
          Serial.print(BOARD_INDEX);
          Serial.print(" sent: ");
          Serial.print(tempF);
          Serial.println(" F");
          lastSend = millis();
      }
    }

    //CODE FOR NODE 1 (MPPT)
    if(BOARD_INDEX == 1){
      //TRANSMIT
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
    }

    //CODE FOR NODE 2 (MOTORS)
    if(BOARD_INDEX == 2){
      // TRANSMIT
      static unsigned long lastSend = 0;
      if (millis() - lastSend > 1000) {
          // 3. Send the float bytes over CAN
          CAN.sendMsgBuf(MY_CAN_ID, 0, 4, (unsigned char*)&tempF);
          
          Serial.print(">>> [TX] Node ");
          Serial.print(BOARD_INDEX);
          Serial.print(" sent: ");
          Serial.print(tempF);
          Serial.println(" F");
          lastSend = millis();
      }
    }

    //CODE FOR NODE 3 (CONTROL)
    if(BOARD_INDEX == 3){
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