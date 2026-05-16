#include <SPI.h>
#include "mcp2515_can.h"
#include "max6675.h"
#include "bms_fault.h"

#if defined(ARDUINO_ARCH_ESP32)
    const int PIN_ID_BIT0 = 14; // Bit 0 for ESP32
    const int PIN_ID_BIT1 = 12; // Bit 1 for ESP32
#else
    const int PIN_ID_BIT0 = 8; // Bit 0 for Arduino
    const int PIN_ID_BIT1 = 9; // Bit 1 for Arduino
    const int CAN_CS_PIN  = 10;      
    const int TEMP_SENSOR_PIN = A0; 
#endif

// --- Node 1 MAX6675 Pin Definitions ---
#define thermo1_DO  4
#define thermo1_CS  5
#define thermo1_CLK 6

#define thermo2_DO  A3
#define thermo2_CS  A2
#define thermo2_CLK A1

#define thermo3_DO  2
#define thermo3_CS  A5
#define thermo3_CLK A4

MAX6675* thermocouple1 = nullptr;
MAX6675* thermocouple2 = nullptr;
MAX6675* thermocouple3 = nullptr;
mcp2515_can CAN(CAN_CS_PIN);

// Orion BMS ID Definitions
#define BMS_ID_A    0x6B0
#define BMS_ID_B    0x6B1
#define BMS_ID_ERR1 0x6B2
#define BMS_ID_ERR2 0x6B3
// ---------------------------------------

#define NODE0_TO_ESP32_FAULT_ID 0x10
const int FAULT_LIGHT_PIN = 7;

unsigned long MY_CAN_ID;
int BOARD_INDEX;

// Forward declarations for compiler stability
void decodeBmsA(byte *buf);
void decodeBmsB(byte *buf);

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_ID_BIT0, INPUT_PULLUP);
    pinMode(PIN_ID_BIT1, INPUT_PULLUP);

    int bit0 = digitalRead(PIN_ID_BIT0); 
    int bit1 = digitalRead(PIN_ID_BIT1);
    
    BOARD_INDEX = (bit1 << 1) | bit0; 
    MY_CAN_ID = (unsigned long)BOARD_INDEX; 

    pinMode(FAULT_LIGHT_PIN, OUTPUT);
    digitalWrite(FAULT_LIGHT_PIN, HIGH);

    if (BOARD_INDEX == 1) {
        thermocouple1 = new MAX6675(thermo1_CLK, thermo1_CS, thermo1_DO);
        thermocouple2 = new MAX6675(thermo2_CLK, thermo2_CS, thermo2_DO);
        thermocouple3 = new MAX6675(thermo3_CLK, thermo3_CS, thermo3_DO);
        delay(500); 
    }
    
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
      if (CAN_MSGAVAIL == CAN.checkReceive()) {
          unsigned char len = 0;
          unsigned char buf[8];
          CAN.readMsgBuf(&len, buf);
          unsigned long incomingId = CAN.getCanId();
          
          if (incomingId == MY_CAN_ID) return;

          // Process Orion BMS Messages
          if (incomingId == BMS_ID_A) {
              decodeBmsA(buf);
          } 
          else if (incomingId == BMS_ID_B) {
              decodeBmsB(buf); 
          }

          // PROCESS FAULT TABLE 1 
          else if (incomingId == BMS_ID_ERR1) {
              uint16_t faultWord1 = (buf[0] << 8) | buf[1]; // Packed in Bytes 0 & 1
              
              // Persistent state tracker to keep the light on across distinct CAN frames
              static bool table1UrgentActive = false; 
              static bool table2UrgentActive = false; 
              
              table1UrgentActive = false; // Reset before checking current frame bits

              for (int i = 0; i < NUM_FAULTS_1; i++) {
                  if (faultWord1 & faultTable1[i].mask) {
                      Serial.print("[BMS Table 1] Active: ");
                      Serial.print(faultTable1[i].dtc);
                      Serial.print(" - ");
                      Serial.print(faultTable1[i].codeName);
                      Serial.println(faultTable1[i].isUrgent ? " [URGENT]" : " [NON-URGENT]");

                      if (faultTable1[i].isUrgent) table1UrgentActive = true;

                      // Send payload to ESP32: [Table ID, Mask High, Mask Low, Urgency]
                      unsigned char txFault[8] = {0}; 
                      txFault[0] = 1; 
                      txFault[1] = (uint8_t)(faultTable1[i].mask >> 8);   
                      txFault[2] = (uint8_t)(faultTable1[i].mask & 0xFF);  

                      if (faultTable1[i].isUrgent) {
                          txFault[3] = 1; 
                      } else {
                          txFault[3] = 0; 
                      }

                      CAN.sendMsgBuf(NODE0_TO_ESP32_FAULT_ID, 0, 4, txFault);
                      delay(5); 
                  }
              }

              // Update the physical light based on combined system states
              digitalWrite(FAULT_LIGHT_PIN, (table1UrgentActive || table2UrgentActive) ? LOW : HIGH);
          }
          
          // PROCESS FAULT TABLE 2 
          else if (incomingId == BMS_ID_ERR2) {
              uint16_t faultWord2 = (buf[0] << 8) | buf[1]; 
              
              static bool table1UrgentActive = false; 
              static bool table2UrgentActive = false; 
              
              table2UrgentActive = false; 

              for (int i = 0; i < NUM_FAULTS_2; i++) {
                  if (faultWord2 & faultTable2[i].mask) {
                      Serial.print("[BMS Table 2] Active: ");
                      Serial.print(faultTable2[i].dtc);
                      Serial.print(" - ");
                      Serial.print(faultTable2[i].codeName);
                      Serial.println(faultTable2[i].isUrgent ? " [URGENT]" : " [NON-URGENT]");

                      if (faultTable2[i].isUrgent) table2UrgentActive = true;

                      // Send payload to ESP32: [Table ID, Mask High, Mask Low, Urgency]
                      unsigned char txFault[8] = {0}; 
                      txFault[0] = 2; 
                      txFault[1] = (uint8_t)(faultTable2[i].mask >> 8);   
                      txFault[2] = (uint8_t)(faultTable2[i].mask & 0xFF);  

                      if (faultTable2[i].isUrgent) {
                          txFault[3] = 1; 
                      } else {
                          txFault[3] = 0; 
                      }

                      CAN.sendMsgBuf(NODE0_TO_ESP32_FAULT_ID, 0, 4, txFault);
                      delay(5); 
                  }
              }

              // Update the physical light based on combined system states
              digitalWrite(FAULT_LIGHT_PIN, (table1UrgentActive || table2UrgentActive) ? LOW : HIGH);
          }
          // Process other sensor nodes (Assumed to be Temp Nodes)
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
      static unsigned long lastSend0 = 0;
      if (millis() - lastSend0 > 1000) {
          // FIXED: Placeholder added to replace undefined tempF variable
          float bmsPlaceholderData = 0.0; 
          
          // CAN.sendMsgBuf(MY_CAN_ID, 0, 4, (unsigned char*)&bmsPlaceholderData);
          
          Serial.print(">>> [TX] Node ");
          Serial.print(BOARD_INDEX);
          Serial.print(" sent: ");
          Serial.print(bmsPlaceholderData);
          Serial.println(" F");
          lastSend0 = millis();
      }
    }

    //CODE FOR NODE 1 (MPPT)
    if(BOARD_INDEX == 1){
      static unsigned long lastSend1 = 0;
      if (millis() - lastSend1 > 1000) {
          
          // Read raw floats from physical sensors
          float t1_float = thermocouple1->readFahrenheit();
          float t2_float = thermocouple2->readFahrenheit();
          float t3_float = thermocouple3->readFahrenheit();

          // 8-byte payload array (all initialized to 0)
          unsigned char txPayload[8] = {0}; 

          // Cast floats directly to single-byte integers (0-255)
          txPayload[0] = (uint8_t)t1_float; // Thermocouple 1 -> Byte 0
          txPayload[1] = (uint8_t)t2_float; // Thermocouple 2 -> Byte 1
          txPayload[2] = (uint8_t)t3_float; // Thermocouple 3 -> Byte 2

          // Send the single 3-byte message over CAN
          CAN.sendMsgBuf(MY_CAN_ID, 0, 3, txPayload);

          // Local debug print
          Serial.print(">>> [TX] Node 1 sent single frame. T1: ");
          Serial.print(txPayload[0]);
          Serial.print("F | T2: ");
          Serial.print(txPayload[1]);
          Serial.print("F | T3: ");
          Serial.println(txPayload[2]);

          lastSend1 = millis();
      }
    }

    //CODE FOR NODE 2 (MOTORS)
    if(BOARD_INDEX == 2){
      // TRANSMIT
      static unsigned long lastSend2 = 0;
      if (millis() - lastSend2 > 1000) {
          float motorPlaceholderData = 0.0;
          // CAN.sendMsgBuf(MY_CAN_ID, 0, 4, (unsigned char*)&motorPlaceholderData);
          
          Serial.print(">>> [TX] Node ");
          Serial.print(BOARD_INDEX);
          Serial.print(" sent: ");
          Serial.print(motorPlaceholderData);
          Serial.println(" F");
          lastSend2 = millis();
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
          
          if (incomingId == 1) {
              uint8_t mppt_temp1 = buf[0];
              uint8_t mppt_temp2 = buf[1];
              uint8_t mppt_temp3 = buf[2];

              Serial.print("<<< [RX] Node 1 Data Received -> ");
              Serial.print("T1: "); Serial.print(mppt_temp1);
              Serial.print("°F | T2: "); Serial.print(mppt_temp2);
              Serial.print("°F | T3: "); Serial.print(mppt_temp3);
              Serial.println("°F");
          }
          else if (incomingId == NODE0_TO_ESP32_FAULT_ID) {
              uint8_t tableId = buf[0];
              uint16_t rxMask = (buf[1] << 8) | buf[2];
              bool rxUrgent = (buf[3] == 1);

              if (tableId == 1) {
                  for (int i = 0; i < NUM_FAULTS_1; i++) {
                      if (rxMask == faultTable1[i].mask) {
                          Serial.print("[ESP32] Table 1 DTC: ");
                          Serial.print(faultTable1[i].dtc);
                          Serial.print(" (");
                          Serial.print(faultTable1[i].codeName);
                          Serial.println(rxUrgent ? ") -> URGENT" : ")");
                      }
                  }
              }
              else if (tableId == 2) {
                  for (int i = 0; i < NUM_FAULTS_2; i++) {
                      if (rxMask == faultTable2[i].mask) {
                          Serial.print("[ESP32 ALARM] Table 2 DTC: ");
                          Serial.print(faultTable2[i].dtc);
                          Serial.print(" (");
                          Serial.print(faultTable2[i].codeName);
                          Serial.println(rxUrgent ? ") -> URGENT" : ")");
                      }
                  }
              }
          }
      }
    }
}

// Decoding BMS 0x6B0 based on your image (Big Endian)
void decodeBmsA(byte *buf) {
    int rawCurrent = (buf[0] << 8) | buf[1];
    int rawVoltage = (buf[2] << 8) | buf[3];
    int soc = buf[4];

    Serial.print("<<< [BMS 0x6B0] ");
    Serial.print("Current: "); Serial.print(rawCurrent);
    Serial.print(" | Volts: "); Serial.print(rawVoltage);
    Serial.print(" | SOC: "); Serial.print(soc);
    Serial.println("%");
}

// Decoding 0x6B1: Pack DCL, High Temp, Low Temp
void decodeBmsB(byte *buf) {
    int dcl = (buf[0] << 8) | buf[1];
    int highTemp = (int8_t)buf[4]; 
    int lowTemp = (int8_t)buf[5];

    Serial.print("<<< [BMS 0x6B1] DCL: "); Serial.print(dcl);
    Serial.print("A | High T: "); Serial.print(highTemp);
    Serial.print("C | Low T: "); Serial.print(lowTemp);
    Serial.println("C");
}