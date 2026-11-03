#include <SPI.h>
#include "mcp2515_can.h"
#include "max6675.h"
#include "bms_fault.h"

// Arduino configuration (Seeed Studio CAN BUS Shield v2.0)
const int PIN_ID_BIT0 = 8;   
const int PIN_ID_BIT1 = 3;   
const int CAN_CS_PIN  = 9;   

// --- Node 1 Optimized Sensor Bus ---
#define SHARED_DO   A0
#define SHARED_CLK  A1

#define thermo1_CS  A2
#define thermo2_CS  A3
#define thermo3_CS  A4

#define CURRENT_SENSOR_PIN A5 

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

uint16_t currentFaultWord1 = 0;
uint16_t currentFaultWord2 = 0;
uint16_t previousFaultWord1 = 0xFFFF;  
uint16_t previousFaultWord2 = 0xFFFF;  

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
        thermocouple1 = new MAX6675(SHARED_CLK, thermo1_CS, SHARED_DO);
        thermocouple2 = new MAX6675(SHARED_CLK, thermo2_CS, SHARED_DO);
        thermocouple3 = new MAX6675(SHARED_CLK, thermo3_CS, SHARED_DO);
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
    if(BOARD_INDEX == 3){
      if (CAN_MSGAVAIL == CAN.checkReceive()) {
          unsigned char len = 0;
          unsigned char buf[8];
          CAN.readMsgBuf(&len, buf);
          unsigned long incomingId = CAN.getCanId();
          
          if (incomingId == MY_CAN_ID) return;

          if (incomingId == BMS_ID_A) {
              decodeBmsA(buf);
          } 
          else if (incomingId == BMS_ID_B) {
              decodeBmsB(buf); 
          }
          else if (incomingId == BMS_ID_ERR1) {
              uint16_t faultWord1 = (buf[0] << 8) | buf[1]; 
              currentFaultWord1 = faultWord1;

              bool anyUrgentActive = false;

              for (int i = 0; i < NUM_FAULTS_1; i++) {
                  if (faultWord1 & faultTable1[i].mask) {
                      Serial.print("[BMS Table 1] Active: ");
                      Serial.print(faultTable1[i].dtc);
                      Serial.print(" - ");
                      Serial.print(faultTable1[i].codeName);
                      Serial.println(faultTable1[i].isUrgent ? " [URGENT]" : " [NON-URGENT]");

                      if (faultTable1[i].isUrgent) {
                          anyUrgentActive = true;
                      }
                  }
              }

              digitalWrite(FAULT_LIGHT_PIN, anyUrgentActive ? LOW : HIGH);

              if (faultWord1 != previousFaultWord1) {
                  Serial.println("[Table 1 state changed - sending to ESP32]");

                  for (int i = 0; i < NUM_FAULTS_1; i++) {
                      if (faultWord1 & faultTable1[i].mask) {
                          unsigned char txFault[8] = {0};
                          txFault[0] = 1;  
                          txFault[1] = (uint8_t)(faultTable1[i].mask >> 8);
                          txFault[2] = (uint8_t)(faultTable1[i].mask & 0xFF);
                          txFault[3] = faultTable1[i].isUrgent ? 1 : 0;

                          CAN.sendMsgBuf(NODE0_TO_ESP32_FAULT_ID, 0, 4, txFault);

                          Serial.print("    >>> [TX to ESP32] ");
                          Serial.print(faultTable1[i].dtc);
                          Serial.println(faultTable1[i].isUrgent ? " [URGENT]" : " [NON-URGENT]");
                          delay(5);
                      }
                  }
                  previousFaultWord1 = faultWord1;
              }
          }
          else if (incomingId == BMS_ID_ERR2) {
              uint16_t faultWord2 = (buf[0] << 8) | buf[1];
              currentFaultWord2 = faultWord2;

              bool anyUrgentActive = false;

              for (int i = 0; i < NUM_FAULTS_2; i++) {
                  if (faultWord2 & faultTable2[i].mask) {
                      Serial.print("[BMS Table 2] Active: ");
                      Serial.print(faultTable2[i].dtc);
                      Serial.print(" - ");
                      Serial.print(faultTable2[i].codeName);
                      Serial.println(faultTable2[i].isUrgent ? " [URGENT]" : " [NON-URGENT]");

                      if (faultTable2[i].isUrgent) {
                          anyUrgentActive = true;
                      }
                  }
              }

              digitalWrite(FAULT_LIGHT_PIN, anyUrgentActive ? LOW : HIGH);

              if (faultWord2 != previousFaultWord2) {
                  Serial.println("[Table 2 state changed - sending to ESP32]");

                  for (int i = 0; i < NUM_FAULTS_2; i++) {
                      if (faultWord2 & faultTable2[i].mask) {
                          unsigned char txFault[8] = {0};
                          txFault[0] = 2;  
                          txFault[1] = (uint8_t)(faultTable2[i].mask >> 8);
                          txFault[2] = (uint8_t)(faultTable2[i].mask & 0xFF);
                          txFault[3] = faultTable2[i].isUrgent ? 1 : 0;

                          CAN.sendMsgBuf(NODE0_TO_ESP32_FAULT_ID, 0, 4, txFault);

                          Serial.print("    >>> [TX to ESP32] ");
                          Serial.print(faultTable2[i].dtc);
                          Serial.println(faultTable2[i].isUrgent ? " [URGENT]" : " [NON-URGENT]");
                          delay(5);
                      }
                  }
                  previousFaultWord2 = faultWord2;
              }
          }
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

    //CODE FOR NODE 1 (MPPT)
    if(BOARD_INDEX == 1){
      static unsigned long lastSend1 = 0;
      if (millis() - lastSend1 > 1000) {
          
          float t1_float = thermocouple1->readFahrenheit();
          delay(10);
          float t2_float = thermocouple2->readFahrenheit();
          delay(10);
          float t3_float = thermocouple3->readFahrenheit();

          int rawCurrent = analogRead(CURRENT_SENSOR_PIN);

          unsigned char txPayload[8] = {0}; 

          txPayload[0] = (uint8_t)(t1_float - 10.0); 
          txPayload[1] = (uint8_t)(t2_float - 10.0); 
          txPayload[2] = (uint8_t)(t3_float - 10.0); 
          
          txPayload[3] = (uint8_t)(rawCurrent >> 8);   
          txPayload[4] = (uint8_t)(rawCurrent & 0xFF); 

          CAN.sendMsgBuf(MY_CAN_ID, 0, 5, txPayload);

          Serial.print(">>> [TX] Node 1 sent single frame. T1: ");
          Serial.print(txPayload[0]);
          Serial.print("F | T2: ");
          Serial.print(txPayload[1]);
          Serial.print("F | T3: ");
          Serial.print(txPayload[2]);
          Serial.print("F | WCS1800 Raw: ");
          Serial.println(rawCurrent);

          lastSend1 = millis();
      }
    }

    //CODE FOR NODE 2 (MOTORS)
    if(BOARD_INDEX == 2){
      static unsigned long lastSend2 = 0;
      if (millis() - lastSend2 > 1000) {
          float motorPlaceholderData = 0.0;
          
          Serial.print(">>> [TX] Node ");
          Serial.print(BOARD_INDEX);
          Serial.print(" sent: ");
          Serial.print(motorPlaceholderData);
          Serial.println(" F");
          lastSend2 = millis();
      }
    }
}

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

void decodeBmsB(byte *buf) {
    int dcl = (buf[0] << 8) | buf[1];
    int highTemp = (int8_t)buf[4]; 
    int lowTemp = (int8_t)buf[5];

    Serial.print("<<< [BMS 0x6B1] DCL: "); Serial.print(dcl);
    Serial.print("A | High T: "); Serial.print(highTemp);
    Serial.print("C | Low T: "); Serial.print(lowTemp);
    Serial.println("C");
}