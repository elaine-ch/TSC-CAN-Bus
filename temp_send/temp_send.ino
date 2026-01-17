#include <SPI.h>          // Library for Serial Peripheral Interface (how Arduino talks to the CAN shield)
#include "mcp2515_can.h"  // Library specifically for the MCP2515 CAN controller chip

const int SPI_CS_PIN = 9; // Chip Select pin: tells the Arduino which SPI device to talk to
const int sensorPin = A0; // The Analog pin where your sensor's signal wire is connected

mcp2515_can CAN(SPI_CS_PIN); // Initialize the CAN library with our CS pin

void setup() {
    Serial.begin(115200); // Open the Serial Monitor so we can see what's happening

    // Loop here until the CAN shield responds; 500KBPS is the communication speed
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        delay(100);
    }
    Serial.println("CAN Ready!");
}

void loop() {
    digitalWrite(13, HIGH); // Turn on onboard LED
    delay(100);
    digitalWrite(13, LOW);  // Turn off onboard LED
    delay(100);
   // 1. Read raw ADC value (0-1023 for 10-bit Arduino)
    int rawADC = analogRead(sensorPin);
    
    // 2. Convert ADC to Voltage (assuming 5V system, adjust to 3.3V if needed)
    float voltage = rawADC * (3.3 / 1023.0);
    
    // 3. Apply your formula: Temp °C = 100 * (V) - 50
    float tempC = (100.0 * voltage) - 50.0;
    
    // 4. Convert Celsius to Fahrenheit: F = (C * 1.8) + 32
    float tempF = (tempC * 1.8) + 32.0;

    // --- CAN Bus Transmission Logic ---
    // Assuming you are using a library like mcp_can or similar for your 
    // distributed sensor nodes [cite: 19]
    
    unsigned char stmp[4]; // Array to hold the float data
    
    // Pack the float into the byte array to send over CAN bus
    memcpy(stmp, &tempF, sizeof(tempF)); 

    // Send the message (Standard ID: 0x123, 4 bytes of data)
    // This supports your real-time data acquisition pipeline 
    CAN.sendMsgBuf(0x123, 0, 4, stmp);

    delay(100); // 10Hz refresh rate for the dashboard [cite: 20, 35]
}