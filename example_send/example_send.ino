// demo: CAN-BUS Shield, send data
// loovee@seeed.cc


#include <SPI.h>
#define SERIAL_PORT_MONITOR Serial

#define CAN_2515
//#define CAN_2518FD

// Set SPI CS Pin according to your hardware

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif


#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif

void setup() {
    SERIAL_PORT_MONITOR.begin(115200);
    while(!SERIAL_PORT_MONITOR){};

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok! Sending 'HELLO?'");
}

unsigned char stmp[6] = {'H', 'E', 'L', 'L', 'O', '?'};
void loop() {
    // send data:  id = 0x00, standrad frame, data len = 8, stmp: data buf

    CAN.sendMsgBuf(0x00, 0, 6, stmp);
    SERIAL_PORT_MONITOR.println("CAN BUS Sent: HELLO?");
    delay(1000); // Send data every 1000ms
}

// END FILE
// HI!!!!