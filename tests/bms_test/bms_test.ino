#include <SPI.h>
#include "mcp2515_can.h"

#if defined(ARDUINO_ARCH_ESP32)
    const int CAN_CS_PIN  = 9;  // Seeed Studio CAN BUS Shield v2.0 uses pin 9
#else
    const int CAN_CS_PIN  = 9;  // Seeed Studio CAN BUS Shield v2.0 uses pin 9
#endif

mcp2515_can CAN(CAN_CS_PIN);

// BMS Message IDs
#define BMS_ID_A    0x6B0
#define BMS_ID_B    0x6B1
#define BMS_ID_ERR1 0x6B2
#define BMS_ID_ERR2 0x6B3

// Node configuration - set to 4 for test node
unsigned long MY_CAN_ID = 4;
int BOARD_INDEX = 4;

// ===== Configure Tests to Run =====
// Define which test to run - comment/uncomment as needed
#define RUN_SEQUENTIAL_TESTS  // Run through all tests in sequence
// #define TEST_ONE_ERROR_SINGLE        // Test 1: One error being sent
// #define TEST_NO_ERRORS               // Test 2: No errors being sent
// #define TEST_SAME_ERROR_SAME_TYPE    // Test 3: One error of same type (urgent)
// #define TEST_DIFFERENT_ERROR_TYPES   // Test 4: One error urgent, one non-urgent
// #define TEST_MULTIPLE_SAME_TYPE      // Test 5: Multiple errors of same type
// #define TEST_MULTIPLE_DIFFERENT_TYPE // Test 6: Multiple errors of different types

// Test timing
unsigned long lastSendTime = 0;
unsigned long testStartTime = 0;
unsigned long testDuration = 10000; // 10 seconds per test (when ran sequentially)

// Current test being run (when ran sequentially)
int currentTest = 0;
bool testInProgress = false;

// Message timing for individual messages
unsigned long lastMsgSend = 0;
#define MSG_INTERVAL 1000  // Send message every 1 second during test

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Hardcoded to Node 4 - no pin configuration needed
    BOARD_INDEX = 4;
    MY_CAN_ID = 4;

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN Init Fail. Check SPI wiring!");
        delay(1000);
    }

    Serial.print("BMS TEST NODE ONLINE. Identified as ID: ");
    Serial.println(BOARD_INDEX);
    Serial.println("\n========================================");
    Serial.println("BMS FAULT TEST SUITE");
    Serial.println("========================================");
    printTestInfo();
}

void loop() {
    unsigned long now = millis();

#ifdef RUN_SEQUENTIAL_TESTS
    // If toggled, run through all tests
    if (!testInProgress) {
        startTest(currentTest);
        testInProgress = true;
        testStartTime = now;
    }

    // Check if test duration has elapsed
    if (now - testStartTime > testDuration) {
        endTest(currentTest);
        currentTest++;
        if (currentTest > 5) {
            currentTest = 0;  // Loop back to first test
        }
        testInProgress = false;
    }

    // Run current test logic
    if (now - lastMsgSend > MSG_INTERVAL) {
        runTest(currentTest);
        lastMsgSend = now;
    }

#else
    // Single test mode - run the selected test continuously
    if (now - lastMsgSend > MSG_INTERVAL) {
        #ifdef TEST_ONE_ERROR_SINGLE
            test_oneErrorSingle();
        #elif defined(TEST_NO_ERRORS)
            test_noErrors();
        #elif defined(TEST_SAME_ERROR_SAME_TYPE)
            test_sameErrorSameType();
        #elif defined(TEST_DIFFERENT_ERROR_TYPES)
            test_differentErrorTypes();
        #elif defined(TEST_MULTIPLE_SAME_TYPE)
            test_multipleSameType();
        #elif defined(TEST_MULTIPLE_DIFFERENT_TYPE)
            test_multipleDifferentType();
        #endif
        lastMsgSend = now;
    }
#endif
}

void sendFaultTable1(uint16_t faultMask) {
    unsigned char buf[8] = {0};
    buf[0] = (uint8_t)(faultMask >> 8);
    buf[1] = (uint8_t)(faultMask & 0xFF);
    CAN.sendMsgBuf(BMS_ID_ERR1, 0, 2, buf);
    Serial.print("  [TX ERR1] Mask: 0x");
    Serial.println(faultMask, HEX);
}

void sendFaultTable2(uint16_t faultMask) {
    unsigned char buf[8] = {0};
    buf[0] = (uint8_t)(faultMask >> 8);
    buf[1] = (uint8_t)(faultMask & 0xFF);
    CAN.sendMsgBuf(BMS_ID_ERR2, 0, 2, buf);
    Serial.print("  [TX ERR2] Mask: 0x");
    Serial.println(faultMask, HEX);
}

void printTestInfo() {
    Serial.println("\nTest Descriptions:");
    Serial.println("  Test 0: One Error (URGENT) - P0A07");
    Serial.println("  Test 1: No Errors - Only normal BMS data");
    Serial.println("  Test 2: Same Error Type (URGENT) - P0A07 repeatedly");
    Serial.println("  Test 3: Different Types - P0A07 (URGENT) + P0A10 (NON-URGENT)");
    Serial.println("  Test 4: Multiple URGENT - P0A07 + P0A08 from Table 1");
    Serial.println("  Test 5: Mixed Types - Urgent + Non-Urgent from both tables");
    Serial.println("\n========================================\n");
}

void startTest(int testNum) {
    Serial.println("----------------------------------------");
    Serial.print(">>> STARTING TEST ");
    Serial.print(testNum);
    Serial.print(": ");
    switch(testNum) {
        case 0: Serial.println("One Error (URGENT)"); break;
        case 1: Serial.println("No Errors"); break;
        case 2: Serial.println("Same Error Type (URGENT)"); break;
        case 3: Serial.println("Different Error Types"); break;
        case 4: Serial.println("Multiple Same Type (URGENT)"); break;
        case 5: Serial.println("Multiple Different Types"); break;
    }
    Serial.println("----------------------------------------");
}

void endTest(int testNum) {
    Serial.println("----------------------------------------");
    Serial.print("<<< TEST ");
    Serial.print(testNum);
    Serial.println(" COMPLETE");
    Serial.println("----------------------------------------\n");
}

void runTest(int testNum) {
    switch(testNum) {
        case 0: test_oneErrorSingle(); break;
        case 1: test_noErrors(); break;
        case 2: test_sameErrorSameType(); break;
        case 3: test_differentErrorTypes(); break;
        case 4: test_multipleSameType(); break;
        case 5: test_multipleDifferentType(); break;
    }
}

// ===== TEST CASES =====

// TEST 0: One error being sent
void test_oneErrorSingle() {
    sendFaultTable1(0x0001);  // P0A07: Discharge Limit Enforcement (URGENT)
}

// TEST 1: No errors being sent
void test_noErrors() {
    // Send only normal BMS data, no errors
    unsigned char buf[5] = {0};
    buf[0] = 0x01;  // Current high byte
    buf[1] = 0xF4;  // Current low byte (500)
    buf[2] = 0x01;  // Voltage high byte
    buf[3] = 0x90;  // Voltage low byte (400)
    buf[4] = 0x4B;  // SOC (75%)
    CAN.sendMsgBuf(BMS_ID_A, 0, 5, buf);
    Serial.println("  [TX BMS_A] No errors - Normal operation");
}

// TEST 2: One error being sent of the same type (URGENT)
void test_sameErrorSameType() {
    sendFaultTable1(0x0001);  // P0A07: Discharge Limit Enforcement (URGENT)
}

// TEST 3: One error being sent of different types (URGENT vs NON-URGENT)
void test_differentErrorTypes() {
    static bool sendUrgent = true;
    if (sendUrgent) {
        sendFaultTable1(0x0001);  // P0A07: URGENT
        sendUrgent = false;
    } else {
        sendFaultTable1(0x0080);  // P0A10: NON-URGENT (Pack Too Hot)
        sendUrgent = true;
    }
}

// TEST 4: Multiple errors being sent of same type (URGENT)
void test_multipleSameType() {
    static int messageCount = 0;
    messageCount++;

    if (messageCount <= 2) {
        sendFaultTable1(0x0001);  // P0A07: URGENT
    } else if (messageCount <= 4) {
        sendFaultTable1(0x0002);  // P0A08: URGENT
    } else {
        messageCount = 0;
    }
}

// TEST 5: Multiple errors being sent of different types (URGENT + NON-URGENT)
void test_multipleDifferentType() {
    static int messageCount = 0;
    messageCount++;

    // Cycle through:
    // 1. Urgent from Table 1
    // 2. Non-Urgent from Table 1
    // 3. Urgent from Table 2
    // 4. Non-Urgent from Table 2

    if (messageCount == 1) {
        sendFaultTable1(0x0001);  // P0A07: URGENT
    } else if (messageCount == 2) {
        sendFaultTable1(0x0080);  // P0A10: NON-URGENT
    } else if (messageCount == 3) {
        sendFaultTable2(0x0008);  // P0AFA: URGENT
    } else if (messageCount == 4) {
        sendFaultTable2(0x0400);  // P0A9C: NON-URGENT
        messageCount = 0;
    }
}
