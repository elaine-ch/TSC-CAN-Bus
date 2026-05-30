# BMS Test Suite

Comprehensive test suite for simulating BMS (Battery Management System) error messages to test node 0's fault decoding logic.

## Quick Start (30 seconds)

1. Upload `bms_test.ino` to second Arduino (node 4, no config needed!)
2. Upload `universal_CAN.ino` to first Arduino (node 0)
3. Connect via CAN bus (CANH, CANL, GND)
4. Open serial monitors on both (115200 baud)
5. **Done!** Tests run automatically 🚀

## Overview

The `bms_test.ino` script runs on a separate Arduino (hardcoded as **node 4**) and sends CAN messages that node 0 receives and decodes. This allows you to test BMS fault handling without connecting an actual Orion BMS system.

**Key Features:**
- ✅ **Hardcoded to Node 4** (no pin configuration needed!)
- ✅ **6 comprehensive test scenarios** covering all error combinations
- ✅ **Automatic sequential mode** (tests run by themselves)
- ✅ **Single test mode** (for focused testing)
- ✅ **Works with node 0's fault priority system** (only most urgent sent to ESP32)

## Running Tests

### Sequential Mode (Recommended - Default)
By default, the script runs **all tests sequentially**, each for 10 seconds:

```cpp
#define RUN_SEQUENTIAL_TESTS  // Already enabled - tests run automatically
```

With this mode enabled:
- Tests 0-5 run automatically, cycling through each one
- Each test runs for 10 seconds, sending messages every 1 second
- Serial output clearly marks when each test starts/ends
- After test 5 completes, cycle repeats from test 0
- **Just upload and watch!** No configuration needed

### Single Test Mode (For Focused Testing)
To run a single test repeatedly, comment out `RUN_SEQUENTIAL_TESTS` and uncomment one:

```cpp
// #define RUN_SEQUENTIAL_TESTS
#define TEST_ONE_ERROR_SINGLE        // Uncomment to test only this
```

Available options:
- `TEST_ONE_ERROR_SINGLE` - Test 0: One error being sent
- `TEST_NO_ERRORS` - Test 1: No errors being sent  
- `TEST_SAME_ERROR_SAME_TYPE` - Test 2: Same error repeated
- `TEST_DIFFERENT_ERROR_TYPES` - Test 3: Different urgency levels
- `TEST_MULTIPLE_SAME_TYPE` - Test 4: Multiple urgent errors
- `TEST_MULTIPLE_DIFFERENT_TYPE` - Test 5: Mixed urgent/non-urgent

## Test Descriptions

### Test 0: One Error (URGENT)
**Purpose:** Verify node 0 can receive and decode a single urgent fault
- Sends: P0A07 (Discharge Limit Enforcement Fault - URGENT)
- Expected: Fault light turns ON, message printed to serial
- Duration: 10 seconds, every 1 second

### Test 1: No Errors
**Purpose:** Verify node 0 handles normal BMS data without errors
- Sends: Normal BMS A data (Current: 500, Voltage: 400, SOC: 75%)
- Expected: No fault messages, only normal data printed
- Duration: 10 seconds, every 1 second

### Test 2: Same Error Type (URGENT)
**Purpose:** Verify node 0 handles repeated errors of the same type
- Sends: Same error repeatedly (P0A07 - URGENT)
- Expected: Fault light stays ON throughout test
- Duration: 10 seconds, every 1 second

### Test 3: Different Error Types
**Purpose:** Verify node 0 correctly distinguishes urgent vs non-urgent errors
- Sends: Alternates between P0A07 (URGENT) and P0A10 (NON-URGENT)
- Expected: Fault light toggles ON/OFF as urgency changes
- Duration: 10 seconds, alternates every 1 second

### Test 4: Multiple Same Type (URGENT)
**Purpose:** Verify node 0 can handle multiple urgent errors
- Sends: P0A07 then P0A08 (both URGENT)
- Expected: Fault light stays ON, both faults decoded
- Duration: 10 seconds, cycles through errors

### Test 5: Multiple Different Types
**Purpose:** Verify node 0 handles mixed urgent/non-urgent from both tables
- Sends: 
  1. P0A07 (Table 1, URGENT)
  2. P0A10 (Table 1, NON-URGENT)
  3. P0AFA (Table 2, URGENT)
  4. P0A9C (Table 2, NON-URGENT)
- Expected: Fault light toggles based on urgency, all faults decoded
- Duration: 10 seconds, cycles through all 4 errors

## Fault Reference

### Table 1 Faults (0x6B2)
| Mask | DTC | Name | Type |
|------|-----|------|------|
| 0x0001 | P0A07 | Discharge Limit Enforcement Fault | URGENT |
| 0x0002 | P0A08 | Charger Safety Relay Fault | URGENT |
| 0x0004 | P0A09 | Internal Hardware Fault | NON-URGENT |
| 0x0080 | P0A10 | Pack Too Hot Fault | NON-URGENT |

### Table 2 Faults (0x6B3)
| Mask | DTC | Name | Type |
|------|-----|------|------|
| 0x0001 | P0A1F | Internal Communication Fault | URGENT |
| 0x0008 | P0AFA | Low Cell Voltage Fault | URGENT |
| 0x0400 | P0A9C | Thermistor Fault | NON-URGENT |

See `universal_CAN/bms_fault.h` for complete fault tables.

## Setup Instructions

### Simplest Setup Ever ✅
No pin configuration needed! Just upload and go.

1. **Prepare Node 0 Arduino:**
   - Upload `universal_CAN.ino`
   - Connect fault light to pin 7 (optional - for visual feedback)
   - Pin ID configuration: Pin 8 = GND, Pin 9 = GND (or leave floating for default)

2. **Prepare Test Arduino (Node 4):**
   - Upload `bms_test.ino`
   - No pin ID configuration needed (hardcoded to node 4)
   - Don't change anything - just upload!

3. **Connect Hardware:**
   - Connect both Arduinos to the same CAN bus
   - CANH line (both transceiver CANH pins)
   - CANL line (both transceiver CANL pins)
   - Common GND between both boards
   - Add 120Ω terminator between CANH and CANL on one end

4. **Monitor Output:**
   - Open serial monitor on Node 0 (115200 baud) - shows all fault detection
   - Open serial monitor on Node 4 (115200 baud) - shows test progress
   - Watch as tests cycle automatically!

## Expected Serial Output

### Node 4 (Test Arduino) Output
```
BMS TEST NODE ONLINE. Identified as ID: 4

========================================
BMS FAULT TEST SUITE
========================================

Test Descriptions:
  Test 0: One Error (URGENT) - P0A07
  Test 1: No Errors - Only normal BMS data
  Test 2: Same Error Type (URGENT) - P0A07 repeatedly
  Test 3: Different Types - P0A07 (URGENT) + P0A10 (NON-URGENT)
  Test 4: Multiple URGENT - P0A07 + P0A08 from Table 1
  Test 5: Mixed Types - Urgent + Non-Urgent from both tables

========================================

----------------------------------------
>>> STARTING TEST 0: One Error (URGENT)
----------------------------------------
  [TX ERR1] Mask: 0x1
  [TX ERR1] Mask: 0x1
  [TX ERR1] Mask: 0x1
  [TX ERR1] Mask: 0x1
  [TX ERR1] Mask: 0x1
  [TX ERR1] Mask: 0x1
  [TX ERR1] Mask: 0x1
  [TX ERR1] Mask: 0x1
  [TX ERR1] Mask: 0x1
  [TX ERR1] Mask: 0x1
----------------------------------------
<<< TEST 0 COMPLETE
----------------------------------------

>>> STARTING TEST 1: No Errors
  [TX BMS_A] No errors - Normal operation
...
```

### Node 0 (BMS Handler) Output
```
TSC NODE ONLINE. Identified as ID: 0

[BMS Table 1] Active: P0A07 - Discharge Limit Enforcement Fault [URGENT]
    >>> PRIORITIZED FOR ESP32: P0A07
[TX to ESP32] Table: 1 | DTC: P0A07 | Urgency: URGENT

[BMS Table 1] Active: P0A07 - Discharge Limit Enforcement Fault [URGENT]
    >>> PRIORITIZED FOR ESP32: P0A07
[TX to ESP32] Table: 1 | DTC: P0A07 | Urgency: URGENT
...
```

## Customization

### Change Test Duration
Edit the timing constant (in milliseconds):
```cpp
unsigned long testDuration = 10000; // Change to 5000 for 5 seconds, etc.
```

### Change Message Interval
Edit how often messages are sent during each test:
```cpp
#define MSG_INTERVAL 1000  // Change to 500 for every 0.5 seconds
```

### Modify Test Scenarios
Edit the test functions to send different fault combinations:
```cpp
void test_oneErrorSingle() {
    sendFaultTable1(0x0002);  // Change to P0A08 instead of P0A07
}
```

### Change Node ID (if needed)
The test is hardcoded to node 4, but you can change it:
```cpp
void setup() {
    // ...
    BOARD_INDEX = 3;  // Change to different node
    MY_CAN_ID = 3;    // Must match BOARD_INDEX
    // ...
}
```
⚠️ **Note:** If changing to node 3, you'll need to comment out the Node 3 code in `universal_CAN.ino` to avoid conflicts.

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No messages received | Check CAN wiring (CANH, CANL, GND), verify 500 kbps baud rate |
| Node 0 doesn't detect faults | Verify CAN ID 0x6B2 (Table 1) and 0x6B3 (Table 2) are being sent |
| Fault light doesn't respond | Verify pin 7 connection on node 0, check if urgent faults are being sent |
| Serial shows gibberish | Ensure baud rate is 115200 on both boards |
| Tests won't cycle | Ensure `RUN_SEQUENTIAL_TESTS` is defined (it's the default) |
| Want to test single scenario | Comment out `RUN_SEQUENTIAL_TESTS`, uncomment specific `TEST_*` define |
| Node 4 shows wrong ID | Check that bms_test.ino has hardcoded `MY_CAN_ID = 4` in setup() |
