# BMS Fault Priority System

## Overview

The modified `universal_CAN.ino` (node 0) implements a **fault priority system** that ensures only the **most urgent error** is sent to the ESP32, regardless of how many errors the BMS is reporting.

## How It Works

### Three-Layer Processing

#### Layer 1: Receive & Decode (Local)
Node 0 receives all BMS error messages from the CAN bus and prints each one to serial:
```
[BMS Table 1] Active: P0A07 - Discharge Limit Enforcement Fault [URGENT]
[BMS Table 1] Active: P0A10 - Pack Too Hot Fault [NON-URGENT]
[BMS Table 2] Active: P0A9C - Thermistor Fault [NON-URGENT]
```

#### Layer 2: Priority Evaluation
The system tracks which fault is **most urgent** and remembers it:
```
    >>> PRIORITIZED FOR ESP32: P0A07
```

#### Layer 3: Send to ESP32 (Single Message)
Only the most urgent fault is sent to the ESP32/control system:
```
[TX to ESP32] Table: 1 | DTC: P0A07 | Urgency: URGENT
```

### Priority Logic

The system applies these rules **in order**:

1. **Urgent > Non-Urgent**
   - If an urgent fault exists, it has priority over non-urgent
   - Non-urgent faults are ignored if any urgent fault is active

2. **First Urgent Wins**
   - When multiple urgent faults are active, the first one detected is sent
   - New non-urgent faults never override an urgent fault

3. **Non-Urgent Only If No Urgent**
   - Non-urgent faults are only sent to ESP32 if no urgent faults exist
   - (Note: You can modify this behavior if desired)

### Local Fault Light Behavior

The **local fault light** (pin 7) still respects all urgent faults:
- Light is **ON** (LOW) if ANY urgent fault is active in either table
- Light is **OFF** (HIGH) if no urgent faults are active

This allows immediate local alerting while filtering messages to the ESP32.

## Example Scenarios

### Scenario 1: Single Error
**BMS sends:** P0A07 (URGENT)
**Node 0 prints:**
```
[BMS Table 1] Active: P0A07 - Discharge Limit Enforcement Fault [URGENT]
    >>> PRIORITIZED FOR ESP32: P0A07
[TX to ESP32] Table: 1 | DTC: P0A07 | Urgency: URGENT
```
**Result:** Fault light ON, ESP32 receives P0A07

---

### Scenario 2: Multiple Urgent + Non-Urgent (Table 1)
**BMS sends:**
- P0A07 (URGENT)
- P0A08 (URGENT)  
- P0A10 (NON-URGENT)

**Node 0 prints:**
```
[BMS Table 1] Active: P0A07 - Discharge Limit Enforcement Fault [URGENT]
    >>> PRIORITIZED FOR ESP32: P0A07
[BMS Table 1] Active: P0A08 - Charger Safety Relay Fault [URGENT]
    [P0A08 ignored - less urgent than P0A07]
[BMS Table 1] Active: P0A10 - Pack Too Hot Fault [NON-URGENT]
    [P0A10 ignored - non-urgent with urgent active]
[TX to ESP32] Table: 1 | DTC: P0A07 | Urgency: URGENT
```
**Result:** Fault light ON, ESP32 receives only P0A07

---

### Scenario 3: Mixed Tables (Urgent from both)
**BMS sends:**
- P0A07 from Table 1 (URGENT)
- P0A1F from Table 2 (URGENT)

**Node 0 prints:**
```
[BMS Table 1] Active: P0A07 - Discharge Limit Enforcement Fault [URGENT]
    >>> PRIORITIZED FOR ESP32: P0A07
[TX to ESP32] Table: 1 | DTC: P0A07 | Urgency: URGENT

[BMS Table 2] Active: P0A1F - Internal Communication Fault [URGENT]
    [P0A1F ignored - same urgency as P0A07, first wins]
[TX to ESP32] Table: 1 | DTC: P0A07 | Urgency: URGENT
```
**Result:** Fault light ON, ESP32 receives P0A07 (first urgent detected)

---

### Scenario 4: Only Non-Urgent Errors
**BMS sends:**
- P0A10 (NON-URGENT)
- P0A9C (NON-URGENT)

**Node 0 prints:**
```
[BMS Table 1] Active: P0A10 - Pack Too Hot Fault [NON-URGENT]
    >>> PRIORITIZED FOR ESP32: P0A10
[TX to ESP32] Table: 1 | DTC: P0A10 | Urgency: NON-URGENT

[BMS Table 2] Active: P0A9C - Thermistor Fault [NON-URGENT]
    [P0A9C ignored - same urgency as P0A10, first wins]
[TX to ESP32] Table: 1 | DTC: P0A10 | Urgency: NON-URGENT
```
**Result:** Fault light OFF, ESP32 receives P0A10 (one non-urgent error)

## Testing the Priority System

### Using the BMS Test Suite

Run **Test 5: Multiple Different Types** to see the priority system in action:

```
>>> STARTING TEST 5: Multiple Different Types
[BMS Table 1] Active: P0A07 - Discharge Limit... [URGENT]
    >>> PRIORITIZED FOR ESP32: P0A07
[TX to ESP32] Table: 1 | DTC: P0A07 | Urgency: URGENT

[BMS Table 1] Active: P0A10 - Pack Too Hot... [NON-URGENT]
    [Less urgent - ignored]

[BMS Table 2] Active: P0AFA - Low Cell Voltage... [URGENT]
    [Same urgency - already have priority]

[BMS Table 2] Active: P0A9C - Thermistor Fault [NON-URGENT]
    [Less urgent - ignored]
```

Only **P0A07** is sent to ESP32, even though 4 different errors were received.

## Code Implementation Details

### Key Data Structure

```cpp
struct ActiveFault {
    uint16_t mask;           // The raw CAN bitmask
    int tableId;             // Which table (1 or 2)
    bool isUrgent;           // Urgent or non-urgent
    const char* dtc;         // Error code (e.g., "P0A07")
    const char* codeName;    // Human-readable name
};

ActiveFault mostUrgentFault = {0, 0, false, "", ""};
```

### Update Function Logic

```cpp
void updateMostUrgentFault(uint16_t faultMask, int tableId) {
    // Priority rules:
    // 1. If no fault tracked, use this one
    // 2. If current is non-urgent and new is urgent, update to new
    // 3. Otherwise keep current fault
}
```

### Send Function

```cpp
void sendFaultToESP32(const ActiveFault& fault) {
    // Sends: [TableID, MaskHigh, MaskLow, Urgency]
    // Only called once per fault reception with the priority fault
}
```

## Customization Options

### Send All Faults (Original Behavior)
To revert to sending all faults to ESP32, comment out the priority system calls and uncomment the old direct sending logic.

### Higher Priority for Specific Faults
Modify `updateMostUrgentFault()` to give special priority to specific DTCs:

```cpp
// Give P0A1F highest priority
if (strcmp(faultEntry->dtc, "P0A1F") == 0) {
    mostUrgentFault = /* ... */;
    return;
}
```

### Timeout Clearing
Add a timeout to clear the tracked fault if no new messages are received:

```cpp
unsigned long lastFaultTime = 0;
const unsigned long FAULT_TIMEOUT = 5000; // 5 seconds

if (millis() - lastFaultTime > FAULT_TIMEOUT) {
    mostUrgentFault = {0, 0, false, "", ""};
}
```

## Benefits

✅ **ESP32 receives fewer messages** - Only the most important fault  
✅ **Cleaner error reporting** - No noise from minor errors during critical failures  
✅ **Local safety maintained** - Fault light still responds to all urgent errors  
✅ **Clear priority system** - Easy to understand and modify  
✅ **Backward compatible** - Serial output still shows all detected errors  
