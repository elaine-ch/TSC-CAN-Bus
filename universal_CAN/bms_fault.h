#ifndef BMS_FAULT_H
#define BMS_FAULT_H

#include <Arduino.h>

struct FaultEntry {
    uint16_t mask;          // The raw CAN bitmask 
    const char* dtc;        // Error code
    const char* codeName;   // Readable log string
    bool isUrgent;          // Safety evaluation flag
};

const FaultEntry faultTable1[] = {
    {0x0001, "P0A07", "Discharge Limit Enforcement Fault", true},  
    {0x0002, "P0A08", "Charger Safety Relay Fault",       true},  
    {0x0004, "P0A09", "Internal Hardware Fault",          false}, 
    {0x0008, "P0A0A", "Internal Heatsink Thermistor Fault",false}, 
    {0x0010, "P0A0B", "Internal Software Fault",          false}, 
    {0x0020, "P0A0C", "Highest Cell Voltage Too High",     false}, 
    {0x0040, "P0A0E", "Lowest Cell Voltage Too Low",       false}, 
    {0x0080, "P0A10", "Pack Too Hot Fault",               false}

};
const int NUM_FAULTS_1 = sizeof(faultTable1) / sizeof(faultTable1[0]);

const FaultEntry faultTable2[] = {
    {0x0001, "P0A1F", "Internal Communication Fault",     true},
    {0x0002, "P0A12", "Cell Balancing Stuck Off Fault",   false},
    {0x0004, "P0A80", "Weak Cell Fault",                  false},
    {0x0008, "P0AFA", "Low Cell Voltage Fault",            true},
    {0x0010, "P0A04", "Open Wiring Fault",                true},
    {0x0020, "P0AC0", "Current Sensor Fault",             true},
    {0x0040, "P0A0D", "Highest Cell Voltage Over 5V Fault",true},
    {0x0080, "P0A0F", "Cell ASIC Fault",                  true},
    {0x0100, "P0A02", "Weak Pack Fault",                  false},
    {0x0200, "P0A81", "Fan Monitor Fault",                false},
    {0x0400, "P0A9C", "Thermistor Fault",                 false},
    {0x0800, "U0100", "External Communication Fault",      true},
    {0x1000, "P0560", "Redundant Power Supply Fault",     false},
    {0x2000, "P0AA6", "High Voltage Isolation Fault",     false},
    {0x4000, "P0A05", "Input Power Supply Fault",          true},
    {0x8000, "P0A06", "Charge Limit Enforcement Fault",   true}
};
const int NUM_FAULTS_2 = sizeof(faultTable2) / sizeof(faultTable2[0]);

#endif