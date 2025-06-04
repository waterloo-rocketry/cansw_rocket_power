#ifndef ERROR_CHECKS_H
#define ERROR_CHECKS_H

#include <stdbool.h>

#include "canlib/message_types.h"

// Updated all threshold values, left old comments unchange

// For voltages within these ranges, a warning will be sent out over CAN
// values were determined by the given voltage rail range
#define UNDERVOLTAGE_THRESHOLD_BATT_mV 11400
#define OVERVOLTAGE_THRESHOLD_BATT_mV 12700

// At this current, a warning will be sent out over CAN
#define OVERCURRENT_THRESHOLD_BATT_mA 4500 // fuse goes at 6300

// From bus line. At this current, a warning will be sent out over CAN
#define OVERCURRENT_THRESHOLD_5V_mA 1800 // buck rated for 2A max
#define OVERCURRENT_THRESHOLD_12V_mA 2300 // buck rated for 3A max

// Voltage threshold for 5V
#define UNDERVOLTAGE_THRESHOLD_5V_mV 4500 
#define OVERVOLTAGE_THRESHOLD_5V_mV 5200

// Fault signals are active low
#define 12V_Fuse_FLT 0
#define 5V_Fuse_FLT 0

// General board status checkers
bool check_battery_voltage_error(void);
bool check_battery_current_error(void);
bool check_5v_current_error(void);
bool check_12v_current_error(void);

#endif /* ERROR_CHECKS_H */
