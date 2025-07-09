#ifndef ERROR_CHECKS_H
#define ERROR_CHECKS_H

#include <stdbool.h>

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
#define EFUSE_12V_FLT 0 
#define EFUSE_5V_FLT 0

// Standard Error Code
typedef enum {
    EFUSE_12V_FLT_OFFSET = 0,
    EFUSE_5V_FLT_OFFSET
} specific_error_offsets;

// For all error check functions, 
// false means nominal true means error

// General board status checkers
uint32_t check_battery_voltage_error(void);
uint32_t check_battery_current_error(void);
uint32_t check_5v_current_error(void);
uint32_t check_12v_current_error(void);

// Specific board status checkers
uint16_t efuse_5v_error(void);
uint16_t efuse_12v_error(void);

#endif /* ERROR_CHECKS_H */
