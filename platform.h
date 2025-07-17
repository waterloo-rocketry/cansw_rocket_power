#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

#define LED_ON 0
#define CAN_5V_ON 1
#define CAN_12V_ON 1

#define SAMPLE_FREQ (1000.0 / MAX_SENSOR_LOOP_TIME_DIFF_ms)
#define LOW_PASS_ALPHA(TR) ((SAMPLE_FREQ * TR / 5.0) / (1 + SAMPLE_FREQ * TR / 5.0))
#define LOW_PASS_RESPONSE_TIME 1 // seconds

// Time between main loop code execution
#define MAX_LOOP_TIME_DIFF_ms 500
// Time between "high speed" sensor checks
#define MAX_SENSOR_LOOP_TIME_DIFF_ms 25
// Reset if we go this long without seeing any CAN messages (including our own)
#define MAX_BUS_DEAD_TIME_ms 10000

// Voltage monitoring in 100k Ohms
#define BATT_RESISTANCE_DIVIDER 5.6
#define GROUND_RESISTANCE_DIVIDER 5.6
// Current monitoring in units of 10 milliohms
#define CURR_5V_RESISTOR 1.67 // in kiloohms
#define CURR_12V_RESISTOR 1.67 // in kiloohms
#define OPAMP_CURR_GAIN 8.15
#define CURR_GAIN 0.182 // mA/A
#define BATT_CURR_GAIN 1 // 100V/V * 0.01R shunt gives us mA
#define CONVERSION_RATIO_BATT_VOLT 3.2 // (220 + 100)/100
#define CONVERSION_RATIO_5V_VOLT 2.5 // (15 + 10)/10

#define E_12V_EFUSE_FAULT 0x0B
#define E_5V_EFUSE_FAULT 0x0C

#define VREF 4.096
#define MAX_COUNTS 4096
#define CONVERSION_ADC_TO_V VREF * 1000 / MAX_COUNTS

void pin_init(void);

void RED_LED_SET(bool value);
void BLUE_LED_SET(bool value);
void WHITE_LED_SET(bool value);
void CAN_5V_SET(bool value);
void CAN_12V_SET(bool value);
void BATTERY_CHARGER_EN(bool value);

void update_batt_curr_low_pass(void);
// returns the value from the lower cut off frequency filter
uint16_t get_batt_curr_low_pass(void);

void update_12v_curr_low_pass(void);
// returns the value from the lower cut off frequency filter
uint16_t get_12v_curr_low_pass(void);

void update_5v_curr_low_pass(void);
// returns the value from the lower cut off frequency filter
uint16_t get_5v_curr_low_pass(void);

#endif /* BOARD_H */
