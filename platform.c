#include <xc.h>

#include "mcc_generated_files/adcc.h"
#include "platform.h"

void pin_init(void) { 
    // LEDS 
    TRISA4 = 0; // set red LED pin as output 
    ANSELA4 = 0; // enable digital input buffer (Useful for reading the LED state) 
    LATA4 = LED_ON; // start with 5V power enabled 
  
    TRISC3 = 0; // set blue LED pin as output 
    ANSELC3 = 0; // enable digital input buffer (Useful for reading the LED state) 
    LATC3 = !LED_ON; // start with charging disabled 
  
    TRISA5 = 0; // set white LED pin as output 
    ANSELA5 = 0; // enable digital input buffer (Useful for reading the LED state) 
    LATA5 = !LED_ON; // start with disabled
 
    // 5V fuse fault LED 
    TRISC4 = 0; // set 5V FLT the LED pin as output
    ANSELC4 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATC4 = 0; 
    
    // 12V fuse fault LED 
    TRISC5 = 0; // set 12V FLT the LED pin as output
    ANSELC5 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATC5 = 0; 
 
    // Rocket power lines 
    LATC7 = CAN_5V_ON; 
    TRISC7 = 0; // allow 5V current line to be toggle-able (5V_Fuse_RST/EN?) 
    ANSELC7 = 1; // analog 
    LATB0 = CAN_12V_ON; 
    TRISB0 = 0; // allow 12V current line to be toggle-able (12V_RST/EN?) 
    ANSELB0 = 1; 
 
    // Current sensing 
    TRISB4 = 1; // set 12V current draw (battery) to be input 
    ANSELB4 = 1; // enable analog reading 
  
    TRISB3 = 1; // set 5V current draw (payload logic + motor) to be input 
    ANSELB3 = 1; // enable analog reading 
  
    TRISC6 = 1; // set +BATT current draw (battery) to be input 
    ANSELC6 = 1; // enable analog reading 
 
    // Voltage sensing 
    TRISB1 = 1; // set +BATT voltage to be input 
    ANSELB1 = 1; // enable analog reading 
  
    TRISA0 = 1; // set +5V voltage to be input 
    ANSELA0 = 1; // enable analog reading 
} 

void RED_LED_SET(bool value) {
    LATA2 = !value ^ LED_ON;
}

void BLUE_LED_SET(bool value) {
    LATA1 = !value ^ LED_ON;
}

void WHITE_LED_SET(bool value) {
    LATA0 = !value ^ LED_ON;
}

void CAN_5V_SET(bool value) {
    LATC7 = !value ^ CAN_5V_ON;
}

// I don't think we need this anymore
//void BATTERY_CHARGER_EN(bool value) {
//    LATA5 = !value ^ CHG_BATT_ON;
//}



// this code is for low pass filter stuff for current
// the following code was yoinked from cansw_arming
// zach derived the equation alpha = (Fs*T/5)/ 1 + (Fs*T/5)
// where Fs = sampling frequency and T = response time
// response time is equivalent to 5*tau or 5/2pi*Fc, where Fc is cutoff frequency

double alpha_low = LOW_PASS_ALPHA(LOW_PASS_RESPONSE_TIME);
double low_pass_curr_batt = 0;
double low_pass_curr_motor = 0;
double low_pass_curr_12v = 0;
double low_pass_curr_5v = 0;
//i think this is needed for 13V BATT Motor and 5V current readings? not sure tho
void update_batt_curr_low_pass(void) {
    double new_curr_reading = ADCC_GetSingleConversion(channel_BATT_CURR) / CURR_BATT_RESISTOR;
    low_pass_curr_batt = alpha_low * low_pass_curr_batt + (1.0 - alpha_low) * new_curr_reading;
}

uint16_t get_batt_curr_low_pass(void) {
    return (uint16_t)low_pass_curr_batt;
}

void update_12v_curr_low_pass(void) {
    double new_curr_reading = (ADCC_GetSingleConversion(channel_POWER_V12) * CONVERSION_ADC_TO_V / OPAMP_CURR_GAIN) 
        / (CURR_12V_RESISTOR * CURR_GAIN);
    low_pass_curr_12v = alpha_low * low_pass_curr_12v + (1.0 - alpha_low) * new_curr_reading;
}

uint16_t get_12v_curr_low_pass(void) {
    return (uint16_t)low_pass_curr_12v;
}

void update_5v_curr_low_pass(void) {
    double new_curr_reading = (ADCC_GetSingleConversion(channel_POWER_V5) * CONVERSION_ADC_TO_V / OPAMP_CURR_GAIN) 
        / (CURR_5V_RESISTOR * CURR_GAIN);
    low_pass_curr_5v = alpha_low * low_pass_curr_5v + (1.0 - alpha_low) * new_curr_reading;
}

uint16_t get_5v_curr_low_pass(void) {
    return (uint16_t)low_pass_curr_5v;
}

