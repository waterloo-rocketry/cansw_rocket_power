#include "mcc_generated_files/adcc.h"
#include "error_checks.h"
#include "platform.h"
#include "message_types.h"

//******************************************************************************
//                              STATUS CHECKS                                 //
//******************************************************************************

uint32_t check_battery_voltage_error(void) {
    uint32_t gen_err_bitfield = 0;
    uint16_t adc_result = ADCC_GetSingleConversion(channel_BATT_VOLT) * CONVERSION_ADC_TO_V;
    uint16_t batt_voltage_mV = (uint16_t)(adc_result * CONVERSION_RATIO_BATT_VOLT);
    
    // undervoltage
    if (batt_voltage_mV < UNDERVOLTAGE_THRESHOLD_BATT_mV) {
        gen_err_bitfield |= (1 << E_BATT_UNDER_VOLTAGE_OFFSET);
    } 
    // overvoltage
    else if(batt_voltage_mV > OVERVOLTAGE_THRESHOLD_BATT_mV) {
        gen_err_bitfield |= (1 << E_BATT_OVER_VOLTAGE_OFFSET);
    }

    // return the bitfield, 0 if no errors
    return gen_err_bitfield;
}

uint32_t check_battery_current_error(void) {
    uint16_t curr_draw_mA = get_batt_curr_low_pass();
    
    if (curr_draw_mA > OVERCURRENT_THRESHOLD_BATT_mA) {
        return (1 << E_BATT_OVER_CURRENT_OFFSET);
    }

    return 0;
}

uint32_t check_5v_current_error(void) {
    uint16_t curr_draw_mA = get_5v_curr_low_pass();

    if (curr_draw_mA > OVERCURRENT_THRESHOLD_5V_mA) {
        return (1 << E_5V_OVER_CURRENT_OFFSET);
    }

    return 0;
}

uint32_t check_12v_current_error(void) {
    uint16_t curr_draw_mA = get_12v_curr_low_pass();

    if (curr_draw_mA > OVERCURRENT_THRESHOLD_12V_mA) {
        return (1 << E_12V_OVER_CURRENT_OFFSET);
    }

    return 0;
}

// new health check
uint32_t check_5v_voltage_error(void) {
    uint32_t gen_err_bitfield = 0;
    uint16_t adc_result = (ADCC_GetSingleConversion(channel_5V_VOLT) * CONVERSION_ADC_TO_V);
    uint16_t voltage_mV = (adc_result * CONVERSION_RATIO_5V_VOLT);

    // overvoltage
    if (voltage_mV > OVERVOLTAGE_THRESHOLD_5V_mV) {
        gen_err_bitfield |= (1 << E_5V_OVER_VOLTAGE_OFFSET);
    } 
    // undervoltage
    else if (voltage_mV < UNDERVOLTAGE_THRESHOLD_5V_mV) {
        gen_err_bitfield |= (1 << E_5V_UNDER_VOLTAGE_OFFSET);
    }

    return gen_err_bitfield;
}

uint16_t efuse_5v_error(void) {
    // read data from pin
    uint8_t efuse_flt_5v = RC4;
    return (efuse_flt_5v == EFUSE_5V_FLT) << EFUSE_5V_FLT_OFFSET;
}

uint16_t efuse_12v_error(void) {
    // read data from pin
    uint8_t efuse_flt_12v = RC5;
    return (efuse_flt_12v == EFUSE_12V_FLT) << EFUSE_12V_FLT_OFFSET;
}
