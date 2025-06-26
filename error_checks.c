#include "canlib/canlib.h"

#include "mcc_generated_files/adcc.h"
#include "mcc_generated_files/fvr.h"

#include "error_checks.h"
#include "platform.h"

#include "timer.h"

//******************************************************************************
//                              STATUS CHECKS                                 //
//******************************************************************************

uint32_t check_battery_voltage_error(void) {
    uint32_t gen_err_bitfield = 0;
    uint16_t adc_result = ADCC_GetSingleConversion(channel_BATT_VOLT);
    uint16_t batt_voltage_mV = (uint16_t)(adc_result * BATT_RESISTANCE_DIVIDER);

    if (batt_voltage_mV < UNDERVOLTAGE_THRESHOLD_BATT_mV ||
        batt_voltage_mV > OVERVOLTAGE_THRESHOLD_BATT_mV) {
        uint8_t batt_data[2] = {0};
        batt_data[0] = (batt_voltage_mV >> 8) & 0xff;
        batt_data[1] = (batt_voltage_mV >> 0) & 0xff;
      
        if (batt_voltage_mV < UNDERVOLTAGE_THRESHOLD_BATT_mV) {
            gen_err_bitfield |= (1 << E_BATT_UNDER_VOLTAGE_OFFSET);
        } else {
            gen_err_bitfield |= (1 << E_BATT_OVER_VOLTAGE_OFFSET);
        }
    }

    can_msg_t batt_volt_msg; // lipo battery voltage
    build_analog_data_msg(
        PRIO_LOW,
        millis(),
        SENSOR_BATT_VOLT,
        (uint16_t)(ADCC_GetSingleConversion(channel_BATT_VOLT) 
            * BATT_RESISTANCE_DIVIDER), &batt_volt_msg
    );
    txb_enqueue(&batt_volt_msg);

    // return the bitfield, 0 if no errors
    return gen_err_bitfield;
}

uint32_t check_battery_current_error(void) {
    uint32_t gen_err_bitfield = 0;
    uint16_t curr_draw_mA = get_batt_curr_low_pass();

    if (curr_draw_mA > OVERCURRENT_THRESHOLD_BATT_mA) {
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff; // what's this for? delete?
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        gen_err_bitfield |= (1 << E_BATT_OVER_CURRENT_OFFSET);
    }

    return gen_err_bitfield;
}

uint32_t check_5v_current_error(void) {
    uint32_t gen_err_bitfield = 0;
    uint16_t curr_draw_mA = get_5v_curr_low_pass();

    if (curr_draw_mA > OVERCURRENT_THRESHOLD_5V_mA) {
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff;
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        gen_err_bitfield |= (1 << E_5V_OVER_CURRENT_OFFSET);
    }

    return gen_err_bitfield;
}

uint32_t check_12v_current_error(void) {
    uint32_t gen_err_bitfield = 0;
    uint16_t curr_draw_mA = get_12v_curr_low_pass();

    if (curr_draw_mA > OVERCURRENT_THRESHOLD_12V_mA) {
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff;
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        gen_err_bitfield |= (1 << E_12V_OVER_CURRENT_OFFSET);
    }

    return gen_err_bitfield;
}

// new health check
uint32_t check_5v_voltage_error(void) {
    uint32_t gen_err_bitfield = 0;
    uint16_t volt_draw_mV = get_5v_volt_low_pass();

    // overvoltage
    if (volt_draw_mV > OVERVOLTAGE_THRESHOLD_5V_mV) {
        uint8_t volt_data[2] = {0};
        volt_data[0] = (volt_draw_mV >> 8) & 0xff;
        volt_data[1] = (volt_draw_mV >> 0) & 0xff;

        gen_err_bitfield |= (1 << E_5V_OVER_VOLTAGE_OFFSET);
    } 
    // undervoltage
    else if (volt_draw_mV < UNDERVOLTAGE_THRESHOLD_5V_mV) {
        uint8_t volt_data[2] = {0};
        volt_data[0] = (volt_draw_mV >> 8) & 0xff;
        volt_data[1] = (volt_draw_mV >> 0) & 0xff;

        gen_err_bitfield |= (1 << E_5V_UNDER_VOLTAGE_OFFSET);
    }

    return gen_err_bitfield;
}

uint32_t efuse_5v_error(void) {
    uint32_t spe_err_bitfield = 0;
    
    // read data from pin
    uint8_t efuse_flt_5v = RC4;
    
    if (!efuse_flt_5v) {
        spe_err_bitfield |= (1 << E_5V_EFUSE_FAULT);
    }
    
    return spe_err_bitfield;
}

uint32_t efuse_12v_error(void) {
    uint32_t spe_err_bitfield = 0;
    
    // read data from pin
    uint8_t efuse_flt_12v = RC5;
    
    if (!efuse_flt_12v) {
        spe_err_bitfield |= (1 << E_12V_EFUSE_FAULT);
    }
    
    return spe_err_bitfield;
}

void generic_health_check(void) {
    static uint32_t last_check_time = 0;
    uint32_t current_time = millis();
    bool heartbeat = false;
    
    uint32_t gen_err_bitfield = 0;
    uint32_t battery_voltage_result;
    uint32_t battery_current_result;
    uint32_t current_5v_result;
    uint32_t current_12v_result;
    uint32_t voltage_5v_result;
    
    uint32_t spe_err_bitfield = 0;
    uint32_t efuse_5v_result;
    uint32_t efuse_12v_result;
    
    if (current_time - last_check_time >= 500) {
        last_check_time = current_time;
        uint32_t timestamp = millis();
        
        // visual heartbeat indicator, changed from white to blue
        BLUE_LED_SET(heartbeat);
        heartbeat = !heartbeat;
        
        // call functions to get general error bitfield
        battery_voltage_result = check_battery_voltage_error();
        battery_current_result = check_battery_current_error();
        current_5v_result = check_5v_current_error();
        current_12v_result = check_12v_current_error();
        voltage_5v_result = check_5v_voltage_error();
        
        gen_err_bitfield = battery_voltage_result | battery_current_result | 
                current_5v_result | current_12v_result | voltage_5v_result;
        
        // send general error message
        can_msg_t error_msg;
        build_general_board_status_msg(PRIO_MEDIUM, timestamp, gen_err_bitfield, 
            0, &error_msg);
        txb_enqueue(&error_msg);
        
        // call functions to get specific error bitfield
        efuse_5v_result = efuse_5v_error();
        efuse_12v_result = efuse_12v_error();
        
        spe_err_bitfield = efuse_5v_result | efuse_12v_result;
        
        // send specific error message
        can_msg_t spe_error_msg;
        // this should build board specific msg, but I didn't find an existing function
        build_general_board_status_msg(PRIO_MEDIUM, timestamp, spe_err_bitfield, 
            0, &spe_error_msg);
        txb_enqueue(&spe_error_msg);
    }

    return;
}
