#include <xc.h>
#include "canlib.h"
#include "device_config.h"
#include "error_checks.h"
#include "mcc_generated_files/adcc.h"
#include "mcc_generated_files/fvr.h"
#include "platform.h"
#include "pwm.h"
#include "stdint.h"
#include "timer.h"

#ifndef BOARD_INST_UNIQUE_ID
#error "Error: No board instance ID"
#endif
#ifndef BOARD_TYPE_UNIQUE_ID
#error "Error: No board type ID"
#endif

#if !(BOARD_TYPE_UNIQUE_ID == BOARD_TYPE_ID_POWER)
#error "Error: Invalid board type ID"
#elif !(BOARD_INST_UNIQUE_ID == BOARD_INST_ID_PAYLOAD || BOARD_UNIQUE_ID == BOARD_INST_ID_ROCKET)
#error "Error: Invalid board instance ID"
#endif

static void can_msg_handler(const can_msg_t *msg);

// memory pool for the CAN tx buffer
uint8_t tx_pool[200];
volatile bool seen_can_message = false;

// LEDs: BLUE is heartbeat, WHITE is 5V enable, RED is 12V enable
int main(void) {
    // initialize mcc functions
    ADCC_Initialize();
    FVR_Initialize();

    pin_init(); // init pins
    oscillator_init(); // init the external oscillator
    timer0_init(); // init our millis() function

    // Enable global interrupts
    INTCON0bits.GIE = 1;

    // Set up CAN TX
    TRISC1 = 0;
    RC1PPS = 0x33; // Set C1 to transmit (page 267)

    // Set up CAN RX
    TRISC2 = 1;
    ANSELC2 = 0;
    CANRXPPS = 0x12; // Set C2 to receive (page 264-265)

    // set up CAN module
    can_timing_t can_setup;
    can_generate_timing_params(_XTAL_FREQ, &can_setup);
    can_init(&can_setup, can_msg_handler);
    // set up CAN tx buffer
    txb_init(tx_pool, sizeof(tx_pool), can_send, can_send_rdy);

    // loop timer
    uint32_t last_millis = 0;
    uint32_t sensor_last_millis = millis();
    uint32_t last_message_millis = millis();

    bool heartbeat = false; // moved to error checks
    
    while (1) {
        CLRWDT(); // feed the watchdog, which is set for 256ms

        if (OSCCON2 != 0x70) { // If the fail-safe clock monitor has triggered
            oscillator_init();
        }

        if (seen_can_message) {
            seen_can_message = false;
            last_message_millis = millis();
        }

        if (millis() - last_message_millis > MAX_BUS_DEAD_TIME_ms) {
            // We've got too long without seeing a valid CAN message (including
            // one of ours)
            RESET();
        }

        uint32_t mls = millis();
        if ((mls - last_millis) > MAX_LOOP_TIME_DIFF_ms) {
            // update our loop counter
            last_millis = millis();

            // visual heartbeat indicator, changed from white to blue
            BLUE_LED_SET(heartbeat); // moved to error checks
            heartbeat = !heartbeat;
            
            // visual power line indicators
            WHITE_LED_SET(LATC7 == CAN_5V_ON);
            RED_LED_SET(LATB0 == CAN_12V_ON);

            // check for general board status
            uint32_t gen_err_bitfield = 0;
            uint16_t board_specific_err_bitfield = 0;
            gen_err_bitfield |= check_battery_voltage_error();
            gen_err_bitfield |= check_battery_current_error();
            gen_err_bitfield |= check_5v_current_error();
            gen_err_bitfield |= check_12v_current_error();
            
            board_specific_err_bitfield |= efuse_5v_error();
            board_specific_err_bitfield |= efuse_12v_error();

            // Build and sent error message!
            can_msg_t board_stat_msg;

            build_general_board_status_msg(
                PRIO_LOW, millis(), gen_err_bitfield, board_specific_err_bitfield, &board_stat_msg
            );

            txb_enqueue(&board_stat_msg);

            // measures current of the CAN 5V_SW line
            can_msg_t curr_msg_5v; // measures current going into CAN 5V
            build_analog_data_msg(
                PRIO_LOW, millis(), SENSOR_5V_CURR, get_5v_curr_low_pass(), &curr_msg_5v
            );
            txb_enqueue(&curr_msg_5v);
            
            // measures voltage of the boards 5V line
            can_msg_t volt_msg_5v; // measures current going into CAN 5V
            build_analog_data_msg(
                PRIO_LOW,
                millis(),
                SENSOR_5V_VOLT,
                (uint16_t) (ADCC_GetSingleConversion(channel_5V_VOLT) * CONVERSION_ADC_TO_V) * CONVERSION_RATIO_5V_VOLT,
                &volt_msg_5v
            );
            txb_enqueue(&volt_msg_5v);
            
            can_msg_t curr_msg_12v; // measures 12V current
            build_analog_data_msg(
                PRIO_LOW, millis(), SENSOR_12V_CURR, get_12v_curr_low_pass(), &curr_msg_12v
            );
            txb_enqueue(&curr_msg_12v);

            bool result;
            
            // LiPo voltage
            can_msg_t volt_msg_batt; // voltage draw from lipo
            build_analog_data_msg(
                PRIO_LOW,
                millis(),
                SENSOR_BATT_VOLT,
                (uint16_t) (ADCC_GetSingleConversion(channel_BATT_VOLT) * CONVERSION_ADC_TO_V) * CONVERSION_RATIO_BATT_VOLT,
                &volt_msg_batt
            );
            result = txb_enqueue(&volt_msg_batt); 
            
            // LiPo current
            can_msg_t curr_msg_batt; // current draw from lipo
            build_analog_data_msg(
                PRIO_LOW, millis(), SENSOR_BATT_CURR, get_batt_curr_low_pass(), &curr_msg_batt
            );
            result = txb_enqueue(&curr_msg_batt); 

        } // ended here

        // send any queued CAN messages
        txb_heartbeat();

        // update high speed sensor lowpass
        if (millis() - sensor_last_millis > MAX_SENSOR_LOOP_TIME_DIFF_ms) {
            sensor_last_millis = millis();
            update_batt_curr_low_pass();
            update_5v_curr_low_pass();
            update_12v_curr_low_pass();
        }
    }
}

static void can_msg_handler(const can_msg_t *msg) {
    seen_can_message = true;
    uint16_t msg_type = get_message_type(msg);

    int act_id;
    int act_state;
    int dest_id;

    switch (msg_type) {
        case MSG_ACTUATOR_CMD: // this will toggle *all* battery chargers, not just CHARGING_CAN
            act_id = get_actuator_id(msg);
            act_state = get_cmd_actuator_state(msg);

            #if (BOARD_INST_UNIQUE_ID == BOARD_INST_ID_ROCKET)
            // RocketCAN 5V Line On/Off
            if (act_id == ACTUATOR_5V_RAIL_ROCKET) {
                if (act_state == ACT_STATE_ON) {
                    CAN_5V_SET(true);
                    // send error message or change 5V_EFUSE_FAULT = 0?
                } else if (act_state == ACT_STATE_OFF) {
                    CAN_5V_SET(false);
                }
            }
            // RocketCan 12V Line On/Off
            if (act_id == ACTUATOR_12V_RAIL_ROCKET) {
                if (act_state == ACT_STATE_ON) {
                    CAN_12V_SET(true);
                } else if (act_state == ACT_STATE_OFF) {
                    CAN_12V_SET(false);
                }
            }
            
            #elif (BOARD_INST_UNIQUE_ID == BOARD_INST_ID_PAYLOAD)
            // Payload 5V Line On/Off
            if (act_id == ACTUATOR_5V_RAIL_PAYLOAD) {
                if (act_state == ACT_STATE_ON) {
                    CAN_5V_SET(true);
                } else if (act_state == ACT_STATE_OFF) {
                    CAN_5V_SET(false);
                }
            }
            // Payload 12V Line On/Off
            if (act_id == ACTUATOR_12V_RAIL_PAYLOAD) {
                if (act_state == ACT_STATE_ON) {
                    CAN_12V_SET(true);
                } else if (act_state == ACT_STATE_OFF) {
                    CAN_12V_SET(false);
                }
            }
            #endif
            
            break;
            
        case MSG_LEDS_ON:
            RED_LED_SET(true);
            BLUE_LED_SET(true);
            WHITE_LED_SET(true);
            break;

        case MSG_LEDS_OFF:
            RED_LED_SET(false);
            BLUE_LED_SET(false);
            WHITE_LED_SET(false);
            break;

        case MSG_RESET_CMD:
            if (check_board_need_reset(msg)) {
                RESET();
            }
            break;

        default:
            break;
    }
}

static void __interrupt() interrupt_handler(void) {
    if (PIR5) {
        can_handle_interrupt();
    }

    // Timer0 has overflowed - update millis() function
    // This happens approximately every 500us
    if (PIE3bits.TMR0IE == 1 && PIR3bits.TMR0IF == 1) {
        timer0_handle_interrupt();
        PIR3bits.TMR0IF = 0;
    }
}
