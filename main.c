#include <xc.h>
#include "canlib/canlib.h"
//#include "device_config.h"
//#include "error_checks.h"
//#include "platform.h"
#include "stdint.h"

// device config, error checks, platform to be written
// device config: 
// error checks: all functions to check for errors
//

#ifndef BOARD_UNIQUE_ID
#error "Error: No board ID"
#endif
#if !(BOARD_UNIQUE_ID == BOARD_ID_POWER_MAIN || BOARD_UNIQUE_ID == BOARD_ID_POWER_PAYLOAD)
#error "Error: Invalid board ID"
#endif

static void can_msg_handler(const can_msg_t *msg);
static void send_status_ok(void);
// memory pool for the CAN tx buffer
uint8_t tx_pool[100];
volatile bool seen_can_message = false;


int main(int argc, char** argv) {

    return (EXIT_SUCCESS);
}

static void send_status_ok(void) {
    can_msg_t board_stat_msg;
    build_board_stat_msg(millis(), E_NOMINAL, NULL, 0, &board_stat_msg);
    txb_enqueue(&board_stat_msg);
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

