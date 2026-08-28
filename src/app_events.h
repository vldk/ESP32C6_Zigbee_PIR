/*
 * The one message type every module posts into the application queue.
 *
 * Modules (board_io, zb_sensor, the timers in main.c) never call each other
 * directly - they only push app_event_t onto a queue handed to them at init.
 * That keeps ISR context, the Zigbee stack task and the application state
 * machine on separate threads without any shared mutable state.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_EVT_PIR_CHANGED,        /* uses level_asserted: true = motion         */
    APP_EVT_BUTTON_CHANGED,     /* uses level_asserted: true = pressed        */
    APP_EVT_HOLD_EXPIRED,       /* occupancy hold timer fired                 */
    APP_EVT_RESET_HOLD_ELAPSED, /* button was held for RESET_HOLD_S           */
    APP_EVT_HEARTBEAT,          /* periodic battery + keepalive report        */
    APP_EVT_ZB_JOINED,          /* stack reports the device is on a network   */
    APP_EVT_ZB_HOLD_SETPOINT,   /* uses setpoint: z2m wrote a new hold value  */
} app_event_id_t;

typedef struct {
    app_event_id_t id;
    union {
        bool  level_asserted;
        float setpoint;
    };
} app_event_t;
