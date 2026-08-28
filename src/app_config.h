/*
 * Single source of truth for every pin and tunable in the sensor.
 * Nothing else in the project hard-codes a GPIO number or a timeout.
 */
#pragma once

#include "driver/gpio.h"

/* ---------------- Pins ---------------------------------------------------
 * AM312 OUT --[100k]--> Base (2N3904)
 *                       Emitter --> GND
 *                       Collector --> D1 / GPIO1
 * D1 / GPIO1 --[1M]--> 3V3
 *   No motion : AM312 OUT = 0V   -> NPN off -> D1 = HIGH
 *   Motion    : AM312 OUT = 1.8V -> NPN on  -> D1 = LOW
 * The NPN buffer inverts the signal, hence the active-LOW levels below.
 */
#define PIN_PIR                 GPIO_NUM_1      /* D1, buffered PIR signal   */
#define PIN_BATTERY             GPIO_NUM_0      /* D0, 2x1M divider midpoint */
#define PIN_BUTTON              GPIO_NUM_7      /* D7, external switch to GND */
#define PIN_LED                 GPIO_NUM_15     /* on-board user LED         */

#define PIR_ACTIVE_LEVEL        0               /* LOW = motion              */
#define BUTTON_ACTIVE_LEVEL     0               /* LOW = pressed             */
#define LED_ACTIVE_LEVEL        0               /* LOW = lit                 */

/* ---------------- Behaviour --------------------------------------------- */
#define OCCUPANCY_HOLD_DEFAULT_S    30U         /* factory default stay-occupied time */
#define OCCUPANCY_HOLD_MIN_S        1U          /* clamp: report even brief motion    */
#define OCCUPANCY_HOLD_MAX_S        3600U       /* clamp: one hour                    */

/* Safety net for a buffered PIR line that is stuck asserted (dead transistor,
 * shorted collector). Without it the hold timer would re-arm forever and the
 * sensor would stay "occupied" until the battery ran out. */
#define OCCUPANCY_MAX_HOLD_S        180U

#define HEARTBEAT_S                 (60U * 60U) /* battery + keepalive report period */
#define RESET_HOLD_S                5U          /* button hold to factory reset      */

/* ---------------- Battery divider --------------------------------------- */
#define BATTERY_DIVIDER_RATIO   2.0f            /* 2x1M -> divide by 2 */
#define BATTERY_MIN_MV          3000.0f         /* 0%   (~empty LiPo)  */
#define BATTERY_MAX_MV          4200.0f         /* 100% (full LiPo)    */
#define BATTERY_SAMPLES         8               /* averaged per reading */

/* ---------------- Zigbee ------------------------------------------------- */
#define ZB_ENDPOINT_OCCUPANCY   10              /* occupancy + battery percentage    */
#define ZB_ENDPOINT_BATTERY     11              /* battery volts as Analog Input     */
#define ZB_ENDPOINT_CONFIG      12              /* hold setpoint as Analog Output    */

#define ZB_MANUFACTURER_NAME    "DIY"
#define ZB_MODEL_IDENTIFIER     "AM312_Presence_v3"

/* How long the parent buffers data for us, and how often we poll it. Shorter
 * = z2m writes land sooner, longer = less radio time and more battery life. */
#define ZB_ED_KEEPALIVE_MS      3000U

/* Do not bother light sleeping for anything shorter than this - the wake-up
 * itself would cost more energy than staying awake. */
#define ZB_SLEEP_THRESHOLD_MS   20U
