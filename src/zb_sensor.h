/*
 * Zigbee end device: three endpoints, all reports pushed unsolicited.
 *
 *   EP 10  occupancy sensing + power config (battery percentage)
 *   EP 11  analog input   - battery volts   (readable AND reportable, unlike
 *                           the read-only Power-Config BatteryVoltage attr)
 *   EP 12  analog output  - occupancy hold setpoint, writable from z2m
 *
 * The stack runs in its own task. Everything public here is safe to call from
 * the application task: each entry point takes the Zigbee lock internally.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    QueueHandle_t event_sink;   /**< receives APP_EVT_ZB_JOINED / APP_EVT_ZB_HOLD_SETPOINT */
    uint32_t      initial_hold_s;
} zb_sensor_config_t;

/** Initialise the stack and start its task. Returns once the task is running. */
esp_err_t zb_sensor_start(const zb_sensor_config_t *config);

bool zb_sensor_is_joined(void);

esp_err_t zb_sensor_report_occupancy(bool occupied);
esp_err_t zb_sensor_report_battery(uint16_t millivolts, uint8_t percent);
esp_err_t zb_sensor_report_hold_setpoint(uint32_t seconds);

/** Leave the network, wipe the datasets and reboot into pairing. Never returns. */
void zb_sensor_factory_reset(void);
