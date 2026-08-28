/*
 * NVS-backed store for the one setting the user can change at runtime: how
 * long the sensor stays "occupied" after the last motion.
 *
 * Replaces the Arduino Preferences usage from the original sketch.
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

/** Loads the persisted value, falling back to OCCUPANCY_HOLD_DEFAULT_S. */
esp_err_t settings_init(void);

uint32_t settings_get_hold_s(void);

/**
 * @brief Clamp, store and publish a new hold value.
 *
 * Writes to NVS only when the clamped value actually differs, so a z2m client
 * that re-sends the same setpoint does not wear the flash.
 *
 * @param[in]  requested_s Raw value as received from the network.
 * @param[out] out_applied_s The clamped value now in effect. May be NULL.
 */
esp_err_t settings_set_hold_s(uint32_t requested_s, uint32_t *out_applied_s);

uint32_t settings_clamp_hold_s(uint32_t seconds);
