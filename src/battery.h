/*
 * Battery sense: LiPo -> 2x1M divider -> ADC1 on PIN_BATTERY.
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t battery_init(void);

/**
 * @brief Take an averaged reading of the pack voltage.
 *
 * @param[out] out_mv      Pack millivolts (already multiplied back up through
 *                         the divider). May be NULL.
 * @param[out] out_percent 0-100, linear between BATTERY_MIN_MV and
 *                         BATTERY_MAX_MV. May be NULL.
 */
esp_err_t battery_read(uint16_t *out_mv, uint8_t *out_percent);
