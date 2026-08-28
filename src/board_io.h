/*
 * Board I/O: the PIR line, the factory-reset button and the status LED.
 *
 * Also owns the light-sleep wake configuration for those two inputs, because
 * the wake trigger and the interrupt trigger have to stay in lock-step (see
 * board_io.c for why they are level-triggered rather than edge-triggered).
 */
#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    BOARD_INPUT_PIR,
    BOARD_INPUT_BUTTON,
} board_input_t;

/**
 * @brief Configure the LED, both inputs and the GPIO light-sleep wake source.
 *
 * @param[in] event_sink Queue that receives APP_EVT_PIR_CHANGED /
 *                       APP_EVT_BUTTON_CHANGED events from ISR context.
 *                       Must outlive this module.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG on a NULL queue, or the first failing
 *         GPIO driver error.
 */
esp_err_t board_io_init(QueueHandle_t event_sink);

/**
 * @brief Re-arm one input after its event has been consumed.
 *
 * The ISR masks the input when it fires (a level-triggered interrupt would
 * otherwise re-enter forever), so the application MUST call this once it has
 * acted on the event, or that input goes deaf.
 */
esp_err_t board_io_rearm(board_input_t input);

bool board_io_motion_active(void);
bool board_io_button_pressed(void);

void board_io_led_set(bool on);

/** Blink the LED @p times, @p period_ms per on+off cycle. Blocking. */
void board_io_led_blink(int times, int period_ms);
