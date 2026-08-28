/*
 * Zigbee battery PIR presence sensor - ESP-IDF port of
 * docs/esp32c6_AM312_pullup_v3_2.ino
 * ---------------------------------------------------------------------------
 * Board : Seeed XIAO ESP32-C6
 * Sensor: AM312 mini PIR -> NPN buffer -> D1/GPIO1
 * Power : LiPo on the BAT pads, sensed through a 2x1M divider on D0/GPIO0
 *
 * What changed against the Arduino sketch
 * ---------------------------------------
 * The sketch ran its whole life inside setup() and ended every wake in
 * esp_deep_sleep_start(), so each motion event cost a full boot, radio init
 * and parent rejoin before the report could leave. This port keeps the device
 * running and joined, and lets the Zigbee stack drop into automatic light
 * sleep between events (see the CAN_SLEEP handler in zb_sensor.c). Practical
 * consequences:
 *
 *   - Motion is reported in milliseconds, not after a boot cycle.
 *   - No more "was this a motion wake or a button wake?" guessing: the GPIO
 *     that woke us also raises a normal interrupt, so the two are distinct.
 *   - The config-receive window is gone. A queued genAnalogOutput write from
 *     z2m is delivered on the next ordinary parent poll, so it no longer has
 *     to be waited for explicitly.
 *
 * Runtime shape - every module is a producer, this file is the only consumer:
 *
 *   board_io (GPIO ISR) --.
 *   esp_timer alarms -----+--> s_events queue --> app_task state machine
 *   zb_sensor (stack) ----'
 */

#include <inttypes.h>

#include "app_config.h"
#include "app_events.h"
#include "battery.h"
#include "board_io.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "settings.h"
#include "zb_sensor.h"

static const char *TAG = "app";

#define APP_QUEUE_LENGTH    12
#define APP_TASK_STACK_SIZE 4096
#define APP_TASK_PRIORITY   4

static QueueHandle_t     s_events;
static esp_timer_handle_t s_hold_timer;       /* occupancy stay-on          */
static esp_timer_handle_t s_heartbeat_timer;  /* periodic battery report    */
static esp_timer_handle_t s_reset_timer;      /* button held long enough    */

static bool    s_occupied;
static int64_t s_occupied_since_us;

/* ------------------------------------------------------------------ timers */

/* esp_timer callbacks run in the timer task, so they only enqueue - all state
 * lives in app_task and needs no locking. */
static void timer_post_event(void *arg)
{
    const app_event_t evt = { .id = (app_event_id_t)(intptr_t)arg };
    if (xQueueSend(s_events, &evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "application queue full, dropped timer event %d", (int)evt.id);
    }
}

static esp_err_t timer_create(esp_timer_handle_t *out, app_event_id_t event_id, const char *name)
{
    const esp_timer_create_args_t args = {
        .callback = timer_post_event,
        .arg      = (void *)(intptr_t)event_id,
        .name     = name,
        /* ESP_TIMER_TASK, not ISR: the callback touches a FreeRTOS queue. */
        .dispatch_method = ESP_TIMER_TASK,
    };
    return esp_timer_create(&args, out);
}

static void timer_restart_once(esp_timer_handle_t timer, uint64_t timeout_us)
{
    /* esp_timer_stop() returns ESP_ERR_INVALID_STATE when the timer is not
     * armed, which is a normal case here, not an error. */
    esp_timer_stop(timer);

    const esp_err_t err = esp_timer_start_once(timer, timeout_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cannot arm timer: %s", esp_err_to_name(err));
    }
}

/* -------------------------------------------------------------- occupancy */

static void set_occupancy(bool occupied)
{
    if (s_occupied == occupied) {
        return;
    }
    s_occupied = occupied;
    if (occupied) {
        s_occupied_since_us = esp_timer_get_time();
    }

    board_io_led_set(occupied);
    ESP_LOGI(TAG, "occupancy -> %s", occupied ? "occupied" : "clear");

    const esp_err_t err = zb_sensor_report_occupancy(occupied);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "occupancy report failed: %s", esp_err_to_name(err));
    }
}

static void extend_hold(void)
{
    timer_restart_once(s_hold_timer, (uint64_t)settings_get_hold_s() * 1000000ULL);
}

static void handle_pir_changed(bool motion)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(board_io_rearm(BOARD_INPUT_PIR));

    if (motion) {
        set_occupancy(true);
    }
    /* Restarted on the trailing edge too, so the hold is measured from the end
     * of the AM312 pulse rather than from its start. */
    extend_hold();
}

static void handle_hold_expired(void)
{
    if (!s_occupied) {
        return;
    }

    if (board_io_motion_active()) {
        const int64_t held_us = esp_timer_get_time() - s_occupied_since_us;
        if (held_us < (int64_t)OCCUPANCY_MAX_HOLD_S * 1000000LL) {
            extend_hold();
            return;
        }
        /* The buffered line has been asserted for longer than any real person
         * triggers it - most likely the transistor or the pull-up has failed.
         * Clear anyway so the sensor does not latch "occupied" forever. */
        ESP_LOGW(TAG, "PIR line asserted for over %u s, forcing clear (is D1 stuck LOW?)",
                 (unsigned)OCCUPANCY_MAX_HOLD_S);
    }

    set_occupancy(false);
}

/* ----------------------------------------------------------------- button */

static void handle_button_changed(bool pressed)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(board_io_rearm(BOARD_INPUT_BUTTON));

    if (pressed) {
        timer_restart_once(s_reset_timer, (uint64_t)RESET_HOLD_S * 1000000ULL);
    } else {
        esp_timer_stop(s_reset_timer);
    }
}

static void handle_reset_hold_elapsed(void)
{
    if (!board_io_button_pressed()) {
        return;   /* released during the last tick; ignore */
    }

    ESP_LOGW(TAG, "reset button held for %u s", (unsigned)RESET_HOLD_S);
    board_io_led_blink(5, 600);   /* ~3 s of blinking so the user sees it registered */
    zb_sensor_factory_reset();    /* erases the datasets and reboots; never returns */
}

/* -------------------------------------------------------------- reporting */

static void report_battery(void)
{
    uint16_t millivolts = 0;
    uint8_t  percent    = 0;

    const esp_err_t err = battery_read(&millivolts, &percent);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "battery read failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "battery: %u mV -> %u%% (%.3f V), hold %" PRIu32 " s",
             millivolts, percent, millivolts / 1000.0f, settings_get_hold_s());

    const esp_err_t report_err = zb_sensor_report_battery(millivolts, percent);
    if (report_err != ESP_OK) {
        ESP_LOGE(TAG, "battery report failed: %s", esp_err_to_name(report_err));
    }
}

static void handle_joined(void)
{
    report_battery();

    const esp_err_t err = zb_sensor_report_hold_setpoint(settings_get_hold_s());
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hold setpoint report failed: %s", esp_err_to_name(err));
    }

    /* Publish the current state so the coordinator is not left guessing after
     * a rejoin, then start the keepalive/battery cadence. */
    zb_sensor_report_occupancy(s_occupied);

    esp_timer_stop(s_heartbeat_timer);
    const esp_err_t timer_err =
        esp_timer_start_periodic(s_heartbeat_timer, (uint64_t)HEARTBEAT_S * 1000000ULL);
    if (timer_err != ESP_OK) {
        ESP_LOGE(TAG, "cannot start the heartbeat timer: %s", esp_err_to_name(timer_err));
    }
}

/* z2m wrote genAnalogOutput.presentValue on the config endpoint. Clamp it,
 * persist it and echo back what was actually accepted. */
static void handle_hold_setpoint(float requested)
{
    if (requested < 0.0f) {
        requested = 0.0f;
    }

    uint32_t applied = 0;
    const esp_err_t err = settings_set_hold_s((uint32_t)(requested + 0.5f), &applied);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cannot store the hold setpoint: %s", esp_err_to_name(err));
    }

    const esp_err_t report_err = zb_sensor_report_hold_setpoint(applied);
    if (report_err != ESP_OK) {
        ESP_LOGE(TAG, "hold setpoint echo failed: %s", esp_err_to_name(report_err));
    }

    /* A shortened hold should take effect on the current occupancy too. */
    if (s_occupied) {
        extend_hold();
    }
}

/* ------------------------------------------------------------- app task */

static void app_task(void *arg)
{
    /* If the PIR line is already asserted at boot, treat it as motion: the
     * original sketch made the same call for a GPIO deep-sleep wake. */
    if (board_io_motion_active()) {
        ESP_LOGI(TAG, "PIR asserted at start-up");
        set_occupancy(true);
        extend_hold();
    }

    app_event_t evt;
    while (xQueueReceive(s_events, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
        case APP_EVT_PIR_CHANGED:
            handle_pir_changed(evt.level_asserted);
            break;
        case APP_EVT_BUTTON_CHANGED:
            handle_button_changed(evt.level_asserted);
            break;
        case APP_EVT_HOLD_EXPIRED:
            handle_hold_expired();
            break;
        case APP_EVT_RESET_HOLD_ELAPSED:
            handle_reset_hold_elapsed();
            break;
        case APP_EVT_HEARTBEAT:
            report_battery();
            break;
        case APP_EVT_ZB_JOINED:
            handle_joined();
            break;
        case APP_EVT_ZB_HOLD_SETPOINT:
            handle_hold_setpoint(evt.setpoint);
            break;
        default:
            ESP_LOGW(TAG, "unknown event %d", (int)evt.id);
            break;
        }
    }

    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------- init */

static esp_err_t power_management_init(void)
{
#if CONFIG_PM_ENABLE
    /* esp_zb_sleep_now() only releases the stack power-management lock. It is
     * this configuration - with light_sleep_enable - that turns the resulting
     * idle into an actual light sleep. Without it the CPU would simply spin. */
    const esp_pm_config_t pm_config = {
        .max_freq_mhz       = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz       = CONFIG_XTAL_FREQ,
        .light_sleep_enable = true,
    };
    ESP_RETURN_ON_ERROR(esp_pm_configure(&pm_config), TAG, "cannot configure power management");
    ESP_LOGI(TAG, "automatic light sleep enabled (%d..%d MHz)",
             CONFIG_XTAL_FREQ, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    return ESP_OK;
#else
#warning "CONFIG_PM_ENABLE is off: the device will stay awake and drain the battery"
    ESP_LOGW(TAG, "power management disabled at build time - no light sleep");
    return ESP_OK;
#endif
}

static esp_err_t nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS unusable (%s), erasing it", esp_err_to_name(err));
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "cannot erase NVS");
        err = nvs_flash_init();
    }
    return err;
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_init());
    ESP_ERROR_CHECK(settings_init());

    s_events = xQueueCreate(APP_QUEUE_LENGTH, sizeof(app_event_t));
    ESP_ERROR_CHECK(s_events != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_ERROR_CHECK(timer_create(&s_hold_timer, APP_EVT_HOLD_EXPIRED, "hold"));
    ESP_ERROR_CHECK(timer_create(&s_heartbeat_timer, APP_EVT_HEARTBEAT, "heartbeat"));
    ESP_ERROR_CHECK(timer_create(&s_reset_timer, APP_EVT_RESET_HOLD_ELAPSED, "reset"));

    ESP_ERROR_CHECK(board_io_init(s_events));
    ESP_ERROR_CHECK(battery_init());
    ESP_ERROR_CHECK(power_management_init());

    /* Same two short flashes the sketch used to signal "firmware is alive". */
    board_io_led_blink(2, 200);

    const zb_sensor_config_t zb_config = {
        .event_sink     = s_events,
        .initial_hold_s = settings_get_hold_s(),
    };
    ESP_ERROR_CHECK(zb_sensor_start(&zb_config));

    ESP_ERROR_CHECK(xTaskCreate(app_task, "app", APP_TASK_STACK_SIZE, NULL,
                                APP_TASK_PRIORITY, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);

    ESP_LOGI(TAG, "started: hold %" PRIu32 " s, heartbeat %u s, model %s",
             settings_get_hold_s(), (unsigned)HEARTBEAT_S, ZB_MODEL_IDENTIFIER);
}
