#include "settings.h"

#include "app_config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "settings";

#define SETTINGS_NAMESPACE "pir"
#define SETTINGS_KEY_HOLD  "hold_s"

static uint32_t s_hold_s = OCCUPANCY_HOLD_DEFAULT_S;

uint32_t settings_clamp_hold_s(uint32_t seconds)
{
    if (seconds < OCCUPANCY_HOLD_MIN_S) {
        return OCCUPANCY_HOLD_MIN_S;
    }
    if (seconds > OCCUPANCY_HOLD_MAX_S) {
        return OCCUPANCY_HOLD_MAX_S;
    }
    return seconds;
}

esp_err_t settings_init(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        /* First boot after a flash erase: nothing stored yet, keep the default. */
        ESP_LOGI(TAG, "no stored settings, using default hold of %u s",
                 (unsigned)OCCUPANCY_HOLD_DEFAULT_S);
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "cannot open NVS namespace '%s'", SETTINGS_NAMESPACE);

    uint32_t stored = OCCUPANCY_HOLD_DEFAULT_S;
    err = nvs_get_u32(handle, SETTINGS_KEY_HOLD, &stored);
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "key '%s' absent, using default hold of %u s",
                 SETTINGS_KEY_HOLD, (unsigned)OCCUPANCY_HOLD_DEFAULT_S);
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "cannot read '%s'", SETTINGS_KEY_HOLD);

    s_hold_s = settings_clamp_hold_s(stored);
    ESP_LOGI(TAG, "occupancy hold loaded: %u s", (unsigned)s_hold_s);
    return ESP_OK;
}

uint32_t settings_get_hold_s(void)
{
    return s_hold_s;
}

esp_err_t settings_set_hold_s(uint32_t requested_s, uint32_t *out_applied_s)
{
    const uint32_t applied = settings_clamp_hold_s(requested_s);
    if (out_applied_s != NULL) {
        *out_applied_s = applied;
    }

    if (applied == s_hold_s) {
        return ESP_OK;
    }

    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle), TAG,
                        "cannot open NVS namespace '%s' for writing", SETTINGS_NAMESPACE);

    esp_err_t err = nvs_set_u32(handle, SETTINGS_KEY_HOLD, applied);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    ESP_RETURN_ON_ERROR(err, TAG, "cannot persist '%s' = %u",
                        SETTINGS_KEY_HOLD, (unsigned)applied);

    s_hold_s = applied;
    ESP_LOGI(TAG, "occupancy hold updated: %u s (requested %u s, saved to NVS)",
             (unsigned)applied, (unsigned)requested_s);
    return ESP_OK;
}
