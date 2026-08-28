#include "battery.h"

#include "app_config.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "battery";

/* GPIO0..GPIO6 on the ESP32-C6 are ADC1_CH0..CH6, so the channel is just the
 * pin number. Asserted at compile time so a pin change in app_config.h that
 * moves the divider off ADC1 fails the build instead of reading garbage. */
_Static_assert(PIN_BATTERY <= 6, "PIN_BATTERY must be GPIO0..GPIO6 (ADC1)");
#define BATTERY_ADC_UNIT     ADC_UNIT_1
#define BATTERY_ADC_CHANNEL  ((adc_channel_t)PIN_BATTERY)

/* 12 dB attenuation gives a ~0-3.1 V input range; the divider halves a 4.2 V
 * pack down to 2.1 V, which sits comfortably inside it. */
#define BATTERY_ADC_ATTEN    ADC_ATTEN_DB_12
#define BATTERY_ADC_BITWIDTH ADC_BITWIDTH_DEFAULT

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;   /* NULL = uncalibrated, raw fallback */

static esp_err_t calibration_init(void)
{
    const adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = BATTERY_ADC_UNIT,
        .chan     = BATTERY_ADC_CHANNEL,
        .atten    = BATTERY_ADC_ATTEN,
        .bitwidth = BATTERY_ADC_BITWIDTH,
    };

    const esp_err_t err = adc_cali_create_scheme_curve_fitting(&cfg, &s_cali);
    if (err != ESP_OK) {
        /* Not fatal: a chip without eFuse calibration data still reports a
         * usable trend, it is just a few percent off. Say so loudly once. */
        ESP_LOGW(TAG, "no ADC calibration (%s) - readings will be approximate",
                 esp_err_to_name(err));
        s_cali = NULL;
    }
    return ESP_OK;
}

esp_err_t battery_init(void)
{
    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = BATTERY_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG,
                        "cannot open ADC1");

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = BATTERY_ADC_ATTEN,
        .bitwidth = BATTERY_ADC_BITWIDTH,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, BATTERY_ADC_CHANNEL, &chan_cfg),
                        TAG, "cannot configure ADC1 channel %d", BATTERY_ADC_CHANNEL);

    return calibration_init();
}

static uint8_t mv_to_percent(uint16_t mv)
{
    float pct = ((float)mv - BATTERY_MIN_MV) * 100.0f / (BATTERY_MAX_MV - BATTERY_MIN_MV);
    if (pct < 0.0f) {
        pct = 0.0f;
    } else if (pct > 100.0f) {
        pct = 100.0f;
    }
    return (uint8_t)(pct + 0.5f);
}

esp_err_t battery_read(uint16_t *out_mv, uint8_t *out_percent)
{
    ESP_RETURN_ON_FALSE(s_adc != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "battery_init() has not run");

    uint32_t accumulated_mv = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++) {
        int raw = 0;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc, BATTERY_ADC_CHANNEL, &raw), TAG,
                            "ADC read %d/%d failed", i + 1, BATTERY_SAMPLES);

        int mv = raw;
        if (s_cali != NULL) {
            ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_cali, raw, &mv), TAG,
                                "ADC calibration conversion failed");
        }
        accumulated_mv += (uint32_t)mv;

        /* The divider is 2x1M, so the node is high-impedance; give the ADC
         * sample-and-hold time to settle between conversions. */
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    const uint16_t pack_mv =
        (uint16_t)((accumulated_mv / (float)BATTERY_SAMPLES) * BATTERY_DIVIDER_RATIO);

    if (out_mv != NULL) {
        *out_mv = pack_mv;
    }
    if (out_percent != NULL) {
        *out_percent = mv_to_percent(pack_mv);
    }
    return ESP_OK;
}
