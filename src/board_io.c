#include "board_io.h"

#include <string.h>

#include "app_config.h"
#include "app_events.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/task.h"

static const char *TAG = "board_io";

/*
 * Why level-triggered instead of edge-triggered interrupts
 * -------------------------------------------------------
 * The GPIO peripheral is clocked from APB, which stops during light sleep, so
 * an edge that happens while the chip sleeps is simply lost. The only GPIO
 * event that survives sleep is a *level*: gpio_wakeup_enable() wakes the chip
 * when the pin sits at the configured level, and that level is still there
 * when the ISR is finally serviced.
 *
 * So each input is armed for the level it does NOT currently have, for both
 * the wake source and the interrupt. The ISR masks the pin (a level interrupt
 * would re-enter forever) and hands the transition to the application, which
 * calls board_io_rearm() to flip the watch to the other level.
 *
 * Race note: if the line flips back between the ISR read and the re-arm, the
 * newly armed level already matches and the interrupt fires immediately. That
 * costs one spurious wake and then self-corrects - it never loses an edge.
 */
typedef struct {
    board_input_t   id;
    gpio_num_t      pin;
    int             active_level;
    app_event_id_t  event_id;
} input_desc_t;

static const input_desc_t s_inputs[] = {
    [BOARD_INPUT_PIR]    = { BOARD_INPUT_PIR,    PIN_PIR,    PIR_ACTIVE_LEVEL,    APP_EVT_PIR_CHANGED    },
    [BOARD_INPUT_BUTTON] = { BOARD_INPUT_BUTTON, PIN_BUTTON, BUTTON_ACTIVE_LEVEL, APP_EVT_BUTTON_CHANGED },
};

static QueueHandle_t s_event_sink;

static void IRAM_ATTR input_isr(void *arg)
{
    const input_desc_t *desc = (const input_desc_t *)arg;

    gpio_intr_disable(desc->pin);

    const app_event_t evt = {
        .id             = desc->event_id,
        .level_asserted = gpio_get_level(desc->pin) == desc->active_level,
    };

    BaseType_t task_woken = pdFALSE;
    xQueueSendFromISR(s_event_sink, &evt, &task_woken);
    if (task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* Arm both the interrupt and the light-sleep wake for the level the pin does
 * not have right now, i.e. for the next transition. */
static esp_err_t watch_next_transition(const input_desc_t *desc)
{
    const gpio_int_type_t type = gpio_get_level(desc->pin)
                                     ? GPIO_INTR_LOW_LEVEL
                                     : GPIO_INTR_HIGH_LEVEL;

    ESP_RETURN_ON_ERROR(gpio_set_intr_type(desc->pin, type), TAG,
                        "GPIO%d: cannot set interrupt type %d", desc->pin, type);
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(desc->pin, type), TAG,
                        "GPIO%d: cannot arm light-sleep wake on level %d", desc->pin, type);
    ESP_RETURN_ON_ERROR(gpio_intr_enable(desc->pin), TAG,
                        "GPIO%d: cannot enable interrupt", desc->pin);
    return ESP_OK;
}

static esp_err_t configure_input(const input_desc_t *desc)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << desc->pin,
        .mode         = GPIO_MODE_INPUT,
        /* The PIR line has an external 1M pull-up and the button has none, so
         * the internal pull-up is a backup for one and the only idle-level
         * source for the other. */
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "GPIO%d: config failed", desc->pin);

    /* CONFIG_PM_SLP_DISABLE_GPIO (on by default with tickless idle) parks every
     * pad during automatic light sleep to save ~200-300 uA. gpio_sleep_sel_dis()
     * is the documented opt-out: without it this pin would lose its pull-up and
     * its wake capability the moment the chip sleeps. */
    ESP_RETURN_ON_ERROR(gpio_sleep_sel_dis(desc->pin), TAG,
                        "GPIO%d: cannot keep pad config during sleep", desc->pin);

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(desc->pin, input_isr, (void *)desc), TAG,
                        "GPIO%d: cannot install ISR", desc->pin);

    return watch_next_transition(desc);
}

esp_err_t board_io_init(QueueHandle_t event_sink)
{
    ESP_RETURN_ON_FALSE(event_sink != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "event_sink queue is required");
    s_event_sink = event_sink;

    const gpio_config_t led_cfg = {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&led_cfg), TAG, "LED GPIO%d: config failed", PIN_LED);

    /* Same opt-out as the inputs, for the opposite reason: the LED marks the
     * occupied state for as long as the hold lasts, and the device light-sleeps
     * right through that. Left parked, the pad would drop and the indicator
     * would stutter instead of staying lit. */
    ESP_RETURN_ON_ERROR(gpio_sleep_sel_dis(PIN_LED), TAG,
                        "LED GPIO%d: cannot keep pad driven during sleep", PIN_LED);
    board_io_led_set(false);

    ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1), TAG,
                        "cannot install GPIO ISR service");

    for (size_t i = 0; i < sizeof(s_inputs) / sizeof(s_inputs[0]); i++) {
        ESP_RETURN_ON_ERROR(configure_input(&s_inputs[i]), TAG,
                            "input %d: setup failed", (int)i);
    }

    /* Turns the per-pin gpio_wakeup_enable() calls above into an actual
     * light-sleep wake source. Requires PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP=n. */
    ESP_RETURN_ON_ERROR(esp_sleep_enable_gpio_wakeup(), TAG,
                        "cannot enable GPIO light-sleep wakeup");

    ESP_LOGI(TAG, "PIR=GPIO%d button=GPIO%d led=GPIO%d, GPIO light-sleep wake armed",
             PIN_PIR, PIN_BUTTON, PIN_LED);
    return ESP_OK;
}

esp_err_t board_io_rearm(board_input_t input)
{
    ESP_RETURN_ON_FALSE(input == BOARD_INPUT_PIR || input == BOARD_INPUT_BUTTON,
                        ESP_ERR_INVALID_ARG, TAG, "unknown input %d", input);
    return watch_next_transition(&s_inputs[input]);
}

bool board_io_motion_active(void)
{
    return gpio_get_level(PIN_PIR) == PIR_ACTIVE_LEVEL;
}

bool board_io_button_pressed(void)
{
    return gpio_get_level(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL;
}

void board_io_led_set(bool on)
{
    gpio_set_level(PIN_LED, on ? LED_ACTIVE_LEVEL : !LED_ACTIVE_LEVEL);
}

void board_io_led_blink(int times, int period_ms)
{
    for (int i = 0; i < times; i++) {
        board_io_led_set(true);
        vTaskDelay(pdMS_TO_TICKS(period_ms / 2));
        board_io_led_set(false);
        vTaskDelay(pdMS_TO_TICKS(period_ms / 2));
    }
}
