#include "zb_sensor.h"

#include <string.h>

#include "app_config.h"
#include "app_events.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_power_config.h"

static const char *TAG = "zb";

/* HA "Occupancy Sensor" device id (ZCL HA spec 1.2, table 5.1). The SDK enum
 * stops at the IAS devices, so it is spelled out here. */
#define HA_OCCUPANCY_SENSOR_DEVICE_ID   0x0107
#define HA_SIMPLE_SENSOR_DEVICE_ID      ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID

#define ZB_TASK_STACK_SIZE  4096
#define ZB_TASK_PRIORITY    5
#define ZB_LOCK_TIMEOUT     pdMS_TO_TICKS(2000)
#define ZB_RETRY_DELAY_MS   1000

/* BatteryPercentageRemaining is expressed in half-percent steps (0..200). */
#define ZCL_PERCENT_TO_HALF_PERCENT(p)  ((uint8_t)((p) * 2))
/* BatteryVoltage is expressed in units of 100 mV. */
#define ZCL_MV_TO_DECIVOLT(mv)          ((uint8_t)(((mv) + 50) / 100))

static QueueHandle_t s_event_sink;
static uint32_t      s_initial_hold_s;
static volatile bool s_joined;

/* ------------------------------------------------------------------ helpers */

/* ZCL character strings are length-prefixed, not NUL-terminated. The buffers
 * are static because the cluster keeps pointing at them after registration. */
#define ZCL_STRING_DEF(name, text)                                  \
    static char name[1 + sizeof(text) - 1] = { sizeof(text) - 1 };  \
    memcpy(&name[1], (text), sizeof(text) - 1)

static void post_event(const app_event_t *evt)
{
    if (s_event_sink == NULL) {
        return;
    }
    if (xQueueSend(s_event_sink, evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "application queue full, dropped event %d", (int)evt->id);
    }
}

/*
 * Send a one-shot, unsolicited report.
 *
 * A sleepy end device never completes the bind + configureReporting handshake
 * z2m normally does (it times out while the device is asleep), so the device
 * pushes reports on its own instead of waiting to be asked. z2m accepts them
 * either way.
 */
static esp_err_t report_attribute(uint8_t endpoint, uint16_t cluster_id, uint16_t attr_id)
{
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = { .src_endpoint = endpoint },
        .address_mode  = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID     = cluster_id,
        .attributeID   = attr_id,
        .direction     = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .manuf_code    = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    return esp_zb_zcl_report_attr_cmd_req(&cmd);
}

static esp_err_t set_and_report(uint8_t endpoint, uint16_t cluster_id, uint16_t attr_id,
                                void *value, const char *what)
{
    ESP_RETURN_ON_FALSE(esp_zb_lock_acquire(ZB_LOCK_TIMEOUT), ESP_ERR_TIMEOUT, TAG,
                        "%s: timed out waiting for the Zigbee lock", what);

    esp_err_t err = ESP_OK;
    const esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
        endpoint, cluster_id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr_id, value, false);

    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "%s: cannot set ep%u cluster 0x%04x attr 0x%04x (zcl status %d)",
                 what, endpoint, cluster_id, attr_id, status);
        err = ESP_FAIL;
    } else if (!s_joined) {
        /* The local attribute is up to date but nothing left the radio. This
         * used to return ESP_OK, which made a silently skipped report
         * indistinguishable from a delivered one - the caller had no way to
         * know it needed to try again once we rejoined. */
        ESP_LOGW(TAG, "%s: not joined, report skipped", what);
        err = ESP_ERR_INVALID_STATE;
    } else {
        err = report_attribute(endpoint, cluster_id, attr_id);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: report failed: %s", what, esp_err_to_name(err));
        }
    }

    esp_zb_lock_release();
    return err;
}

/* ------------------------------------------------------------ endpoint model */

static esp_zb_cluster_list_t *create_common_clusters(void)
{
    ZCL_STRING_DEF(manufacturer, ZB_MANUFACTURER_NAME);
    ZCL_STRING_DEF(model, ZB_MODEL_IDENTIFIER);

    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version  = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_BATTERY,
    };
    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
        basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
        basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model));

    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
    };

    esp_zb_cluster_list_t *clusters = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(
        clusters, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(
        clusters, esp_zb_identify_cluster_create(&identify_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return clusters;
}

static esp_zb_cluster_list_t *create_occupancy_endpoint(void)
{
    esp_zb_cluster_list_t *clusters = create_common_clusters();

    esp_zb_occupancy_sensing_cluster_cfg_t occupancy_cfg = {
        .occupancy          = 0,
        .sensor_type        = ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_PIR,
        .sensor_type_bitmap = 1 << ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_PIR,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_occupancy_sensing_cluster(
        clusters, esp_zb_occupancy_sensing_cluster_create(&occupancy_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* The Power-Config cluster is created with its mains attributes only, so
     * the battery ones have to be added by hand. */
    esp_zb_attribute_list_t *power = esp_zb_power_config_cluster_create(NULL);
    uint8_t battery_voltage = 0;
    uint8_t battery_percent = 0;
    uint8_t battery_size    = ESP_ZB_ZCL_POWER_CONFIG_BATTERY_SIZE_BUILT_IN;
    uint8_t battery_qty     = 1;
    uint8_t battery_rated   = ZCL_MV_TO_DECIVOLT((uint16_t)BATTERY_MAX_MV);
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(
        power, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage));
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(
        power, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &battery_percent));
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(
        power, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, &battery_size));
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(
        power, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID, &battery_qty));
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(
        power, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_RATED_VOLTAGE_ID, &battery_rated));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(
        clusters, power, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return clusters;
}

static esp_zb_cluster_list_t *create_battery_voltage_endpoint(void)
{
    ZCL_STRING_DEF(description, "Battery Voltage");

    esp_zb_cluster_list_t *clusters = create_common_clusters();

    esp_zb_analog_input_cluster_cfg_t analog_cfg = {
        .out_of_service = false,
        .present_value  = 0.0f,
        .status_flags   = 0,
    };
    esp_zb_attribute_list_t *analog = esp_zb_analog_input_cluster_create(&analog_cfg);

    static float resolution = 0.001f;   /* 1 mV */
    ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(
        analog, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, description));
    ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(
        analog, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_RESOLUTION_ID, &resolution));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(
        clusters, analog, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return clusters;
}

static esp_zb_cluster_list_t *create_config_endpoint(uint32_t initial_hold_s)
{
    ZCL_STRING_DEF(description, "Occupancy hold (s)");

    esp_zb_cluster_list_t *clusters = create_common_clusters();

    esp_zb_analog_output_cluster_cfg_t analog_cfg = {
        .out_of_service = false,
        .present_value  = (float)initial_hold_s,
        .status_flags   = 0,
    };
    esp_zb_attribute_list_t *analog = esp_zb_analog_output_cluster_create(&analog_cfg);

    static float resolution  = 1.0f;                        /* whole seconds */
    static float min_present = (float)OCCUPANCY_HOLD_MIN_S;
    static float max_present = (float)OCCUPANCY_HOLD_MAX_S;
    ESP_ERROR_CHECK(esp_zb_analog_output_cluster_add_attr(
        analog, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_DESCRIPTION_ID, description));
    ESP_ERROR_CHECK(esp_zb_analog_output_cluster_add_attr(
        analog, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_RESOLUTION_ID, &resolution));
    ESP_ERROR_CHECK(esp_zb_analog_output_cluster_add_attr(
        analog, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID, &min_present));
    ESP_ERROR_CHECK(esp_zb_analog_output_cluster_add_attr(
        analog, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID, &max_present));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_output_cluster(
        clusters, analog, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return clusters;
}

static esp_zb_ep_list_t *create_endpoints(uint32_t initial_hold_s)
{
    esp_zb_ep_list_t *endpoints = esp_zb_ep_list_create();

    esp_zb_endpoint_config_t occupancy_ep = {
        .endpoint           = ZB_ENDPOINT_OCCUPANCY,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = HA_OCCUPANCY_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_endpoint_config_t battery_ep = {
        .endpoint           = ZB_ENDPOINT_BATTERY,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_endpoint_config_t config_ep = {
        .endpoint           = ZB_ENDPOINT_CONFIG,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };

    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpoints, create_occupancy_endpoint(), occupancy_ep));
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpoints, create_battery_voltage_endpoint(), battery_ep));
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpoints, create_config_endpoint(initial_hold_s), config_ep));
    return endpoints;
}

/* ----------------------------------------------------------- stack callbacks */

/* A remote node wrote one of our attributes. Only the config endpoint
 * PresentValue is actionable; hand it to the application task rather than
 * touching NVS from inside a stack callback. */
static esp_err_t on_set_attribute(const esp_zb_zcl_set_attr_value_message_t *message)
{
    ESP_RETURN_ON_FALSE(message != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "empty set-attribute message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG,
                        TAG, "set-attribute failed upstream, zcl status %d", message->info.status);

    if (message->info.dst_endpoint != ZB_ENDPOINT_CONFIG ||
        message->info.cluster != ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT ||
        message->attribute.id != ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID) {
        ESP_LOGD(TAG, "ignoring write to ep%u cluster 0x%04x attr 0x%04x",
                 message->info.dst_endpoint, message->info.cluster, message->attribute.id);
        return ESP_OK;
    }

    ESP_RETURN_ON_FALSE(message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_SINGLE &&
                            message->attribute.data.value != NULL,
                        ESP_ERR_INVALID_ARG, TAG,
                        "hold setpoint has unexpected type 0x%02x", message->attribute.data.type);

    const app_event_t evt = {
        .id       = APP_EVT_ZB_HOLD_SETPOINT,
        .setpoint = *(float *)message->attribute.data.value,
    };
    ESP_LOGI(TAG, "hold setpoint written: %.1f s", evt.setpoint);
    post_event(&evt);
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                   const void *message)
{
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        return on_set_attribute((const esp_zb_zcl_set_attr_value_message_t *)message);
    default:
        ESP_LOGD(TAG, "unhandled Zigbee action 0x%x", callback_id);
        return ESP_OK;
    }
}

static void retry_commissioning(uint8_t mode_mask)
{
    esp_zb_bdb_start_top_level_commissioning(mode_mask);
}

static void announce_joined(void)
{
    if (s_joined) {
        return;
    }
    s_joined = true;

    ESP_LOGI(TAG, "joined: PAN 0x%04hx, channel %d, short address 0x%04hx",
             esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());

    const app_event_t evt = { .id = APP_EVT_ZB_JOINED };
    post_event(&evt);
}

/* Called from the stack task; the SDK declares it, the application defines it. */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    const uint32_t *signal = signal_struct->p_app_signal;
    const esp_err_t status = signal_struct->esp_err_status;
    const esp_zb_app_signal_type_t sig_type = *signal;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "stack initialised, starting commissioning");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (status != ESP_OK) {
            ESP_LOGE(TAG, "stack start failed (%s), retrying in %d ms",
                     esp_err_to_name(status), ZB_RETRY_DELAY_MS);
            esp_zb_scheduler_alarm(retry_commissioning, ESP_ZB_BDB_MODE_INITIALIZATION,
                                   ZB_RETRY_DELAY_MS);
            break;
        }
        if (esp_zb_bdb_is_factory_new()) {
            ESP_LOGI(TAG, "not commissioned yet, starting network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            announce_joined();
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (status == ESP_OK) {
            announce_joined();
        } else {
            ESP_LOGW(TAG, "network steering failed (%s), retrying in %d ms",
                     esp_err_to_name(status), ZB_RETRY_DELAY_MS);
            esp_zb_scheduler_alarm(retry_commissioning, ESP_ZB_BDB_MODE_NETWORK_STEERING,
                                   ZB_RETRY_DELAY_MS);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "left the network");
        s_joined = false;
        break;

    /*
     * THE light-sleep hook, and the reason this port exists. The stack raises
     * this whenever it has nothing left to do; esp_zb_sleep_now() drops the
     * power-management lock it holds, which lets the IDF tickless idle take the
     * SoC into automatic light sleep until the next parent poll, GPIO wake or
     * esp_timer alarm. RAM and the radio state survive, so the next motion
     * event is reported in milliseconds instead of after a boot + rejoin.
     */
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        esp_zb_sleep_now();
        break;

    default:
        ESP_LOGD(TAG, "signal 0x%x (%s), status %s", sig_type,
                 esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(status));
        break;
    }
}

/* ----------------------------------------------------------------- zb task */

static void zb_task(void *arg)
{
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role         = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
            .keep_alive = ZB_ED_KEEPALIVE_MS,
        },
    };

    /* Must be enabled before esp_zb_init(): it decides whether the stack keeps
     * a permanent power-management lock from start-up. */
    esp_zb_sleep_enable(true);
    esp_zb_init(&zb_cfg);
    ESP_ERROR_CHECK(esp_zb_sleep_set_threshold(ZB_SLEEP_THRESHOLD_MS));

    /* A sleepy end device cannot listen continuously; the parent buffers
     * anything addressed to us until our next poll. */
    esp_zb_set_rx_on_when_idle(false);

    ESP_ERROR_CHECK(esp_zb_device_register(create_endpoints(s_initial_hold_s)));
    esp_zb_core_action_handler_register(zb_action_handler);
    ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK));

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();

    /* Only reached if the stack is stopped. */
    ESP_LOGE(TAG, "Zigbee main loop exited");
    vTaskDelete(NULL);
}

/* --------------------------------------------------------------- public API */

esp_err_t zb_sensor_start(const zb_sensor_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL && config->event_sink != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "event_sink queue is required");

    s_event_sink     = config->event_sink;
    s_initial_hold_s = config->initial_hold_s;

    esp_zb_platform_config_t platform_cfg = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config  = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_RETURN_ON_ERROR(esp_zb_platform_config(&platform_cfg), TAG,
                        "cannot configure the 802.15.4 radio");

    ESP_RETURN_ON_FALSE(
        xTaskCreate(zb_task, "zigbee", ZB_TASK_STACK_SIZE, NULL, ZB_TASK_PRIORITY, NULL) == pdPASS,
        ESP_ERR_NO_MEM, TAG, "cannot create the Zigbee task");

    return ESP_OK;
}

bool zb_sensor_is_joined(void)
{
    return s_joined;
}

esp_err_t zb_sensor_report_occupancy(bool occupied)
{
    uint8_t value = occupied ? 1 : 0;
    return set_and_report(ZB_ENDPOINT_OCCUPANCY, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                          ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &value, "occupancy");
}

esp_err_t zb_sensor_report_battery(uint16_t millivolts, uint8_t percent)
{
    /* BatteryVoltage (0x0020) is read-only and non-reportable in this stack,
     * which is why the volts also go out as an Analog Input on EP 11. Keep the
     * ZCL attribute in sync anyway so an explicit read returns something sane. */
    uint8_t decivolts = ZCL_MV_TO_DECIVOLT(millivolts);
    if (esp_zb_lock_acquire(ZB_LOCK_TIMEOUT)) {
        esp_zb_zcl_set_attribute_val(ZB_ENDPOINT_OCCUPANCY,
                                     ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
                                     &decivolts, false);
        esp_zb_lock_release();
    }

    uint8_t half_percent = ZCL_PERCENT_TO_HALF_PERCENT(percent);
    esp_err_t percent_err =
        set_and_report(ZB_ENDPOINT_OCCUPANCY, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                       ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                       &half_percent, "battery percentage");

    float volts = millivolts / 1000.0f;
    esp_err_t volts_err =
        set_and_report(ZB_ENDPOINT_BATTERY, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                       ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &volts,
                       "battery voltage");

    return percent_err != ESP_OK ? percent_err : volts_err;
}

esp_err_t zb_sensor_report_hold_setpoint(uint32_t seconds)
{
    float value = (float)seconds;
    return set_and_report(ZB_ENDPOINT_CONFIG, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
                          ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &value,
                          "hold setpoint");
}

void zb_sensor_factory_reset(void)
{
    ESP_LOGW(TAG, "factory reset: leaving the network and rebooting to re-pair");
    esp_zb_factory_reset();
}
