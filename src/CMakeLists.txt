idf_component_register(
    SRCS
        "main.c"
        "board_io.c"
        "battery.c"
        "settings.c"
        "zb_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_adc
        esp_pm
        esp_timer
        nvs_flash
        espressif__esp-zigbee-lib
)
