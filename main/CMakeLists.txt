idf_component_register(
    SRCS
        "main.c"
        "motor.c"
        "tmc2208_uart.c"
        "storage.c"
        "zb_network.c"
        "led.c"
        "battery.c"
        "ble_ota.c"
        "diagnostics.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash driver esp_driver_rmt
    REQUIRES esp-zigbee-lib led_strip esp_adc app_update bt
)
