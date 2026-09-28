idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES zigbee_host wifi_manager web nvs_flash esp_event homekit_bridge thermostat
)
