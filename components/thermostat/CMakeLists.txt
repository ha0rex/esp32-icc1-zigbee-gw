idf_component_register(
    SRCS "thermostat.c"
    INCLUDE_DIRS "include"
    REQUIRES zigbee_host nvs_flash freertos esp_system
    PRIV_REQUIRES log
)
