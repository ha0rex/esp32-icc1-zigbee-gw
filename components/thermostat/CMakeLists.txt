idf_component_register(
    SRCS "thermostat.c"
    INCLUDE_DIRS "include"
    REQUIRES zigbee_host nvs_flash freertos esp_system esp_timer peer_link
    PRIV_REQUIRES log
)
