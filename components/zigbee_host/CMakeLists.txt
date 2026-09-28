idf_component_register(
    SRCS "zigbee_host.c"
    INCLUDE_DIRS "include"
    REQUIRES ezsp ash freertos esp_common nvs_flash
    PRIV_REQUIRES log esp_timer
)
