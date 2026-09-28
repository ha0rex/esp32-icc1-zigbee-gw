idf_component_register(
    SRCS "homekit_bridge.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_hap_core esp_hap_apple_profiles zigbee_host thermostat freertos nvs_flash
    PRIV_REQUIRES log
)
