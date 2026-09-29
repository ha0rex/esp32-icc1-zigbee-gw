idf_component_register(
    SRCS "web_server.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server zigbee_host wifi_manager esp_timer freertos homekit_bridge thermostat ezsp fw_ota app_update
    PRIV_REQUIRES log
)
