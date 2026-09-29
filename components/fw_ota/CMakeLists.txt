idf_component_register(
    SRCS "fw_ota.c"
    INCLUDE_DIRS "include"
    REQUIRES app_update esp_https_ota esp_http_client
    PRIV_REQUIRES log freertos mbedtls
)
