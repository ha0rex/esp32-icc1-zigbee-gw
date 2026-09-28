idf_component_register(
    SRCS "ash.c" "ash_crc.c"
    INCLUDE_DIRS "include"
    REQUIRES icc_uart freertos esp_common
    PRIV_REQUIRES log
)
