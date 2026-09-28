idf_component_register(
    SRCS "ezsp.c" "ezsp_v8.c"
    INCLUDE_DIRS "include"
    REQUIRES ash freertos esp_common esp_timer
    PRIV_REQUIRES log
)
