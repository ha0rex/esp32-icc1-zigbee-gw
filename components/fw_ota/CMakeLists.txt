idf_component_register(
    SRCS "fw_ota.c"
    INCLUDE_DIRS "include"
    REQUIRES app_update
    PRIV_REQUIRES log freertos
)
