idf_component_register(
    SRCS "fw_ota.c"
    INCLUDE_DIRS "include"
    REQUIRES app_update nvs_flash
    PRIV_REQUIRES log freertos
)
