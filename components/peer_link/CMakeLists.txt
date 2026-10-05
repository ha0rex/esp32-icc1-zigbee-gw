idf_component_register(
    SRCS "peer_link.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash esp_http_client esp_netif esp_timer freertos lwip
    PRIV_REQUIRES log esp_system
)
