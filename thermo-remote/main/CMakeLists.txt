idf_component_register(
    SRCS "app_main.c" "epd.c" "ui.c" "buttons.c" "link.c" "climate.c" "battery.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_wifi esp_event esp_netif esp_http_server esp_http_client driver esp_timer esp_adc esp_driver_usb_serial_jtag
)
