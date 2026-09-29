# ESP32-C3 Super Mini + IKEA ICC-1 Zigbee NCP bridge (EZSP/ASH host)
cmake_minimum_required(VERSION 3.16)

# Embedded in esp_app_desc; compared against GitHub OTA manifest "version".
set(PROJECT_VER "0.3.1")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_icc1_zigbee_gw)
