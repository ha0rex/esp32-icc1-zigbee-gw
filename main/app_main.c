/**
 * @file app_main.c
 * @brief ESP32-C3 host for IKEA ICC-1 EmberZNet NCP (EZSP over ASH over UART).
 */

#include <stdio.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "web_server.h"
#include "wifi_manager.h"
#include "zigbee_host.h"
#include "thermostat.h"

#if CONFIG_HK_ENABLED
#include "homekit_bridge.h"
#endif

static const char *TAG = "app_main";

#if CONFIG_HK_ENABLED
/** Defer HomeKit: HAP/mDNS right after STA IP wedges C3 TX. */
static void delayed_hk_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(45000));
    esp_err_t e = homekit_bridge_start();
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "HomeKit bridge start failed: %s", esp_err_to_name(e));
    }
    vTaskDelete(NULL);
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "ESP32-C3 + ICC-1 Zigbee NCP gateway");
    ESP_LOGI(TAG, "================================================");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if CONFIG_WIFI_ENABLED
    ESP_ERROR_CHECK(wifi_manager_start());
    if (wifi_manager_is_started()) {
        ESP_ERROR_CHECK(web_server_start());
    }
#else
    ESP_LOGW(TAG, "Wi-Fi disabled in menuconfig; ICC probing continues");
#endif

    ESP_ERROR_CHECK(zigbee_host_start());
    ESP_ERROR_CHECK(thermostat_start());

#if CONFIG_HK_ENABLED
    if (xTaskCreate(delayed_hk_task, "hk_delay", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGW(TAG, "HomeKit delay task failed — starting immediately");
        err = homekit_bridge_start();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "HomeKit bridge start failed: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGI(TAG, "HomeKit start deferred 45s (Wi‑Fi settle)");
    }
#endif
}
