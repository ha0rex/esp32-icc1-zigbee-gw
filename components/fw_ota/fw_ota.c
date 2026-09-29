/**
 * @file fw_ota.c
 * @brief HTTPS OTA from the GitHub rolling "ota" release (published from main).
 */

#include "fw_ota.h"

#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "fw_ota";

#ifndef CONFIG_FW_OTA_MANIFEST_URL
#define CONFIG_FW_OTA_MANIFEST_URL \
    "https://github.com/ha0rex/esp32-icc1-zigbee-gw/releases/download/ota/manifest.json"
#endif

typedef struct {
    fw_ota_state_t state;
    char running_version[32];
    char available_version[32];
    char firmware_url[256];
    char message[96];
    bool update_available;
    int progress_pct;
} fw_ota_ctx_t;

static fw_ota_ctx_t s_ctx;
static SemaphoreHandle_t s_mu;
static TaskHandle_t s_upg_task;

static void lock(void)
{
    if (!s_mu) {
        s_mu = xSemaphoreCreateMutex();
    }
    if (s_mu) {
        xSemaphoreTake(s_mu, portMAX_DELAY);
    }
}

static void unlock(void)
{
    if (s_mu) {
        xSemaphoreGive(s_mu);
    }
}

static void set_running_version(void)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    if (desc && desc->version[0]) {
        snprintf(s_ctx.running_version, sizeof(s_ctx.running_version), "%s", desc->version);
    } else {
        snprintf(s_ctx.running_version, sizeof(s_ctx.running_version), "unknown");
    }
}

const char *fw_ota_state_str(fw_ota_state_t st)
{
    switch (st) {
    case FW_OTA_IDLE:
        return "idle";
    case FW_OTA_CHECKING:
        return "checking";
    case FW_OTA_AVAILABLE:
        return "available";
    case FW_OTA_UP_TO_DATE:
        return "up_to_date";
    case FW_OTA_UPDATING:
        return "updating";
    case FW_OTA_FAILED:
        return "failed";
    case FW_OTA_REBOOTING:
        return "rebooting";
    default:
        return "unknown";
    }
}

void fw_ota_get_status(fw_ota_status_t *out)
{
    if (!out) {
        return;
    }
    lock();
    if (!s_ctx.running_version[0]) {
        set_running_version();
    }
    out->state = s_ctx.state;
    snprintf(out->running_version, sizeof(out->running_version), "%s", s_ctx.running_version);
    snprintf(out->available_version, sizeof(out->available_version), "%s", s_ctx.available_version);
    snprintf(out->message, sizeof(out->message), "%s", s_ctx.message);
    out->update_available = s_ctx.update_available;
    out->progress_pct = s_ctx.progress_pct;
    unlock();
}

/** Minimal JSON string extractor: "key":"value" (no escaped quotes in value). */
static bool json_get_string(const char *json, const char *key, char *out, size_t out_len)
{
    if (!json || !key || !out || out_len == 0) {
        return false;
    }
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (!p) {
        return false;
    }
    p = strchr(p + strlen(pat), ':');
    if (!p) {
        return false;
    }
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
        p++;
    }
    if (*p != '"') {
        return false;
    }
    p++;
    size_t n = 0;
    while (*p && *p != '"' && n + 1 < out_len) {
        if (*p == '\\' && p[1]) {
            p++;
        }
        out[n++] = *p++;
    }
    out[n] = '\0';
    return n > 0;
}

typedef struct {
    char *buf;
    size_t cap;
    size_t len;
} http_buf_t;

static esp_err_t http_event(esp_http_client_event_t *evt)
{
    http_buf_t *hb = (http_buf_t *)evt->user_data;
    if (!hb || !hb->buf) {
        return ESP_OK;
    }
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data_len > 0) {
        size_t copy = (size_t)evt->data_len;
        if (hb->len + copy >= hb->cap) {
            copy = hb->cap - hb->len - 1;
        }
        if (copy > 0) {
            memcpy(hb->buf + hb->len, evt->data, copy);
            hb->len += copy;
            hb->buf[hb->len] = '\0';
        }
    }
    return ESP_OK;
}

static esp_err_t http_get_text(const char *url, char *out, size_t out_len, int *http_status)
{
    if (http_status) {
        *http_status = 0;
    }
    if (!url || !out || out_len < 8) {
        return ESP_ERR_INVALID_ARG;
    }
    out[0] = '\0';
    http_buf_t hb = {.buf = out, .cap = out_len, .len = 0};

    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 20000,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = http_event,
        .user_data = &hb,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    if (http_status) {
        *http_status = status;
    }
    esp_http_client_cleanup(client);
    if (err != ESP_OK) {
        return err;
    }
    if (status < 200 || status >= 300) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t fw_ota_check(void)
{
#if !CONFIG_FW_OTA_ENABLED
    lock();
    s_ctx.state = FW_OTA_FAILED;
    snprintf(s_ctx.message, sizeof(s_ctx.message), "OTA disabled in config");
    unlock();
    return ESP_ERR_NOT_SUPPORTED;
#else
    lock();
    if (s_ctx.state == FW_OTA_UPDATING || s_ctx.state == FW_OTA_REBOOTING) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx.state = FW_OTA_CHECKING;
    s_ctx.update_available = false;
    s_ctx.available_version[0] = '\0';
    s_ctx.firmware_url[0] = '\0';
    s_ctx.progress_pct = -1;
    set_running_version();
    snprintf(s_ctx.message, sizeof(s_ctx.message), "Fetching manifest…");
    unlock();

    char *body = calloc(1, 1536);
    if (!body) {
        lock();
        s_ctx.state = FW_OTA_FAILED;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "Out of memory");
        unlock();
        return ESP_ERR_NO_MEM;
    }

    int status = 0;
    esp_err_t err = http_get_text(CONFIG_FW_OTA_MANIFEST_URL, body, 1536, &status);
    if (err != ESP_OK) {
        lock();
        s_ctx.state = FW_OTA_FAILED;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "Manifest fetch failed (%s, HTTP %d)",
                 esp_err_to_name(err), status);
        unlock();
        free(body);
        return err;
    }

    char version[32] = {0};
    char url[256] = {0};
    if (!json_get_string(body, "version", version, sizeof(version)) ||
        !json_get_string(body, "url", url, sizeof(url))) {
        lock();
        s_ctx.state = FW_OTA_FAILED;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "Invalid manifest JSON");
        unlock();
        free(body);
        return ESP_ERR_INVALID_RESPONSE;
    }
    free(body);

    lock();
    snprintf(s_ctx.available_version, sizeof(s_ctx.available_version), "%s", version);
    snprintf(s_ctx.firmware_url, sizeof(s_ctx.firmware_url), "%s", url);
    if (strcmp(s_ctx.running_version, version) == 0) {
        s_ctx.state = FW_OTA_UP_TO_DATE;
        s_ctx.update_available = false;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "Already on %s", version);
    } else {
        s_ctx.state = FW_OTA_AVAILABLE;
        s_ctx.update_available = true;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "Update available: %s", version);
    }
    unlock();
    ESP_LOGI(TAG, "check: running=%s available=%s", s_ctx.running_version, version);
    return ESP_OK;
#endif
}

static void upgrade_task(void *arg)
{
    (void)arg;
    char url[256];
    lock();
    snprintf(url, sizeof(url), "%s", s_ctx.firmware_url);
    s_ctx.state = FW_OTA_UPDATING;
    s_ctx.progress_pct = 0;
    snprintf(s_ctx.message, sizeof(s_ctx.message), "Downloading firmware…");
    unlock();

    if (!url[0]) {
        lock();
        s_ctx.state = FW_OTA_FAILED;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "No firmware URL — check first");
        unlock();
        s_upg_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Starting OTA from %s", url);
    esp_http_client_config_t http = {
        .url = url,
        .timeout_ms = 60000,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota = {
        .http_config = &http,
    };

    esp_err_t err = esp_https_ota(&ota);
    lock();
    if (err == ESP_OK) {
        s_ctx.state = FW_OTA_REBOOTING;
        s_ctx.progress_pct = 100;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "Update OK — rebooting…");
        unlock();
        ESP_LOGI(TAG, "OTA success — restarting");
        vTaskDelay(pdMS_TO_TICKS(800));
        esp_restart();
    } else {
        s_ctx.state = FW_OTA_FAILED;
        s_ctx.progress_pct = -1;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "OTA failed: %s", esp_err_to_name(err));
        unlock();
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
    }
    s_upg_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t fw_ota_start_upgrade(void)
{
#if !CONFIG_FW_OTA_ENABLED
    return ESP_ERR_NOT_SUPPORTED;
#else
    lock();
    if (s_ctx.state == FW_OTA_UPDATING || s_ctx.state == FW_OTA_REBOOTING) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_ctx.update_available || !s_ctx.firmware_url[0]) {
        snprintf(s_ctx.message, sizeof(s_ctx.message), "No update queued — check first");
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    if (s_upg_task) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    unlock();

    if (xTaskCreate(upgrade_task, "fw_ota", 8192, NULL, 5, &s_upg_task) != pdPASS) {
        lock();
        s_ctx.state = FW_OTA_FAILED;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "Failed to start OTA task");
        unlock();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
#endif
}
