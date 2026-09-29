/**
 * @file fw_ota.c
 * @brief OTA without on-device HTTPS (ESP32-C3 lacks DRAM for mbedTLS + HomeKit).
 *
 * The portal browser fetches the GitHub manifest/binary (CORS-friendly raw URL),
 * then streams the image to the gateway over local HTTP.
 */

#include "fw_ota.h"

#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "fw_ota";

typedef struct {
    fw_ota_state_t state;
    char running_version[48];
    char available_version[48];
    char firmware_url[256];
    char message[96];
    bool update_available;
    int progress_pct;
    const esp_partition_t *part;
    esp_ota_handle_t handle;
    size_t image_len;
    size_t written;
    bool uploading;
} fw_ota_ctx_t;

static fw_ota_ctx_t s_ctx;
static SemaphoreHandle_t s_mu;

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
    snprintf(out->firmware_url, sizeof(out->firmware_url), "%s", s_ctx.firmware_url);
    snprintf(out->message, sizeof(out->message), "%s", s_ctx.message);
    out->update_available = s_ctx.update_available;
    out->progress_pct = s_ctx.progress_pct;
    unlock();
}

esp_err_t fw_ota_offer(const char *version, const char *url)
{
    if (!version || !version[0] || !url || !url[0]) {
        return ESP_ERR_INVALID_ARG;
    }
    lock();
    set_running_version();
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
    s_ctx.progress_pct = -1;
    unlock();
    ESP_LOGI(TAG, "offer running=%s available=%s", s_ctx.running_version, version);
    return ESP_OK;
}

esp_err_t fw_ota_upload_begin(size_t image_len)
{
    lock();
    if (s_ctx.uploading || s_ctx.state == FW_OTA_REBOOTING) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_ctx.update_available) {
        snprintf(s_ctx.message, sizeof(s_ctx.message), "No update offered — check first");
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) {
        s_ctx.state = FW_OTA_FAILED;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "No OTA partition");
        unlock();
        return ESP_ERR_NOT_FOUND;
    }
    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(part, image_len ? image_len : OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) {
        s_ctx.state = FW_OTA_FAILED;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "ota_begin: %s", esp_err_to_name(err));
        unlock();
        return err;
    }
    s_ctx.part = part;
    s_ctx.handle = handle;
    s_ctx.image_len = image_len;
    s_ctx.written = 0;
    s_ctx.uploading = true;
    s_ctx.state = FW_OTA_UPDATING;
    s_ctx.progress_pct = 0;
    snprintf(s_ctx.message, sizeof(s_ctx.message), "Uploading firmware…");
    unlock();
    ESP_LOGI(TAG, "upload begin part=%s size=%u", part->label, (unsigned)image_len);
    return ESP_OK;
}

esp_err_t fw_ota_upload_write(const void *data, size_t len)
{
    if (!data || !len) {
        return ESP_OK;
    }
    lock();
    if (!s_ctx.uploading) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    esp_ota_handle_t handle = s_ctx.handle;
    unlock();

    esp_err_t err = esp_ota_write(handle, data, len);
    if (err != ESP_OK) {
        lock();
        s_ctx.state = FW_OTA_FAILED;
        s_ctx.uploading = false;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "ota_write: %s", esp_err_to_name(err));
        unlock();
        esp_ota_abort(handle);
        return err;
    }

    lock();
    s_ctx.written += len;
    if (s_ctx.image_len > 0) {
        s_ctx.progress_pct = (int)((s_ctx.written * 100) / s_ctx.image_len);
    }
    unlock();
    return ESP_OK;
}

esp_err_t fw_ota_upload_finish(void)
{
    lock();
    if (!s_ctx.uploading) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    esp_ota_handle_t handle = s_ctx.handle;
    const esp_partition_t *part = s_ctx.part;
    s_ctx.uploading = false;
    unlock();

    esp_err_t err = esp_ota_end(handle);
    if (err != ESP_OK) {
        lock();
        s_ctx.state = FW_OTA_FAILED;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "ota_end: %s", esp_err_to_name(err));
        unlock();
        return err;
    }
    err = esp_ota_set_boot_partition(part);
    if (err != ESP_OK) {
        lock();
        s_ctx.state = FW_OTA_FAILED;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "set_boot: %s", esp_err_to_name(err));
        unlock();
        return err;
    }

    lock();
    s_ctx.state = FW_OTA_REBOOTING;
    s_ctx.progress_pct = 100;
    snprintf(s_ctx.message, sizeof(s_ctx.message), "Update OK — rebooting…");
    unlock();
    ESP_LOGI(TAG, "upload complete (%u bytes) — reboot after HTTP response", (unsigned)s_ctx.written);
    return ESP_OK;
}

esp_err_t fw_ota_upload_abort(void)
{
    lock();
    if (s_ctx.uploading) {
        esp_ota_handle_t handle = s_ctx.handle;
        s_ctx.uploading = false;
        unlock();
        esp_ota_abort(handle);
        lock();
        s_ctx.state = FW_OTA_FAILED;
        snprintf(s_ctx.message, sizeof(s_ctx.message), "Upload aborted");
        unlock();
        return ESP_OK;
    }
    unlock();
    return ESP_OK;
}
