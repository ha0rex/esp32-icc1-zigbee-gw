#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FW_OTA_IDLE = 0,
    FW_OTA_CHECKING,
    FW_OTA_AVAILABLE,
    FW_OTA_UP_TO_DATE,
    FW_OTA_UPDATING,
    FW_OTA_FAILED,
    FW_OTA_REBOOTING,
} fw_ota_state_t;

/** Which published firmware channel the portal should check. */
typedef enum {
    FW_OTA_CHANNEL_STABLE = 0,  /**< Builds from main → ota branch */
    FW_OTA_CHANNEL_NIGHTLY = 1, /**< Builds from dev → ota-nightly branch */
} fw_ota_channel_t;

typedef struct {
    fw_ota_state_t state;
    fw_ota_channel_t channel;
    char running_version[48];
    char available_version[48];
    char firmware_url[256];
    char manifest_url[256];
    char message[96];
    bool update_available;
    int progress_pct;
} fw_ota_status_t;

void fw_ota_get_status(fw_ota_status_t *out);

fw_ota_channel_t fw_ota_get_channel(void);
esp_err_t fw_ota_set_channel(fw_ota_channel_t channel);
const char *fw_ota_channel_str(fw_ota_channel_t channel);
const char *fw_ota_channel_label(fw_ota_channel_t channel);
const char *fw_ota_manifest_url_for(fw_ota_channel_t channel);

/** Browser-fed offer after the portal fetched the GitHub manifest (no device TLS). */
esp_err_t fw_ota_offer(const char *version, const char *url);

/** Stream firmware image written by the portal (HTTP upload, no device TLS). */
esp_err_t fw_ota_upload_begin(size_t image_len);
esp_err_t fw_ota_upload_write(const void *data, size_t len);
esp_err_t fw_ota_upload_finish(void);
esp_err_t fw_ota_upload_abort(void);

const char *fw_ota_state_str(fw_ota_state_t st);

#ifdef __cplusplus
}
#endif
