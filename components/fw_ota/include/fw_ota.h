#pragma once

#include <stdbool.h>
#include <stddef.h>

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

typedef struct {
    fw_ota_state_t state;
    char running_version[32];
    char available_version[32];
    char message[96];
    bool update_available;
    int progress_pct; /**< 0–100 while updating; -1 if unknown */
} fw_ota_status_t;

/** Read running app version into status; safe to call before any check. */
void fw_ota_get_status(fw_ota_status_t *out);

/** Fetch OTA manifest from the configured GitHub release URL (blocking ~few s). */
esp_err_t fw_ota_check(void);

/** Start HTTPS OTA in a background task (call after fw_ota_check finds an update). */
esp_err_t fw_ota_start_upgrade(void);

const char *fw_ota_state_str(fw_ota_state_t st);

#ifdef __cplusplus
}
#endif
