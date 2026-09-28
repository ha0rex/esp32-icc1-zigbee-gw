#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool started;
    bool paired;
    uint16_t accessory_count;
    char setup_code[16];
    char setup_id[8];
    char status[48];
} homekit_bridge_status_t;

esp_err_t homekit_bridge_start(void);
void homekit_bridge_get_status(homekit_bridge_status_t *out);
/** Rebuild bridged accessories from Zigbee device table (call after edits). */
esp_err_t homekit_bridge_sync_devices(void);

#ifdef __cplusplus
}
#endif
