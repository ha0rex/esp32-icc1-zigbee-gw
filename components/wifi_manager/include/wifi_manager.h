#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WIFI_MGR_DISABLED = 0,
    WIFI_MGR_AP_MODE,       /**< SoftAP provisioning portal */
    WIFI_MGR_CONNECTING,    /**< STA associating */
    WIFI_MGR_CONNECTED,     /**< STA has IP */
    WIFI_MGR_FAILED,        /**< STA failed; SoftAP still available for setup */
} wifi_mgr_state_t;

typedef struct {
    wifi_mgr_state_t state;
    char ssid[33];          /**< Home STA SSID (if configured) */
    char ap_ssid[33];       /**< SoftAP SSID */
    char ip[16];            /**< STA IP when connected, else SoftAP IP */
    char ap_ip[16];         /**< SoftAP gateway IP (usually 192.168.4.1) */
    int rssi;
    uint32_t reconnects;
    bool has_sta_credentials;
    bool ap_active;
} wifi_manager_status_t;

/**
 * Start Wi-Fi: STA-only when home credentials exist (no SoftAP beacon).
 * SoftAP setup portal is enabled only with no credentials, or after STA loss.
 * SoftAP used during portal provisioning is disabled ~60s after home IP.
 * Failures never block ICC probing.
 */
esp_err_t wifi_manager_start(void);

void wifi_manager_get_status(wifi_manager_status_t *out);
bool wifi_manager_is_connected(void);
bool wifi_manager_is_started(void);
bool wifi_manager_is_ap_active(void);

/**
 * Save home Wi-Fi credentials to NVS and begin STA connection.
 * SoftAP remains up so the client can see the result.
 */
esp_err_t wifi_manager_apply_sta(const char *ssid, const char *password);

/** Erase saved STA credentials and stay in / return to SoftAP setup mode. */
esp_err_t wifi_manager_clear_sta(void);

/**
 * Trigger an AP scan (blocking briefly). Fills ssids[] with unique SSIDs.
 * @return number of SSIDs written (0..max_ssids)
 */
int wifi_manager_scan(char ssids[][33], int max_ssids);

const char *wifi_manager_state_str(wifi_mgr_state_t s);

#ifdef __cplusplus
}
#endif
