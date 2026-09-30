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

/** One scanned access point (mesh APs share SSID — use bssid to pick a node). */
typedef struct {
    char ssid[33];
    char bssid[18]; /**< "aa:bb:cc:dd:ee:ff" */
    int8_t rssi;
    uint8_t channel;
} wifi_scan_ap_t;

typedef struct {
    wifi_mgr_state_t state;
    char ssid[33];          /**< Currently associated / target SSID */
    char bssid[18];         /**< Currently associated BSSID (or empty) */
    char ap_ssid[33];       /**< SoftAP SSID */
    char ip[16];            /**< STA IP when connected, else SoftAP IP */
    char ap_ip[16];         /**< SoftAP gateway IP (usually 192.168.4.1) */
    int rssi;
    uint32_t reconnects;
    bool has_sta_credentials;
    bool ap_active;
    /* Saved slots (for portal). */
    char primary_ssid[33];
    char primary_bssid[18];
    bool primary_bssid_set;
    char secondary_ssid[33];
    char secondary_bssid[18];
    bool secondary_bssid_set;
    bool has_secondary;
    uint8_t active_slot; /**< 0 = primary, 1 = secondary */
} wifi_manager_status_t;

/**
 * Start Wi-Fi: STA-only when home credentials exist (no SoftAP beacon).
 * SoftAP setup portal is enabled only with no credentials, or after STA loss.
 * SoftAP used during portal provisioning is disabled a few seconds after home IP.
 * Failures never block ICC probing.
 */
esp_err_t wifi_manager_start(void);

void wifi_manager_get_status(wifi_manager_status_t *out);
bool wifi_manager_is_connected(void);
bool wifi_manager_is_started(void);
bool wifi_manager_is_ap_active(void);

/** True when SoftAP is up and STA is not serving the LAN (setup / SoftAP-only lifeline). */
bool wifi_manager_is_setup_portal(void);

/**
 * Mark that application traffic succeeded on STA (portal / HomeKit).
 * Used by the Wi‑Fi health watchdog to detect “associated but silent” C3 stalls.
 */
void wifi_manager_note_traffic(void);

/**
 * Pause STA health recovery (soft-reconnect / radio cycle) while busy is true.
 * Use around large portal HTML sends and Wi‑Fi scans — those briefly starve TX
 * and otherwise look like silent STA.
 */
void wifi_manager_set_busy(bool busy);

/**
 * Mark that an app-level STA TX failed (e.g. HomeKit EVENT send).
 * Does not force reconnect by itself.
 */
void wifi_manager_note_tx_fail(void);

/** Soft STA reconnect (less disruptive than wifi stop/start). */
void wifi_manager_soft_reconnect(void);

/** Full wifi stop/start — clears C3 associated-but-silent TX stalls. */
void wifi_manager_force_radio_cycle(void);

/**
 * Save STA credentials and connect.
 * @param ssid  required
 * @param password  may be empty for open networks
 * @param bssid_colon  optional "aa:bb:cc:dd:ee:ff" to pin a mesh AP; NULL/empty = any BSSID
 * @param secondary  false = primary slot, true = secondary (fallback only)
 */
esp_err_t wifi_manager_apply_sta(const char *ssid, const char *password, const char *bssid_colon,
                                 bool secondary);

/** Erase primary + secondary and return to SoftAP setup mode. */
esp_err_t wifi_manager_clear_sta(void);

/** Erase secondary only; keep primary and reconnect to it. */
esp_err_t wifi_manager_clear_secondary(void);

/**
 * Scan APs (passive while associated — active scans wedge C3 STA TX).
 * Returns number written (sorted by RSSI, strongest first).
 * Duplicate SSIDs are kept when BSSIDs differ (mesh).
 */
int wifi_manager_scan_aps(wifi_scan_ap_t *out, int max_aps);

const char *wifi_manager_state_str(wifi_mgr_state_t s);

#ifdef __cplusplus
}
#endif
