/**
 * @file wifi_manager.c
 * @brief SoftAP provisioning portal + STA with credentials in NVS.
 *
 * Boot behaviour:
 *   1. If home Wi-Fi is in NVS → STA-only (no SoftAP beacon)
 *   2. If STA fails / disconnects → SoftAP comes back for setup
 *   3. SoftAP-only when no credentials are saved
 *   4. Portal save of home Wi-Fi keeps SoftAP briefly, then STA-only after IP
 */

#include "wifi_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "lwip/tcpip.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include <stdlib.h>
#include <unistd.h>

static const char *TAG = "wifi_mgr";

#define NVS_NS              "wifi_cfg"
#define NVS_KEY_SSID        "ssid"
#define NVS_KEY_PASS        "pass"
#define NVS_KEY_BSSID       "bssid"   /* 6-byte blob; absent = any AP */
#define NVS_KEY_SSID2       "ssid2"
#define NVS_KEY_PASS2       "pass2"
#define NVS_KEY_BSSID2      "bssid2"
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
/** Tear SoftAP down soon after STA IP — APSTA on C3 starves portal/HAP TX. */
#define SOFTAP_STOP_DELAY_MS 8000
/** SoftAP only after this many failed STA retries (not on the first blip). */
#define SOFTAP_AFTER_RETRY 5
/** After this many fails with a BSSID pin, drop the pin so any mesh node works. */
#define BSSID_PIN_CLEAR_RETRY 3
/** Health loop period. */
#define WIFI_HEALTH_MS 15000
/** Ignore health shortly after join. */
#define GOT_IP_GRACE_US 15000000LL
/** Recent portal/HAP hit ⇒ radio healthy (skip probes). */
#define INBOUND_OK_US 45000000LL
/** Cap how long soft-reconnect / radio-cycle owns the STA stack. */
#define RECOVERY_TIMEOUT_US 20000000LL
/** Associated but DHCP never completes (common on a distant mesh AP). */
#define ASSOC_NO_IP_US 15000000LL
/** Roam / failover threshold (mesh node too weak). */
#define WEAK_RSSI_ROAM_DBM (-80)
/** Above this: skip idle reconnect — only gateway-ARP silence can escalate. */
#define HEALTHY_RSSI_DBM (-78)
#define WEAK_RSSI_STRIKES 4
/** Consecutive gateway ARP misses (while quiet) before soft-reconnect. */
#define GW_ARP_SILENT_STRIKES 4
/** Minimum gap between weak-RSSI reconnects. */
#define ROAM_COOLDOWN_US 120000000LL
/** Refresh ARP for wired clients while STA is healthy. */
#define GARP_PERIOD_US 45000000LL

typedef struct {
    char ssid[33];
    char pass[65];
    uint8_t bssid[6];
    bool bssid_set;
} wifi_slot_t;

static EventGroupHandle_t s_wifi_events;
static wifi_manager_status_t s_status;
static int s_retry;
static bool s_started;
static esp_netif_t *s_netif_ap;
static esp_netif_t *s_netif_sta;
static TimerHandle_t s_softap_stop_timer;
static TaskHandle_t s_softap_work_task;
static TaskHandle_t s_wifi_health_task;
static int64_t s_last_traffic_us;
static int64_t s_got_ip_us;
static int64_t s_assoc_no_ip_us;
static int64_t s_last_roam_us;
static int64_t s_last_garp_us;
static uint8_t s_silent_strikes;
static uint8_t s_weak_rssi_strikes;
static uint8_t s_gw_arp_miss;
static uint8_t s_recovery_depth; /* 0=none, 1=reconnect done, 2=stop/start done */
static volatile bool s_in_recovery;
/** Portal HTML / scan in progress — health must not soft-reconnect. */
static volatile bool s_app_busy;
static int64_t s_recovery_started_us;
/** SoftAP raised because STA inbound is dead — unused (SoftAP lifeline regresses C3). */
static bool s_softap_lifeline;
/** Consecutive successful portal/HAP hits while SoftAP lifeline is up. */
static uint8_t s_traffic_good_streak;
static wifi_slot_t s_primary;
static wifi_slot_t s_secondary;
static bool s_has_secondary;
static uint8_t s_active_slot; /* 0 primary, 1 secondary */
/** Hold STA connect while SoftAP briefly goes APSTA for a Wi‑Fi scan. */
static volatile bool s_scan_hold_sta;

static esp_err_t softap_start(void);
static esp_err_t softap_stop(void);
static void schedule_softap_stop(void);
static void cancel_softap_stop(void);
static esp_err_t configure_sta_slot(const wifi_slot_t *slot);
static void status_sync_slots(void);
static void format_bssid(const uint8_t bssid[6], char *out, size_t out_len);
static bool parse_bssid(const char *colon, uint8_t out[6]);
static wifi_slot_t *active_slot_ptr(void);
static void wifi_soft_reconnect(const char *why);
static void wifi_force_radio_cycle(const char *why);

static void format_bssid(const uint8_t bssid[6], char *out, size_t out_len)
{
    if (!out || out_len < 18 || !bssid) {
        return;
    }
    snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x", bssid[0], bssid[1], bssid[2], bssid[3],
             bssid[4], bssid[5]);
}

static bool parse_bssid(const char *colon, uint8_t out[6])
{
    if (!colon || !colon[0] || !out) {
        return false;
    }
    unsigned v[6];
    if (sscanf(colon, "%02x:%02x:%02x:%02x:%02x:%02x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) !=
        6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        out[i] = (uint8_t)v[i];
    }
    return true;
}

static wifi_slot_t *active_slot_ptr(void)
{
    return (s_active_slot == 1 && s_has_secondary) ? &s_secondary : &s_primary;
}

static void status_sync_slots(void)
{
    strncpy(s_status.primary_ssid, s_primary.ssid, sizeof(s_status.primary_ssid) - 1);
    s_status.primary_bssid_set = s_primary.bssid_set;
    if (s_primary.bssid_set) {
        format_bssid(s_primary.bssid, s_status.primary_bssid, sizeof(s_status.primary_bssid));
    } else {
        s_status.primary_bssid[0] = '\0';
    }
    s_status.has_secondary = s_has_secondary && s_secondary.ssid[0] != '\0';
    if (s_status.has_secondary) {
        strncpy(s_status.secondary_ssid, s_secondary.ssid, sizeof(s_status.secondary_ssid) - 1);
        s_status.secondary_bssid_set = s_secondary.bssid_set;
        if (s_secondary.bssid_set) {
            format_bssid(s_secondary.bssid, s_status.secondary_bssid,
                         sizeof(s_status.secondary_bssid));
        } else {
            s_status.secondary_bssid[0] = '\0';
        }
    } else {
        s_status.secondary_ssid[0] = '\0';
        s_status.secondary_bssid[0] = '\0';
        s_status.secondary_bssid_set = false;
    }
    s_status.active_slot = s_active_slot;
    s_status.has_sta_credentials = (s_primary.ssid[0] != '\0');
}

/** Refresh associated BSSID string in status (no auto pin/unpin). */
static void refresh_assoc_bssid(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        return;
    }
    format_bssid(ap.bssid, s_status.bssid, sizeof(s_status.bssid));
    s_status.rssi = ap.rssi;
}

void wifi_manager_set_busy(bool busy)
{
    s_app_busy = busy;
    if (busy) {
        s_last_traffic_us = esp_timer_get_time();
        s_silent_strikes = 0;
    }
}

void wifi_manager_note_traffic(void)
{
    s_last_traffic_us = esp_timer_get_time();
    s_silent_strikes = 0;
    s_gw_arp_miss = 0;
    if (s_traffic_good_streak < 255) {
        s_traffic_good_streak++;
    }
    /* Do not clear s_recovery_depth here — HAP can note "traffic" without the
     * LAN being reachable, which blocked SoftAP lifeline escalation. Depth is
     * cleared in the health task once quiet_us shows real inbound. */
    refresh_assoc_bssid();
    if (s_softap_lifeline && s_status.ap_active && s_traffic_good_streak >= 10) {
        ESP_LOGI(TAG, "STA inbound stable (%u hits) — releasing SoftAP lifeline",
                 (unsigned)s_traffic_good_streak);
        s_softap_lifeline = false;
        schedule_softap_stop();
    }
}

void wifi_manager_note_tx_fail(void)
{
    /* HAP EVENT send fails are common (stale sessions). Do NOT clear inbound
     * traffic timestamps — that made quiet_us look huge and forced reconnects
     * while the portal was mid-transfer. */
    ESP_LOGW(TAG, "STA TX fail noted (no reconnect strike)");
}

static void wifi_ensure_public_dns(void)
{
    if (!s_netif_sta) {
        return;
    }
    esp_netif_dns_info_t dns = {0};
    bool have_main = false;
    if (esp_netif_get_dns_info(s_netif_sta, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK &&
        dns.ip.type == ESP_IPADDR_TYPE_V4 && dns.ip.u_addr.ip4.addr != 0) {
        have_main = true;
    }
    if (!have_main) {
        dns.ip.type = ESP_IPADDR_TYPE_V4;
        dns.ip.u_addr.ip4.addr = ESP_IP4TOADDR(8, 8, 8, 8);
        if (esp_netif_set_dns_info(s_netif_sta, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK) {
            ESP_LOGI(TAG, "DNS main → 8.8.8.8");
        }
    }
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = ESP_IP4TOADDR(1, 1, 1, 1);
    esp_netif_set_dns_info(s_netif_sta, ESP_NETIF_DNS_BACKUP, &dns);
}

static void wifi_apply_sta_radio_quirks(void)
{
    esp_err_t ps = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ps != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_ps(NONE) failed: %s", esp_err_to_name(ps));
    }
    /* 11n/HT AMPDU BA with this AP → associated-but-silent / TX death. */
    uint8_t proto = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;
    esp_err_t pr = esp_wifi_set_protocol(WIFI_IF_STA, proto);
    if (pr != ESP_OK) {
        ESP_LOGW(TAG, "set_protocol(b/g) failed: %s", esp_err_to_name(pr));
    }
}

/** Refresh L2 mappings after DHCP — helps wired clients that never see our ARP. */
static void wifi_announce_garp(void)
{
    struct netif *nf = netif_default;
    if (!nf) {
        return;
    }
    LOCK_TCPIP_CORE();
    etharp_gratuitous(nf);
    UNLOCK_TCPIP_CORE();
    ESP_LOGD(TAG, "Gratuitous ARP announced");
}

static bool wifi_gateway_arp_cached(void)
{
    struct netif *nf = netif_default;
    if (!nf || !s_netif_sta) {
        return false;
    }
    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(s_netif_sta, &info) != ESP_OK || info.gw.addr == 0) {
        return false;
    }
    ip4_addr_t gw;
    gw.addr = info.gw.addr;
    struct eth_addr *eth_ret = NULL;
    const ip4_addr_t *ip_ret = NULL;
    bool ok;
    LOCK_TCPIP_CORE();
    ok = etharp_find_addr(nf, &gw, &eth_ret, &ip_ret) >= 0;
    UNLOCK_TCPIP_CORE();
    return ok;
}

static void wifi_gateway_arp_probe(void)
{
    struct netif *nf = netif_default;
    if (!nf || !s_netif_sta) {
        return;
    }
    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(s_netif_sta, &info) != ESP_OK || info.gw.addr == 0) {
        return;
    }
    ip4_addr_t gw;
    gw.addr = info.gw.addr;
    LOCK_TCPIP_CORE();
    (void)etharp_request(nf, &gw);
    UNLOCK_TCPIP_CORE();
}

/**
 * Recover associated-but-silent STA without TCP probes (mesh APs often ignore those).
 * Only escalates when portal/HAP have been quiet AND the default gateway never answers ARP.
 */
static void wifi_check_gateway_arp_liveness(int64_t now)
{
    const int64_t quiet_us =
        s_last_traffic_us ? (now - s_last_traffic_us)
                          : (s_got_ip_us ? (now - s_got_ip_us) : 0);
    if (quiet_us < INBOUND_OK_US) {
        s_gw_arp_miss = 0;
        return;
    }
    if (wifi_gateway_arp_cached()) {
        s_gw_arp_miss = 0;
        return;
    }
    wifi_gateway_arp_probe();
    if (s_gw_arp_miss < 255) {
        s_gw_arp_miss++;
    }
    ESP_LOGW(TAG, "STA health: gateway ARP miss %u (quiet %lld s)", (unsigned)s_gw_arp_miss,
             (long long)(quiet_us / 1000000LL));
    if (s_gw_arp_miss < GW_ARP_SILENT_STRIKES) {
        return;
    }
    s_gw_arp_miss = 0;
    if (s_recovery_depth == 0) {
        s_recovery_depth = 1;
        wifi_soft_reconnect("gateway ARP silent");
    } else {
        wifi_force_radio_cycle("gateway ARP silent");
    }
}

static void wifi_soft_reconnect(const char *why)
{
    ESP_LOGW(TAG, "STA recovery (%s) — disconnect/reconnect", why ? why : "unknown");
    s_status.state = WIFI_MGR_CONNECTING;
    snprintf(s_status.ip, sizeof(s_status.ip), "-");
    s_silent_strikes = 0;
    s_gw_arp_miss = 0;
    s_got_ip_us = 0;
    s_last_traffic_us = 0; /* quiet must restart after recovery */
    s_traffic_good_streak = 0;
    s_last_roam_us = esp_timer_get_time();
    s_in_recovery = true;
    s_recovery_started_us = esp_timer_get_time();
    /* Disconnect first; STA_DISCONNECTED must NOT auto-connect while s_in_recovery —
     * otherwise it races and rejoins the old BSSID before we apply the new slot. */
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(400));
    (void)configure_sta_slot(active_slot_ptr());
    esp_wifi_connect();
    /* Keep s_in_recovery until GOT_IP or RECOVERY_TIMEOUT — do not clear here. */
}

static void wifi_force_radio_cycle(const char *why)
{
    ESP_LOGW(TAG, "STA recovery (%s) — wifi stop/start", why ? why : "unknown");
    s_status.state = WIFI_MGR_CONNECTING;
    snprintf(s_status.ip, sizeof(s_status.ip), "-");
    s_retry = 0;
    s_silent_strikes = 0;
    s_gw_arp_miss = 0;
    s_got_ip_us = 0;
    s_last_traffic_us = 0;
    s_traffic_good_streak = 0;
    s_last_roam_us = esp_timer_get_time();
    s_in_recovery = true;
    s_recovery_started_us = esp_timer_get_time();
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(300));
    (void)configure_sta_slot(active_slot_ptr());
    esp_wifi_start(); /* STA_START handler calls esp_wifi_connect() */
    /* Keep s_in_recovery until GOT_IP or timeout. */
}

void wifi_manager_soft_reconnect(void)
{
    if (!s_started) {
        return;
    }
    wifi_soft_reconnect("app request");
}

void wifi_manager_force_radio_cycle(void)
{
    if (!s_started) {
        return;
    }
    s_recovery_depth = 2;
    wifi_force_radio_cycle("app request");
}

/**
 * Detect associated-but-silent C3 STA carefully.
 *
 * TCP probes to the AP are unreliable on mesh (ignored → false “TX dead”). Soft-
 * reconnect on a busy healthy link also caused ping death. Escalation paths:
 *   - no AP info / DHCP timeout
 *   - weak RSSI roam / secondary failover
 *   - gateway ARP silence after portal/HAP have been quiet (true TX death)
 */
static void wifi_health_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(WIFI_HEALTH_MS));
        if (!s_started) {
            continue;
        }

        int64_t now = esp_timer_get_time();
        if (s_softap_lifeline) {
            continue;
        }
        if (s_app_busy || s_scan_hold_sta) {
            s_last_traffic_us = now;
            s_silent_strikes = 0;
            s_gw_arp_miss = 0;
            continue;
        }
        if (s_in_recovery) {
            if (s_recovery_started_us && (now - s_recovery_started_us) > RECOVERY_TIMEOUT_US) {
                ESP_LOGW(TAG, "STA recovery timeout — releasing ownership");
                s_in_recovery = false;
                s_recovery_started_us = 0;
            } else {
                continue;
            }
        }

        /* Associated (or stuck CONNECTING) without DHCP — often a distant mesh AP. */
        if (s_status.has_sta_credentials && s_status.state != WIFI_MGR_CONNECTED) {
            wifi_ap_record_t ap_wait;
            if (esp_wifi_sta_get_ap_info(&ap_wait) == ESP_OK) {
                if (!s_assoc_no_ip_us) {
                    s_assoc_no_ip_us = now;
                    ESP_LOGW(TAG, "STA associated rssi=%d — waiting for DHCP", (int)ap_wait.rssi);
                } else if ((now - s_assoc_no_ip_us) > ASSOC_NO_IP_US) {
                    ESP_LOGW(TAG, "STA DHCP timeout (rssi=%d) — reconnect (no scan)",
                             (int)ap_wait.rssi);
                    s_assoc_no_ip_us = 0;
                    s_recovery_depth = 1;
                    /* Avoid scan-based rebind here — scans wedge C3 STA TX. */
                    if (s_active_slot == 0 && s_has_secondary && s_secondary.ssid[0] &&
                        ap_wait.rssi < WEAK_RSSI_ROAM_DBM) {
                        s_active_slot = 1;
                        wifi_soft_reconnect("assoc DHCP timeout → secondary");
                    } else {
                        wifi_soft_reconnect("assoc without DHCP");
                    }
                }
            } else {
                s_assoc_no_ip_us = 0;
            }
            continue;
        }

        if (s_status.state != WIFI_MGR_CONNECTED) {
            continue;
        }
        if (s_got_ip_us && (now - s_got_ip_us) < GOT_IP_GRACE_US) {
            continue;
        }

        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
            s_silent_strikes++;
            ESP_LOGW(TAG, "STA health: no AP info (strike %u)", (unsigned)s_silent_strikes);
            if (s_silent_strikes >= 3) {
                wifi_force_radio_cycle("no AP info");
                s_silent_strikes = 0;
            }
            continue;
        }
        s_status.rssi = ap.rssi;

        /* Usable RSSI: GARP + gateway ARP liveness (only when quiet). */
        if (ap.rssi >= HEALTHY_RSSI_DBM) {
            s_silent_strikes = 0;
            s_weak_rssi_strikes = 0;
            s_recovery_depth = 0;
            if (s_status.ap_active && !s_softap_lifeline) {
                schedule_softap_stop();
            }
            if (!s_last_garp_us || (now - s_last_garp_us) >= GARP_PERIOD_US) {
                wifi_announce_garp();
                s_last_garp_us = now;
            }
            wifi_check_gateway_arp_liveness(now);
            continue;
        }

        /* Multi-AP: weak link. Prefer secondary; avoid scan-rebind (wedges C3). */
        if (ap.rssi < WEAK_RSSI_ROAM_DBM) {
            s_weak_rssi_strikes++;
            ESP_LOGW(TAG, "STA health: weak rssi=%d (strike %u)", (int)ap.rssi,
                     (unsigned)s_weak_rssi_strikes);
            if (s_weak_rssi_strikes >= WEAK_RSSI_STRIKES &&
                (!s_last_roam_us || (now - s_last_roam_us) > ROAM_COOLDOWN_US)) {
                s_weak_rssi_strikes = 0;
                if (s_active_slot == 0 && s_has_secondary && s_secondary.ssid[0]) {
                    ESP_LOGW(TAG, "Weak primary — failing over to secondary '%s'",
                             s_secondary.ssid);
                    s_active_slot = 1;
                    s_recovery_depth = 1;
                    s_retry = 0;
                    wifi_soft_reconnect("primary weak → secondary");
                } else if (!active_slot_ptr()->bssid_set) {
                    s_recovery_depth = 1;
                    wifi_soft_reconnect("weak RSSI roam");
                } else {
                    ESP_LOGW(TAG, "Weak RSSI with pinned BSSID — keep link, GARP only");
                    wifi_announce_garp();
                    s_last_garp_us = now;
                    wifi_check_gateway_arp_liveness(now);
                }
            }
            continue;
        }
        s_weak_rssi_strikes = 0;

        /* Marginal RSSI (WEAK…HEALTHY): stay associated, GARP + ARP liveness. */
        if (s_status.ap_active && !s_softap_lifeline) {
            const int64_t quiet_us =
                s_last_traffic_us ? (now - s_last_traffic_us)
                                  : (s_got_ip_us ? (now - s_got_ip_us) : INBOUND_OK_US);
            if (quiet_us < INBOUND_OK_US) {
                schedule_softap_stop();
            }
        }
        if (!s_last_garp_us || (now - s_last_garp_us) >= GARP_PERIOD_US) {
            wifi_announce_garp();
            s_last_garp_us = now;
        }
        wifi_check_gateway_arp_liveness(now);
    }
}

/** Dedicated task — never call esp_wifi_* from the timer service or Wi-Fi event loop. */
static void softap_work_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_status.state != WIFI_MGR_CONNECTED || !s_status.ap_active || s_softap_lifeline) {
            continue;
        }
        ESP_LOGI(TAG, "Home Wi-Fi stable — disabling SoftAP setup portal");
        softap_stop();
    }
}

static void softap_stop_timer_cb(TimerHandle_t t)
{
    (void)t;
    if (s_softap_work_task) {
        xTaskNotifyGive(s_softap_work_task);
    }
}

static void schedule_softap_stop(void)
{
    if (!s_softap_stop_timer) {
        return;
    }
    xTimerStop(s_softap_stop_timer, 0);
    xTimerChangePeriod(s_softap_stop_timer, pdMS_TO_TICKS(SOFTAP_STOP_DELAY_MS), 0);
    xTimerStart(s_softap_stop_timer, 0);
}

static void cancel_softap_stop(void)
{
    if (s_softap_stop_timer) {
        xTimerStop(s_softap_stop_timer, 0);
    }
}

const char *wifi_manager_state_str(wifi_mgr_state_t s)
{
    switch (s) {
    case WIFI_MGR_DISABLED:
        return "disabled";
    case WIFI_MGR_AP_MODE:
        return "ap_setup";
    case WIFI_MGR_CONNECTING:
        return "connecting";
    case WIFI_MGR_CONNECTED:
        return "connected";
    case WIFI_MGR_FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

static void set_ip_from_netif(esp_netif_t *netif, char *buf, size_t buflen)
{
    if (!netif || !buf || buflen == 0) {
        return;
    }
    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr != 0) {
        snprintf(buf, buflen, IPSTR, IP2STR(&ip.ip));
    } else {
        snprintf(buf, buflen, "-");
    }
}

static esp_err_t nvs_load_slots(void)
{
    memset(&s_primary, 0, sizeof(s_primary));
    memset(&s_secondary, 0, sizeof(s_secondary));
    s_has_secondary = false;
    s_active_slot = 0;

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    size_t sl = sizeof(s_primary.ssid);
    size_t pl = sizeof(s_primary.pass);
    if (nvs_get_str(h, NVS_KEY_SSID, s_primary.ssid, &sl) != ESP_OK || !s_primary.ssid[0]) {
        nvs_close(h);
        return ESP_ERR_NOT_FOUND;
    }
    if (nvs_get_str(h, NVS_KEY_PASS, s_primary.pass, &pl) != ESP_OK) {
        s_primary.pass[0] = '\0';
    }
    size_t bl = 6;
    if (nvs_get_blob(h, NVS_KEY_BSSID, s_primary.bssid, &bl) == ESP_OK && bl == 6) {
        s_primary.bssid_set = true;
    }

    char ssid2[33] = {0};
    sl = sizeof(ssid2);
    if (nvs_get_str(h, NVS_KEY_SSID2, ssid2, &sl) == ESP_OK && ssid2[0]) {
        strncpy(s_secondary.ssid, ssid2, sizeof(s_secondary.ssid) - 1);
        pl = sizeof(s_secondary.pass);
        if (nvs_get_str(h, NVS_KEY_PASS2, s_secondary.pass, &pl) != ESP_OK) {
            s_secondary.pass[0] = '\0';
        }
        bl = 6;
        if (nvs_get_blob(h, NVS_KEY_BSSID2, s_secondary.bssid, &bl) == ESP_OK && bl == 6) {
            s_secondary.bssid_set = true;
        }
        s_has_secondary = true;
    }
    nvs_close(h);
    status_sync_slots();
    return ESP_OK;
}

static esp_err_t nvs_save_slots(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, NVS_KEY_SSID, s_primary.ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(h, NVS_KEY_PASS, s_primary.pass);
    }
    if (err == ESP_OK) {
        if (s_primary.bssid_set) {
            err = nvs_set_blob(h, NVS_KEY_BSSID, s_primary.bssid, 6);
        } else {
            nvs_erase_key(h, NVS_KEY_BSSID);
        }
    }
    if (err == ESP_OK) {
        if (s_has_secondary && s_secondary.ssid[0]) {
            err = nvs_set_str(h, NVS_KEY_SSID2, s_secondary.ssid);
            if (err == ESP_OK) {
                err = nvs_set_str(h, NVS_KEY_PASS2, s_secondary.pass);
            }
            if (err == ESP_OK) {
                if (s_secondary.bssid_set) {
                    err = nvs_set_blob(h, NVS_KEY_BSSID2, s_secondary.bssid, 6);
                } else {
                    nvs_erase_key(h, NVS_KEY_BSSID2);
                }
            }
        } else {
            nvs_erase_key(h, NVS_KEY_SSID2);
            nvs_erase_key(h, NVS_KEY_PASS2);
            nvs_erase_key(h, NVS_KEY_BSSID2);
        }
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

static esp_err_t nvs_erase_sta(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    nvs_erase_key(h, NVS_KEY_SSID);
    nvs_erase_key(h, NVS_KEY_PASS);
    nvs_erase_key(h, NVS_KEY_BSSID);
    nvs_erase_key(h, NVS_KEY_SSID2);
    nvs_erase_key(h, NVS_KEY_PASS2);
    nvs_erase_key(h, NVS_KEY_BSSID2);
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static void build_ap_ssid(char *out, size_t out_len)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(out, out_len, "%s-%02X%02X", CONFIG_WIFI_AP_SSID_PREFIX, mac[4], mac[5]);
}

static esp_err_t configure_softap(void)
{
    build_ap_ssid(s_status.ap_ssid, sizeof(s_status.ap_ssid));

    wifi_config_t ap = {0};
    strncpy((char *)ap.ap.ssid, s_status.ap_ssid, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = (uint8_t)strlen(s_status.ap_ssid);
    /* APSTA requires SoftAP on the same channel as STA. */
    uint8_t ch = CONFIG_WIFI_AP_CHANNEL;
    wifi_ap_record_t sta_ap;
    if (esp_wifi_sta_get_ap_info(&sta_ap) == ESP_OK && sta_ap.primary >= 1) {
        ch = sta_ap.primary;
    }
    ap.ap.channel = ch;
    ap.ap.max_connection = 4;
    ap.ap.beacon_interval = 100;

#if CONFIG_WIFI_AP_OPEN
    ap.ap.authmode = WIFI_AUTH_OPEN;
    ap.ap.password[0] = '\0';
#else
    strncpy((char *)ap.ap.password, CONFIG_WIFI_AP_PASSWORD, sizeof(ap.ap.password) - 1);
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    if (strlen((char *)ap.ap.password) < 8) {
        ESP_LOGW(TAG, "AP password < 8 chars — forcing open SoftAP");
        ap.ap.authmode = WIFI_AUTH_OPEN;
        ap.ap.password[0] = '\0';
    }
#endif

    esp_err_t err = esp_wifi_set_config(WIFI_IF_AP, &ap);
    if (err != ESP_OK) {
        return err;
    }

    set_ip_from_netif(s_netif_ap, s_status.ap_ip, sizeof(s_status.ap_ip));
    if (s_status.ap_ip[0] == '-' || s_status.ap_ip[0] == '\0') {
        snprintf(s_status.ap_ip, sizeof(s_status.ap_ip), "192.168.4.1");
    }
    s_status.ap_active = true;
    ESP_LOGI(TAG, "SoftAP SSID='%s' IP=%s ch=%u (open setup portal)", s_status.ap_ssid,
             s_status.ap_ip, (unsigned)ch);
    return ESP_OK;
}

static esp_err_t configure_sta_slot(const wifi_slot_t *slot)
{
    if (!slot || !slot->ssid[0]) {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t sta = {0};
    strncpy((char *)sta.sta.ssid, slot->ssid, sizeof(sta.sta.ssid) - 1);
    strncpy((char *)sta.sta.password, slot->pass, sizeof(sta.sta.password) - 1);
    if (slot->pass[0] != '\0') {
        sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        sta.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }
    if (slot->bssid_set) {
        memcpy(sta.sta.bssid, slot->bssid, 6);
        sta.sta.bssid_set = true;
    } else {
        sta.sta.bssid_set = false;
    }

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &sta);
    if (err != ESP_OK) {
        return err;
    }

    strncpy(s_status.ssid, slot->ssid, sizeof(s_status.ssid) - 1);
    if (slot->bssid_set) {
        format_bssid(slot->bssid, s_status.bssid, sizeof(s_status.bssid));
    } else {
        s_status.bssid[0] = '\0';
    }
    s_status.has_sta_credentials = true;
    s_status.active_slot = s_active_slot;
    s_retry = 0;
    s_status.state = WIFI_MGR_CONNECTING;
    snprintf(s_status.ip, sizeof(s_status.ip), "-");
    if (slot->bssid_set) {
        char b[18];
        format_bssid(slot->bssid, b, sizeof(b));
        ESP_LOGI(TAG, "STA config SSID='%s' BSSID=%s (slot %u)", slot->ssid, b,
                 (unsigned)s_active_slot);
    } else {
        ESP_LOGI(TAG, "STA config SSID='%s' (any BSSID, slot %u)", slot->ssid,
                 (unsigned)s_active_slot);
    }
    return ESP_OK;
}

static esp_err_t softap_start(void)
{
    esp_err_t err;
    if (s_softap_lifeline) {
        /* Stay APSTA with STA disconnected. SoftAP-only (WIFI_MODE_AP) needs a
         * temporary APSTA flip to scan, and that mode switch wedges the C3 radio
         * (ping + SoftAP die). Compact setup HTML is small enough for SoftAP TX
         * even while the STA iface exists idle. */
        ESP_LOGW(TAG, "SoftAP lifeline — pausing STA (APSTA kept for scan)");
        s_in_recovery = false;
        s_retry = CONFIG_WIFI_MAX_RETRY; /* stop STA reconnect loop */
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(200));
        err = esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (err != ESP_OK) {
            return err;
        }
        s_status.state = WIFI_MGR_AP_MODE;
        snprintf(s_status.ip, sizeof(s_status.ip), "-");
        s_status.rssi = 0;
        s_status.bssid[0] = '\0';
    } else {
        err = esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (err != ESP_OK) {
            return err;
        }
    }
    err = configure_softap();
    if (err != ESP_OK) {
        return err;
    }
    s_status.ap_active = true;
    if (s_status.state != WIFI_MGR_CONNECTED && s_status.state != WIFI_MGR_CONNECTING) {
        s_status.state = WIFI_MGR_AP_MODE;
    }
    ESP_LOGI(TAG, "SoftAP enabled for setup%s", s_softap_lifeline ? " (STA paused)" : "");
    return ESP_OK;
}

static esp_err_t softap_stop(void)
{
    if (!s_status.ap_active) {
        return ESP_OK;
    }
    if (s_softap_lifeline) {
        ESP_LOGW(TAG, "SoftAP stop ignored — lifeline active (join SoftAP to reconfigure)");
        return ESP_OK;
    }
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to switch to STA-only: %s", esp_err_to_name(err));
        return err;
    }
    s_status.ap_active = false;
    s_status.ap_ip[0] = '\0';
    ESP_LOGI(TAG, "SoftAP stopped — home Wi-Fi only (secured)");
    return ESP_OK;
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        set_ip_from_netif(s_netif_ap, s_status.ap_ip, sizeof(s_status.ap_ip));
        s_status.ap_active = true;
        if (s_status.state != WIFI_MGR_CONNECTED && s_status.state != WIFI_MGR_CONNECTING) {
            s_status.state = WIFI_MGR_AP_MODE;
        }
        ESP_LOGI(TAG, "SoftAP started");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STOP) {
        s_status.ap_active = false;
        ESP_LOGI(TAG, "SoftAP stopped");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "Client joined SoftAP, AID=%d", e->aid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_softap_lifeline || s_scan_hold_sta) {
            /* SoftAP-only recovery / temporary scan APSTA — do not auto-join. */
            return;
        }
        if (s_status.has_sta_credentials) {
            s_status.state = WIFI_MGR_CONNECTING;
            esp_wifi_connect();
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_status.reconnects++;
        snprintf(s_status.ip, sizeof(s_status.ip), "-");
        cancel_softap_stop();
        if (s_softap_lifeline || s_scan_hold_sta) {
            s_status.state = WIFI_MGR_AP_MODE;
            return;
        }
        if (s_in_recovery) {
            /* soft_reconnect / radio_cycle apply config then connect — do not race here. */
            s_status.state = WIFI_MGR_CONNECTING;
            return;
        }
        if (!s_status.has_sta_credentials) {
            softap_start();
            s_status.state = WIFI_MGR_AP_MODE;
            return;
        }
        if (s_retry < CONFIG_WIFI_MAX_RETRY) {
            s_retry++;
            s_status.state = WIFI_MGR_CONNECTING;
            /* Pinned mesh node gone → "Haven't to connect to a suitable AP" forever.
             * After a few fails, clear the pin and join any same-SSID AP (no scan). */
            if (s_retry == BSSID_PIN_CLEAR_RETRY) {
                wifi_slot_t *slot = active_slot_ptr();
                if (slot && slot->bssid_set) {
                    char was[18];
                    format_bssid(slot->bssid, was, sizeof(was));
                    ESP_LOGW(TAG, "Pinned BSSID %s unreachable — clearing pin (any '%s' node)",
                             was, slot->ssid);
                    slot->bssid_set = false;
                    memset(slot->bssid, 0, 6);
                    (void)nvs_save_slots();
                    status_sync_slots();
                    wifi_config_t sta = {0};
                    strncpy((char *)sta.sta.ssid, slot->ssid, sizeof(sta.sta.ssid) - 1);
                    strncpy((char *)sta.sta.password, slot->pass, sizeof(sta.sta.password) - 1);
                    sta.sta.bssid_set = false;
                    if (slot->pass[0] != '\0') {
                        sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
                    } else {
                        sta.sta.threshold.authmode = WIFI_AUTH_OPEN;
                    }
                    (void)esp_wifi_set_config(WIFI_IF_STA, &sta);
                    s_status.bssid[0] = '\0';
                }
            }
            /* APSTA while STA is flapping kills C3 TX (portal + HomeKit hang).
             * Only bring SoftAP back after several STA failures. */
            if (s_retry >= SOFTAP_AFTER_RETRY && !s_status.ap_active) {
                softap_start();
            }
            ESP_LOGW(TAG, "STA disconnected, retry %d/%d (slot %u)", s_retry, CONFIG_WIFI_MAX_RETRY,
                     (unsigned)s_active_slot);
            esp_wifi_connect();
        } else if (s_active_slot == 0 && s_has_secondary && s_secondary.ssid[0]) {
            ESP_LOGW(TAG, "Primary '%s' failed — trying secondary '%s'", s_primary.ssid,
                     s_secondary.ssid);
            s_active_slot = 1;
            s_retry = 0;
            s_status.state = WIFI_MGR_CONNECTING;
            (void)configure_sta_slot(&s_secondary);
            status_sync_slots();
            esp_wifi_connect();
        } else {
            s_status.state = WIFI_MGR_FAILED;
            softap_start();
            ESP_LOGW(TAG, "STA failed — SoftAP '%s' available at %s", s_status.ap_ssid,
                     s_status.ap_ip);
            if (s_wifi_events) {
                xEventGroupSetBits(s_wifi_events, WIFI_FAIL_BIT);
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        s_retry = 0;
        s_status.state = WIFI_MGR_CONNECTED;
        snprintf(s_status.ip, sizeof(s_status.ip), IPSTR, IP2STR(&event->ip_info.ip));
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            s_status.rssi = ap.rssi;
            format_bssid(ap.bssid, s_status.bssid, sizeof(s_status.bssid));
        }
        ESP_LOGI(TAG, "STA connected to '%s' BSSID=%s IP %s RSSI %d (slot %u)", s_status.ssid,
                 s_status.bssid[0] ? s_status.bssid : "-", s_status.ip, s_status.rssi,
                 (unsigned)s_active_slot);
        s_got_ip_us = esp_timer_get_time();
        s_assoc_no_ip_us = 0;
        s_weak_rssi_strikes = 0;
        s_in_recovery = false;
        s_recovery_started_us = 0;
        s_silent_strikes = 0;
        s_last_traffic_us = 0; /* measure quiet from this join, not pre-reconnect HAP noise */
        /* Re-apply after every join — stop/start and roam drop these. */
        wifi_apply_sta_radio_quirks();
        wifi_ensure_public_dns();
        wifi_announce_garp();
        /* Do not seed s_last_traffic_us here — that blocked TX-dead recovery. */
        if (s_status.ap_active && !s_softap_lifeline) {
            schedule_softap_stop();
        }
        if (s_wifi_events) {
            xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_AP_STAIPASSIGNED) {
        set_ip_from_netif(s_netif_ap, s_status.ap_ip, sizeof(s_status.ap_ip));
    }
}

esp_err_t wifi_manager_start(void)
{
#if !CONFIG_WIFI_ENABLED
    s_status.state = WIFI_MGR_DISABLED;
    ESP_LOGI(TAG, "Wi-Fi disabled in menuconfig");
    return ESP_OK;
#else
    memset(&s_status, 0, sizeof(s_status));
    snprintf(s_status.ip, sizeof(s_status.ip), "-");
    snprintf(s_status.ap_ip, sizeof(s_status.ap_ip), "192.168.4.1");
    s_status.state = WIFI_MGR_AP_MODE;

    s_wifi_events = xEventGroupCreate();
    if (!s_wifi_events) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    s_netif_ap = esp_netif_create_default_wifi_ap();
    s_netif_sta = esp_netif_create_default_wifi_sta();
    /* Stable DHCP client id / router hostname — helps leases stick across reconnect. */
    (void)esp_netif_set_hostname(s_netif_sta, "icc-gateway");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                                        &on_wifi_event, NULL, NULL));

    bool have_creds = (nvs_load_slots() == ESP_OK) && (s_primary.ssid[0] != '\0');

    /* Optional compile-time seed if NVS empty (dev convenience only). */
    if (!have_creds && strlen(CONFIG_WIFI_SSID) > 0) {
        memset(&s_primary, 0, sizeof(s_primary));
        strncpy(s_primary.ssid, CONFIG_WIFI_SSID, sizeof(s_primary.ssid) - 1);
        strncpy(s_primary.pass, CONFIG_WIFI_PASSWORD, sizeof(s_primary.pass) - 1);
        have_creds = true;
        ESP_LOGI(TAG, "Seeding STA credentials from menuconfig into NVS");
        (void)nvs_save_slots();
        status_sync_slots();
    }

    /*
     * Prefer STA-only when home Wi-Fi is known. Booting APSTA then tearing SoftAP
     * down has left the C3 associated but silent; SoftAP is only for setup / fail.
     */
    if (have_creds) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        s_active_slot = 0;
        ESP_ERROR_CHECK(configure_sta_slot(&s_primary));
        s_status.ap_active = false;
        s_status.state = WIFI_MGR_CONNECTING;
        ESP_LOGI(TAG, "Connecting to home Wi-Fi '%s' (SoftAP off until STA fails)",
                 s_primary.ssid);
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(configure_softap());
        s_status.state = WIFI_MGR_AP_MODE;
        ESP_LOGI(TAG, "No home Wi-Fi saved — SoftAP setup mode only");
    }

    ESP_ERROR_CHECK(esp_wifi_start());
    /* Modem sleep drops TCP sessions (portal / HomeKit) on this AP — keep radio awake.
     * Also force 11b/g (no HT/AMPDU). Re-applied on every GOT_IP. */
    wifi_apply_sta_radio_quirks();
    ESP_LOGI(TAG, "Wi-Fi power save disabled; STA protocol 11b/g only (no HT/AMPDU)");

    /* Do NOT scan here — active scan during STA bring-up wedges C3 TX. */

    if (!s_softap_work_task) {
        if (xTaskCreate(softap_work_task, "ap_work", 3072, NULL, 5, &s_softap_work_task) !=
            pdPASS) {
            s_softap_work_task = NULL;
            ESP_LOGW(TAG, "SoftAP work task create failed");
        }
    }
    if (!s_softap_stop_timer) {
        s_softap_stop_timer = xTimerCreate("ap_stop", pdMS_TO_TICKS(SOFTAP_STOP_DELAY_MS), pdFALSE,
                                           NULL, softap_stop_timer_cb);
        if (!s_softap_stop_timer) {
            ESP_LOGW(TAG, "SoftAP stop timer create failed");
        }
    }
    if (!s_wifi_health_task) {
        if (xTaskCreate(wifi_health_task, "wifi_health", 8192, NULL, 4, &s_wifi_health_task) !=
            pdPASS) {
            s_wifi_health_task = NULL;
            ESP_LOGW(TAG, "Wi-Fi health task create failed");
        }
    }

    s_started = true;
    return ESP_OK;
#endif
}

esp_err_t wifi_manager_apply_sta(const char *ssid, const char *password, const char *bssid_colon,
                                 bool secondary)
{
    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!ssid || ssid[0] == '\0' || strlen(ssid) > 32) {
        return ESP_ERR_INVALID_ARG;
    }
    if (password && strlen(password) > 63) {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_slot_t slot = {0};
    strncpy(slot.ssid, ssid, sizeof(slot.ssid) - 1);
    /* Empty password = keep existing secret when editing the same slot. */
    const wifi_slot_t *prev = secondary ? (s_has_secondary ? &s_secondary : NULL) : &s_primary;
    if (password && password[0]) {
        strncpy(slot.pass, password, sizeof(slot.pass) - 1);
    } else if (prev && prev->ssid[0] && strcmp(prev->ssid, ssid) == 0) {
        strncpy(slot.pass, prev->pass, sizeof(slot.pass) - 1);
    } else if (password) {
        strncpy(slot.pass, password, sizeof(slot.pass) - 1);
    }
    if (bssid_colon && bssid_colon[0] && parse_bssid(bssid_colon, slot.bssid)) {
        slot.bssid_set = true;
    }

    if (secondary) {
        if (!s_primary.ssid[0]) {
            return ESP_ERR_INVALID_STATE; /* need primary first */
        }
        s_secondary = slot;
        s_has_secondary = true;
    } else {
        s_primary = slot;
        s_active_slot = 0;
    }

    esp_err_t err = nvs_save_slots();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(err));
        return err;
    }
    status_sync_slots();

    if (secondary) {
        /* Saving secondary does not disconnect primary. */
        ESP_LOGI(TAG, "Secondary Wi-Fi saved '%s'%s", slot.ssid,
                 slot.bssid_set ? " (BSSID pinned)" : "");
        return ESP_OK;
    }

    /* Keep SoftAP up briefly so setup portal clients see the result; also helps
     * when changing Wi‑Fi from the home portal if the new AP fails. */
    s_softap_lifeline = false;
    s_recovery_depth = 0;
    s_retry = 0;
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (!s_status.ap_active) {
        configure_softap();
    }

    err = configure_sta_slot(&s_primary);
    if (err != ESP_OK) {
        return err;
    }

    s_retry = 0;
    xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    ESP_LOGI(TAG, "Connecting to home Wi-Fi '%s'%s...", ssid,
             slot.bssid_set ? " (BSSID pinned)" : "");
    esp_wifi_disconnect();
    return esp_wifi_connect();
}

esp_err_t wifi_manager_clear_sta(void)
{
    nvs_erase_sta();
    memset(&s_primary, 0, sizeof(s_primary));
    memset(&s_secondary, 0, sizeof(s_secondary));
    s_has_secondary = false;
    s_active_slot = 0;
    s_status.has_sta_credentials = false;
    s_status.ssid[0] = '\0';
    s_status.bssid[0] = '\0';
    s_status.rssi = 0;
    status_sync_slots();
    snprintf(s_status.ip, sizeof(s_status.ip), "-");
    s_retry = CONFIG_WIFI_MAX_RETRY; /* stop reconnect loop */
    esp_wifi_disconnect();
    softap_start();
    s_status.state = WIFI_MGR_AP_MODE;
    ESP_LOGI(TAG, "Cleared home Wi-Fi — SoftAP setup mode restored");
    return ESP_OK;
}

esp_err_t wifi_manager_clear_secondary(void)
{
    memset(&s_secondary, 0, sizeof(s_secondary));
    s_has_secondary = false;
    if (s_active_slot == 1) {
        s_active_slot = 0;
    }
    esp_err_t err = nvs_save_slots();
    status_sync_slots();
    if (err != ESP_OK) {
        return err;
    }
    if (s_status.state == WIFI_MGR_CONNECTED && s_active_slot == 0) {
        return ESP_OK;
    }
    if (s_primary.ssid[0]) {
        esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (!s_status.ap_active) {
            configure_softap();
        }
        (void)configure_sta_slot(&s_primary);
        esp_wifi_disconnect();
        return esp_wifi_connect();
    }
    return ESP_OK;
}

int wifi_manager_scan_aps(wifi_scan_ap_t *out, int max_aps)
{
    if (!s_started || !out || max_aps <= 0) {
        return 0;
    }

    wifi_mode_t mode = WIFI_MODE_NULL;
    (void)esp_wifi_get_mode(&mode);
    /* Never flip SoftAP-only ↔ APSTA here — that mode switch has wedged the C3
     * (SoftAP + ICMP die until power cycle). Setup/lifeline stays APSTA. */
    if (mode == WIFI_MODE_AP) {
        ESP_LOGW(TAG, "Scan skipped — SoftAP-only (enter SSID manually, or reboot)");
        return 0;
    }
    if (mode != WIFI_MODE_STA && mode != WIFI_MODE_APSTA) {
        ESP_LOGW(TAG, "Scan skipped — wifi mode %d", (int)mode);
        return 0;
    }

    wifi_manager_set_busy(true);
    /* Hold STA connect while SoftAP is up so STA_START mid-scan does not race. */
    bool hold = s_status.ap_active || s_softap_lifeline;
    if (hold) {
        s_scan_hold_sta = true;
    }

    /* Passive while associated — active scans have repeatedly killed C3 STA ICMP/portal. */
    const bool associated = (s_status.state == WIFI_MGR_CONNECTED);
    wifi_scan_config_t scan = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = (associated || s_status.ap_active) ? WIFI_SCAN_TYPE_PASSIVE
                                                         : WIFI_SCAN_TYPE_ACTIVE,
        .scan_time =
            {
                .active = {.min = 50, .max = 90},
                .passive = associated ? 120 : 80,
            },
    };

    esp_err_t err = esp_wifi_scan_start(&scan, true);
    int written = 0;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scan_start failed: %s", esp_err_to_name(err));
    } else {
        uint16_t ap_count = 0;
        esp_wifi_scan_get_ap_num(&ap_count);
        if (ap_count > 32) {
            ap_count = 32;
        }
        if (ap_count > 0) {
            wifi_ap_record_t *records = calloc(ap_count, sizeof(wifi_ap_record_t));
            if (records) {
                uint16_t n = ap_count;
                if (esp_wifi_scan_get_ap_records(&n, records) == ESP_OK) {
                    for (uint16_t i = 1; i < n; i++) {
                        wifi_ap_record_t key = records[i];
                        int j = (int)i - 1;
                        while (j >= 0 && records[j].rssi < key.rssi) {
                            records[j + 1] = records[j];
                            j--;
                        }
                        records[j + 1] = key;
                    }
                    for (uint16_t i = 0; i < n && written < max_aps; i++) {
                        if (records[i].ssid[0] == '\0') {
                            continue;
                        }
                        strncpy(out[written].ssid, (char *)records[i].ssid,
                                sizeof(out[written].ssid) - 1);
                        format_bssid(records[i].bssid, out[written].bssid,
                                     sizeof(out[written].bssid));
                        out[written].rssi = records[i].rssi;
                        out[written].channel = records[i].primary;
                        written++;
                    }
                }
                free(records);
            }
        }
    }

    if (hold) {
        s_scan_hold_sta = false;
    }
    wifi_manager_set_busy(false);
    wifi_manager_note_traffic();

    return written;
}

void wifi_manager_get_status(wifi_manager_status_t *out)
{
    if (!out) {
        return;
    }
    status_sync_slots();
    *out = s_status;
    if (s_status.state == WIFI_MGR_CONNECTED) {
        refresh_assoc_bssid();
        out->rssi = s_status.rssi;
        strncpy(out->bssid, s_status.bssid, sizeof(out->bssid) - 1);
        set_ip_from_netif(s_netif_sta, out->ip, sizeof(out->ip));
    }
    set_ip_from_netif(s_netif_ap, out->ap_ip, sizeof(out->ap_ip));
}

bool wifi_manager_is_connected(void)
{
    return s_status.state == WIFI_MGR_CONNECTED;
}

bool wifi_manager_is_started(void)
{
    return s_started;
}

bool wifi_manager_is_ap_active(void)
{
    return s_status.ap_active;
}

bool wifi_manager_is_setup_portal(void)
{
    return s_status.ap_active && s_status.state != WIFI_MGR_CONNECTED;
}
