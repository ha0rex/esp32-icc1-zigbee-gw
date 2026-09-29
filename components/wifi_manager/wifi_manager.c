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
#include "lwip/sockets.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

static const char *TAG = "wifi_mgr";

#define NVS_NS              "wifi_cfg"
#define NVS_KEY_SSID        "ssid"
#define NVS_KEY_PASS        "pass"
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
/** Tear SoftAP down soon after STA IP — APSTA on C3 starves portal/HAP TX. */
#define SOFTAP_STOP_DELAY_MS 8000
/** SoftAP only after this many failed STA retries (not on the first blip). */
#define SOFTAP_AFTER_RETRY 5

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
static uint8_t s_silent_strikes;
static uint8_t s_recovery_depth; /* 0=none, 1=reconnect done, 2=stop/start done */
static bool s_bssid_locked;
static uint8_t s_locked_bssid[6];
static volatile bool s_in_recovery;
/** SoftAP raised because STA inbound is dead — do not auto-tear it down. */
static bool s_softap_lifeline;
/** Consecutive successful portal/HAP hits while SoftAP lifeline is up. */
static uint8_t s_traffic_good_streak;

static esp_err_t softap_start(void);
static esp_err_t softap_stop(void);
static void schedule_softap_stop(void);
static void cancel_softap_stop(void);

/**
 * Remember current BSSID for the *next* connect only.
 * Never call esp_wifi_set_config() while associated — that wedges C3.
 * Skip weak links and clear any prior pin so multi-AP roam stays free.
 */
static void remember_sta_bssid(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        return;
    }
    /* Pinning a BSSID on this mesh/roaming AP made TX die after a few minutes. */
    if (ap.rssi < -55) {
        if (s_bssid_locked) {
            s_bssid_locked = false;
            memset(s_locked_bssid, 0, sizeof(s_locked_bssid));
            ESP_LOGW(TAG, "Cleared BSSID pin at weak rssi=%d (allow roam)", (int)ap.rssi);
        }
        return;
    }
    /* Do not lock BSSID — CREA SPACE has multiple APs; pin caused silent TX. */
    if (s_bssid_locked) {
        s_bssid_locked = false;
        memset(s_locked_bssid, 0, sizeof(s_locked_bssid));
        ESP_LOGI(TAG, "BSSID pin disabled (multi-AP roam)");
    }
}

void wifi_manager_note_traffic(void)
{
    s_last_traffic_us = esp_timer_get_time();
    s_silent_strikes = 0;
    if (s_traffic_good_streak < 255) {
        s_traffic_good_streak++;
    }
    /* Only clear recovery depth after sustained good hits — a single HAP EVENT
     * after soft-reconnect used to reset depth and trap us in reconnect loops. */
    if (s_traffic_good_streak >= 5) {
        s_recovery_depth = 0;
    }
    remember_sta_bssid();
    if (s_softap_lifeline && s_status.ap_active && s_traffic_good_streak >= 10) {
        ESP_LOGI(TAG, "STA inbound stable (%u hits) — releasing SoftAP lifeline",
                 (unsigned)s_traffic_good_streak);
        s_softap_lifeline = false;
        schedule_softap_stop();
    }
}

void wifi_manager_note_tx_fail(void)
{
    /* Invalidate traffic grace so health can recover quickly. */
    s_last_traffic_us = 0;
    s_silent_strikes++;
    ESP_LOGW(TAG, "STA TX fail noted (strike %u)", (unsigned)s_silent_strikes);
}

/** True if STA can open a TCP connection toward the gateway (TX path alive).
 * Returns true on success OR when the check cannot run (no free sockets) —
 * never treat resource exhaustion as a dead radio.
 * Try several ports — many routers ignore TCP/53 (UDP DNS only), which used to
 * look like “TX dead” and force reconnect loops every couple of minutes. */
static int probe_one_port(uint32_t gw, uint16_t port)
{
    int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s < 0) {
        return -1;
    }
    int flags = fcntl(s, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(s, F_SETFL, flags | O_NONBLOCK);
    }
    struct sockaddr_in dest = {0};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    dest.sin_addr.s_addr = gw;
    int cr = connect(s, (struct sockaddr *)&dest, sizeof(dest));
    if (cr == 0) {
        close(s);
        return 1;
    }
    if (errno == ECONNREFUSED) {
        close(s);
        return 1; /* RST = our SYN reached the LAN */
    }
    if (errno != EINPROGRESS && errno != EALREADY) {
        close(s);
        return 0;
    }
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(s, &wfds);
    struct timeval tv = {.tv_sec = 0, .tv_usec = 600000};
    int sel = select(s + 1, NULL, &wfds, NULL, &tv);
    int soerr = 0;
    socklen_t sl = sizeof(soerr);
    if (sel > 0) {
        getsockopt(s, SOL_SOCKET, SO_ERROR, &soerr, &sl);
        close(s);
        /* Connected, refused, or other ICMP-unreachable — TX worked. */
        return 1;
    }
    close(s);
    return 0;
}

static int probe_gateway_tx(void)
{
    if (!s_netif_sta) {
        return -1; /* skip */
    }
    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(s_netif_sta, &info) != ESP_OK || info.gw.addr == 0) {
        return -1;
    }
    static const uint16_t ports[] = {80, 443, 8080, 53};
    bool any_socket = false;
    for (size_t i = 0; i < sizeof(ports) / sizeof(ports[0]); i++) {
        int r = probe_one_port(info.gw.addr, ports[i]);
        if (r < 0) {
            continue; /* no socket */
        }
        any_socket = true;
        if (r > 0) {
            return 1;
        }
    }
    return any_socket ? 0 : -1;
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

static void wifi_soft_reconnect(const char *why)
{
    ESP_LOGW(TAG, "STA recovery (%s) — disconnect/reconnect", why ? why : "unknown");
    s_status.state = WIFI_MGR_CONNECTING;
    snprintf(s_status.ip, sizeof(s_status.ip), "-");
    s_silent_strikes = 0;
    s_got_ip_us = 0;
    s_traffic_good_streak = 0;
    s_in_recovery = true;
    /* Drop any BSSID pin so reconnect can pick the stronger AP. */
    s_bssid_locked = false;
    memset(s_locked_bssid, 0, sizeof(s_locked_bssid));
    wifi_config_t cfg = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &cfg) == ESP_OK && cfg.sta.bssid_set) {
        cfg.sta.bssid_set = false;
        memset(cfg.sta.bssid, 0, sizeof(cfg.sta.bssid));
        esp_wifi_set_config(WIFI_IF_STA, &cfg);
    }
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_wifi_connect();
    s_in_recovery = false;
}

static void wifi_force_radio_cycle(const char *why)
{
    ESP_LOGW(TAG, "STA recovery (%s) — wifi stop/start", why ? why : "unknown");
    s_status.state = WIFI_MGR_CONNECTING;
    snprintf(s_status.ip, sizeof(s_status.ip), "-");
    s_retry = 0;
    s_silent_strikes = 0;
    s_got_ip_us = 0;
    s_traffic_good_streak = 0;
    s_bssid_locked = false;
    memset(s_locked_bssid, 0, sizeof(s_locked_bssid));
    s_in_recovery = true;
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_wifi_start(); /* STA_START handler calls esp_wifi_connect() */
    s_in_recovery = false;
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

/** Keep STA alive without false reconnects.
 * TCP probes to the gateway often fail on filtered LAN ports while the portal
 * still works — reconnect then kills a healthy STA ~1–2 min after boot.
 * Only treat outbound probe failure as TX-dead after a long inbound quiet
 * window (real silence). SoftAP stays setup/fail-only (APSTA starves C3 TX). */
static void wifi_health_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(15000));
        if (!s_started || s_in_recovery) {
            continue;
        }
        if (s_status.state != WIFI_MGR_CONNECTED) {
            continue;
        }
        int64_t now = esp_timer_get_time();
        if (s_got_ip_us && (now - s_got_ip_us) < 20000000LL) {
            continue;
        }

        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
            s_silent_strikes++;
            ESP_LOGW(TAG, "STA health: no AP info (strike %u)", (unsigned)s_silent_strikes);
            if (s_silent_strikes >= 2) {
                wifi_force_radio_cycle("no AP info");
                s_silent_strikes = 0;
            }
            continue;
        }
        s_status.rssi = ap.rssi;

        /* Any successful HTTP/HAP in the last 2 minutes ⇒ leave the radio alone. */
        const bool recent_inbound =
            s_last_traffic_us && (now - s_last_traffic_us) < 120000000LL;
        if (recent_inbound) {
            s_silent_strikes = 0;
            if (s_status.ap_active && !s_softap_lifeline) {
                schedule_softap_stop();
            }
            continue;
        }

        int pr = probe_gateway_tx();
        if (pr < 0) {
            continue;
        }
        if (pr == 0) {
            s_silent_strikes++;
            ESP_LOGW(TAG, "STA health: quiet + gateway unreachable (strike %u, rssi=%d depth=%u)",
                     (unsigned)s_silent_strikes, (int)ap.rssi, (unsigned)s_recovery_depth);
            if (s_silent_strikes >= 3) {
                s_silent_strikes = 0;
                s_traffic_good_streak = 0;
                if (s_status.ap_active) {
                    softap_stop();
                }
                if (s_recovery_depth < 1) {
                    s_recovery_depth = 1;
                    wifi_soft_reconnect("gateway TX dead");
                } else {
                    s_recovery_depth = 2;
                    wifi_force_radio_cycle("gateway TX dead");
                }
            }
            continue;
        }

        s_silent_strikes = 0;
        remember_sta_bssid();
        if (s_status.ap_active && !s_softap_lifeline) {
            schedule_softap_stop();
        }
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

static esp_err_t nvs_load_sta(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;
    }
    size_t sl = ssid_len;
    size_t pl = pass_len;
    err = nvs_get_str(h, NVS_KEY_SSID, ssid, &sl);
    if (err == ESP_OK) {
        err = nvs_get_str(h, NVS_KEY_PASS, pass, &pl);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            if (pass_len) {
                pass[0] = '\0';
            }
            err = ESP_OK;
        }
    }
    nvs_close(h);
    return err;
}

static esp_err_t nvs_save_sta(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, NVS_KEY_SSID, ssid ? ssid : "");
    if (err == ESP_OK) {
        err = nvs_set_str(h, NVS_KEY_PASS, pass ? pass : "");
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

static esp_err_t configure_sta(const char *ssid, const char *pass)
{
    if (!ssid || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t sta = {0};
    strncpy((char *)sta.sta.ssid, ssid, sizeof(sta.sta.ssid) - 1);
    if (pass) {
        strncpy((char *)sta.sta.password, pass, sizeof(sta.sta.password) - 1);
    }
    if (pass && pass[0] != '\0') {
        sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        sta.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }
    /* Never pin BSSID — multi-AP roam on CREA SPACE; pin caused silent TX. */
    sta.sta.bssid_set = false;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &sta);
    if (err != ESP_OK) {
        return err;
    }

    strncpy(s_status.ssid, ssid, sizeof(s_status.ssid) - 1);
    s_status.has_sta_credentials = true;
    s_retry = 0;
    s_status.state = WIFI_MGR_CONNECTING;
    snprintf(s_status.ip, sizeof(s_status.ip), "-");
    return ESP_OK;
}

static esp_err_t softap_start(void)
{
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) {
        return err;
    }
    err = configure_softap();
    if (err != ESP_OK) {
        return err;
    }
    s_status.ap_active = true;
    if (s_status.state != WIFI_MGR_CONNECTED && s_status.state != WIFI_MGR_CONNECTING) {
        s_status.state = WIFI_MGR_AP_MODE;
    }
    ESP_LOGI(TAG, "SoftAP enabled for setup");
    return ESP_OK;
}

static esp_err_t softap_stop(void)
{
    if (!s_status.ap_active) {
        return ESP_OK;
    }
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to switch to STA-only: %s", esp_err_to_name(err));
        return err;
    }
    s_status.ap_active = false;
    s_status.ap_ip[0] = '\0';
    s_softap_lifeline = false;
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
        if (s_status.has_sta_credentials) {
            s_status.state = WIFI_MGR_CONNECTING;
            esp_wifi_connect();
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_status.reconnects++;
        snprintf(s_status.ip, sizeof(s_status.ip), "-");
        cancel_softap_stop();
        if (s_in_recovery) {
            /* wifi_soft_reconnect / force_radio_cycle owns the reconnect. */
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
            /* APSTA while STA is flapping kills C3 TX (portal + HomeKit hang).
             * Only bring SoftAP back after several STA failures. */
            if (s_retry >= SOFTAP_AFTER_RETRY && !s_status.ap_active) {
                softap_start();
            }
            ESP_LOGW(TAG, "STA disconnected, retry %d/%d", s_retry, CONFIG_WIFI_MAX_RETRY);
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
        }
        ESP_LOGI(TAG, "STA connected to '%s', IP %s RSSI %d", s_status.ssid, s_status.ip,
                 s_status.rssi);
        s_got_ip_us = esp_timer_get_time();
        /* Re-apply after every join — stop/start and roam drop these. */
        wifi_apply_sta_radio_quirks();
        /* Do not seed s_last_traffic_us here — that blocked TX-dead recovery for 2 minutes. */
        /* If SoftAP was up (portal / reconnect), tear it down after HAP/httpd settle.
         * Booting with saved credentials uses STA-only from the start — no teardown. */
        if (s_status.ap_active) {
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

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                                        &on_wifi_event, NULL, NULL));

    char nvs_ssid[33] = {0};
    char nvs_pass[65] = {0};
    bool have_creds = (nvs_load_sta(nvs_ssid, sizeof(nvs_ssid), nvs_pass, sizeof(nvs_pass)) ==
                       ESP_OK) &&
                      (nvs_ssid[0] != '\0');

    /* Optional compile-time seed if NVS empty (dev convenience only). */
    if (!have_creds && strlen(CONFIG_WIFI_SSID) > 0) {
        strncpy(nvs_ssid, CONFIG_WIFI_SSID, sizeof(nvs_ssid) - 1);
        strncpy(nvs_pass, CONFIG_WIFI_PASSWORD, sizeof(nvs_pass) - 1);
        have_creds = true;
        ESP_LOGI(TAG, "Seeding STA credentials from menuconfig into NVS");
        nvs_save_sta(nvs_ssid, nvs_pass);
    }

    /*
     * Prefer STA-only when home Wi-Fi is known. Booting APSTA then tearing SoftAP
     * down has left the C3 associated but silent; SoftAP is only for setup / fail.
     */
    if (have_creds) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(configure_sta(nvs_ssid, nvs_pass));
        s_status.ap_active = false;
        s_status.state = WIFI_MGR_CONNECTING;
        ESP_LOGI(TAG, "Connecting to home Wi-Fi '%s' (SoftAP off until STA fails)", nvs_ssid);
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

esp_err_t wifi_manager_apply_sta(const char *ssid, const char *password)
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

    esp_err_t err = nvs_save_sta(ssid, password ? password : "");
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Keep SoftAP up only while associating from the setup portal. */
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (!s_status.ap_active) {
        configure_softap();
    }

    err = configure_sta(ssid, password ? password : "");
    if (err != ESP_OK) {
        return err;
    }

    s_retry = 0;
    xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    ESP_LOGI(TAG, "Connecting to home Wi-Fi '%s'...", ssid);
    esp_wifi_disconnect();
    return esp_wifi_connect();
}

esp_err_t wifi_manager_clear_sta(void)
{
    nvs_erase_sta();
    s_status.has_sta_credentials = false;
    s_status.ssid[0] = '\0';
    s_status.rssi = 0;
    s_bssid_locked = false;
    memset(s_locked_bssid, 0, sizeof(s_locked_bssid));
    snprintf(s_status.ip, sizeof(s_status.ip), "-");
    s_retry = CONFIG_WIFI_MAX_RETRY; /* stop reconnect loop */
    esp_wifi_disconnect();
    softap_start();
    s_status.state = WIFI_MGR_AP_MODE;
    ESP_LOGI(TAG, "Cleared home Wi-Fi — SoftAP setup mode restored");
    return ESP_OK;
}

int wifi_manager_scan(char ssids[][33], int max_ssids)
{
    /* Optional helper — do not call from SoftAP HTTP handlers.
     * Blocking scans while clients are associated drop SoftAP links. */
    if (!s_started || !ssids || max_ssids <= 0) {
        return 0;
    }

    wifi_scan_config_t scan = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 80,
        .scan_time.active.max = 120,
    };

    esp_err_t err = esp_wifi_scan_start(&scan, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scan_start failed: %s", esp_err_to_name(err));
        return 0;
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count == 0) {
        return 0;
    }
    if (ap_count > 16) {
        ap_count = 16;
    }

    wifi_ap_record_t *records = calloc(ap_count, sizeof(wifi_ap_record_t));
    if (!records) {
        return 0;
    }
    uint16_t n = ap_count;
    if (esp_wifi_scan_get_ap_records(&n, records) != ESP_OK) {
        free(records);
        return 0;
    }

    int out = 0;
    for (uint16_t i = 0; i < n && out < max_ssids; i++) {
        if (records[i].ssid[0] == '\0') {
            continue;
        }
        bool dup = false;
        for (int j = 0; j < out; j++) {
            if (strcmp(ssids[j], (char *)records[i].ssid) == 0) {
                dup = true;
                break;
            }
        }
        if (!dup) {
            strncpy(ssids[out], (char *)records[i].ssid, 32);
            ssids[out][32] = '\0';
            out++;
        }
    }
    free(records);
    return out;
}

void wifi_manager_get_status(wifi_manager_status_t *out)
{
    if (!out) {
        return;
    }
    *out = s_status;
    if (s_status.state == WIFI_MGR_CONNECTED) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            out->rssi = ap.rssi;
        }
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
