/**
 * @file web_server.c
 * @brief Production portal + JSON control APIs.
 */

#include "web_server.h"
#include "web_app.h"
#include "web_setup.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <strings.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "wifi_manager.h"
#include "zigbee_host.h"
#include "homekit_bridge.h"
#include "thermostat.h"
#include "peer_link.h"
#include "ezsp.h"
#include "fw_ota.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_system.h"

static const char *TAG = "web";
static httpd_handle_t s_server;
static SemaphoreHandle_t s_scan_mutex;
static SemaphoreHandle_t s_status_mutex; /**< Serializes shared s_json / s_zb_snap */
static SemaphoreHandle_t s_root_mu; /**< At most one full portal HTML send */
/** Enough for a dense 2.4 GHz neighbourhood; must match scan call max. */
#define WEB_SCAN_AP_MAX 32
static wifi_scan_ap_t s_scan_aps[WEB_SCAN_AP_MAX];
/** ~100 bytes/AP worst case + wrapper; 32 APs need headroom past 2 KiB. */
static char s_scan_json[4096];
static char s_json[8192];
/* Light status snapshot — devices are fetched via zigbee_host_get_device_at(). */
static zigbee_host_status_t s_zb_snap;
static int64_t s_last_handler_us; /**< Last successful URI handler completion */
static int64_t s_root_held_us;    /**< When root HTML mutex was taken (0 = free) */
static TaskHandle_t s_watch_task;

static const char *thermo_mode_json(const group_t *g);

static const char *device_kind_json(const zb_device_t *d)
{
    switch (zigbee_host_device_kind(d)) {
    case ZB_DEVICE_KIND_LIGHT:
        return "light";
    case ZB_DEVICE_KIND_SWITCH:
        return "switch";
    case ZB_DEVICE_KIND_OUTLET:
        return "outlet";
    case ZB_DEVICE_KIND_IRRIGATION:
        return "irrigation";
    case ZB_DEVICE_KIND_SENSOR:
        return "sensor";
    case ZB_DEVICE_KIND_REMOTE:
        return "remote";
    case ZB_DEVICE_KIND_CONTACT:
        return "contact";
    case ZB_DEVICE_KIND_MOTION:
        return "motion";
    case ZB_DEVICE_KIND_LEAK:
        return "leak";
    case ZB_DEVICE_KIND_SMOKE:
        return "smoke";
    default:
        return "unknown";
    }
}

static void web_note_handler_ok(void)
{
    s_last_handler_us = esp_timer_get_time();
    wifi_manager_note_traffic();
}

static void web_watchdog_task(void *arg);
static esp_err_t web_server_restart(void);
static bool json_get_string(const char *body, const char *key, char *out, size_t out_len);

/** snprintf into buffer with truncation-safe position advance. */
static size_t json_append(char *buf, size_t bufsz, size_t pos, const char *fmt, ...)
{
    if (!buf || pos >= bufsz) {
        return pos;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + pos, bufsz - pos, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return pos;
    }
    if ((size_t)n >= bufsz - pos) {
        if (bufsz > 0) {
            buf[bufsz - 1] = '\0';
        }
        return bufsz;
    }
    return pos + (size_t)n;
}

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static void url_decode(char *s)
{
    char *r = s, *w = s;
    while (*r) {
        if (*r == '+') {
            *w++ = ' ';
            r++;
        } else if (*r == '%' && r[1] && r[2]) {
            int hi = hex_val(r[1]), lo = hex_val(r[2]);
            if (hi >= 0 && lo >= 0) {
                *w++ = (char)((hi << 4) | lo);
                r += 3;
            } else {
                *w++ = *r++;
            }
        } else {
            *w++ = *r++;
        }
    }
    *w = '\0';
}

static bool form_get(const char *body, const char *key, char *out, size_t out_len)
{
    size_t key_len = strlen(key);
    const char *p = body;
    while (p && *p) {
        if ((p == body || p[-1] == '&') && strncmp(p, key, key_len) == 0 && p[key_len] == '=') {
            p += key_len + 1;
            size_t i = 0;
            while (*p && *p != '&' && i + 1 < out_len) {
                out[i++] = *p++;
            }
            out[i] = '\0';
            url_decode(out);
            return true;
        }
        p = strchr(p, '&');
        if (p) {
            p++;
        }
    }
    out[0] = '\0';
    return false;
}

static void json_escape(const char *in, char *out, size_t out_len)
{
    size_t o = 0;
    if (!in) {
        out[0] = '\0';
        return;
    }
    for (size_t i = 0; in[i] && o + 2 < out_len; i++) {
        char c = in[i];
        if (c == '"' || c == '\\') {
            if (o + 3 >= out_len) {
                break;
            }
            out[o++] = '\\';
            out[o++] = c;
        } else if ((unsigned char)c < 0x20) {
            continue;
        } else {
            out[o++] = c;
        }
    }
    out[o] = '\0';
}

static esp_err_t root_get(httpd_req_t *req)
{
    /* SoftAP setup: serve the compact page — full portal (~85 KiB) never finishes on C3 SoftAP. */
    if (wifi_manager_is_setup_portal()) {
        web_note_handler_ok();
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        httpd_resp_set_hdr(req, "Connection", "close");
        return httpd_resp_sendstr(req, WEB_SETUP_HTML);
    }

    /* Only one full HTML transfer at a time — concurrent tab/captive downloads
     * used to occupy every httpd socket until send timeouts, hanging the portal. */
    if (!s_root_mu || xSemaphoreTake(s_root_mu, 0) != pdTRUE) {
        web_note_handler_ok();
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Connection", "close");
        httpd_resp_set_hdr(req, "Retry-After", "1");
        return httpd_resp_sendstr(req, "busy");
    }
    /* Count the request as inbound immediately so Wi‑Fi health does not soft-
     * reconnect mid-transfer when the GW TCP probe times out under load. */
    web_note_handler_ok();
    wifi_manager_set_busy(true);
    s_root_held_us = esp_timer_get_time();
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    const char *html = WEB_APP_HTML;
    size_t len = strlen(html);
    const size_t chunk = 1024; /* smaller chunks — less stall risk on C3 TX */
    esp_err_t err = ESP_OK;
    unsigned chunk_i = 0;
    for (size_t off = 0; off < len; off += chunk) {
        size_t n = len - off;
        if (n > chunk) {
            n = chunk;
        }
        err = httpd_resp_send_chunk(req, html + off, n);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "root HTML chunk @%u failed: %s", (unsigned)off, esp_err_to_name(err));
            httpd_resp_send_chunk(req, NULL, 0);
            break;
        }
        chunk_i++;
        if ((chunk_i & 7u) == 0u) {
            web_note_handler_ok();
            vTaskDelay(pdMS_TO_TICKS(2));
        } else {
            vTaskDelay(1);
        }
    }
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, NULL, 0);
    }
    web_note_handler_ok();
    s_root_held_us = 0;
    wifi_manager_set_busy(false);
    xSemaphoreGive(s_root_mu);
    return err;
}

/** Captive-portal probes must NOT get the full UI — that pinned all web sockets. */
static esp_err_t captive_ok(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    web_note_handler_ok();
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t api_ping(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    web_note_handler_ok();
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static const char *reset_reason_json(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
        return "power_on";
    case ESP_RST_EXT:
        return "external";
    case ESP_RST_SW:
        return "software";
    case ESP_RST_PANIC:
        return "panic";
    case ESP_RST_INT_WDT:
        return "int_wdt";
    case ESP_RST_TASK_WDT:
        return "task_wdt";
    case ESP_RST_WDT:
        return "wdt";
    case ESP_RST_DEEPSLEEP:
        return "deepsleep";
    case ESP_RST_BROWNOUT:
        return "brownout";
    case ESP_RST_SDIO:
        return "sdio";
    case ESP_RST_USB:
        return "usb";
    case ESP_RST_JTAG:
        return "jtag";
    case ESP_RST_EFUSE:
        return "efuse";
    case ESP_RST_PWR_GLITCH:
        return "pwr_glitch";
    case ESP_RST_CPU_LOCKUP:
        return "cpu_lockup";
    default:
        return "unknown";
    }
}

static esp_err_t api_status(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Connection", "close");
    /* Portal polls /api/status every few seconds; overlapping requests used to
     * race on s_json/s_zb_snap and panic (heap/stack corruption). */
    if (!s_status_mutex || xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(800)) != pdTRUE) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Retry-After", "1");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"busy\":true}");
    }
    wifi_manager_status_t wifi;
    zigbee_host_get_status(&s_zb_snap);
    wifi_manager_get_status(&wifi);
    zigbee_host_status_t *zb = &s_zb_snap;

    char ver[32] = "", eui[40] = "", epid[40] = "";
    if (zb->version_ok) {
        ezsp_format_stack_version(zb->ncp.stack_version, ver, sizeof(ver));
    }
    if (zb->ncp.eui64_valid) {
        ezsp_format_eui64(zb->ncp.eui64, eui, sizeof(eui));
    }
    if (zb->ncp.net_params_valid) {
        ezsp_format_epid(zb->ncp.net.extended_pan_id, epid, sizeof(epid));
    }

    uint64_t uptime_s = (uint64_t)(esp_timer_get_time() / 1000000ULL);
    char uptime[32];
    snprintf(uptime, sizeof(uptime), "%lluh %llum %llus", uptime_s / 3600ULL,
             (uptime_s / 60ULL) % 60ULL, uptime_s % 60ULL);

    char err_esc[128], ssid_esc[64], ap_esc[64], bssid_esc[40];
    char pssid_esc[64], pbssid_esc[40], sssid_esc[64], sbssid_esc[40];
    json_escape(zb->last_error, err_esc, sizeof(err_esc));
    json_escape(wifi.ssid, ssid_esc, sizeof(ssid_esc));
    json_escape(wifi.ap_ssid, ap_esc, sizeof(ap_esc));
    json_escape(wifi.bssid, bssid_esc, sizeof(bssid_esc));
    json_escape(wifi.primary_ssid, pssid_esc, sizeof(pssid_esc));
    json_escape(wifi.primary_bssid, pbssid_esc, sizeof(pbssid_esc));
    json_escape(wifi.secondary_ssid, sssid_esc, sizeof(sssid_esc));
    json_escape(wifi.secondary_bssid, sbssid_esc, sizeof(sbssid_esc));

    homekit_bridge_status_t hk;
    homekit_bridge_get_status(&hk);
    char hk_code[24], hk_id[12], hk_st[64];
    json_escape(hk.setup_code, hk_code, sizeof(hk_code));
    json_escape(hk.setup_id, hk_id, sizeof(hk_id));
    json_escape(hk.status, hk_st, sizeof(hk_st));

    size_t pos = 0;
#if CONFIG_IDF_TARGET_ESP32S3
    const char *board_chip = "esp32s3";
    const char *board_name = "ESP32-S3";
#elif CONFIG_IDF_TARGET_ESP32C3
    const char *board_chip = "esp32c3";
    const char *board_name = "ESP32-C3";
#else
    const char *board_chip = CONFIG_IDF_TARGET;
    const char *board_name = CONFIG_IDF_TARGET;
#endif
    pos += (size_t)snprintf(
        s_json + pos, sizeof(s_json) - pos,
        "{\"board\":{\"chip\":\"%s\",\"name\":\"%s\",\"uart_tx\":%d,\"uart_rx\":%d},"
        "\"icc\":\"%s\",\"ash\":\"%s\",\"ezsp\":%u,\"ember\":\"%s\",\"eui64\":\"%s\","
        "\"network\":\"%s\",\"node_type\":\"%s\",\"net_valid\":%s,\"channel\":%u,\"pan_id\":%u,"
        "\"epid\":\"%s\",\"tx_power\":%d,\"permit_join\":%u,\"device_count\":%u,"
        "\"wifi\":\"%s\",\"has_sta\":%s,\"ssid\":\"%s\",\"bssid\":\"%s\",\"ip\":\"%s\","
        "\"ap_ssid\":\"%s\",\"ap_ip\":\"%s\",\"ap_active\":%s,\"rssi\":%d,"
        "\"primary_ssid\":\"%s\",\"primary_bssid\":\"%s\",\"primary_bssid_set\":%s,"
        "\"secondary_ssid\":\"%s\",\"secondary_bssid\":\"%s\",\"secondary_bssid_set\":%s,"
        "\"has_secondary\":%s,\"active_slot\":%u,\"uptime\":\"%s\","
        "\"reset_reason\":\"%s\",\"free_heap\":%u,\"min_free_heap\":%u,\"largest_heap\":%u,"
        "\"tx\":%lu,\"rx\":%lu,\"ash_ok\":%lu,\"ash_bad\":%lu,\"crc_err\":%lu,\"timeouts\":%lu,"
        "\"ezsp_cmd\":%lu,\"ezsp_rsp\":%lu,\"last_error\":\"%s\","
        "\"homekit\":{\"started\":%s,\"paired\":%s,\"setup_code\":\"%s\",\"setup_id\":\"%s\","
        "\"accessories\":%u,\"status\":\"%s\"},\"devices\":[",
        board_chip, board_name, CONFIG_ICC_UART_TX_GPIO, CONFIG_ICC_UART_RX_GPIO,
        zigbee_host_icc_status_str(zb->icc_status), zigbee_host_ash_state_str(zb->ash_state),
        (unsigned)zb->ncp.protocol_version, ver, eui,
        zb->ncp.network_state_valid ? zigbee_host_network_state_str(zb->ncp.network_state) : "UNKNOWN",
        ezsp_node_type_str(zb->ncp.node_type), zb->ncp.net_params_valid ? "true" : "false",
        zb->ncp.net_params_valid ? zb->ncp.net.radio_channel : 0,
        zb->ncp.net_params_valid ? zb->ncp.net.pan_id : 0, epid,
        zb->ncp.net_params_valid ? zb->ncp.net.radio_tx_power : 0, zb->permit_join_remaining,
        zb->device_count, wifi_manager_state_str(wifi.state),
        wifi.has_sta_credentials ? "true" : "false", ssid_esc, bssid_esc, wifi.ip, ap_esc, wifi.ap_ip,
        wifi.ap_active ? "true" : "false", wifi.rssi, pssid_esc, pbssid_esc,
        wifi.primary_bssid_set ? "true" : "false", sssid_esc, sbssid_esc,
        wifi.secondary_bssid_set ? "true" : "false", wifi.has_secondary ? "true" : "false",
        (unsigned)wifi.active_slot, uptime, reset_reason_json(), (unsigned)esp_get_free_heap_size(),
        (unsigned)esp_get_minimum_free_heap_size(),
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
        (unsigned long)zb->ash_stats.tx_bytes,
        (unsigned long)zb->ash_stats.rx_bytes, (unsigned long)zb->ash_stats.valid_frames,
        (unsigned long)zb->ash_stats.invalid_frames, (unsigned long)zb->ash_stats.crc_errors,
        (unsigned long)zb->ash_stats.timeouts, (unsigned long)zb->ezsp_stats.commands_sent,
        (unsigned long)zb->ezsp_stats.responses_received, err_esc, hk.started ? "true" : "false",
        hk.paired ? "true" : "false", hk_code, hk_id, (unsigned)hk.accessory_count, hk_st);

    bool first = true;
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES && pos + 720 < sizeof(s_json); i++) {
        zb_device_t dslot;
        if (!zigbee_host_get_device_at(i, &dslot)) {
            continue;
        }
        const zb_device_t *d = &dslot;
        char deui[40], dlab[80], dname[80], dman[80], dmodel[80], dfw[80];
        ezsp_format_eui64(d->eui64, deui, sizeof(deui));
        json_escape(d->label, dlab, sizeof(dlab));
        json_escape(d->name, dname, sizeof(dname));
        json_escape(d->manufacturer, dman, sizeof(dman));
        json_escape(d->model, dmodel, sizeof(dmodel));
        json_escape(d->firmware, dfw, sizeof(dfw));
        pos += (size_t)snprintf(
            s_json + pos, sizeof(s_json) - pos,
            "%s{\"eui64\":\"%s\",\"node_id\":%u,\"type\":\"%s\",\"label\":\"%s\","
            "\"name\":\"%s\",\"manufacturer\":\"%s\",\"model\":\"%s\",\"firmware\":\"%s\","
            "\"kind\":\"%s\",\"has_temp\":%s,\"temperature_c\":%.2f,\"has_humidity\":%s,\"humidity_pct\":%.2f,"
            "\"has_battery\":%s,\"battery_pct\":%u,\"has_onoff\":%s,\"onoff_on\":%s,"
            "\"binary_on\":%s,\"has_level\":%s,\"brightness\":%u,\"buttons\":%u,\"button_modes\":[",
            first ? "" : ",", deui, d->node_id, ezsp_node_type_str(d->node_type), dlab, dname, dman,
            dmodel, dfw, device_kind_json(d),
            d->has_temp ? "true" : "false", (double)d->temperature_c,
            d->has_humidity ? "true" : "false", (double)d->humidity_pct,
            d->has_battery ? "true" : "false", (unsigned)d->battery_pct,
            d->has_onoff ? "true" : "false", d->onoff_on ? "true" : "false",
            d->binary_on ? "true" : "false",
            d->has_level ? "true" : "false",
            (unsigned)(d->has_level ? zigbee_host_level_to_brightness(d->level)
                                    : (d->onoff_on ? 100 : 0)),
            (unsigned)zigbee_host_remote_button_count(d));
        {
            uint8_t nb = zigbee_host_remote_button_count(d);
            bool climate = (nb == 0) && (d->has_temp || d->has_humidity ||
                                         zigbee_host_device_kind(d) == ZB_DEVICE_KIND_SENSOR);
            if (nb > ZB_REMOTE_MAX_BUTTONS) {
                nb = ZB_REMOTE_MAX_BUTTONS;
            }
            if (climate) {
                nb = 1; /* wake-button mode in btn_mode[0] */
            }
            for (uint8_t bi = 0; bi < nb; bi++) {
                uint8_t mode = d->btn_mode[bi];
                if (climate) {
                    if (mode != ZB_SENSOR_BTN_STATELESS && mode != ZB_SENSOR_BTN_STATEFUL) {
                        mode = ZB_SENSOR_BTN_NONE;
                    }
                } else {
                    mode = (mode == ZB_BTN_MODE_STATEFUL) ? 1 : 0;
                }
                pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "%s%u",
                                        bi ? "," : "", (unsigned)mode);
            }
            pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "],\"button_on\":[");
            for (uint8_t bi = 0; bi < nb; bi++) {
                pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "%s%s", bi ? "," : "",
                                        d->btn_on[bi] ? "true" : "false");
            }
            pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "],\"button_names\":[");
            for (uint8_t bi = 0; bi < nb; bi++) {
                char bn[48];
                if (climate) {
                    json_escape("Button", bn, sizeof(bn));
                } else {
                    json_escape(zigbee_host_remote_button_name(d, bi), bn, sizeof(bn));
                }
                pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "%s\"%s\"", bi ? "," : "",
                                        bn);
            }
            /* Close button_names[]; sensor wake-button mode is a sibling field. */
            pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "]");
            if (climate) {
                uint8_t sm = d->btn_mode[0];
                if (sm != ZB_SENSOR_BTN_STATELESS && sm != ZB_SENSOR_BTN_STATEFUL) {
                    sm = ZB_SENSOR_BTN_NONE;
                }
                pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos,
                                        ",\"sensor_btn_mode\":%u", (unsigned)sm);
            }
        }
        {
            uint8_t tc = d->target_count;
            if (tc == 0) {
                bool has_t = false;
                for (int ti = 0; ti < 8; ti++) {
                    if (d->target_eui64[ti]) {
                        has_t = true;
                        break;
                    }
                }
                if (has_t) {
                    tc = 1;
                }
            }
            if (tc > ZB_REMOTE_MAX_TARGETS) {
                tc = ZB_REMOTE_MAX_TARGETS;
            }
            /* button_names already closed above — continue the device object. */
            pos = json_append(s_json, sizeof(s_json), pos, ",\"remote_type\":%u,\"targets\":[",
                              (unsigned)((d->remote_type == ZB_REMOTE_TYPE_CONTROL)
                                             ? ZB_REMOTE_TYPE_CONTROL
                                             : ZB_REMOTE_TYPE_HOMEKIT));
            for (uint8_t ti = 0; ti < tc; ti++) {
                char teui[40];
                const uint8_t *te =
                    (d->target_count > 0) ? d->targets[ti] : d->target_eui64;
                ezsp_format_eui64(te, teui, sizeof(teui));
                pos = json_append(s_json, sizeof(s_json), pos, "%s\"%s\"", ti ? "," : "", teui);
            }
            pos = json_append(s_json, sizeof(s_json), pos, "],\"target_groups\":[");
            uint8_t tgc = d->target_group_count;
            if (tgc > ZB_REMOTE_MAX_TARGETS) {
                tgc = ZB_REMOTE_MAX_TARGETS;
            }
            for (uint8_t gi = 0; gi < tgc; gi++) {
                pos = json_append(s_json, sizeof(s_json), pos, "%s%u", gi ? "," : "",
                                  (unsigned)d->target_groups[gi]);
            }
            pos = json_append(
                s_json, sizeof(s_json), pos,
                "],\"rssi\":%d,\"lqi\":%u,\"homekit_expose\":%s,\"last_seen_ms\":%lld}",
                (int)d->last_rssi, (unsigned)d->last_lqi, d->homekit_expose ? "true" : "false",
                (long long)d->last_seen_ms);
        }
        first = false;
    }
    pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "],\"groups\":[");
    first = true;
    for (uint16_t i = 0; i < GROUP_MAX && pos + 700 < sizeof(s_json); i++) {
        group_t g;
        if (!group_get_at(i, &g)) {
            continue;
        }
        char gname[80];
        json_escape(g.name, gname, sizeof(gname));
        pos += (size_t)snprintf(
            s_json + pos, sizeof(s_json) - pos,
            "%s{\"id\":%u,\"name\":\"%s\",\"type\":\"%s\",\"homekit_expose\":%s,"
            "\"on\":%s,\"brightness\":%u,\"is_light\":%s,\"power_fail\":\"%s\",",
            first ? "" : ",", (unsigned)g.id, gname,
            g.type == GROUP_TYPE_THERMOSTAT ? "thermostat" : "general",
            g.homekit_expose ? "true" : "false", g.on ? "true" : "false",
            (unsigned)g.brightness_pct, g.is_light_group ? "true" : "false",
            g.power_fail_mode == GROUP_POWER_FAIL_ON
                ? "on"
                : (g.power_fail_mode == GROUP_POWER_FAIL_OFF ? "off" : "previous"));
        if (g.type == GROUP_TYPE_THERMOSTAT) {
            char seui[40], sweui[40], ceui[40], heui[40], weui[40];
            if (g.sensor_ref[0]) {
                snprintf(seui, sizeof(seui), "%s", g.sensor_ref);
            } else {
                ezsp_format_eui64(g.sensor_eui, seui, sizeof(seui));
            }
            if (g.switch_ref[0]) {
                snprintf(sweui, sizeof(sweui), "%s", g.switch_ref);
            } else if (group_thermo_has_heater(&g)) {
                ezsp_format_eui64(g.switch_eui, sweui, sizeof(sweui));
            } else {
                sweui[0] = '\0';
            }
            if (g.cooler_ref[0]) {
                snprintf(ceui, sizeof(ceui), "%s", g.cooler_ref);
            } else if (group_thermo_has_cooler(&g)) {
                ezsp_format_eui64(g.cooler_eui, ceui, sizeof(ceui));
            } else {
                ceui[0] = '\0';
            }
            snprintf(weui, sizeof(weui), "%s", g.window_ref);
            bool has_hum_sensor = false;
            for (int bi = 0; bi < 8; bi++) {
                if (g.humidity_sensor_eui[bi] != 0) {
                    has_hum_sensor = true;
                    break;
                }
            }
            if (has_hum_sensor) {
                ezsp_format_eui64(g.humidity_sensor_eui, heui, sizeof(heui));
            } else {
                heui[0] = '\0';
            }
            pos += (size_t)snprintf(
                s_json + pos, sizeof(s_json) - pos,
                "\"regulation\":\"%s\",\"sensor_eui\":\"%s\",\"humidity_sensor_eui\":\"%s\","
                "\"switch_eui\":\"%s\",\"cooler_eui\":\"%s\",\"window_eui\":\"%s\","
                "\"window_open\":%s,"
                "\"target_c\":%.1f,\"target_humidity\":%.0f,\"gap_c\":%.1f,\"hysteresis_c\":%.1f,"
                "\"humidity_force_heat\":%s,\"mode\":\"%s\","
                "\"heating\":%s,\"cooling\":%s,\"has_current_temp\":%s,\"current_temp_c\":%.2f,"
                "\"has_current_humidity\":%s,\"current_humidity_pct\":%.1f,\"members\":[]}",
                g.thermo_kind == GROUP_THERMO_KIND_HUMIDITY ? "humidity" : "temperature", seui,
                heui, sweui, ceui, weui, g.window_open ? "true" : "false",
                (double)g.target_c, (double)g.target_humidity_pct,
                (double)g.gap_c, (double)g.hysteresis_c, g.humidity_force_heat ? "true" : "false",
                thermo_mode_json(&g), g.heating ? "true" : "false", g.cooling ? "true" : "false",
                g.has_current_temp ? "true" : "false", (double)g.current_temp_c,
                g.has_current_humidity ? "true" : "false", (double)g.current_humidity_pct);
        } else {
            pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "\"members\":[");
            for (uint8_t m = 0; m < g.member_count; m++) {
                char meui[40];
                ezsp_format_eui64(g.members[m], meui, sizeof(meui));
                pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "%s\"%s\"", m ? "," : "",
                                        meui);
            }
            pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "]}");
        }
        first = false;
    }
    /* Keep legacy key for any cached UI. */
    pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "],\"thermostats\":[");
    first = true;
    for (uint16_t i = 0; i < GROUP_MAX && pos + 360 < sizeof(s_json); i++) {
        group_t t;
        if (!group_get_at(i, &t) || t.type != GROUP_TYPE_THERMOSTAT) {
            continue;
        }
        char tname[80], seui[40], sweui[40];
        json_escape(t.name, tname, sizeof(tname));
        ezsp_format_eui64(t.sensor_eui, seui, sizeof(seui));
        ezsp_format_eui64(t.switch_eui, sweui, sizeof(sweui));
        pos += (size_t)snprintf(
            s_json + pos, sizeof(s_json) - pos,
            "%s{\"id\":%u,\"name\":\"%s\",\"sensor_eui\":\"%s\",\"switch_eui\":\"%s\","
            "\"target_c\":%.1f,\"mode\":\"%s\",\"heating\":%s,"
            "\"has_current_temp\":%s,\"current_temp_c\":%.2f,\"homekit_expose\":%s}",
            first ? "" : ",", (unsigned)t.id, tname, seui, sweui, (double)t.target_c,
            t.mode == THERMO_MODE_HEAT ? "heat" : "off", t.heating ? "true" : "false",
            t.has_current_temp ? "true" : "false", (double)t.current_temp_c,
            t.homekit_expose ? "true" : "false");
        first = false;
    }
    pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "],\"remote_presses\":[");
    zb_remote_press_t presses[ZB_REMOTE_PRESS_LOG];
    uint8_t np = zigbee_host_copy_remote_presses(presses, ZB_REMOTE_PRESS_LOG);
    int64_t now_ms_host = esp_timer_get_time() / 1000;
    first = true;
    for (uint8_t i = 0; i < np && pos + 140 < sizeof(s_json); i++) {
        char peui[40];
        ezsp_format_eui64(presses[i].eui64, peui, sizeof(peui));
        const char *ev = presses[i].event == HK_BTN_EVENT_LONG
                             ? "long"
                             : (presses[i].event == HK_BTN_EVENT_DOUBLE ? "double" : "single");
        int64_t age = now_ms_host - presses[i].ms;
        if (age < 0) {
            age = 0;
        }
        pos += (size_t)snprintf(
            s_json + pos, sizeof(s_json) - pos,
            "%s{\"eui64\":\"%s\",\"button\":%u,\"event\":\"%s\",\"age_ms\":%lld}",
            first ? "" : ",", peui, (unsigned)presses[i].button, ev, (long long)age);
        first = false;
    }
    {
        const esp_app_desc_t *ad = esp_app_get_description();
        const char *ver = (ad && ad->version[0]) ? ad->version : "unknown";
        snprintf(s_json + pos, sizeof(s_json) - pos, "],\"fw_version\":\"%s\"}", ver);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_sendstr(req, s_json);
    if (err == ESP_OK) {
        web_note_handler_ok();
    }
    xSemaphoreGive(s_status_mutex);
    return err;
}

static esp_err_t read_body(httpd_req_t *req, char *buf, size_t buflen);

static esp_err_t api_ota_status(httpd_req_t *req)
{
    fw_ota_status_t st;
    fw_ota_get_status(&st);
    char msg[128], run[64], av[64], url[280], man[280];
    json_escape(st.message, msg, sizeof(msg));
    json_escape(st.running_version, run, sizeof(run));
    json_escape(st.available_version, av, sizeof(av));
    json_escape(st.firmware_url, url, sizeof(url));
    json_escape(st.manifest_url, man, sizeof(man));
    char body[1200];
    snprintf(body, sizeof(body),
             "{\"ok\":true,\"state\":\"%s\",\"channel\":\"%s\",\"channel_label\":\"%s\","
             "\"running\":\"%s\",\"available\":\"%s\",\"url\":\"%s\",\"manifest_url\":\"%s\","
             "\"update_available\":%s,\"progress\":%d,\"message\":\"%s\"}",
             fw_ota_state_str(st.state), fw_ota_channel_str(st.channel),
             fw_ota_channel_label(st.channel), run, av, url, man,
             st.update_available ? "true" : "false", st.progress_pct, msg);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t api_ota_channel(httpd_req_t *req)
{
    char body[160] = {0};
    if (read_body(req, body, sizeof(body)) != ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"bad body\"}");
    }
    char ch_s[24] = {0};
    json_get_string(body, "channel", ch_s, sizeof(ch_s));
    fw_ota_channel_t ch = FW_OTA_CHANNEL_STABLE;
    if (strcmp(ch_s, "nightly") == 0) {
        ch = FW_OTA_CHANNEL_NIGHTLY;
    } else if (strcmp(ch_s, "stable") != 0 && ch_s[0]) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"channel must be stable or nightly\"}");
    }
    esp_err_t err = fw_ota_set_channel(ch);
    fw_ota_status_t st;
    fw_ota_get_status(&st);
    char msg[128], man[280];
    json_escape(st.message, msg, sizeof(msg));
    json_escape(st.manifest_url, man, sizeof(man));
    char resp[640];
    snprintf(resp, sizeof(resp),
             "{\"ok\":%s,\"channel\":\"%s\",\"channel_label\":\"%s\",\"manifest_url\":\"%s\","
             "\"message\":\"%s\"}",
             err == ESP_OK ? "true" : "false", fw_ota_channel_str(st.channel),
             fw_ota_channel_label(st.channel), man, msg);
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
    }
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t api_ota_offer(httpd_req_t *req)
{
    char body[512];
    if (read_body(req, body, sizeof(body)) != ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"bad body\"}");
    }
    char version[48] = {0}, url[256] = {0};
    json_get_string(body, "version", version, sizeof(version));
    json_get_string(body, "url", url, sizeof(url));
    esp_err_t err = fw_ota_offer(version, url);
    fw_ota_status_t st;
    fw_ota_get_status(&st);
    char msg[128], run[64], av[64];
    json_escape(st.message, msg, sizeof(msg));
    json_escape(st.running_version, run, sizeof(run));
    json_escape(st.available_version, av, sizeof(av));
    char resp[420];
    snprintf(resp, sizeof(resp),
             "{\"ok\":%s,\"state\":\"%s\",\"running\":\"%s\",\"available\":\"%s\","
             "\"update_available\":%s,\"message\":\"%s\"}",
             err == ESP_OK ? "true" : "false", fw_ota_state_str(st.state), run, av,
             st.update_available ? "true" : "false", msg);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t api_ota_upload(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"empty body\"}");
    }
    esp_err_t err = fw_ota_upload_begin((size_t)total);
    if (err != ESP_OK) {
        fw_ota_status_t st;
        fw_ota_get_status(&st);
        char msg[128];
        json_escape(st.message, msg, sizeof(msg));
        char resp[200];
        snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", msg);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, resp);
    }

    char buf[1024];
    int remaining = total;
    while (remaining > 0) {
        int n = remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining;
        int r = httpd_req_recv(req, buf, n);
        if (r <= 0) {
            fw_ota_upload_abort();
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"recv failed\"}");
        }
        err = fw_ota_upload_write(buf, (size_t)r);
        if (err != ESP_OK) {
            char resp[120];
            snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", esp_err_to_name(err));
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, resp);
        }
        remaining -= r;
    }

    err = fw_ota_upload_finish();
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        char resp[120];
        snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", esp_err_to_name(err));
        return httpd_resp_sendstr(req, resp);
    }
    /* Reply first; reboot after the TCP send completes. */
    esp_err_t send_err =
        httpd_resp_sendstr(req, "{\"ok\":true,\"rebooting\":true,\"message\":\"Update OK — rebooting\"}");
    vTaskDelay(pdMS_TO_TICKS(400));
    esp_restart();
    return send_err;
}

static esp_err_t api_wifi_scan(httpd_req_t *req)
{
    if (!s_scan_mutex || xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"aps\":[],\"busy\":true}");
    }
    memset(s_scan_aps, 0, sizeof(s_scan_aps));
    int n = wifi_manager_scan_aps(s_scan_aps, WEB_SCAN_AP_MAX);
    if (n < 0) {
        n = 0;
    }
    if (n > WEB_SCAN_AP_MAX) {
        n = WEB_SCAN_AP_MAX;
    }
    size_t pos = 0;
    pos += (size_t)snprintf(s_scan_json + pos, sizeof(s_scan_json) - pos, "{\"aps\":[");
    int written = 0;
    for (int i = 0; i < n; i++) {
        /* Leave room for closing `],"count":NN}` (~16 bytes) + one AP (~120). */
        if (pos + 140 >= sizeof(s_scan_json)) {
            break;
        }
        char sessid[80], sebssid[48];
        json_escape(s_scan_aps[i].ssid, sessid, sizeof(sessid));
        json_escape(s_scan_aps[i].bssid, sebssid, sizeof(sebssid));
        int w = snprintf(s_scan_json + pos, sizeof(s_scan_json) - pos,
                         "%s{\"ssid\":\"%s\",\"bssid\":\"%s\",\"rssi\":%d,\"channel\":%u}",
                         written ? "," : "", sessid, sebssid, (int)s_scan_aps[i].rssi,
                         (unsigned)s_scan_aps[i].channel);
        if (w < 0 || (size_t)w >= sizeof(s_scan_json) - pos) {
            break;
        }
        pos += (size_t)w;
        written++;
    }
    snprintf(s_scan_json + pos, sizeof(s_scan_json) - pos, "],\"count\":%d}", written);
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, s_scan_json);
    xSemaphoreGive(s_scan_mutex);
    return err;
}

static esp_err_t wifi_save(httpd_req_t *req)
{
    char body[384];
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }
    int r = httpd_req_recv(req, body, total);
    if (r <= 0) {
        return ESP_FAIL;
    }
    body[r] = '\0';
    char ssid[33] = {0}, pass[65] = {0}, bssid[24] = {0}, slot[16] = {0};
    /* Prefer JSON (portal); fall back to form-urlencoded (SoftAP setup). */
    if (body[0] == '{') {
        json_get_string(body, "ssid", ssid, sizeof(ssid));
        json_get_string(body, "password", pass, sizeof(pass));
        json_get_string(body, "bssid", bssid, sizeof(bssid));
        json_get_string(body, "slot", slot, sizeof(slot));
    } else {
        form_get(body, "ssid", ssid, sizeof(ssid));
        form_get(body, "password", pass, sizeof(pass));
        form_get(body, "bssid", bssid, sizeof(bssid));
        form_get(body, "slot", slot, sizeof(slot));
    }
    if (!ssid[0]) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid required");
        return ESP_FAIL;
    }
    bool secondary = (slot[0] && strcasecmp(slot, "secondary") == 0);
    esp_err_t err = wifi_manager_apply_sta(ssid, pass, bssid[0] ? bssid : NULL, secondary);
    httpd_resp_set_type(req, "application/json");
    char msg[160];
    snprintf(msg, sizeof(msg), "{\"ok\":%s,\"slot\":\"%s\"}", err == ESP_OK ? "true" : "false",
             secondary ? "secondary" : "primary");
    return httpd_resp_sendstr(req, msg);
}

static esp_err_t wifi_clear(httpd_req_t *req)
{
    char tmp[64];
    while (httpd_req_recv(req, tmp, sizeof(tmp)) > 0) {
    }
    wifi_manager_clear_sta();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t wifi_clear_secondary(httpd_req_t *req)
{
    char tmp[64];
    while (httpd_req_recv(req, tmp, sizeof(tmp)) > 0) {
    }
    esp_err_t err = wifi_manager_clear_secondary();
    httpd_resp_set_type(req, "application/json");
    char msg[64];
    snprintf(msg, sizeof(msg), "{\"ok\":%s}", err == ESP_OK ? "true" : "false");
    return httpd_resp_sendstr(req, msg);
}

static int json_get_int(const char *body, const char *key, int def)
{
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(body, pat);
    if (!p) {
        return def;
    }
    p = strchr(p, ':');
    if (!p) {
        return def;
    }
    return atoi(p + 1);
}

static bool json_get_bool(const char *body, const char *key, bool def)
{
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(body, pat);
    if (!p) {
        return def;
    }
    p = strchr(p, ':');
    if (!p) {
        return def;
    }
    while (*p && (*p == ':' || *p == ' ')) {
        p++;
    }
    if (strncmp(p, "true", 4) == 0) {
        return true;
    }
    if (strncmp(p, "false", 5) == 0) {
        return false;
    }
    return def;
}

static float json_get_float(const char *body, const char *key, float def)
{
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(body, pat);
    if (!p) {
        return def;
    }
    p = strchr(p, ':');
    if (!p) {
        return def;
    }
    return (float)strtod(p + 1, NULL);
}

static bool json_get_string(const char *body, const char *key, char *out, size_t out_len)
{
    if (!body || !key || !out || out_len == 0) {
        if (out && out_len) {
            out[0] = '\0';
        }
        return false;
    }
    /* Require "key": so short keys like "name" do not match inside other keys. */
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = body;
    while ((p = strstr(p, pat)) != NULL) {
        const char *after = p + strlen(pat);
        while (*after == ' ' || *after == '\t' || *after == '\n' || *after == '\r') {
            after++;
        }
        if (*after != ':') {
            p += strlen(pat);
            continue;
        }
        after++;
        while (*after == ' ' || *after == '\t') {
            after++;
        }
        if (*after != '"') {
            return false;
        }
        after++;
        size_t i = 0;
        while (*after && *after != '"' && i + 1 < out_len) {
            if (*after == '\\' && after[1]) {
                after++;
            }
            out[i++] = *after++;
        }
        out[i] = '\0';
        return true;
    }
    out[0] = '\0';
    return false;
}

static bool parse_eui64(const char *s, uint8_t out[8])
{
    if (!s || !out) {
        return false;
    }
    uint8_t tmp[8];
    int n = 0;
    const char *p = s;
    while (*p && n < 8) {
        while (*p == ':' || *p == '-' || *p == ' ') {
            p++;
        }
        if (!p[0] || !p[1]) {
            return false;
        }
        int hi = hex_val(p[0]), lo = hex_val(p[1]);
        if (hi < 0 || lo < 0) {
            return false;
        }
        tmp[n++] = (uint8_t)((hi << 4) | lo);
        p += 2;
    }
    if (n != 8) {
        return false;
    }
    /* Display format is big-endian; Ember stores EUI64 little-endian. */
    for (int i = 0; i < 8; i++) {
        out[i] = tmp[7 - i];
    }
    return true;
}

/** Parse JSON int array under key (e.g. "button_modes":[0,1,0]). Returns count. */
static int json_get_int_array(const char *body, const char *key, int *out, int max)
{
    if (!body || !key || !out || max <= 0) {
        return 0;
    }
    char pat[40];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(body, pat);
    if (!p) {
        return 0;
    }
    p = strchr(p + strlen(pat), ':');
    if (!p) {
        return 0;
    }
    p = strchr(p, '[');
    if (!p) {
        return 0;
    }
    p++;
    int n = 0;
    while (*p && n < max) {
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == ',') {
            p++;
        }
        if (*p == ']') {
            break;
        }
        char *end = NULL;
        long v = strtol(p, &end, 10);
        if (end == p) {
            break;
        }
        out[n++] = (int)v;
        p = end;
    }
    return n;
}

/** Parse JSON string array (e.g. "button_names":["Power","Left"]). Returns count. */
static int json_get_string_array(const char *body, const char *key, char out[][24], int max)
{
    if (!body || !key || !out || max <= 0) {
        return 0;
    }
    char pat[40];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(body, pat);
    if (!p) {
        return 0;
    }
    p = strchr(p + strlen(pat), ':');
    if (!p) {
        return 0;
    }
    p = strchr(p, '[');
    if (!p) {
        return 0;
    }
    p++;
    int n = 0;
    while (*p && n < max) {
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == ',') {
            p++;
        }
        if (*p == ']') {
            break;
        }
        if (*p != '"') {
            break;
        }
        p++;
        size_t i = 0;
        while (*p && *p != '"' && i + 1 < 24) {
            if (*p == '\\' && p[1]) {
                p++;
            }
            out[n][i++] = *p++;
        }
        out[n][i] = '\0';
        if (*p == '"') {
            p++;
        }
        n++;
    }
    return n;
}

/** Parse JSON string array of EUI64s under key (e.g. "members":["aa:..","bb:.."]). */
static int json_get_eui_array(const char *body, const char *key, uint8_t out[][8], int max)
{
    if (!body || !key || !out || max <= 0) {
        return 0;
    }
    char pat[40];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(body, pat);
    if (!p) {
        return 0;
    }
    p = strchr(p + strlen(pat), ':');
    if (!p) {
        return 0;
    }
    p = strchr(p, '[');
    if (!p) {
        return 0;
    }
    p++;
    int n = 0;
    while (*p && n < max) {
        while (*p && (*p == ' ' || *p == ',' || *p == '\n' || *p == '\r' || *p == '\t')) {
            p++;
        }
        if (*p == ']') {
            break;
        }
        if (*p != '"') {
            break;
        }
        p++;
        char eui_s[40];
        size_t i = 0;
        while (*p && *p != '"' && i + 1 < sizeof(eui_s)) {
            eui_s[i++] = *p++;
        }
        eui_s[i] = '\0';
        if (*p == '"') {
            p++;
        }
        if (parse_eui64(eui_s, out[n])) {
            n++;
        }
    }
    return n;
}

static esp_err_t read_body(httpd_req_t *req, char *buf, size_t buflen)
{
    int total = req->content_len;
    if (total < 0 || total >= (int)buflen) {
        total = (int)buflen - 1;
    }
    int got = 0;
    while (got < total) {
        int r = httpd_req_recv(req, buf + got, total - got);
        if (r <= 0) {
            break;
        }
        got += r;
    }
    buf[got] = '\0';
    return ESP_OK;
}

static esp_err_t api_form(httpd_req_t *req)
{
    char body[128] = {0};
    read_body(req, body, sizeof(body));
    zigbee_form_options_t opt = {
        .channel = (uint8_t)json_get_int(body, "channel", 15),
        .tx_power = (int8_t)json_get_int(body, "tx_power", 8),
        .pan_id = 0,
    };
    esp_err_t err = zigbee_host_form_network(&opt);
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        zigbee_host_get_status(&s_zb_snap);
        char esc[128], msg[192];
        json_escape(s_zb_snap.last_error, esc, sizeof(esc));
        snprintf(msg, sizeof(msg), "{\"ok\":false,\"error\":\"%s\"}", esc[0] ? esc : "form failed");
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, msg);
    }
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t api_leave(httpd_req_t *req)
{
    char tmp[32];
    while (httpd_req_recv(req, tmp, sizeof(tmp)) > 0) {
    }
    zigbee_host_leave_network();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t api_permit(httpd_req_t *req)
{
    char body[64] = {0};
    read_body(req, body, sizeof(body));
    int dur = json_get_int(body, "duration", 120);
    if (dur < 0) {
        dur = 0;
    }
    if (dur > 254) {
        dur = 254;
    }
    esp_err_t err = zigbee_host_permit_join((uint8_t)dur);
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        zigbee_host_get_status(&s_zb_snap);
        char esc[128], msg[192];
        json_escape(s_zb_snap.last_error, esc, sizeof(esc));
        snprintf(msg, sizeof(msg), "{\"ok\":false,\"error\":\"%s\"}",
                 esc[0] ? esc : "permit join failed");
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, msg);
    }
    char msg[64];
    snprintf(msg, sizeof(msg), "{\"ok\":true,\"duration\":%d}", dur);
    return httpd_resp_sendstr(req, msg);
}

static esp_err_t api_devices_refresh(httpd_req_t *req)
{
    char tmp[32];
    while (httpd_req_recv(req, tmp, sizeof(tmp)) > 0) {
    }
    zigbee_host_refresh_devices();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t api_device_register(httpd_req_t *req)
{
    char body[256] = {0};
    read_body(req, body, sizeof(body));
    char eui_s[40] = {0};
    uint8_t eui[8];
    json_get_string(body, "eui64", eui_s, sizeof(eui_s));
    if (!parse_eui64(eui_s, eui)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid eui64\"}");
    }
    int node_id = json_get_int(body, "node_id", 0);
    int node_type = json_get_int(body, "node_type", 4); /* 4 = sleepy end device */
    esp_err_t err = zigbee_host_register_device(eui, (uint16_t)node_id, (uint8_t)node_type);
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"register failed\"}");
    }
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t api_device_update(httpd_req_t *req)
{
    char body[1536] = {0};
    read_body(req, body, sizeof(body));
    char eui_s[40] = {0}, name[32] = {0};
    uint8_t eui[8];
    json_get_string(body, "eui64", eui_s, sizeof(eui_s));
    if (!parse_eui64(eui_s, eui)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid eui64\"}");
    }
    zb_device_update_t upd = {0};
    /* Always apply name when the field is present (including empty → clear). */
    if (strstr(body, "\"name\"") && json_get_string(body, "name", name, sizeof(name))) {
        upd.set_name = true;
        snprintf(upd.name, sizeof(upd.name), "%s", name);
    }
    if (strstr(body, "\"homekit_expose\"")) {
        upd.set_homekit = true;
        upd.homekit_expose = json_get_bool(body, "homekit_expose", true);
    }
    if (strstr(body, "\"button_modes\"")) {
        int modes[ZB_REMOTE_MAX_BUTTONS] = {0};
        int n = json_get_int_array(body, "button_modes", modes, ZB_REMOTE_MAX_BUTTONS);
        if (n > 0) {
            upd.set_btn_modes = true;
            upd.btn_mode_count = (uint8_t)n;
            for (int i = 0; i < n; i++) {
                upd.btn_modes[i] =
                    (modes[i] == ZB_BTN_MODE_STATEFUL) ? ZB_BTN_MODE_STATEFUL : ZB_BTN_MODE_STATELESS;
            }
        }
    } else if (strstr(body, "\"sensor_btn_mode\"")) {
        /* Climate sensor wake button — 0=None, 1=Stateless, 2=Stateful. */
        upd.set_btn_modes = true;
        upd.btn_mode_count = 1;
        int m = json_get_int(body, "sensor_btn_mode", ZB_SENSOR_BTN_NONE);
        if (m == ZB_SENSOR_BTN_STATEFUL) {
            upd.btn_modes[0] = ZB_SENSOR_BTN_STATEFUL;
        } else if (m == ZB_SENSOR_BTN_STATELESS) {
            upd.btn_modes[0] = ZB_SENSOR_BTN_STATELESS;
        } else {
            upd.btn_modes[0] = ZB_SENSOR_BTN_NONE;
        }
    }
    if (strstr(body, "\"button_names\"")) {
        char names[ZB_REMOTE_MAX_BUTTONS][24];
        memset(names, 0, sizeof(names));
        int n = json_get_string_array(body, "button_names", names, ZB_REMOTE_MAX_BUTTONS);
        if (n > 0) {
            upd.set_btn_names = true;
            upd.btn_name_count = (uint8_t)n;
            for (int i = 0; i < n; i++) {
                strncpy(upd.btn_names[i], names[i], sizeof(upd.btn_names[i]) - 1);
                upd.btn_names[i][sizeof(upd.btn_names[i]) - 1] = '\0';
            }
        }
    }
    if (strstr(body, "\"remote_type\"")) {
        upd.set_remote_type = true;
        int rt = json_get_int(body, "remote_type", ZB_REMOTE_TYPE_HOMEKIT);
        upd.remote_type =
            (rt == ZB_REMOTE_TYPE_CONTROL) ? ZB_REMOTE_TYPE_CONTROL : ZB_REMOTE_TYPE_HOMEKIT;
    }
    if (strstr(body, "\"targets\"") || strstr(body, "\"target_groups\"") ||
        strstr(body, "\"target_eui64\"")) {
        upd.set_targets = true;
        uint8_t euis[ZB_REMOTE_MAX_TARGETS][8];
        int n = json_get_eui_array(body, "targets", euis, ZB_REMOTE_MAX_TARGETS);
        /* Legacy single field */
        if (n == 0 && strstr(body, "\"target_eui64\"")) {
            char teui_s[40] = {0};
            json_get_string(body, "target_eui64", teui_s, sizeof(teui_s));
            if (teui_s[0] && parse_eui64(teui_s, euis[0])) {
                n = 1;
            }
        }
        int gids[ZB_REMOTE_MAX_TARGETS] = {0};
        int ng = json_get_int_array(body, "target_groups", gids, ZB_REMOTE_MAX_TARGETS);
        upd.target_group_count = 0;
        for (int i = 0; i < ng && upd.target_group_count < ZB_REMOTE_MAX_TARGETS; i++) {
            if (gids[i] <= 0 || gids[i] > 255) {
                continue;
            }
            uint8_t gid = (uint8_t)gids[i];
            upd.target_groups[upd.target_group_count++] = gid;
            /* Expand general/light groups into member EUIs. */
            group_t g;
            if (group_get_by_id(gid, &g) && g.type == GROUP_TYPE_GENERAL) {
                for (uint8_t m = 0; m < g.member_count && n < ZB_REMOTE_MAX_TARGETS; m++) {
                    bool dup = false;
                    for (int k = 0; k < n; k++) {
                        if (memcmp(euis[k], g.members[m], 8) == 0) {
                            dup = true;
                            break;
                        }
                    }
                    if (!dup) {
                        memcpy(euis[n++], g.members[m], 8);
                    }
                }
            }
        }
        upd.target_count = (uint8_t)((n < 0) ? 0 : n);
        if (upd.target_count > ZB_REMOTE_MAX_TARGETS) {
            upd.target_count = ZB_REMOTE_MAX_TARGETS;
        }
        for (uint8_t i = 0; i < upd.target_count; i++) {
            memcpy(upd.targets[i], euis[i], 8);
        }
    }
    esp_err_t err = zigbee_host_update_device(eui, &upd);
    if (err == ESP_OK) {
        homekit_bridge_sync_devices();
    }
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"device not found\"}");
    }
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t api_device_interview(httpd_req_t *req)
{
    char body[128] = {0};
    read_body(req, body, sizeof(body));
    char eui_s[40] = {0};
    uint8_t eui[8];
    json_get_string(body, "eui64", eui_s, sizeof(eui_s));
    if (!parse_eui64(eui_s, eui)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid eui64\"}");
    }
    esp_err_t err = zigbee_host_interview_device(eui);
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"interview failed\"}");
    }
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t api_zigbee_sniff(httpd_req_t *req)
{
    /* Must be static: EZSP_SNIFF_LOG entries are too large for the httpd task stack
     * (stack overflow → random reboot whenever the Sniffer tab polls). */
    static char sniff_json[1536];
    static ezsp_sniff_entry_t s_entries[EZSP_SNIFF_LOG];
    static SemaphoreHandle_t s_sniff_api_mu;
    const uint32_t seq = ezsp_sniff_seq();

    /* ?since=<seq> → tiny ack when nothing new (UI used to pull large JSON often
     * and wedge C3 STA TX the moment the Sniffer tab opened). */
    char qbuf[48];
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        char since_s[16];
        if (httpd_query_key_value(qbuf, "since", since_s, sizeof(since_s)) == ESP_OK) {
            unsigned long since = strtoul(since_s, NULL, 10);
            if (since == (unsigned long)seq) {
                char tiny[64];
                snprintf(tiny, sizeof(tiny), "{\"ok\":true,\"seq\":%lu,\"unchanged\":true}",
                         (unsigned long)seq);
                httpd_resp_set_type(req, "application/json");
                httpd_resp_set_hdr(req, "Cache-Control", "no-store");
                web_note_handler_ok();
                return httpd_resp_sendstr(req, tiny);
            }
        }
    }

    if (!s_sniff_api_mu) {
        s_sniff_api_mu = xSemaphoreCreateMutex();
    }
    if (s_sniff_api_mu && xSemaphoreTake(s_sniff_api_mu, pdMS_TO_TICKS(30)) != pdTRUE) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":true,\"busy\":true,\"seq\":0,\"entries\":[]}");
    }

    uint8_t n = ezsp_copy_sniff_log(s_entries, EZSP_SNIFF_LOG);
    /* Newest 12 frames, short hex — keep SoftAP/STA TX light under remote presses. */
    uint8_t start = 0;
    if (n > 12) {
        start = (uint8_t)(n - 12);
        n = 12;
    }
    size_t pos = 0;
    pos += (size_t)snprintf(sniff_json + pos, sizeof(sniff_json) - pos,
                            "{\"ok\":true,\"seq\":%lu,\"count\":%u,\"entries\":[",
                            (unsigned long)seq, (unsigned)n);
    for (uint8_t i = 0; i < n && pos + 160 < sizeof(sniff_json); i++) {
        const ezsp_sniff_entry_t *e = &s_entries[start + i];
        char hex[16 * 3 + 1];
        size_t h = 0;
        uint8_t show = e->len > 16 ? 16 : e->len;
        for (uint8_t b = 0; b < show && h + 3 < sizeof(hex); b++) {
            h += (size_t)snprintf(hex + h, sizeof(hex) - h, "%s%02X", b ? " " : "", e->data[b]);
        }
        hex[h] = '\0';
        const char *dir = e->dir == EZSP_SNIFF_TX ? "TX" : e->dir == EZSP_SNIFF_EVT ? "EVT" : "RX";
        pos += (size_t)snprintf(
            sniff_json + pos, sizeof(sniff_json) - pos,
            "%s{\"t\":%lld,\"dir\":\"%s\",\"type\":%u,\"node\":%u,\"profile\":%u,\"cluster\":%u,"
            "\"group\":%u,\"src_ep\":%u,\"dst_ep\":%u,\"rssi\":%d,\"status\":%u,\"hex\":\"%s\"}",
            i ? "," : "", (long long)e->ms, dir, (unsigned)e->msg_type, (unsigned)e->node,
            (unsigned)e->profile, (unsigned)e->cluster, (unsigned)e->group, (unsigned)e->src_ep,
            (unsigned)e->dst_ep, (int)e->rssi, (unsigned)e->status, hex);
    }
    snprintf(sniff_json + pos, sizeof(sniff_json) - pos, "]}");
    if (s_sniff_api_mu) {
        xSemaphoreGive(s_sniff_api_mu);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    web_note_handler_ok();
    return httpd_resp_sendstr(req, sniff_json);
}

static esp_err_t api_zigbee_sniff_clear(httpd_req_t *req)
{
    ezsp_clear_sniff_log();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t api_device_remove(httpd_req_t *req)
{
    char body[128] = {0};
    read_body(req, body, sizeof(body));
    char eui_s[40] = {0};
    uint8_t eui[8];
    json_get_string(body, "eui64", eui_s, sizeof(eui_s));
    if (!parse_eui64(eui_s, eui)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid eui64\"}");
    }
    zigbee_host_remove_device(eui);
    homekit_bridge_sync_devices();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static uint8_t parse_thermo_kind_str(const char *s)
{
    if (s && strcmp(s, "humidity") == 0) {
        return GROUP_THERMO_KIND_HUMIDITY;
    }
    return GROUP_THERMO_KIND_TEMP;
}

static uint8_t parse_thermo_mode_str(const char *s, uint8_t thermo_kind)
{
    if (!s || !s[0]) {
        return THERMO_MODE_OFF;
    }
    if (thermo_kind == GROUP_THERMO_KIND_HUMIDITY) {
        if (strcmp(s, "humidify") == 0) {
            return THERMO_MODE_HUMIDIFY;
        }
        if (strcmp(s, "dehumidify") == 0) {
            return THERMO_MODE_DEHUMIDIFY;
        }
        return THERMO_MODE_OFF;
    }
    if (strcmp(s, "off") == 0) {
        return THERMO_MODE_OFF;
    }
    if (strcmp(s, "cool") == 0) {
        return THERMO_MODE_COOL;
    }
    if (strcmp(s, "auto") == 0) {
        return THERMO_MODE_AUTO;
    }
    return THERMO_MODE_HEAT;
}

static const char *thermo_mode_json(const group_t *g)
{
    if (!g) {
        return "off";
    }
    if (g->thermo_kind == GROUP_THERMO_KIND_HUMIDITY) {
        if (g->mode == THERMO_MODE_HUMIDIFY) {
            return "humidify";
        }
        if (g->mode == THERMO_MODE_DEHUMIDIFY) {
            return "dehumidify";
        }
        return "off";
    }
    if (g->mode == THERMO_MODE_COOL) {
        return "cool";
    }
    if (g->mode == THERMO_MODE_AUTO) {
        return "auto";
    }
    if (g->mode == THERMO_MODE_HEAT) {
        return "heat";
    }
    return "off";
}

/** Local hex EUI, or "remote:<id>" into ref with a zero EUI. Empty is ok when optional. */
static bool parse_dev_ref(const char *s, uint8_t eui[8], char *ref, size_t refn, bool optional)
{
    memset(eui, 0, 8);
    if (ref && refn) {
        ref[0] = '\0';
    }
    if (!s || !s[0] || strcmp(s, "none") == 0) {
        return optional;
    }
    if (strncmp(s, "remote:", 7) == 0 && s[7]) {
        if (!ref || refn < 8) {
            return false;
        }
        snprintf(ref, refn, "%s", s);
        return true;
    }
    return parse_eui64(s, eui);
}

/** Parse optional EUI: empty / "none" → zero EUI (ok). Invalid non-empty → false. */
static bool parse_optional_eui64(const char *s, uint8_t out[8])
{
    memset(out, 0, 8);
    if (!s || !s[0] || strcmp(s, "none") == 0) {
        return true;
    }
    return parse_eui64(s, out);
}

static esp_err_t api_group_create(httpd_req_t *req)
{
    char body[768] = {0};
    read_body(req, body, sizeof(body));
    char name[32] = {0}, type_s[24] = {0};
    json_get_string(body, "name", name, sizeof(name));
    json_get_string(body, "type", type_s, sizeof(type_s));
    bool hk = json_get_bool(body, "homekit_expose", true);
    if (!name[0]) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"name required\"}");
    }
    uint8_t id = 0;
    esp_err_t err = ESP_ERR_INVALID_ARG;
    if (strcmp(type_s, "thermostat") == 0) {
        char seui_s[40] = {0}, sweui_s[40] = {0}, ceui_s[40] = {0}, heui_s[40] = {0};
        char weui_s[40] = {0};
        char sref[40] = {0}, href[40] = {0}, cref[40] = {0}, wref[40] = {0};
        char mode_s[16] = {0}, reg_s[16] = {0};
        uint8_t seui[8], sweui[8], ceui[8], heui[8];
        json_get_string(body, "sensor_eui", seui_s, sizeof(seui_s));
        json_get_string(body, "switch_eui", sweui_s, sizeof(sweui_s));
        json_get_string(body, "cooler_eui", ceui_s, sizeof(ceui_s));
        json_get_string(body, "humidity_sensor_eui", heui_s, sizeof(heui_s));
        json_get_string(body, "window_eui", weui_s, sizeof(weui_s));
        json_get_string(body, "mode", mode_s, sizeof(mode_s));
        json_get_string(body, "regulation", reg_s, sizeof(reg_s));
        float target = json_get_float(body, "target_c", 21.0f);
        float target_rh = json_get_float(body, "target_humidity", 45.0f);
        float gap = json_get_float(body, "gap_c", THERMOSTAT_DEFAULT_GAP_C);
        float hyst = json_get_float(body, "hysteresis_c", THERMOSTAT_DEFAULT_HYSTERESIS_C);
        bool force_rh = json_get_bool(body, "humidity_force_heat", false);
        if (!parse_dev_ref(seui_s, seui, sref, sizeof(sref), false) ||
            !parse_dev_ref(sweui_s, sweui, href, sizeof(href), true) ||
            !parse_dev_ref(ceui_s, ceui, cref, sizeof(cref), true) ||
            !parse_dev_ref(weui_s, (uint8_t[8]){0}, wref, sizeof(wref), true) ||
            !parse_optional_eui64(heui_s, heui)) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid sensor/actuator eui\"}");
        }
        if (wref[0] == 0 && weui_s[0] && strncmp(weui_s, "remote:", 7) != 0) {
            snprintf(wref, sizeof(wref), "%s", weui_s);
        }
        uint8_t kind = parse_thermo_kind_str(reg_s[0] ? reg_s : "temperature");
        if (kind == GROUP_THERMO_KIND_HUMIDITY) {
            bool have_sw = href[0] || sweui[0] || sweui[1] || sweui[2] || sweui[3] || sweui[4] ||
                           sweui[5] || sweui[6] || sweui[7];
            if (!have_sw) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json");
                return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"switch_eui required\"}");
            }
        }
        uint8_t mode = parse_thermo_mode_str(mode_s, kind);
        err = group_create_thermostat(name, kind, seui, heui, sweui, ceui, target, target_rh, gap,
                                      hyst, force_rh, mode, hk, sref, href, cref, wref, &id);
    } else {
        /* Default: general grouped device */
        uint8_t members[GROUP_MAX_MEMBERS][8];
        int n = json_get_eui_array(body, "members", members, GROUP_MAX_MEMBERS);
        if (n <= 0) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"members required\"}");
        }
        char pf_s[16] = {0};
        uint8_t power_fail = GROUP_POWER_FAIL_PREVIOUS;
        if (json_get_string(body, "power_fail", pf_s, sizeof(pf_s))) {
            if (strcmp(pf_s, "on") == 0) {
                power_fail = GROUP_POWER_FAIL_ON;
            } else if (strcmp(pf_s, "off") == 0) {
                power_fail = GROUP_POWER_FAIL_OFF;
            }
        }
        err = group_create_general(name, members, (uint8_t)n, hk, power_fail, &id);
    }
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"create failed\"}");
    }
    homekit_bridge_sync_devices();
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"id\":%u}", (unsigned)id);
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t api_thermo_create(httpd_req_t *req)
{
    /* Legacy path: force thermostat type. */
    char body[384] = {0};
    read_body(req, body, sizeof(body));
    char name[32] = {0}, seui_s[40] = {0}, sweui_s[40] = {0}, mode_s[16] = {0};
    uint8_t seui[8], sweui[8];
    json_get_string(body, "name", name, sizeof(name));
    json_get_string(body, "sensor_eui", seui_s, sizeof(seui_s));
    json_get_string(body, "switch_eui", sweui_s, sizeof(sweui_s));
    json_get_string(body, "mode", mode_s, sizeof(mode_s));
    float target = json_get_float(body, "target_c", 21.0f);
    bool hk = json_get_bool(body, "homekit_expose", true);
    if (!name[0] || !parse_eui64(seui_s, seui) || !parse_eui64(sweui_s, sweui)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req,
                                  "{\"ok\":false,\"error\":\"name, sensor_eui, switch_eui required\"}");
    }
    uint8_t kind = GROUP_THERMO_KIND_TEMP;
    uint8_t mode = parse_thermo_mode_str(mode_s, kind);
    uint8_t id = 0;
    uint8_t zero[8] = {0};
    esp_err_t err = group_create_thermostat(name, kind, seui, zero, sweui, zero, target, 45.0f,
                                            THERMOSTAT_DEFAULT_GAP_C, THERMOSTAT_DEFAULT_HYSTERESIS_C,
                                            false, mode, hk, NULL, NULL, NULL, NULL, &id);
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"create failed\"}");
    }
    homekit_bridge_sync_devices();
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"id\":%u}", (unsigned)id);
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t api_group_update(httpd_req_t *req)
{
    char body[768] = {0};
    read_body(req, body, sizeof(body));
    int id = json_get_int(body, "id", 0);
    if (id <= 0 || id > 255) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid id\"}");
    }
    group_update_t upd = {0};
    char name[32] = {0}, seui_s[40] = {0}, sweui_s[40] = {0}, ceui_s[40] = {0}, heui_s[40] = {0};
    char weui_s[40] = {0};
    char mode_s[16] = {0}, reg_s[16] = {0};
    if (json_get_string(body, "name", name, sizeof(name)) && name[0]) {
        upd.set_name = true;
        snprintf(upd.name, sizeof(upd.name), "%s", name);
    }
    if (json_get_string(body, "sensor_eui", seui_s, sizeof(seui_s))) {
        if (!parse_dev_ref(seui_s, upd.sensor_eui, upd.sensor_ref, sizeof(upd.sensor_ref), false)) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid sensor\"}");
        }
        upd.set_sensor = upd.sensor_ref[0] == 0;
        upd.set_sensor_ref = true;
    }
    if (json_get_string(body, "humidity_sensor_eui", heui_s, sizeof(heui_s)) &&
        parse_optional_eui64(heui_s, upd.humidity_sensor_eui)) {
        upd.set_humidity_sensor = true;
    }
    if (json_get_string(body, "switch_eui", sweui_s, sizeof(sweui_s))) {
        if (!parse_dev_ref(sweui_s, upd.switch_eui, upd.switch_ref, sizeof(upd.switch_ref), true)) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid heater\"}");
        }
        upd.set_switch = true;
        upd.set_switch_ref = true;
    }
    if (json_get_string(body, "cooler_eui", ceui_s, sizeof(ceui_s))) {
        if (!parse_dev_ref(ceui_s, upd.cooler_eui, upd.cooler_ref, sizeof(upd.cooler_ref), true)) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid cooler\"}");
        }
        upd.set_cooler = true;
        upd.set_cooler_ref = true;
    }
    if (json_get_string(body, "window_eui", weui_s, sizeof(weui_s))) {
        uint8_t ignore[8];
        if (!parse_dev_ref(weui_s, ignore, upd.window_ref, sizeof(upd.window_ref), true)) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid window sensor\"}");
        }
        if (!upd.window_ref[0] && weui_s[0] && strcmp(weui_s, "none") != 0) {
            snprintf(upd.window_ref, sizeof(upd.window_ref), "%s", weui_s);
        }
        upd.set_window_ref = true;
    }
    if (json_get_string(body, "regulation", reg_s, sizeof(reg_s)) && reg_s[0]) {
        upd.set_thermo_kind = true;
        upd.thermo_kind = parse_thermo_kind_str(reg_s);
    }
    if (strstr(body, "\"target_c\"")) {
        upd.set_target = true;
        upd.target_c = json_get_float(body, "target_c", 21.0f);
    }
    if (strstr(body, "\"target_humidity\"")) {
        upd.set_target_humidity = true;
        upd.target_humidity_pct = json_get_float(body, "target_humidity", 45.0f);
    }
    if (strstr(body, "\"gap_c\"")) {
        upd.set_gap = true;
        upd.gap_c = json_get_float(body, "gap_c", THERMOSTAT_DEFAULT_GAP_C);
    }
    if (strstr(body, "\"hysteresis_c\"")) {
        upd.set_hysteresis = true;
        upd.hysteresis_c = json_get_float(body, "hysteresis_c", THERMOSTAT_DEFAULT_HYSTERESIS_C);
    }
    if (strstr(body, "\"humidity_force_heat\"")) {
        upd.set_humidity_force_heat = true;
        upd.humidity_force_heat = json_get_bool(body, "humidity_force_heat", false);
    }
    if (json_get_string(body, "mode", mode_s, sizeof(mode_s)) && mode_s[0]) {
        uint8_t kind = GROUP_THERMO_KIND_TEMP;
        group_t existing;
        if (group_get_by_id((uint8_t)id, &existing)) {
            kind = existing.thermo_kind;
        }
        if (upd.set_thermo_kind) {
            kind = upd.thermo_kind;
        }
        upd.set_mode = true;
        upd.mode = parse_thermo_mode_str(mode_s, kind);
    }
    if (strstr(body, "\"homekit_expose\"")) {
        upd.set_homekit = true;
        upd.homekit_expose = json_get_bool(body, "homekit_expose", true);
    }
    int n = json_get_eui_array(body, "members", upd.members, GROUP_MAX_MEMBERS);
    if (n > 0) {
        upd.set_members = true;
        upd.member_count = (uint8_t)n;
    }
    char pf_s[16] = {0};
    if (json_get_string(body, "power_fail", pf_s, sizeof(pf_s)) && pf_s[0]) {
        upd.set_power_fail = true;
        if (strcmp(pf_s, "on") == 0) {
            upd.power_fail_mode = GROUP_POWER_FAIL_ON;
        } else if (strcmp(pf_s, "off") == 0) {
            upd.power_fail_mode = GROUP_POWER_FAIL_OFF;
        } else {
            upd.power_fail_mode = GROUP_POWER_FAIL_PREVIOUS;
        }
    }
    esp_err_t err = group_update((uint8_t)id, &upd);
    if (err == ESP_OK) {
        homekit_bridge_sync_devices();
    }
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"not found\"}");
    }
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t api_thermo_update(httpd_req_t *req)
{
    return api_group_update(req);
}

static esp_err_t api_group_remove(httpd_req_t *req)
{
    char body[64] = {0};
    read_body(req, body, sizeof(body));
    int id = json_get_int(body, "id", 0);
    esp_err_t err = group_remove((uint8_t)id);
    if (err == ESP_OK) {
        homekit_bridge_sync_devices();
    }
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"not found\"}");
    }
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t api_thermo_remove(httpd_req_t *req)
{
    return api_group_remove(req);
}

static const char *zb_peer_kind(zb_device_kind_t kind)
{
    switch (kind) {
    case ZB_DEVICE_KIND_SENSOR:
        return "climate";
    case ZB_DEVICE_KIND_SWITCH:
        return "switch";
    case ZB_DEVICE_KIND_LIGHT:
        return "light";
    case ZB_DEVICE_KIND_OUTLET:
        return "plug";
    case ZB_DEVICE_KIND_CONTACT:
        return "contact";
    case ZB_DEVICE_KIND_MOTION:
        return "motion";
    case ZB_DEVICE_KIND_IRRIGATION:
        return "valve";
    default:
        return "device";
    }
}

static int zb_write_peer_devices(char *buf, size_t len)
{
    size_t pos = 0;
    if (!buf || len < 3) {
        return -1;
    }
    buf[pos++] = '[';
    bool first = true;
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        zb_device_t d;
        if (!zigbee_host_get_device_at(i, &d) || !d.used) {
            continue;
        }
        zb_device_kind_t kind = zigbee_host_device_kind(&d);
        bool contact = kind == ZB_DEVICE_KIND_CONTACT;
        bool onoff = zigbee_host_is_onoff_actuator(&d);
        if (!contact && !d.has_temp && !onoff) {
            continue;
        }
        char id[24], name[80];
        ezsp_format_eui64(d.eui64, id, sizeof(id));
        json_escape(d.name[0] ? d.name : id, name, sizeof(name));
        int n = snprintf(buf + pos, len - pos,
                         "%s{\"id\":\"%s\",\"name\":\"%s\",\"kind\":\"%s\","
                         "\"has_temp\":%s,\"temp_c\":%.2f,\"has_onoff\":%s,\"on\":%s,"
                         "\"has_contact\":%s,\"contact_open\":%s}",
                         first ? "" : ",", id, name, zb_peer_kind(kind),
                         d.has_temp ? "true" : "false", d.has_temp ? (double)d.temperature_c : 0.0,
                         onoff ? "true" : "false", d.onoff_on ? "true" : "false",
                         contact ? "true" : "false", (contact && d.binary_on) ? "true" : "false");
        if (n < 0 || (size_t)n >= len - pos) {
            return -1;
        }
        pos += (size_t)n;
        first = false;
    }
    if (pos + 2 > len) {
        return -1;
    }
    buf[pos++] = ']';
    buf[pos] = '\0';
    return (int)pos;
}

static esp_err_t zb_peer_set_on(const char *id, bool on)
{
    uint8_t eui[8];
    if (!id || !parse_eui64(id, eui)) {
        return ESP_ERR_INVALID_ARG;
    }
    return zigbee_host_set_onoff(eui, on);
}

static bool peer_token(httpd_req_t *req)
{
    char token[48];
    if (httpd_req_get_hdr_value_str(req, "X-Peer-Token", token, sizeof(token)) != ESP_OK) {
        return false;
    }
    return peer_link_token_ok(token);
}

static esp_err_t peer_json(httpd_req_t *req, const char *body)
{
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t peer_fail(httpd_req_t *req, const char *msg)
{
    httpd_resp_set_status(req, "400 Bad Request");
    char body[160];
    snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}", msg);
    return peer_json(req, body);
}

static esp_err_t peer_result(httpd_req_t *req, esp_err_t err, const char *fail)
{
    if (err != ESP_OK) {
        return peer_fail(req, fail);
    }
    return peer_json(req, "{\"ok\":true}");
}

static esp_err_t api_peer_get(httpd_req_t *req)
{
    char *json = malloc(3072);
    if (!json) {
        return peer_fail(req, "Link status is unavailable");
    }
    int n = peer_link_status_json(json, 3072);
    esp_err_t err = (n < 0) ? peer_fail(req, "Link status is unavailable") : peer_json(req, json);
    free(json);
    return err;
}

static esp_err_t api_peer_devices_get(httpd_req_t *req)
{
    if (!peer_token(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        return peer_json(req, "{\"ok\":false,\"error\":\"Not linked\"}");
    }
    char *json = malloc(3072);
    if (!json) {
        return peer_fail(req, "Device list is unavailable");
    }
    int n = peer_link_write_local_devices(json, 3072);
    esp_err_t err = (n < 0) ? peer_fail(req, "Device list is unavailable") : peer_json(req, json);
    free(json);
    return err;
}

static esp_err_t api_peer_discover(httpd_req_t *req)
{
    return peer_result(req, peer_link_discover(), "Could not look for a gateway");
}

static esp_err_t api_peer_pair(httpd_req_t *req)
{
    char body[160] = {0}, host[48] = {0};
    read_body(req, body, sizeof(body));
    if (!json_get_string(body, "host", host, sizeof(host)) || !host[0]) {
        return peer_fail(req, "Address is required");
    }
    esp_err_t err = peer_link_pair(host);
    if (err == ESP_ERR_INVALID_STATE) {
        return peer_fail(req, "Already linked, or this gateway has no IP yet");
    }
    return peer_result(req, err, "The other gateway did not answer");
}

static esp_err_t api_peer_accept(httpd_req_t *req)
{
    return peer_result(req, peer_link_accept(), "Could not accept the link");
}

static esp_err_t api_peer_decline(httpd_req_t *req)
{
    return peer_result(req, peer_link_decline(), "Could not decline");
}

static esp_err_t api_peer_unpair(httpd_req_t *req)
{
    return peer_result(req, peer_link_unpair(), "Could not unpair");
}

static esp_err_t api_peer_invite(httpd_req_t *req)
{
    char body[320] = {0};
    char id[20] = {0}, name[40] = {0}, role[12] = {0}, host[48] = {0}, token[40] = {0};
    read_body(req, body, sizeof(body));
    json_get_string(body, "id", id, sizeof(id));
    json_get_string(body, "name", name, sizeof(name));
    json_get_string(body, "role", role, sizeof(role));
    json_get_string(body, "host", host, sizeof(host));
    json_get_string(body, "token", token, sizeof(token));
    esp_err_t err = peer_link_on_invite(id, name, role, host, token);
    if (err != ESP_OK) {
        return peer_fail(req, "This gateway is already linked");
    }
    return peer_json(req, "{\"ok\":true,\"name\":\"ICC Zigbee Gateway\",\"role\":\"zigbee\"}");
}

static esp_err_t api_peer_confirm(httpd_req_t *req)
{
    char body[320] = {0};
    char id[20] = {0}, name[40] = {0}, role[12] = {0}, host[48] = {0}, token[48] = {0};
    read_body(req, body, sizeof(body));
    json_get_string(body, "id", id, sizeof(id));
    json_get_string(body, "name", name, sizeof(name));
    json_get_string(body, "role", role, sizeof(role));
    json_get_string(body, "host", host, sizeof(host));
    json_get_string(body, "token", token, sizeof(token));
    char hdr[48];
    if (httpd_req_get_hdr_value_str(req, "X-Peer-Token", hdr, sizeof(hdr)) == ESP_OK && hdr[0]) {
        snprintf(token, sizeof(token), "%s", hdr);
    }
    return peer_result(req, peer_link_on_confirm(id, name, role, host, token),
                       "This link request is no longer waiting");
}

static esp_err_t api_peer_bye(httpd_req_t *req)
{
    char token[48] = {0};
    if (httpd_req_get_hdr_value_str(req, "X-Peer-Token", token, sizeof(token)) != ESP_OK) {
        char body[120] = {0};
        read_body(req, body, sizeof(body));
        json_get_string(body, "token", token, sizeof(token));
    }
    return peer_result(req, peer_link_on_bye(token), "Not linked");
}

static esp_err_t api_peer_set(httpd_req_t *req)
{
    if (!peer_token(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        return peer_json(req, "{\"ok\":false,\"error\":\"Not linked\"}");
    }
    char body[120] = {0}, id[24] = {0};
    read_body(req, body, sizeof(body));
    if (!json_get_string(body, "id", id, sizeof(id)) || !id[0]) {
        return peer_fail(req, "Device id is required");
    }
    bool on = json_get_bool(body, "on", false);
    return peer_result(req, peer_link_set_local(id, on), "Could not reach that device");
}

static esp_err_t api_peer_command(httpd_req_t *req)
{
    char body[120] = {0}, id[24] = {0};
    read_body(req, body, sizeof(body));
    if (!json_get_string(body, "id", id, sizeof(id)) || !id[0]) {
        return peer_fail(req, "Device id is required");
    }
    bool on = json_get_bool(body, "on", false);
    return peer_result(req, peer_link_set_remote(id, on), "The other gateway did not accept the command");
}

esp_err_t web_server_start(void)
{
#if !CONFIG_WIFI_ENABLED
    return ESP_OK;
#else
    if (s_server) {
        return ESP_OK;
    }
    if (!s_scan_mutex) {
        s_scan_mutex = xSemaphoreCreateMutex();
    }
    if (!s_status_mutex) {
        s_status_mutex = xSemaphoreCreateMutex();
    }
    if (!s_root_mu) {
        s_root_mu = xSemaphoreCreateMutex();
    }
    static const peer_link_cfg_t peer_cfg = {
        .role = "zigbee",
        .name = "ICC Zigbee Gateway",
        .write_devices = zb_write_peer_devices,
        .set_on = zb_peer_set_on,
    };
    peer_link_start(&peer_cfg);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_WEB_HTTP_PORT;
    /* Above HAP httpd (idle+5) so a busy Home session cannot starve the portal. */
    config.task_priority = tskIDLE_PRIORITY + 6;
    config.lru_purge_enable = true;
    config.max_uri_handlers = 48;
    /* 16384 does not fit in internal RAM beside the HomeKit server. */
    config.stack_size = 12288;
    /* OTA upload streams ~1 MiB; keep headroom between flash-write chunks. */
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 8; /* Fail stuck portal sends fast; long waits starved sockets */
    config.max_open_sockets = 5; /* Headroom for status/ping while one HTML send runs */
    config.keep_alive_enable = false;
    config.backlog_conn = 2;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        s_server = NULL;
        return err;
    }

    const httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = root_get},
        {.uri = "/api/ping", .method = HTTP_GET, .handler = api_ping},
        {.uri = "/api/status", .method = HTTP_GET, .handler = api_status},
        {.uri = "/api/wifi/scan", .method = HTTP_GET, .handler = api_wifi_scan},
        {.uri = "/wifi/save", .method = HTTP_POST, .handler = wifi_save},
        {.uri = "/api/wifi/save", .method = HTTP_POST, .handler = wifi_save},
        {.uri = "/wifi/clear", .method = HTTP_POST, .handler = wifi_clear},
        {.uri = "/api/wifi/clear-secondary", .method = HTTP_POST, .handler = wifi_clear_secondary},
        {.uri = "/api/zigbee/form", .method = HTTP_POST, .handler = api_form},
        {.uri = "/api/zigbee/leave", .method = HTTP_POST, .handler = api_leave},
        {.uri = "/api/zigbee/permit", .method = HTTP_POST, .handler = api_permit},
        {.uri = "/api/zigbee/devices/refresh", .method = HTTP_POST, .handler = api_devices_refresh},
        {.uri = "/api/zigbee/devices/register", .method = HTTP_POST, .handler = api_device_register},
        {.uri = "/api/zigbee/devices/update", .method = HTTP_POST, .handler = api_device_update},
        {.uri = "/api/zigbee/devices/interview", .method = HTTP_POST, .handler = api_device_interview},
        {.uri = "/api/zigbee/devices/remove", .method = HTTP_POST, .handler = api_device_remove},
        {.uri = "/api/zigbee/sniff", .method = HTTP_GET, .handler = api_zigbee_sniff},
        {.uri = "/api/zigbee/sniff/clear", .method = HTTP_POST, .handler = api_zigbee_sniff_clear},
        {.uri = "/api/groups/create", .method = HTTP_POST, .handler = api_group_create},
        {.uri = "/api/groups/update", .method = HTTP_POST, .handler = api_group_update},
        {.uri = "/api/groups/remove", .method = HTTP_POST, .handler = api_group_remove},
        {.uri = "/api/thermostats/create", .method = HTTP_POST, .handler = api_thermo_create},
        {.uri = "/api/thermostats/update", .method = HTTP_POST, .handler = api_thermo_update},
        {.uri = "/api/thermostats/remove", .method = HTTP_POST, .handler = api_thermo_remove},
        {.uri = "/api/ota", .method = HTTP_GET, .handler = api_ota_status},
        {.uri = "/api/ota/channel", .method = HTTP_POST, .handler = api_ota_channel},
        {.uri = "/api/ota/offer", .method = HTTP_POST, .handler = api_ota_offer},
        {.uri = "/api/ota/upload", .method = HTTP_POST, .handler = api_ota_upload},
        {.uri = "/generate_204", .method = HTTP_GET, .handler = captive_ok},
        {.uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = captive_ok},
        {.uri = "/library/test/success.html", .method = HTTP_GET, .handler = captive_ok},
        {.uri = "/ncsi.txt", .method = HTTP_GET, .handler = captive_ok},
        {.uri = "/connecttest.txt", .method = HTTP_GET, .handler = captive_ok},
        {.uri = "/api/peer", .method = HTTP_GET, .handler = api_peer_get},
        {.uri = "/api/peer/devices", .method = HTTP_GET, .handler = api_peer_devices_get},
        {.uri = "/api/peer/discover", .method = HTTP_POST, .handler = api_peer_discover},
        {.uri = "/api/peer/pair", .method = HTTP_POST, .handler = api_peer_pair},
        {.uri = "/api/peer/accept", .method = HTTP_POST, .handler = api_peer_accept},
        {.uri = "/api/peer/decline", .method = HTTP_POST, .handler = api_peer_decline},
        {.uri = "/api/peer/unpair", .method = HTTP_POST, .handler = api_peer_unpair},
        {.uri = "/api/peer/invite", .method = HTTP_POST, .handler = api_peer_invite},
        {.uri = "/api/peer/confirm", .method = HTTP_POST, .handler = api_peer_confirm},
        {.uri = "/api/peer/bye", .method = HTTP_POST, .handler = api_peer_bye},
        {.uri = "/api/peer/set", .method = HTTP_POST, .handler = api_peer_set},
        {.uri = "/api/peer/command", .method = HTTP_POST, .handler = api_peer_command},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_register_uri_handler(s_server, &routes[i]);
    }

    /* Do not call web_note_handler_ok() here — that seeded fake STA traffic and
     * delayed silent-STA SoftAP recovery by ~25s after every boot/restart. */
    if (!s_watch_task) {
        xTaskCreate(web_watchdog_task, "web_wd", 3072, NULL, 3, &s_watch_task);
    }

    ESP_LOGI(TAG, "Production portal on port %d", CONFIG_WEB_HTTP_PORT);
    return ESP_OK;
#endif
}

#if CONFIG_WIFI_ENABLED
static esp_err_t web_server_restart(void)
{
    if (s_server) {
        ESP_LOGW(TAG, "Restarting portal httpd (socket recovery)");
        httpd_stop(s_server);
        s_server = NULL;
        s_root_held_us = 0;
        if (s_root_mu) {
            /* Ensure HTML mutex is free after stop. */
            xSemaphoreGive(s_root_mu);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return web_server_start();
}

static void web_watchdog_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(15000));
        if (!s_server) {
            (void)web_server_start();
            continue;
        }
        /* Only bounce if a full HTML send has been stuck past send timeouts. */
        if (s_root_held_us > 0) {
            int64_t held = esp_timer_get_time() - s_root_held_us;
            if (held > 20000000LL) { /* 20s */
                ESP_LOGW(TAG, "Portal HTML send stuck for %lld ms — restarting httpd",
                         (long long)(held / 1000LL));
                (void)web_server_restart();
            }
        }
    }
}
#endif
