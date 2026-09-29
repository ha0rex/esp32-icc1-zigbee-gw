/**
 * @file web_server.c
 * @brief Production portal + JSON control APIs.
 */

#include "web_server.h"
#include "web_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

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
#include "ezsp.h"

static const char *TAG = "web";
static httpd_handle_t s_server;
static SemaphoreHandle_t s_scan_mutex;
static SemaphoreHandle_t s_root_mu; /**< At most one full portal HTML send */
static char s_scan_json[2048];
static char s_scan_ssids[16][33];
static char s_json[24576];
/* Heap buffer for status snapshot — avoids multi-KB stack copies. */
static zigbee_host_status_t s_zb_snap;
static int64_t s_last_handler_us; /**< Last successful URI handler completion */
static int64_t s_root_held_us;    /**< When root HTML mutex was taken (0 = free) */
static TaskHandle_t s_watch_task;

static void web_note_handler_ok(void)
{
    s_last_handler_us = esp_timer_get_time();
    wifi_manager_note_traffic();
}

static void web_watchdog_task(void *arg);
static esp_err_t web_server_restart(void);

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
    /* Only one full HTML transfer at a time — concurrent tab/captive downloads
     * used to occupy every httpd socket until send timeouts, hanging the portal. */
    if (!s_root_mu || xSemaphoreTake(s_root_mu, 0) != pdTRUE) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Connection", "close");
        httpd_resp_set_hdr(req, "Retry-After", "1");
        return httpd_resp_sendstr(req, "busy");
    }
    s_root_held_us = esp_timer_get_time();
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    const char *html = WEB_APP_HTML;
    size_t len = strlen(html);
    const size_t chunk = 2048;
    esp_err_t err = ESP_OK;
    for (size_t off = 0; off < len; off += chunk) {
        size_t n = len - off;
        if (n > chunk) {
            n = chunk;
        }
        err = httpd_resp_send_chunk(req, html + off, n);
        if (err != ESP_OK) {
            httpd_resp_send_chunk(req, NULL, 0);
            break;
        }
        if ((off & 0xFFF) == 0) {
            vTaskDelay(1);
        }
    }
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, NULL, 0);
        if (err == ESP_OK) {
            web_note_handler_ok();
        }
    }
    s_root_held_us = 0;
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

static esp_err_t api_status(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Connection", "close");
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

    char err_esc[128], ssid_esc[64], ap_esc[64];
    json_escape(zb->last_error, err_esc, sizeof(err_esc));
    json_escape(wifi.ssid, ssid_esc, sizeof(ssid_esc));
    json_escape(wifi.ap_ssid, ap_esc, sizeof(ap_esc));

    homekit_bridge_status_t hk;
    homekit_bridge_get_status(&hk);
    char hk_code[24], hk_id[12], hk_st[64];
    json_escape(hk.setup_code, hk_code, sizeof(hk_code));
    json_escape(hk.setup_id, hk_id, sizeof(hk_id));
    json_escape(hk.status, hk_st, sizeof(hk_st));

    size_t pos = 0;
    pos += (size_t)snprintf(
        s_json + pos, sizeof(s_json) - pos,
        "{\"icc\":\"%s\",\"ash\":\"%s\",\"ezsp\":%u,\"ember\":\"%s\",\"eui64\":\"%s\","
        "\"network\":\"%s\",\"node_type\":\"%s\",\"net_valid\":%s,\"channel\":%u,\"pan_id\":%u,"
        "\"epid\":\"%s\",\"tx_power\":%d,\"permit_join\":%u,\"device_count\":%u,"
        "\"wifi\":\"%s\",\"has_sta\":%s,\"ssid\":\"%s\",\"ip\":\"%s\",\"ap_ssid\":\"%s\","
        "\"ap_ip\":\"%s\",\"ap_active\":%s,\"rssi\":%d,\"uptime\":\"%s\","
        "\"tx\":%lu,\"rx\":%lu,\"ash_ok\":%lu,\"ash_bad\":%lu,\"crc_err\":%lu,\"timeouts\":%lu,"
        "\"ezsp_cmd\":%lu,\"ezsp_rsp\":%lu,\"last_error\":\"%s\","
        "\"homekit\":{\"started\":%s,\"paired\":%s,\"setup_code\":\"%s\",\"setup_id\":\"%s\","
        "\"accessories\":%u,\"status\":\"%s\"},\"devices\":[",
        zigbee_host_icc_status_str(zb->icc_status), zigbee_host_ash_state_str(zb->ash_state),
        (unsigned)zb->ncp.protocol_version, ver, eui,
        zb->ncp.network_state_valid ? zigbee_host_network_state_str(zb->ncp.network_state) : "UNKNOWN",
        ezsp_node_type_str(zb->ncp.node_type), zb->ncp.net_params_valid ? "true" : "false",
        zb->ncp.net_params_valid ? zb->ncp.net.radio_channel : 0,
        zb->ncp.net_params_valid ? zb->ncp.net.pan_id : 0, epid,
        zb->ncp.net_params_valid ? zb->ncp.net.radio_tx_power : 0, zb->permit_join_remaining,
        zb->device_count, wifi_manager_state_str(wifi.state),
        wifi.has_sta_credentials ? "true" : "false", ssid_esc, wifi.ip, ap_esc, wifi.ap_ip,
        wifi.ap_active ? "true" : "false", wifi.rssi, uptime, (unsigned long)zb->ash_stats.tx_bytes,
        (unsigned long)zb->ash_stats.rx_bytes, (unsigned long)zb->ash_stats.valid_frames,
        (unsigned long)zb->ash_stats.invalid_frames, (unsigned long)zb->ash_stats.crc_errors,
        (unsigned long)zb->ash_stats.timeouts, (unsigned long)zb->ezsp_stats.commands_sent,
        (unsigned long)zb->ezsp_stats.responses_received, err_esc, hk.started ? "true" : "false",
        hk.paired ? "true" : "false", hk_code, hk_id, (unsigned)hk.accessory_count, hk_st);

    bool first = true;
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES && pos + 720 < sizeof(s_json); i++) {
        const zb_device_t *d = &zb->devices[i];
        if (!d->used) {
            continue;
        }
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
            "\"has_level\":%s,\"brightness\":%u,\"buttons\":%u,\"button_modes\":[",
            first ? "" : ",", deui, d->node_id, ezsp_node_type_str(d->node_type), dlab, dname, dman,
            dmodel, dfw,
            zigbee_host_device_kind(d) == ZB_DEVICE_KIND_LIGHT
                ? "light"
                : (zigbee_host_device_kind(d) == ZB_DEVICE_KIND_SWITCH
                       ? "switch"
                       : (zigbee_host_device_kind(d) == ZB_DEVICE_KIND_SENSOR
                              ? "sensor"
                              : (zigbee_host_device_kind(d) == ZB_DEVICE_KIND_REMOTE ? "remote"
                                                                                     : "unknown"))),
            d->has_temp ? "true" : "false", (double)d->temperature_c,
            d->has_humidity ? "true" : "false", (double)d->humidity_pct,
            d->has_battery ? "true" : "false", (unsigned)d->battery_pct,
            d->has_onoff ? "true" : "false", d->onoff_on ? "true" : "false",
            d->has_level ? "true" : "false",
            (unsigned)(d->has_level ? zigbee_host_level_to_brightness(d->level)
                                    : (d->onoff_on ? 100 : 0)),
            (unsigned)zigbee_host_remote_button_count(d));
        {
            uint8_t nb = zigbee_host_remote_button_count(d);
            if (nb > ZB_REMOTE_MAX_BUTTONS) {
                nb = ZB_REMOTE_MAX_BUTTONS;
            }
            for (uint8_t bi = 0; bi < nb; bi++) {
                pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "%s%u",
                                        bi ? "," : "",
                                        (unsigned)(d->btn_mode[bi] == ZB_BTN_MODE_STATEFUL ? 1 : 0));
            }
            pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "],\"button_on\":[");
            for (uint8_t bi = 0; bi < nb; bi++) {
                pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "%s%s", bi ? "," : "",
                                        d->btn_on[bi] ? "true" : "false");
            }
            pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "],\"button_names\":[");
            for (uint8_t bi = 0; bi < nb; bi++) {
                char bn[48];
                json_escape(zigbee_host_remote_button_name(d, bi), bn, sizeof(bn));
                pos += (size_t)snprintf(s_json + pos, sizeof(s_json) - pos, "%s\"%s\"", bi ? "," : "",
                                        bn);
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
            pos = json_append(s_json, sizeof(s_json), pos, "],\"remote_type\":%u,\"targets\":[",
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
    for (uint16_t i = 0; i < GROUP_MAX && pos + 520 < sizeof(s_json); i++) {
        group_t g;
        if (!group_get_at(i, &g)) {
            continue;
        }
        char gname[80];
        json_escape(g.name, gname, sizeof(gname));
        pos += (size_t)snprintf(
            s_json + pos, sizeof(s_json) - pos,
            "%s{\"id\":%u,\"name\":\"%s\",\"type\":\"%s\",\"homekit_expose\":%s,"
            "\"on\":%s,\"brightness\":%u,\"is_light\":%s,",
            first ? "" : ",", (unsigned)g.id, gname,
            g.type == GROUP_TYPE_THERMOSTAT ? "thermostat" : "general",
            g.homekit_expose ? "true" : "false", g.on ? "true" : "false",
            (unsigned)g.brightness_pct, g.is_light_group ? "true" : "false");
        if (g.type == GROUP_TYPE_THERMOSTAT) {
            char seui[40], sweui[40];
            ezsp_format_eui64(g.sensor_eui, seui, sizeof(seui));
            ezsp_format_eui64(g.switch_eui, sweui, sizeof(sweui));
            pos += (size_t)snprintf(
                s_json + pos, sizeof(s_json) - pos,
                "\"sensor_eui\":\"%s\",\"switch_eui\":\"%s\",\"target_c\":%.1f,\"mode\":\"%s\","
                "\"heating\":%s,\"has_current_temp\":%s,\"current_temp_c\":%.2f,\"members\":[]}",
                seui, sweui, (double)g.target_c, g.mode == THERMO_MODE_HEAT ? "heat" : "off",
                g.heating ? "true" : "false", g.has_current_temp ? "true" : "false",
                (double)g.current_temp_c);
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
    snprintf(s_json + pos, sizeof(s_json) - pos, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_sendstr(req, s_json);
    if (err == ESP_OK) {
        web_note_handler_ok();
    }
    return err;
}

static esp_err_t api_wifi_scan(httpd_req_t *req)
{
    if (!s_scan_mutex || xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ssids\":[],\"busy\":true}");
    }
    memset(s_scan_ssids, 0, sizeof(s_scan_ssids));
    int n = wifi_manager_scan(s_scan_ssids, 16);
    size_t pos = 0;
    pos += (size_t)snprintf(s_scan_json + pos, sizeof(s_scan_json) - pos, "{\"ssids\":[");
    for (int i = 0; i < n && pos + 48 < sizeof(s_scan_json); i++) {
        char esc[80];
        json_escape(s_scan_ssids[i], esc, sizeof(esc));
        pos += (size_t)snprintf(s_scan_json + pos, sizeof(s_scan_json) - pos, "%s\"%s\"", i ? "," : "",
                                esc);
    }
    snprintf(s_scan_json + pos, sizeof(s_scan_json) - pos, "],\"count\":%d}", n);
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, s_scan_json);
    xSemaphoreGive(s_scan_mutex);
    return err;
}

static esp_err_t wifi_save(httpd_req_t *req)
{
    char body[192];
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
    char ssid[33] = {0}, pass[65] = {0};
    form_get(body, "ssid", ssid, sizeof(ssid));
    form_get(body, "password", pass, sizeof(pass));
    if (!ssid[0]) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid required");
        return ESP_FAIL;
    }
    esp_err_t err = wifi_manager_apply_sta(ssid, pass);
    httpd_resp_set_type(req, "application/json");
    char msg[128];
    snprintf(msg, sizeof(msg), "{\"ok\":%s}", err == ESP_OK ? "true" : "false");
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
    }
    if (strstr(body, "\"button_names\"")) {
        char names[ZB_REMOTE_MAX_BUTTONS][24];
        memset(names, 0, sizeof(names));
        int n = json_get_string_array(body, "button_names", names, ZB_REMOTE_MAX_BUTTONS);
        if (n > 0) {
            upd.set_btn_names = true;
            upd.btn_name_count = (uint8_t)n;
            for (int i = 0; i < n; i++) {
                snprintf(upd.btn_names[i], sizeof(upd.btn_names[i]), "%s", names[i]);
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
    static char sniff_json[12288];
    static ezsp_sniff_entry_t s_entries[EZSP_SNIFF_LOG];
    static SemaphoreHandle_t s_sniff_api_mu;
    if (!s_sniff_api_mu) {
        s_sniff_api_mu = xSemaphoreCreateMutex();
    }
    if (s_sniff_api_mu) {
        xSemaphoreTake(s_sniff_api_mu, portMAX_DELAY);
    }

    uint8_t n = ezsp_copy_sniff_log(s_entries, EZSP_SNIFF_LOG);
    size_t pos = 0;
    pos += (size_t)snprintf(sniff_json + pos, sizeof(sniff_json) - pos,
                            "{\"ok\":true,\"seq\":%lu,\"count\":%u,\"entries\":[",
                            (unsigned long)ezsp_sniff_seq(), (unsigned)n);
    for (uint8_t i = 0; i < n && pos + 220 < sizeof(sniff_json); i++) {
        const ezsp_sniff_entry_t *e = &s_entries[i];
        char hex[EZSP_SNIFF_DATA * 3 + 1];
        size_t h = 0;
        for (uint8_t b = 0; b < e->len && h + 3 < sizeof(hex); b++) {
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
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_sendstr(req, sniff_json);
    if (s_sniff_api_mu) {
        xSemaphoreGive(s_sniff_api_mu);
    }
    return err;
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
        char seui_s[40] = {0}, sweui_s[40] = {0}, mode_s[16] = {0};
        uint8_t seui[8], sweui[8];
        json_get_string(body, "sensor_eui", seui_s, sizeof(seui_s));
        json_get_string(body, "switch_eui", sweui_s, sizeof(sweui_s));
        json_get_string(body, "mode", mode_s, sizeof(mode_s));
        float target = json_get_float(body, "target_c", 21.0f);
        if (!parse_eui64(seui_s, seui) || !parse_eui64(sweui_s, sweui)) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req,
                                      "{\"ok\":false,\"error\":\"sensor_eui and switch_eui required\"}");
        }
        uint8_t mode = (strcmp(mode_s, "off") == 0) ? THERMO_MODE_OFF : THERMO_MODE_HEAT;
        err = group_create_thermostat(name, seui, sweui, target, mode, hk, &id);
    } else {
        /* Default: general grouped device */
        uint8_t members[GROUP_MAX_MEMBERS][8];
        int n = json_get_eui_array(body, "members", members, GROUP_MAX_MEMBERS);
        if (n <= 0) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"members required\"}");
        }
        err = group_create_general(name, members, (uint8_t)n, hk, &id);
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
    uint8_t mode = (strcmp(mode_s, "off") == 0) ? THERMO_MODE_OFF : THERMO_MODE_HEAT;
    uint8_t id = 0;
    esp_err_t err = group_create_thermostat(name, seui, sweui, target, mode, hk, &id);
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
    char name[32] = {0}, seui_s[40] = {0}, sweui_s[40] = {0}, mode_s[16] = {0};
    if (json_get_string(body, "name", name, sizeof(name)) && name[0]) {
        upd.set_name = true;
        snprintf(upd.name, sizeof(upd.name), "%s", name);
    }
    if (json_get_string(body, "sensor_eui", seui_s, sizeof(seui_s)) &&
        parse_eui64(seui_s, upd.sensor_eui)) {
        upd.set_sensor = true;
    }
    if (json_get_string(body, "switch_eui", sweui_s, sizeof(sweui_s)) &&
        parse_eui64(sweui_s, upd.switch_eui)) {
        upd.set_switch = true;
    }
    if (strstr(body, "\"target_c\"")) {
        upd.set_target = true;
        upd.target_c = json_get_float(body, "target_c", 21.0f);
    }
    if (json_get_string(body, "mode", mode_s, sizeof(mode_s)) && mode_s[0]) {
        upd.set_mode = true;
        upd.mode = (strcmp(mode_s, "off") == 0) ? THERMO_MODE_OFF : THERMO_MODE_HEAT;
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
    if (!s_root_mu) {
        s_root_mu = xSemaphoreCreateMutex();
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_WEB_HTTP_PORT;
    /* Above HAP httpd (idle+5) so a busy Home session cannot starve the portal. */
    config.task_priority = tskIDLE_PRIORITY + 6;
    config.lru_purge_enable = true;
    config.max_uri_handlers = 32;
    config.stack_size = 12288;
    config.recv_wait_timeout = 1;
    config.send_wait_timeout = 1;
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
        {.uri = "/wifi/clear", .method = HTTP_POST, .handler = wifi_clear},
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
        {.uri = "/generate_204", .method = HTTP_GET, .handler = captive_ok},
        {.uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = captive_ok},
        {.uri = "/library/test/success.html", .method = HTTP_GET, .handler = captive_ok},
        {.uri = "/ncsi.txt", .method = HTTP_GET, .handler = captive_ok},
        {.uri = "/connecttest.txt", .method = HTTP_GET, .handler = captive_ok},
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
