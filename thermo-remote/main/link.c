#include "link.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <arpa/inet.h>
#include <unistd.h>

static const char *TAG = "link";
static const char *NS = "thermo";

static EventGroupHandle_t s_wifi;
static SemaphoreHandle_t s_saved;
#define BIT_GOT_IP BIT0
#define BIT_FAIL BIT1

static char s_ssid[33];
static char s_pass[65];
static char s_gw[48];
static int s_port = 80;
static bool s_sta;
static int s_retries;
static char s_http[8192];

static void url_decode(char *s)
{
    char *w = s;
    for (char *r = s; *r; r++) {
        if (*r == '+') {
            *w++ = ' ';
        } else if (*r == '%' && isxdigit((unsigned char)r[1]) && isxdigit((unsigned char)r[2])) {
            char hex[3] = {r[1], r[2], 0};
            *w++ = (char)strtol(hex, NULL, 16);
            r += 2;
        } else {
            *w++ = *r;
        }
    }
    *w = '\0';
}

static void load_nvs(void)
{
    nvs_handle_t nvs;
    s_ssid[0] = s_pass[0] = s_gw[0] = '\0';
    s_port = 80;
    if (nvs_open(NS, NVS_READONLY, &nvs) != ESP_OK) {
        return;
    }
    size_t len = sizeof(s_ssid);
    nvs_get_str(nvs, "ssid", s_ssid, &len);
    len = sizeof(s_pass);
    nvs_get_str(nvs, "pass", s_pass, &len);
    len = sizeof(s_gw);
    nvs_get_str(nvs, "gw", s_gw, &len);
    int32_t port = 80;
    if (nvs_get_i32(nvs, "port", &port) == ESP_OK && port > 0 && port < 65536) {
        s_port = (int)port;
    }
    nvs_close(nvs);
}

static void save_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NS, NVS_READWRITE, &nvs) != ESP_OK) {
        return;
    }
    nvs_set_str(nvs, "ssid", s_ssid);
    nvs_set_str(nvs, "pass", s_pass);
    nvs_set_str(nvs, "gw", s_gw);
    nvs_set_i32(nvs, "port", s_port);
    nvs_commit(nvs);
    nvs_close(nvs);
}

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retries < 10) {
            s_retries++;
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_wifi, BIT_FAIL);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_sta = true;
        xEventGroupSetBits(s_wifi, BIT_GOT_IP);
    }
}

static esp_err_t portal_get(httpd_req_t *req)
{
    const char *page =
        "<!doctype html><meta name=viewport content=\"width=device-width,initial-scale=1\">"
        "<title>Thermostat remote</title>"
        "<h2>Thermostat remote</h2>"
        "<p>Join the same Wi-Fi as the Zigbee gateway.</p>"
        "<form method=POST action=/save>"
        "Wi-Fi name<br><input name=ssid required maxlength=32><br><br>"
        "Wi-Fi password<br><input name=pass type=password maxlength=64><br><br>"
        "Gateway IP (blank to search)<br><input name=gw maxlength=40 placeholder=192.168.1.20><br><br>"
        "<button type=submit>Save</button></form>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req, page);
}

static esp_err_t portal_post(httpd_req_t *req)
{
    char body[256] = {0};
    int total = req->content_len;
    if (total < 0 || total >= (int)sizeof(body)) {
        total = (int)sizeof(body) - 1;
    }
    int got = 0;
    while (got < total) {
        int n = httpd_req_recv(req, body + got, total - got);
        if (n <= 0) {
            break;
        }
        got += n;
    }
    body[got] = '\0';
    s_ssid[0] = s_pass[0] = s_gw[0] = '\0';
    char *save = NULL;
    for (char *tok = strtok_r(body, "&", &save); tok; tok = strtok_r(NULL, "&", &save)) {
        char *eq = strchr(tok, '=');
        if (!eq) {
            continue;
        }
        *eq = '\0';
        char *val = eq + 1;
        url_decode(val);
        if (strcmp(tok, "ssid") == 0) {
            snprintf(s_ssid, sizeof(s_ssid), "%s", val);
        } else if (strcmp(tok, "pass") == 0) {
            snprintf(s_pass, sizeof(s_pass), "%s", val);
        } else if (strcmp(tok, "gw") == 0) {
            snprintf(s_gw, sizeof(s_gw), "%s", val);
        }
    }
    if (!s_ssid[0]) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Wi-Fi name is required");
    }
    s_port = 80;
    save_nvs();
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, "<p>Saved. The reader is joining Wi-Fi.</p>");
    if (s_saved) {
        xSemaphoreGive(s_saved);
    }
    return ESP_OK;
}

void link_init(void)
{
    s_wifi = xEventGroupCreate();
    s_saved = xSemaphoreCreateBinary();
    load_nvs();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL, NULL));
}

bool link_have_creds(void)
{
    return s_ssid[0] != '\0';
}

const char *link_ssid(void)
{
    return s_ssid;
}

bool link_sta_up(void)
{
    return s_sta;
}

bool link_have_gateway(void)
{
    return s_gw[0] != '\0';
}

void link_portal_until_saved(void)
{
    wifi_config_t ap = {0};
    snprintf((char *)ap.ap.ssid, sizeof(ap.ap.ssid), "ThermoRemote");
    snprintf((char *)ap.ap.password, sizeof(ap.ap.password), "thermostat");
    ap.ap.ssid_len = strlen((char *)ap.ap.ssid);
    ap.ap.channel = 1;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.max_connection = 2;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    httpd_handle_t server = NULL;
    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    hc.server_port = 80;
    hc.lru_purge_enable = true;
    if (httpd_start(&server, &hc) != ESP_OK) {
        ESP_LOGE(TAG, "setup page failed");
        return;
    }
    httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = portal_get};
    httpd_uri_t save = {.uri = "/save", .method = HTTP_POST, .handler = portal_post};
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &save);
    ESP_LOGI(TAG, "setup AP ThermoRemote password thermostat at 192.168.4.1");
    xSemaphoreTake(s_saved, portMAX_DELAY);
    httpd_stop(server);
    esp_wifi_stop();
}

bool link_connect(int timeout_ms)
{
    wifi_config_t sta = {0};
    memcpy(sta.sta.ssid, s_ssid, sizeof(sta.sta.ssid));
    memcpy(sta.sta.password, s_pass, sizeof(sta.sta.password));
    sta.sta.threshold.authmode = s_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    s_retries = 0;
    s_sta = false;
    xEventGroupClearBits(s_wifi, BIT_GOT_IP | BIT_FAIL);
    esp_wifi_disconnect();
    esp_wifi_stop();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    EventBits_t bits = xEventGroupWaitBits(s_wifi, BIT_GOT_IP | BIT_FAIL, pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(timeout_ms));
    if (bits & BIT_GOT_IP) {
        ESP_LOGI(TAG, "joined %s", s_ssid);
        return true;
    }
    ESP_LOGW(TAG, "could not join %s", s_ssid);
    s_sta = false;
    return false;
}

bool link_discover(void)
{
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        return false;
    }
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    struct timeval tv = {.tv_sec = 2, .tv_usec = 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(41241),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };
    const char *q = "ICC-THERMO?";
    sendto(fd, q, strlen(q), 0, (struct sockaddr *)&dest, sizeof(dest));
    char buf[64];
    struct sockaddr_in from;
    socklen_t flen = sizeof(from);
    int n = recvfrom(fd, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &flen);
    close(fd);
    if (n < 11) {
        return false;
    }
    buf[n] = '\0';
    if (strncmp(buf, "ICC-THERMO", 10) != 0) {
        return false;
    }
    inet_ntoa_r(from.sin_addr, s_gw, sizeof(s_gw));
    int port = 80;
    const char *p = strstr(buf, "port=");
    if (p) {
        port = atoi(p + 5);
    }
    if (port > 0 && port < 65536) {
        s_port = port;
    }
    save_nvs();
    ESP_LOGI(TAG, "gateway %s:%d", s_gw, s_port);
    return true;
}

static int http_call(const char *path, const char *post, int *status_out)
{
    char url[96];
    snprintf(url, sizeof(url), "http://%s:%d%s", s_gw, s_port, path);
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 4000,
        .method = post ? HTTP_METHOD_POST : HTTP_METHOD_GET,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        return -1;
    }
    if (post) {
        esp_http_client_set_header(client, "Content-Type", "application/json");
    }
    int err = esp_http_client_open(client, post ? (int)strlen(post) : 0);
    if (err != ESP_OK) {
        esp_http_client_cleanup(client);
        return -1;
    }
    if (post) {
        esp_http_client_write(client, post, (int)strlen(post));
    }
    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status_out) {
        *status_out = status;
    }
    int total = 0;
    while (total < (int)sizeof(s_http) - 1) {
        int n = esp_http_client_read(client, s_http + total, (int)sizeof(s_http) - 1 - total);
        if (n <= 0) {
            break;
        }
        total += n;
    }
    s_http[total] = '\0';
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return total;
}

static void copy_name(const char *obj, char *out, size_t out_len)
{
    out[0] = '\0';
    const char *p = strstr(obj, "\"name\":\"");
    if (!p) {
        return;
    }
    p += 8;
    size_t n = 0;
    while (*p && *p != '"' && n + 1 < out_len) {
        if (*p == '\\' && p[1]) {
            p++;
        }
        out[n++] = *p++;
    }
    out[n] = '\0';
}

static bool object_field(const char *obj, const char *key, char *out, size_t out_len)
{
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(obj, pat);
    if (!p) {
        return false;
    }
    p += strlen(pat);
    while (*p == ' ') {
        p++;
    }
    size_t n = 0;
    if (*p == '"') {
        p++;
        while (*p && *p != '"' && n + 1 < out_len) {
            out[n++] = *p++;
        }
    } else {
        while (*p && *p != ',' && *p != '}' && n + 1 < out_len) {
            out[n++] = *p++;
        }
    }
    out[n] = '\0';
    return n > 0;
}

static int parse_list(const char *json, bool compact, link_list_t *out)
{
    out->count = 0;
    const char *p = json;
    while (out->count < LINK_MAX && (p = strchr(p, '{')) != NULL) {
        const char *end = strchr(p, '}');
        if (!end) {
            break;
        }
        char obj[384];
        size_t len = (size_t)(end - p + 1);
        if (len >= sizeof(obj)) {
            len = sizeof(obj) - 1;
        }
        memcpy(obj, p, len);
        obj[len] = '\0';
        p = end + 1;
        if (!compact) {
            if (!strstr(obj, "\"type\":\"thermostat\"") || !strstr(obj, "\"regulation\":\"temperature\"")) {
                continue;
            }
        }
        char idb[8], cur[16], tgt[16], has[8];
        if (!object_field(obj, "id", idb, sizeof(idb))) {
            continue;
        }
        if (!object_field(obj, compact ? "target_c" : "target_c", tgt, sizeof(tgt))) {
            continue;
        }
        link_thermo_t *t = &out->items[out->count];
        memset(t, 0, sizeof(*t));
        t->id = (uint8_t)atoi(idb);
        t->target_c = strtof(tgt, NULL);
        copy_name(obj, t->name, sizeof(t->name));
        const char *has_key = compact ? "has_current" : "has_current_temp";
        const char *cur_key = compact ? "current_c" : "current_temp_c";
        if (object_field(obj, has_key, has, sizeof(has)) && strcmp(has, "true") == 0 &&
            object_field(obj, cur_key, cur, sizeof(cur))) {
            t->has_current = true;
            t->current_c = strtof(cur, NULL);
        }
        out->count++;
    }
    return out->count;
}

bool link_fetch(link_list_t *out)
{
    if (!out || !s_gw[0]) {
        return false;
    }
    int status = 0;
    int n = http_call("/api/thermo-remote", NULL, &status);
    if (n > 0 && status == 200 && strstr(s_http, "\"current_c\"")) {
        parse_list(s_http, true, out);
        return true;
    }
    n = http_call("/api/status", NULL, &status);
    if (n <= 0 || status != 200) {
        return false;
    }
    const char *groups = strstr(s_http, "\"groups\":[");
    parse_list(groups ? groups : s_http, false, out);
    return true;
}

bool link_post_climate(float temp_c, float humidity)
{
    uint8_t mac[6];
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) != ESP_OK) {
        return false;
    }
    char body[96];
    snprintf(body, sizeof(body),
             "{\"mac\":\"%02x%02x%02x%02x%02x%02x\",\"temp_c\":%.2f,\"humidity\":%.1f}", mac[0], mac[1],
             mac[2], mac[3], mac[4], mac[5], (double)temp_c, (double)humidity);
    int status = 0;
    http_call("/api/thermo-remote/climate", body, &status);
    return status == 200;
}

bool link_set_target(uint8_t id, float target_c)
{
    char body[64];
    snprintf(body, sizeof(body), "{\"id\":%u,\"target_c\":%.1f}", (unsigned)id, (double)target_c);
    int status = 0;
    http_call("/api/thermo-remote", body, &status);
    if (status == 200) {
        return true;
    }
    http_call("/api/thermostats/update", body, &status);
    return status == 200;
}

void link_forget(void)
{
    nvs_flash_erase();
    nvs_flash_init();
    s_ssid[0] = s_pass[0] = s_gw[0] = '\0';
    s_sta = false;
}
