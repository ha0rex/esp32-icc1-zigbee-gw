#include "peer_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs.h"

static const char *TAG = "peer";
#define NVS_NS "peerlink"
#define SEEN_MAX 4

typedef struct {
    char id[20];
    char name[40];
    char role[12];
    char host[48];
    int64_t seen_us;
    bool used;
} seen_t;

static peer_link_cfg_t s_cfg;
static SemaphoreHandle_t s_mu;
static char s_self_id[20];
static char s_token[40];
static char s_peer_id[20];
static char s_peer_name[40];
static char s_peer_role[12];
static char s_peer_host[48];
static bool s_paired;

static char s_out_token[40];
static char s_out_host[48];
static char s_out_name[40];
static char s_out_role[12];
static char s_out_id[20];
static bool s_waiting;

static char s_in_token[40];
static char s_in_host[48];
static char s_in_name[40];
static char s_in_role[12];
static char s_in_id[20];
static bool s_invite;

static seen_t s_seen[SEEN_MAX];
static int s_sock = -1;
static peer_dev_t s_devs[PEER_DEV_MAX];
static int s_dev_count;
static int64_t s_dev_us;

static void lock(void)
{
    if (s_mu) {
        xSemaphoreTake(s_mu, portMAX_DELAY);
    }
}

static void unlock(void)
{
    if (s_mu) {
        xSemaphoreGive(s_mu);
    }
}

static bool ipv4_ok(const char *host)
{
    int a, b, c, d;
    char tail;
    if (!host || sscanf(host, "%d.%d.%d.%d%c", &a, &b, &c, &d, &tail) != 4) {
        return false;
    }
    return a >= 0 && a <= 255 && b >= 0 && b <= 255 && c >= 0 && c <= 255 && d >= 0 && d <= 255;
}

static void json_str(const char *body, const char *key, char *out, size_t n)
{
    if (!out || n == 0) {
        return;
    }
    out[0] = 0;
    if (!body || !key) {
        return;
    }
    char pat[40];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(body, pat);
    if (!p) {
        return;
    }
    p += strlen(pat);
    while (*p == ' ' || *p == '\t' || *p == ':') {
        p++;
    }
    if (*p != '"') {
        return;
    }
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i + 1 < n) {
        out[i++] = *p++;
    }
    out[i] = 0;
}

static const char *after_colon(const char *keypos)
{
    const char *c = keypos ? strchr(keypos, ':') : NULL;
    if (!c) {
        return NULL;
    }
    c++;
    while (*c == ' ' || *c == '\t') {
        c++;
    }
    return c;
}

static bool json_bool_at(const char *keypos)
{
    const char *p = after_colon(keypos);
    return p && strncmp(p, "true", 4) == 0;
}

static void my_ip(char *out, size_t n)
{
    out[0] = 0;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t info;
    if (netif && esp_netif_get_ip_info(netif, &info) == ESP_OK && info.ip.addr) {
        esp_ip4addr_ntoa(&info.ip, out, n);
    }
}

static void save_locked(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_str(h, "id", s_self_id);
    nvs_set_str(h, "token", s_paired ? s_token : "");
    nvs_set_str(h, "pid", s_paired ? s_peer_id : "");
    nvs_set_str(h, "pname", s_paired ? s_peer_name : "");
    nvs_set_str(h, "prole", s_paired ? s_peer_role : "");
    nvs_set_str(h, "phost", s_paired ? s_peer_host : "");
    nvs_commit(h);
    nvs_close(h);
}

static void load_id(void)
{
    nvs_handle_t h;
    s_self_id[0] = 0;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    size_t n = sizeof(s_self_id);
    if (nvs_get_str(h, "id", s_self_id, &n) != ESP_OK || strlen(s_self_id) < 8) {
        snprintf(s_self_id, sizeof(s_self_id), "%08lx%08lx", (unsigned long)esp_random(),
                 (unsigned long)esp_random());
        nvs_set_str(h, "id", s_self_id);
        nvs_commit(h);
    }
    n = sizeof(s_token);
    if (nvs_get_str(h, "token", s_token, &n) == ESP_OK && s_token[0]) {
        n = sizeof(s_peer_id);
        nvs_get_str(h, "pid", s_peer_id, &n);
        n = sizeof(s_peer_name);
        nvs_get_str(h, "pname", s_peer_name, &n);
        n = sizeof(s_peer_role);
        nvs_get_str(h, "prole", s_peer_role, &n);
        n = sizeof(s_peer_host);
        nvs_get_str(h, "phost", s_peer_host, &n);
        s_paired = s_peer_host[0] && s_token[0];
    }
    nvs_close(h);
}

static void note_seen(const char *id, const char *name, const char *role, const char *host)
{
    if (!id || !id[0] || !host || !host[0] || strcmp(id, s_self_id) == 0) {
        return;
    }
    if (!role || !role[0]) {
        role = "gateway";
    }
    if (!name || !name[0]) {
        name = "Gateway";
    }
    lock();
    int slot = 0;
    int64_t oldest = INT64_MAX;
    for (int i = 0; i < SEEN_MAX; i++) {
        if (s_seen[i].used && strcmp(s_seen[i].id, id) == 0) {
            slot = i;
            goto fill;
        }
        if (!s_seen[i].used) {
            slot = i;
            goto fill;
        }
        if (s_seen[i].seen_us < oldest) {
            oldest = s_seen[i].seen_us;
            slot = i;
        }
    }
fill:
    s_seen[slot].used = true;
    snprintf(s_seen[slot].id, sizeof(s_seen[slot].id), "%s", id);
    snprintf(s_seen[slot].name, sizeof(s_seen[slot].name), "%s", name);
    snprintf(s_seen[slot].role, sizeof(s_seen[slot].role), "%s", role);
    snprintf(s_seen[slot].host, sizeof(s_seen[slot].host), "%s", host);
    s_seen[slot].seen_us = esp_timer_get_time();
    unlock();
}

static void send_hello(void)
{
    if (s_sock < 0) {
        return;
    }
    char ip[20];
    my_ip(ip, sizeof(ip));
    char msg[180];
    snprintf(msg, sizeof(msg),
             "{\"t\":\"icc1\",\"id\":\"%s\",\"name\":\"%s\",\"role\":\"%s\",\"port\":80}",
             s_self_id, s_cfg.name ? s_cfg.name : "Gateway", s_cfg.role ? s_cfg.role : "gateway");
    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(PEER_LINK_PORT);
    dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    sendto(s_sock, msg, strlen(msg), 0, (struct sockaddr *)&dest, sizeof(dest));
    (void)ip;
}

static void handle_packet(const char *msg, const struct sockaddr_in *from)
{
    if (!msg || !strstr(msg, "\"t\":\"icc1\"")) {
        return;
    }
    char id[20], name[40], role[12];
    json_str(msg, "id", id, sizeof(id));
    json_str(msg, "name", name, sizeof(name));
    json_str(msg, "role", role, sizeof(role));
    char host[48];
    inet_ntop(AF_INET, &from->sin_addr, host, sizeof(host));
    note_seen(id, name, role, host);
}

static int http_json(const char *host, const char *path, const char *token, const char *body,
                     char *resp, size_t resp_len)
{
    if (resp && resp_len) {
        resp[0] = 0;
    }
    char url[96];
    snprintf(url, sizeof(url), "http://%s%s", host, path);
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 2500,
        .method = body ? HTTP_METHOD_POST : HTTP_METHOD_GET,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        return -1;
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    if (token && token[0]) {
        esp_http_client_set_header(client, "X-Peer-Token", token);
    }
    int body_len = body ? (int)strlen(body) : 0;
    if (esp_http_client_open(client, body_len) != ESP_OK) {
        esp_http_client_cleanup(client);
        return -1;
    }
    if (body_len > 0) {
        esp_http_client_write(client, body, body_len);
    }
    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (resp && resp_len > 1) {
        int n = esp_http_client_read(client, resp, (int)resp_len - 1);
        if (n < 0) {
            n = 0;
        }
        resp[n] = 0;
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return status;
}

static void parse_devices(const char *json)
{
    peer_dev_t fresh[PEER_DEV_MAX];
    memset(fresh, 0, sizeof(fresh));
    int count = 0;
    const char *p = json;
    while (p && count < PEER_DEV_MAX && (p = strchr(p, '{')) != NULL) {
        const char *end = strchr(p, '}');
        if (!end) {
            break;
        }
        char obj[320];
        size_t n = (size_t)(end - p + 1);
        if (n >= sizeof(obj)) {
            n = sizeof(obj) - 1;
        }
        memcpy(obj, p, n);
        obj[n] = 0;
        peer_dev_t *d = &fresh[count];
        json_str(obj, "id", d->id, sizeof(d->id));
        json_str(obj, "name", d->name, sizeof(d->name));
        json_str(obj, "kind", d->kind, sizeof(d->kind));
        const char *ht = strstr(obj, "\"has_temp\"");
        const char *ho = strstr(obj, "\"has_onoff\"");
        const char *hc = strstr(obj, "\"has_contact\"");
        d->has_temp = json_bool_at(ht);
        d->has_onoff = json_bool_at(ho);
        d->has_contact = json_bool_at(hc);
        const char *on = strstr(obj, "\"on\"");
        const char *co = strstr(obj, "\"contact_open\"");
        d->on = json_bool_at(on);
        d->contact_open = json_bool_at(co);
        const char *tc = after_colon(strstr(obj, "\"temp_c\""));
        if (tc) {
            d->temp_c = strtof(tc, NULL);
        }
        d->ok = d->id[0] != 0;
        if (d->ok) {
            count++;
        }
        p = end + 1;
    }
    lock();
    memcpy(s_devs, fresh, sizeof(s_devs));
    s_dev_count = count;
    s_dev_us = esp_timer_get_time();
    unlock();
}

void peer_link_refresh(void)
{
    char host[48], token[40];
    lock();
    bool paired = s_paired;
    snprintf(host, sizeof(host), "%s", s_peer_host);
    snprintf(token, sizeof(token), "%s", s_token);
    unlock();
    if (!paired || !host[0]) {
        return;
    }
    char body[1400];
    int status = http_json(host, "/api/peer/devices", token, NULL, body, sizeof(body));
    if (status == 200) {
        parse_devices(body);
    }
}

esp_err_t peer_link_discover(void)
{
    for (int i = 0; i < 3; i++) {
        send_hello();
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    vTaskDelay(pdMS_TO_TICKS(350));
    return ESP_OK;
}

static void become_paired_locked(const char *id, const char *name, const char *role,
                                 const char *host, const char *token)
{
    snprintf(s_token, sizeof(s_token), "%s", token);
    snprintf(s_peer_id, sizeof(s_peer_id), "%s", id ? id : "");
    snprintf(s_peer_name, sizeof(s_peer_name), "%s", name && name[0] ? name : "Gateway");
    snprintf(s_peer_role, sizeof(s_peer_role), "%s", role && role[0] ? role : "gateway");
    snprintf(s_peer_host, sizeof(s_peer_host), "%s", host);
    s_paired = true;
    s_waiting = false;
    s_invite = false;
    s_out_token[0] = s_in_token[0] = 0;
    save_locked();
}

esp_err_t peer_link_pair(const char *host)
{
    if (!ipv4_ok(host)) {
        return ESP_ERR_INVALID_ARG;
    }
    char ip[20];
    my_ip(ip, sizeof(ip));
    if (!ip[0]) {
        return ESP_ERR_INVALID_STATE;
    }
    lock();
    if (s_paired) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    snprintf(s_out_token, sizeof(s_out_token), "%08lx%08lx", (unsigned long)esp_random(),
             (unsigned long)esp_random());
    snprintf(s_out_host, sizeof(s_out_host), "%s", host);
    s_out_name[0] = s_out_role[0] = s_out_id[0] = 0;
    s_waiting = true;
    char token[40];
    snprintf(token, sizeof(token), "%s", s_out_token);
    char name[40], role[12], id[20];
    snprintf(name, sizeof(name), "%s", s_cfg.name ? s_cfg.name : "Gateway");
    snprintf(role, sizeof(role), "%s", s_cfg.role ? s_cfg.role : "gateway");
    snprintf(id, sizeof(id), "%s", s_self_id);
    unlock();

    char body[220];
    snprintf(body, sizeof(body),
             "{\"id\":\"%s\",\"name\":\"%s\",\"role\":\"%s\",\"host\":\"%s\",\"token\":\"%s\"}", id,
             name, role, ip, token);
    char resp[180];
    int status = http_json(host, "/api/peer/invite", NULL, body, resp, sizeof(resp));
    if (status != 200) {
        lock();
        s_waiting = false;
        unlock();
        return ESP_FAIL;
    }
    char rname[40], rrole[12], rid[20];
    json_str(resp, "name", rname, sizeof(rname));
    json_str(resp, "role", rrole, sizeof(rrole));
    json_str(resp, "id", rid, sizeof(rid));
    lock();
    if (s_waiting) {
        snprintf(s_out_name, sizeof(s_out_name), "%s", rname);
        snprintf(s_out_role, sizeof(s_out_role), "%s", rrole);
        snprintf(s_out_id, sizeof(s_out_id), "%s", rid);
    }
    unlock();
    return ESP_OK;
}

esp_err_t peer_link_on_invite(const char *id, const char *name, const char *role, const char *host,
                              const char *token)
{
    if (!id || !token || !token[0] || !ipv4_ok(host)) {
        return ESP_ERR_INVALID_ARG;
    }
    lock();
    if (s_paired) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    snprintf(s_in_id, sizeof(s_in_id), "%s", id);
    snprintf(s_in_name, sizeof(s_in_name), "%s", name && name[0] ? name : "Gateway");
    snprintf(s_in_role, sizeof(s_in_role), "%s", role && role[0] ? role : "gateway");
    snprintf(s_in_host, sizeof(s_in_host), "%s", host);
    snprintf(s_in_token, sizeof(s_in_token), "%s", token);
    s_invite = true;
    unlock();
    ESP_LOGI(TAG, "Pairing request from %s", s_in_name);
    return ESP_OK;
}

esp_err_t peer_link_accept(void)
{
    char host[48], token[40], id[20], name[40], role[12], ip[20];
    my_ip(ip, sizeof(ip));
    lock();
    if (!s_invite || !s_in_host[0]) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    snprintf(host, sizeof(host), "%s", s_in_host);
    snprintf(token, sizeof(token), "%s", s_in_token);
    snprintf(id, sizeof(id), "%s", s_in_id);
    snprintf(name, sizeof(name), "%s", s_in_name);
    snprintf(role, sizeof(role), "%s", s_in_role);
    unlock();
    char body[220];
    snprintf(body, sizeof(body),
             "{\"id\":\"%s\",\"name\":\"%s\",\"role\":\"%s\",\"host\":\"%s\",\"token\":\"%s\"}",
             s_self_id, s_cfg.name ? s_cfg.name : "Gateway", s_cfg.role ? s_cfg.role : "gateway",
             ip, token);
    int status = http_json(host, "/api/peer/confirm", token, body, NULL, 0);
    if (status != 200) {
        return ESP_FAIL;
    }
    lock();
    become_paired_locked(id, name, role, host, token);
    unlock();
    ESP_LOGI(TAG, "Linked with %s", name);
    return ESP_OK;
}

esp_err_t peer_link_decline(void)
{
    lock();
    s_invite = false;
    s_in_token[0] = 0;
    unlock();
    return ESP_OK;
}

esp_err_t peer_link_on_confirm(const char *id, const char *name, const char *role, const char *host,
                               const char *token)
{
    lock();
    if (!s_waiting || !token || strcmp(token, s_out_token) != 0) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    const char *use_host = (host && ipv4_ok(host)) ? host : s_out_host;
    become_paired_locked(id && id[0] ? id : s_out_id, name && name[0] ? name : s_out_name,
                         role && role[0] ? role : s_out_role, use_host, token);
    unlock();
    ESP_LOGI(TAG, "Link accepted");
    return ESP_OK;
}

esp_err_t peer_link_on_bye(const char *token)
{
    lock();
    if (!s_paired || !token || strcmp(token, s_token) != 0) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_paired = false;
    s_token[0] = s_peer_host[0] = 0;
    s_dev_count = 0;
    save_locked();
    unlock();
    return ESP_OK;
}

esp_err_t peer_link_unpair(void)
{
    char host[48], token[40];
    lock();
    bool paired = s_paired;
    snprintf(host, sizeof(host), "%s", s_peer_host);
    snprintf(token, sizeof(token), "%s", s_token);
    s_paired = false;
    s_waiting = false;
    s_invite = false;
    s_token[0] = s_peer_host[0] = 0;
    s_dev_count = 0;
    save_locked();
    unlock();
    if (paired && host[0]) {
        char body[80];
        snprintf(body, sizeof(body), "{\"token\":\"%s\"}", token);
        http_json(host, "/api/peer/bye", token, body, NULL, 0);
    }
    return ESP_OK;
}

bool peer_link_token_ok(const char *token)
{
    lock();
    bool ok = s_paired && token && token[0] && strcmp(token, s_token) == 0;
    unlock();
    return ok;
}

int peer_link_write_local_devices(char *buf, size_t len)
{
    if (!buf || len < 3) {
        return -1;
    }
    if (!s_cfg.write_devices) {
        snprintf(buf, len, "[]");
        return 2;
    }
    return s_cfg.write_devices(buf, len);
}

esp_err_t peer_link_set_local(const char *id, bool on)
{
    if (!s_cfg.set_on) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return s_cfg.set_on(id, on);
}

esp_err_t peer_link_set_remote(const char *id, bool on)
{
    char host[48], token[40];
    lock();
    if (!s_paired) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    snprintf(host, sizeof(host), "%s", s_peer_host);
    snprintf(token, sizeof(token), "%s", s_token);
    unlock();
    char body[80];
    snprintf(body, sizeof(body), "{\"id\":\"%s\",\"on\":%s}", id ? id : "", on ? "true" : "false");
    int status = http_json(host, "/api/peer/set", token, body, NULL, 0);
    return status == 200 ? ESP_OK : ESP_FAIL;
}

bool peer_link_device(const char *id, peer_dev_t *out)
{
    if (!id || !out) {
        return false;
    }
    lock();
    for (int i = 0; i < s_dev_count; i++) {
        if (strcmp(s_devs[i].id, id) == 0) {
            *out = s_devs[i];
            unlock();
            return true;
        }
    }
    unlock();
    return false;
}

int peer_link_copy_devices(peer_dev_t *out, int max)
{
    if (!out || max <= 0) {
        return 0;
    }
    lock();
    int n = s_dev_count < max ? s_dev_count : max;
    memcpy(out, s_devs, (size_t)n * sizeof(peer_dev_t));
    unlock();
    return n;
}

int peer_link_status_json(char *buf, size_t len)
{
    if (!buf || len < 16) {
        return -1;
    }
    lock();
    int64_t now = esp_timer_get_time();
    int used = snprintf(buf, len,
                        "{\"self\":{\"id\":\"%s\",\"name\":\"%s\",\"role\":\"%s\"},"
                        "\"paired\":%s,",
                        s_self_id, s_cfg.name ? s_cfg.name : "", s_cfg.role ? s_cfg.role : "",
                        s_paired ? "true" : "false");
    if (used < 0 || (size_t)used >= len) {
        unlock();
        return -1;
    }
    if (s_paired) {
        used += snprintf(buf + used, len - (size_t)used,
                         "\"peer\":{\"id\":\"%s\",\"name\":\"%s\",\"role\":\"%s\",\"host\":\"%s\"},",
                         s_peer_id, s_peer_name, s_peer_role, s_peer_host);
    } else {
        used += snprintf(buf + used, len - (size_t)used, "\"peer\":null,");
    }
    if (s_invite) {
        used += snprintf(buf + used, len - (size_t)used,
                         "\"invite\":{\"name\":\"%s\",\"role\":\"%s\",\"host\":\"%s\"},", s_in_name,
                         s_in_role, s_in_host);
    } else {
        used += snprintf(buf + used, len - (size_t)used, "\"invite\":null,");
    }
    if (s_waiting) {
        used += snprintf(buf + used, len - (size_t)used,
                         "\"waiting\":{\"name\":\"%s\",\"role\":\"%s\",\"host\":\"%s\"},",
                         s_out_name[0] ? s_out_name : "the other gateway", s_out_role, s_out_host);
    } else {
        used += snprintf(buf + used, len - (size_t)used, "\"waiting\":null,");
    }
    used += snprintf(buf + used, len - (size_t)used, "\"found\":[");
    bool first = true;
    for (int i = 0; i < SEEN_MAX && used > 0 && (size_t)used < len; i++) {
        if (!s_seen[i].used || now - s_seen[i].seen_us > 20LL * 1000000) {
            continue;
        }
        used += snprintf(buf + used, len - (size_t)used,
                         "%s{\"id\":\"%s\",\"name\":\"%s\",\"role\":\"%s\",\"host\":\"%s\"}",
                         first ? "" : ",", s_seen[i].id, s_seen[i].name, s_seen[i].role,
                         s_seen[i].host);
        first = false;
    }
    used += snprintf(buf + used, len - (size_t)used, "],\"devices\":[");
    first = true;
    for (int i = 0; i < s_dev_count && used > 0 && (size_t)used + 160 < len; i++) {
        const peer_dev_t *d = &s_devs[i];
        used += snprintf(buf + used, len - (size_t)used,
                         "%s{\"id\":\"%s\",\"name\":\"%s\",\"kind\":\"%s\",\"has_temp\":%s,"
                         "\"temp_c\":%.2f,\"has_onoff\":%s,\"on\":%s,\"has_contact\":%s,"
                         "\"contact_open\":%s}",
                         first ? "" : ",", d->id, d->name, d->kind, d->has_temp ? "true" : "false",
                         (double)d->temp_c, d->has_onoff ? "true" : "false", d->on ? "true" : "false",
                         d->has_contact ? "true" : "false", d->contact_open ? "true" : "false");
        first = false;
    }
    used += snprintf(buf + used, len - (size_t)used, "]}");
    unlock();
    if (used < 0 || (size_t)used >= len) {
        return -1;
    }
    return used;
}

static void listen_task(void *arg)
{
    (void)arg;
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "UDP socket failed");
        vTaskDelete(NULL);
        return;
    }
    int yes = 1;
    setsockopt(s_sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    setsockopt(s_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PEER_LINK_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "UDP bind %d failed", PEER_LINK_PORT);
        close(s_sock);
        s_sock = -1;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Listening for gateways on UDP %d", PEER_LINK_PORT);
    int64_t last_refresh = 0;
    for (;;) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(s_sock, &fds);
        struct timeval tv = {.tv_sec = 0, .tv_usec = 400000};
        if (select(s_sock + 1, &fds, NULL, NULL, &tv) > 0) {
            char buf[256];
            struct sockaddr_in from;
            socklen_t flen = sizeof(from);
            int n = recvfrom(s_sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &flen);
            if (n > 0) {
                buf[n] = 0;
                handle_packet(buf, &from);
            }
        }
        int64_t now = esp_timer_get_time();
        if (now - last_refresh > 8LL * 1000000) {
            last_refresh = now;
            peer_link_refresh();
        }
    }
}

void peer_link_start(const peer_link_cfg_t *cfg)
{
    if (!s_mu) {
        s_mu = xSemaphoreCreateMutex();
    }
    if (cfg) {
        s_cfg = *cfg;
    }
    load_id();
    if (!s_cfg.name) {
        s_cfg.name = "Gateway";
    }
    if (!s_cfg.role) {
        s_cfg.role = "gateway";
    }
    static bool started;
    if (!started) {
        started = true;
        xTaskCreate(listen_task, "peer", 8192, NULL, 3, NULL);
    }
    ESP_LOGI(TAG, "This gateway is %s (%s)", s_cfg.name, s_self_id);
}
