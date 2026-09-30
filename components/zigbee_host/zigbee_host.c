/**
 * @file zigbee_host.c
 * @brief ICC bring-up + Zigbee network control + real device table.
 */

#include "zigbee_host.h"

#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "sdkconfig.h"

#include <stdlib.h>

static const char *TAG = "zigbee_host";
#define NVS_NS "zb_net"

static zigbee_host_status_t s_status;
/** Heap scratch for NVS pack/migrate — allocated only while saving (frees ~15 KiB BSS). */
static zb_device_t *s_nvs_scratch;
/** Button latch/name changes mark dirty; flush off the hot path (NVS flash freezes status_lock). */
static volatile bool s_devices_nvs_dirty;
static int64_t s_devices_nvs_last_flush_ms;
static SemaphoreHandle_t s_status_mutex;
static SemaphoreHandle_t s_op_mutex;
static TaskHandle_t s_task;
static int64_t s_permit_deadline_ms;
static bool s_pending_permit;
static uint8_t s_pending_permit_dur;
static zb_remote_button_cb_t s_remote_btn_cb;
static zb_sensor_update_cb_t s_sensor_upd_cb;
/** HomeKit button cb must not run under s_op_mutex/status_lock (re-enters those locks). */
#define ZB_HK_BTN_Q 8
typedef struct {
    uint8_t eui64[8];
    uint8_t button_index;
    uint8_t event;
} hk_btn_evt_t;
static hk_btn_evt_t s_hk_btn_q[ZB_HK_BTN_Q];
static uint8_t s_hk_btn_q_len;

/** Defer sensor→HomeKit pushes out of status_lock / ZCL parse. */
#define ZB_SENSOR_UPD_Q 6
static uint8_t s_sensor_upd_q[ZB_SENSOR_UPD_Q][8];
static uint8_t s_sensor_upd_q_len;

static void sensor_upd_q_push(const uint8_t eui64[8])
{
    if (!eui64) {
        return;
    }
    for (uint8_t i = 0; i < s_sensor_upd_q_len; i++) {
        if (memcmp(s_sensor_upd_q[i], eui64, 8) == 0) {
            return;
        }
    }
    if (s_sensor_upd_q_len >= ZB_SENSOR_UPD_Q) {
        memmove(&s_sensor_upd_q[0], &s_sensor_upd_q[1], (ZB_SENSOR_UPD_Q - 1) * 8);
        s_sensor_upd_q_len = ZB_SENSOR_UPD_Q - 1;
    }
    memcpy(s_sensor_upd_q[s_sensor_upd_q_len++], eui64, 8);
}

static void flush_sensor_upd_callbacks(void)
{
    while (s_sensor_upd_q_len > 0) {
        uint8_t eui[8];
        memcpy(eui, s_sensor_upd_q[0], 8);
        memmove(&s_sensor_upd_q[0], &s_sensor_upd_q[1], (s_sensor_upd_q_len - 1) * 8);
        s_sensor_upd_q_len--;
        if (s_sensor_upd_cb) {
            s_sensor_upd_cb(eui);
        }
    }
}

static void hk_btn_q_push(const uint8_t eui64[8], uint8_t button_index, uint8_t event)
{
    if (!eui64) {
        return;
    }
    if (s_hk_btn_q_len >= ZB_HK_BTN_Q) {
        /* Drop oldest. */
        memmove(&s_hk_btn_q[0], &s_hk_btn_q[1], (ZB_HK_BTN_Q - 1) * sizeof(s_hk_btn_q[0]));
        s_hk_btn_q_len = ZB_HK_BTN_Q - 1;
    }
    hk_btn_evt_t *e = &s_hk_btn_q[s_hk_btn_q_len++];
    memcpy(e->eui64, eui64, 8);
    e->button_index = button_index;
    e->event = event;
}

static void flush_hk_btn_callbacks(void)
{
    while (s_hk_btn_q_len > 0) {
        hk_btn_evt_t e = s_hk_btn_q[0];
        memmove(&s_hk_btn_q[0], &s_hk_btn_q[1], (s_hk_btn_q_len - 1) * sizeof(s_hk_btn_q[0]));
        s_hk_btn_q_len--;
        if (s_remote_btn_cb) {
            s_remote_btn_cb(e.eui64, e.button_index, e.event);
        }
    }
}
static uint8_t s_pending_remote_bind_eui[8];
static bool s_pending_remote_bind;
static int64_t s_last_remote_bind_ms;
/**
 * After the remote wakes (button → poll), install binds to our fake bulb.
 * Newer Tradfri FW prefers IEEE device binds; group 901 is a fallback.
 * Touchlink 10s-hold is not available on this ICC-1 NCP image (no Scan Response).
 */
enum {
    REMOTE_BIND_STEP_IEEE_ONOFF = 0, /* bind OnOff → fake bulb ep11 */
    REMOTE_BIND_STEP_IEEE_LEVEL,
    REMOTE_BIND_STEP_IEEE_SCENES,
    REMOTE_BIND_STEP_GROUP_ONOFF, /* fallback groupcast */
    REMOTE_BIND_STEP_GROUP_LEVEL,
    REMOTE_BIND_STEP_DONE
};
static uint8_t s_remote_bind_step;
static uint8_t s_pending_sensor_cfg_eui[8];
static bool s_pending_sensor_cfg;
static int64_t s_last_sensor_cfg_ms;
/** At most one in-flight APS frame per sleepy sensor (indirect queue / timeout). */
static uint16_t s_sensor_tx_node;
static bool s_sensor_tx_pending;
static int64_t s_sensor_tx_ms;
static uint8_t s_pending_interview_eui[8];
static bool s_pending_interview;
static int64_t s_last_poll_interview_ms;

/** One APS frame per step — SNZB-02D sleeps between polls; floods all fail 0x66. */
enum {
    SENSOR_STEP_BIND_TEMP = 0,
    SENSOR_STEP_BIND_HUM,
    SENSOR_STEP_BIND_BATT,
    SENSOR_STEP_CFG_TEMP,
    SENSOR_STEP_CFG_HUM,
    SENSOR_STEP_CFG_BATT,
    SENSOR_STEP_READ_TEMP,
    SENSOR_STEP_READ_HUM,
    SENSOR_STEP_READ_BATT,
    SENSOR_STEP_COUNT
};
/** After Identify Query, skip remote interview so F&B/bind airtime is free. */
static int64_t s_remote_fb_quiet_until_ms;
static zb_remote_press_t s_press_log[ZB_REMOTE_PRESS_LOG];
static uint8_t s_press_log_head; /* next write index */

typedef struct {
    uint8_t eui64[8];
    bool on;
} onoff_req_t;

typedef struct {
    uint8_t eui64[8];
    uint8_t level; /**< 0–254 */
} level_req_t;

#define ZB_CMD_QUEUE_MAX 16
static onoff_req_t s_onoff_q[ZB_CMD_QUEUE_MAX];
static uint8_t s_onoff_q_len;
static level_req_t s_level_q[ZB_CMD_QUEUE_MAX];
static uint8_t s_level_q_len;

static zb_device_t *device_find_by_eui_locked(const uint8_t eui64[8]);
static zb_device_t *device_find_by_node_locked(uint16_t node_id);
static void nvs_save_devices_locked(void);
static void sensor_try_send_one_unlocked(zb_device_t *d);

const char *zigbee_host_icc_status_str(icc_link_status_t s)
{
    switch (s) {
    case ICC_STATUS_CONNECTED:
        return "CONNECTED";
    case ICC_STATUS_NOT_CONNECTED:
        return "NOT CONNECTED";
    default:
        return "UNKNOWN";
    }
}

const char *zigbee_host_ash_state_str(ash_state_t s)
{
    switch (s) {
    case ASH_STATE_CONNECTING:
        return "connecting";
    case ASH_STATE_CONNECTED:
        return "connected";
    case ASH_STATE_FAILED:
        return "failed";
    default:
        return "disconnected";
    }
}

const char *zigbee_host_network_state_str(ezsp_network_status_t s)
{
    switch (s) {
    case EZSP_NO_NETWORK:
        return "NO_NETWORK";
    case EZSP_JOINING_NETWORK:
        return "JOINING";
    case EZSP_JOINED_NETWORK:
        return "JOINED";
    case EZSP_JOINED_NETWORK_NO_PARENT:
        return "JOINED_NO_PARENT";
    default:
        return "UNKNOWN";
    }
}

static void status_lock(void)
{
    if (s_status_mutex) {
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    }
}

static void status_unlock(void)
{
    if (s_status_mutex) {
        xSemaphoreGive(s_status_mutex);
    }
}

static void set_last_error(const char *msg)
{
    status_lock();
    snprintf(s_status.last_error, sizeof(s_status.last_error), "%s", msg ? msg : "");
    status_unlock();
}

static int64_t now_ms(void)
{
    return (int64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void update_permit_remaining(void)
{
    if (s_permit_deadline_ms <= 0) {
        s_status.permit_join_remaining = 0;
        return;
    }
    int64_t left = s_permit_deadline_ms - now_ms();
    if (left <= 0) {
        s_status.permit_join_remaining = 0;
        s_permit_deadline_ms = 0;
    } else {
        s_status.permit_join_remaining = (uint8_t)((left + 999) / 1000);
        if (s_status.permit_join_remaining > 254) {
            s_status.permit_join_remaining = 254;
        }
    }
}

static void nvs_save_network(const ezsp_network_params_t *p)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_blob(h, "params", p, sizeof(*p));
    nvs_commit(h);
    nvs_close(h);
}

static void nvs_clear_network(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_erase_key(h, "params");
    nvs_commit(h);
    nvs_close(h);
}

static void device_refresh_label(zb_device_t *d);
static bool eui_looks_like_ikea_remote(const uint8_t eui64[8]);

static void refresh_network_locked(void)
{
    ezsp_network_status_t st;
    if (ezsp_get_network_state(&st) == ESP_OK) {
        s_status.ncp.network_state = st;
        s_status.ncp.network_state_valid = true;
    }
    uint8_t node_type = 0;
    ezsp_network_params_t params;
    if (ezsp_get_network_parameters(&node_type, &params) == ESP_OK) {
        s_status.ncp.node_type = node_type;
        s_status.ncp.net = params;
        s_status.ncp.net_params_valid = true;
    }
}

static void device_refresh_label(zb_device_t *d)
{
    if (!d) {
        return;
    }
    /* Seed a friendly name for IKEA remotes before Basic interview completes. */
    if (!d->name[0] && eui_looks_like_ikea_remote(d->eui64) &&
        (d->node_type == EMBER_SLEEPY_END_DEVICE || d->node_type == EMBER_END_DEVICE) &&
        !d->has_onoff && !d->has_level && !d->has_temp) {
        snprintf(d->name, sizeof(d->name), "TRADFRI remote");
        if (!d->manufacturer[0]) {
            snprintf(d->manufacturer, sizeof(d->manufacturer), "IKEA of Sweden");
        }
    }
    if (d->model[0]) {
        snprintf(d->label, sizeof(d->label), "%s", d->model);
    } else if (d->name[0]) {
        snprintf(d->label, sizeof(d->label), "%s", d->name);
    } else {
        snprintf(d->label, sizeof(d->label), "%s · 0x%04X", ezsp_node_type_str(d->node_type),
                 d->node_id);
    }
}

static bool str_contains_ci(const char *hay, const char *needle)
{
    if (!hay || !needle || !needle[0]) {
        return false;
    }
    size_t nlen = strlen(needle);
    for (const char *p = hay; *p; p++) {
        size_t i = 0;
        while (i < nlen && p[i] &&
               (char)tolower((unsigned char)p[i]) == (char)tolower((unsigned char)needle[i])) {
            i++;
        }
        if (i == nlen) {
            return true;
        }
    }
    return false;
}

/** Whole-token match so "DOOR" does not hit friendly names like "Outdoors". */
static bool str_has_token_ci(const char *hay, const char *token)
{
    if (!hay || !token || !token[0]) {
        return false;
    }
    size_t nlen = strlen(token);
    for (const char *p = hay; *p; p++) {
        size_t i = 0;
        while (i < nlen && p[i] &&
               (char)tolower((unsigned char)p[i]) == (char)tolower((unsigned char)token[i])) {
            i++;
        }
        if (i == nlen) {
            bool left_ok = (p == hay) || !isalnum((unsigned char)p[-1]);
            bool right_ok = !isalnum((unsigned char)p[nlen]);
            if (left_ok && right_ok) {
                return true;
            }
        }
    }
    return false;
}

static bool model_looks_like_outlet(const char *model)
{
    if (!model || !model[0]) {
        return false;
    }
    return str_contains_ci(model, "PLUG") || str_contains_ci(model, "OUTLET") ||
           str_contains_ci(model, "S31") || str_contains_ci(model, "BASICZBR3") ||
           str_contains_ci(model, "SOCKET") || str_contains_ci(model, "SA-003") ||
           str_contains_ci(model, "TS011F") || str_contains_ci(model, "Smart Plug");
}

static bool model_looks_like_switch(const char *model)
{
    if (!model || !model[0]) {
        return false;
    }
    if (model_looks_like_outlet(model)) {
        return false;
    }
    return str_contains_ci(model, "SWP") || str_contains_ci(model, "SWITCH") ||
           str_contains_ci(model, "RELAY") || str_contains_ci(model, "ZBMINI") ||
           str_contains_ci(model, "ZBMINIL2") || str_contains_ci(model, "MINI");
}

static bool model_looks_like_irrigation(const char *model)
{
    if (!model || !model[0]) {
        return false;
    }
    return str_contains_ci(model, "SWV") || str_contains_ci(model, "WATER VALVE") ||
           str_contains_ci(model, "IRRIG") || str_contains_ci(model, "SPRINKLER") ||
           str_contains_ci(model, "VALVE");
}

static bool model_looks_like_contact(const char *model)
{
    if (!model || !model[0]) {
        return false;
    }
    /* "DOOR" is token-only — substring would mis-hit names like "Outdoors". */
    return str_contains_ci(model, "SNZB-04") || str_has_token_ci(model, "DOOR") ||
           str_contains_ci(model, "CONTACT") || str_contains_ci(model, "WINDOW SENSOR") ||
           str_contains_ci(model, "DS01") || str_contains_ci(model, "MCCGQ");
}

static bool model_looks_like_motion(const char *model)
{
    if (!model || !model[0]) {
        return false;
    }
    return str_contains_ci(model, "SNZB-03") || str_contains_ci(model, "MOTION") ||
           str_contains_ci(model, "PIR") || str_contains_ci(model, "OCCUPANCY") ||
           str_contains_ci(model, "MS01") || str_contains_ci(model, "RTCGQ");
}

static bool model_looks_like_leak(const char *model)
{
    if (!model || !model[0]) {
        return false;
    }
    return str_contains_ci(model, "SNZB-05") || str_contains_ci(model, "LEAK") ||
           str_contains_ci(model, "WATER LEAK") || str_contains_ci(model, "FLOOD") ||
           str_contains_ci(model, "SJCGQ") || str_contains_ci(model, "WL01");
}

static bool model_looks_like_smoke(const char *model)
{
    if (!model || !model[0]) {
        return false;
    }
    return str_contains_ci(model, "SMOKE") || str_contains_ci(model, "FIRE") ||
           str_contains_ci(model, "GS358") || str_contains_ci(model, "JTYJ");
}

static bool model_looks_like_remote(const char *model)
{
    if (!model || !model[0]) {
        return false;
    }
    return str_contains_ci(model, "remote") || str_contains_ci(model, "shortcut") ||
           str_contains_ci(model, "STYRBAR") || str_contains_ci(model, "SYMFONISK") ||
           str_contains_ci(model, "wireless switch") || str_contains_ci(model, "E1810") ||
           str_contains_ci(model, "E1743") || str_contains_ci(model, "E1766") ||
           str_contains_ci(model, "E2001") || str_contains_ci(model, "E2002") ||
           str_contains_ci(model, "Remote Control N2") ||
           str_contains_ci(model, "TRADFRI ON/OFF SWITCH");
}

/** IKEA battery remotes commonly use OUI d0:cf:5e (before Basic interview completes). */
static bool eui_looks_like_ikea_remote(const uint8_t eui64[8])
{
    if (!eui64) {
        return false;
    }
    /* EUI64 stored little-endian in Ember: bytes [7]=OUI0, [6]=OUI1, [5]=OUI2 */
    return eui64[7] == 0xd0 && eui64[6] == 0xcf && eui64[5] == 0x5e;
}

/** Classic Sonoff battery sensors (SNZB-02 / TH01) use TI OUI 00:12:4b. */
static bool eui_looks_like_sonoff_sensor(const uint8_t eui64[8])
{
    if (!eui64) {
        return false;
    }
    return eui64[7] == 0x00 && eui64[6] == 0x12 && eui64[5] == 0x4b;
}

static bool model_looks_like_light(const char *model)
{
    if (!model || !model[0]) {
        return false;
    }
    /* Remotes / controllers are not lights even if branded TRADFRI. */
    if (model_looks_like_remote(model)) {
        return false;
    }
    return str_contains_ci(model, "TRADFRI") || str_contains_ci(model, "Driver") ||
           str_contains_ci(model, "ICPSHC") || str_contains_ci(model, "LED drivers") ||
           str_contains_ci(model, "LED") || str_contains_ci(model, "bulb") ||
           str_contains_ci(model, "BULB") || str_contains_ci(model, "light") ||
           str_contains_ci(model, "LIGHT") || str_contains_ci(model, "DIM") ||
           str_contains_ci(model, "GU10") || str_contains_ci(model, "E27") ||
           str_contains_ci(model, "E14") || str_contains_ci(model, "WS82") ||
           str_contains_ci(model, "L1527") || str_contains_ci(model, "L1529");
}

/** Climate / air-quality style sensors (not binary IAS). */
static bool model_looks_like_climate(const char *model)
{
    if (!model || !model[0]) {
        return false;
    }
    if (model_looks_like_contact(model) || model_looks_like_motion(model) ||
        model_looks_like_leak(model) || model_looks_like_smoke(model)) {
        return false;
    }
    return str_contains_ci(model, "SNZB-02") || str_contains_ci(model, "SNZB-02D") ||
           str_contains_ci(model, "TH01") || str_contains_ci(model, "TH02") ||
           str_contains_ci(model, "TEMP") || str_contains_ci(model, "HUMID") ||
           str_contains_ci(model, "TH ") || str_contains_ci(model, "VINDSTYRKA") ||
           str_contains_ci(model, "air quality") || str_contains_ci(model, "weather") ||
           str_contains_ci(model, "WSDCGQ");
}

static zb_device_kind_t kind_from_ias_zone_type(uint16_t zt)
{
    switch (zt) {
    case ZCL_IAS_ZONE_MOTION:
        return ZB_DEVICE_KIND_MOTION;
    case ZCL_IAS_ZONE_CONTACT:
        return ZB_DEVICE_KIND_CONTACT;
    case ZCL_IAS_ZONE_FIRE:
        return ZB_DEVICE_KIND_SMOKE;
    case ZCL_IAS_ZONE_WATER:
        return ZB_DEVICE_KIND_LEAK;
    default:
        return ZB_DEVICE_KIND_UNKNOWN;
    }
}

static void refresh_binary_on_locked(zb_device_t *d)
{
    if (!d) {
        return;
    }
    if (d->has_ias_zone) {
        d->binary_on = (d->ias_zone_status & 0x0001) != 0; /* Alarm1 */
    } else if (d->has_occupancy) {
        d->binary_on = d->occupancy;
    } else if (d->has_onoff) {
        d->binary_on = d->onoff_on;
    }
}

zb_device_kind_t zigbee_host_device_kind(const zb_device_t *d)
{
    if (!d || !d->used) {
        return ZB_DEVICE_KIND_UNKNOWN;
    }
    /* Remotes first — they advertise On/Off + Level as *client* clusters. */
    if (model_looks_like_remote(d->model) || model_looks_like_remote(d->name) ||
        model_looks_like_remote(d->label)) {
        return ZB_DEVICE_KIND_REMOTE;
    }
    if ((d->node_type == EMBER_SLEEPY_END_DEVICE || d->node_type == EMBER_END_DEVICE) &&
        eui_looks_like_ikea_remote(d->eui64) && !d->has_onoff && !d->has_level && !d->has_temp &&
        !d->has_ias_zone) {
        return ZB_DEVICE_KIND_REMOTE;
    }

    /* IAS ZoneType / occupancy before On/Off (valves still win via model below). */
    if (d->has_ias_zone) {
        zb_device_kind_t ik = kind_from_ias_zone_type(d->ias_zone_type);
        if (ik != ZB_DEVICE_KIND_UNKNOWN) {
            return ik;
        }
    }
    if (d->has_occupancy) {
        return ZB_DEVICE_KIND_MOTION;
    }

    /* Climate before name/model binary fingerprints — "Outdoors" contains "door". */
    if (d->has_temp || d->has_humidity || model_looks_like_climate(d->model) ||
        model_looks_like_climate(d->name) || model_looks_like_climate(d->label) ||
        ((d->node_type == EMBER_SLEEPY_END_DEVICE || d->node_type == EMBER_END_DEVICE) &&
         eui_looks_like_sonoff_sensor(d->eui64) && !d->has_onoff && !d->has_level &&
         !d->has_ias_zone)) {
        return ZB_DEVICE_KIND_SENSOR;
    }

    if (model_looks_like_motion(d->model) || model_looks_like_motion(d->name)) {
        return ZB_DEVICE_KIND_MOTION;
    }
    if (model_looks_like_contact(d->model) || model_looks_like_contact(d->name)) {
        return ZB_DEVICE_KIND_CONTACT;
    }
    if (model_looks_like_leak(d->model) || model_looks_like_leak(d->name)) {
        return ZB_DEVICE_KIND_LEAK;
    }
    if (model_looks_like_smoke(d->model) || model_looks_like_smoke(d->name)) {
        return ZB_DEVICE_KIND_SMOKE;
    }

    if (model_looks_like_irrigation(d->model) || model_looks_like_irrigation(d->name) ||
        model_looks_like_irrigation(d->label)) {
        return ZB_DEVICE_KIND_IRRIGATION;
    }

    if (d->has_level || model_looks_like_light(d->model) || model_looks_like_light(d->name) ||
        model_looks_like_light(d->label)) {
        return ZB_DEVICE_KIND_LIGHT;
    }
    if (d->has_onoff && !model_looks_like_switch(d->model) && !model_looks_like_outlet(d->model) &&
        (str_contains_ci(d->manufacturer, "IKEA") || str_contains_ci(d->manufacturer, "ikea"))) {
        return ZB_DEVICE_KIND_LIGHT;
    }
    if (model_looks_like_outlet(d->model) || model_looks_like_outlet(d->name)) {
        return ZB_DEVICE_KIND_OUTLET;
    }
    if (d->has_onoff || model_looks_like_switch(d->model)) {
        return ZB_DEVICE_KIND_SWITCH;
    }
    return ZB_DEVICE_KIND_UNKNOWN;
}

bool zigbee_host_is_onoff_actuator(const zb_device_t *d)
{
    zb_device_kind_t k = zigbee_host_device_kind(d);
    return k == ZB_DEVICE_KIND_LIGHT || k == ZB_DEVICE_KIND_SWITCH || k == ZB_DEVICE_KIND_OUTLET ||
           k == ZB_DEVICE_KIND_IRRIGATION;
}

bool zigbee_host_is_binary_sensor(const zb_device_t *d)
{
    zb_device_kind_t k = zigbee_host_device_kind(d);
    return k == ZB_DEVICE_KIND_CONTACT || k == ZB_DEVICE_KIND_MOTION || k == ZB_DEVICE_KIND_LEAK ||
           k == ZB_DEVICE_KIND_SMOKE;
}

uint8_t zigbee_host_remote_button_count(const zb_device_t *d)
{
    if (!d || zigbee_host_device_kind(d) != ZB_DEVICE_KIND_REMOTE) {
        return 0;
    }
    const char *m = d->model[0] ? d->model : d->name;
    if (str_contains_ci(m, "shortcut") || str_contains_ci(m, "E1766")) {
        return 1;
    }
    if (str_contains_ci(m, "E1743") || str_contains_ci(m, "ON/OFF SWITCH")) {
        return 2;
    }
    if (str_contains_ci(m, "STYRBAR") || str_contains_ci(m, "E2001") ||
        str_contains_ci(m, "E2002") || str_contains_ci(m, "Remote Control N2")) {
        return 4;
    }
    /* Classic TRADFRI remote (E1810): Power, Dimmer, Brighter, Left, Right */
    return 5;
}

const char *zigbee_host_remote_button_default_name(uint8_t nbtn, uint8_t button_index)
{
    static const char *names5[] = {"Power", "Dimmer", "Brighter", "Left", "Right"};
    static const char *names4[] = {"On", "Off", "Left", "Right"};
    static const char *names2[] = {"On", "Off"};
    if (nbtn >= 5 && button_index < 5) {
        return names5[button_index];
    }
    if (nbtn == 4 && button_index < 4) {
        return names4[button_index];
    }
    if (nbtn == 2 && button_index < 2) {
        return names2[button_index];
    }
    if (nbtn == 1 && button_index == 0) {
        return "Button";
    }
    return "Button";
}

const char *zigbee_host_remote_button_name(const zb_device_t *d, uint8_t button_index)
{
    if (!d || button_index >= ZB_REMOTE_MAX_BUTTONS) {
        return "Button";
    }
    if (d->btn_name[button_index][0]) {
        return d->btn_name[button_index];
    }
    uint8_t n = zigbee_host_remote_button_count(d);
    if (n == 0) {
        n = 5;
    }
    return zigbee_host_remote_button_default_name(n, button_index);
}

esp_err_t zigbee_host_btn_set_name(const uint8_t eui64[8], uint8_t button_index, const char *name)
{
    if (!eui64 || button_index >= ZB_REMOTE_MAX_BUTTONS) {
        return ESP_ERR_INVALID_ARG;
    }
    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui64);
    esp_err_t err = ESP_ERR_NOT_FOUND;
    if (d) {
        if (name && name[0]) {
            snprintf(d->btn_name[button_index], sizeof(d->btn_name[button_index]), "%s", name);
        } else {
            d->btn_name[button_index][0] = '\0';
        }
        s_devices_nvs_dirty = true;
        err = ESP_OK;
    }
    status_unlock();
    if (err == ESP_OK && s_task) {
        xTaskNotifyGive(s_task);
    }
    return err;
}

bool zigbee_host_btn_is_stateful(const zb_device_t *d, uint8_t button_index)
{
    if (!d || button_index >= ZB_REMOTE_MAX_BUTTONS) {
        return false;
    }
    uint8_t n = zigbee_host_remote_button_count(d);
    if (button_index >= n) {
        return false;
    }
    return d->btn_mode[button_index] == ZB_BTN_MODE_STATEFUL;
}

bool zigbee_host_btn_get_on(const uint8_t eui64[8], uint8_t button_index)
{
    if (!eui64 || button_index >= ZB_REMOTE_MAX_BUTTONS || !s_op_mutex) {
        return false;
    }
    bool on = false;
    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui64);
    if (d && button_index < ZB_REMOTE_MAX_BUTTONS) {
        on = d->btn_on[button_index];
    }
    status_unlock();
    return on;
}

esp_err_t zigbee_host_btn_set_on(const uint8_t eui64[8], uint8_t button_index, bool on)
{
    if (!eui64 || button_index >= ZB_REMOTE_MAX_BUTTONS) {
        return ESP_ERR_INVALID_ARG;
    }
    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui64);
    esp_err_t err = ESP_ERR_NOT_FOUND;
    if (d && zigbee_host_btn_is_stateful(d, button_index)) {
        d->btn_on[button_index] = on;
        s_devices_nvs_dirty = true;
        err = ESP_OK;
    }
    status_unlock();
    if (err == ESP_OK && s_task) {
        xTaskNotifyGive(s_task);
    }
    return err;
}

bool zigbee_host_btn_toggle(const uint8_t eui64[8], uint8_t button_index)
{
    if (!eui64 || button_index >= ZB_REMOTE_MAX_BUTTONS) {
        return false;
    }
    bool on = false;
    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui64);
    if (d && zigbee_host_btn_is_stateful(d, button_index)) {
        d->btn_on[button_index] = !d->btn_on[button_index];
        on = d->btn_on[button_index];
        s_devices_nvs_dirty = true;
    }
    status_unlock();
    return on;
}

void zigbee_host_set_remote_button_cb(zb_remote_button_cb_t cb)
{
    s_remote_btn_cb = cb;
}

void zigbee_host_set_sensor_update_cb(zb_sensor_update_cb_t cb)
{
    s_sensor_upd_cb = cb;
}

static void press_log_push(const uint8_t eui64[8], uint8_t button_1based, uint8_t event)
{
    zb_remote_press_t *slot = &s_press_log[s_press_log_head % ZB_REMOTE_PRESS_LOG];
    memset(slot, 0, sizeof(*slot));
    slot->used = true;
    memcpy(slot->eui64, eui64, 8);
    slot->button = button_1based;
    slot->event = event;
    slot->ms = (int64_t)(esp_timer_get_time() / 1000);
    s_press_log_head++;
}

uint8_t zigbee_host_copy_remote_presses(zb_remote_press_t *out, uint8_t max)
{
    if (!out || max == 0) {
        return 0;
    }
    uint8_t n = 0;
    /* Newest first */
    for (uint8_t i = 0; i < ZB_REMOTE_PRESS_LOG && n < max; i++) {
        uint8_t idx = (uint8_t)((s_press_log_head + ZB_REMOTE_PRESS_LOG - 1 - i) % ZB_REMOTE_PRESS_LOG);
        if (!s_press_log[idx].used) {
            continue;
        }
        out[n++] = s_press_log[idx];
    }
    return n;
}

static bool eui_is_zero(const uint8_t eui64[8])
{
    if (!eui64) {
        return true;
    }
    for (int i = 0; i < 8; i++) {
        if (eui64[i]) {
            return false;
        }
    }
    return true;
}

/** Enqueue On/Off for target while status_lock is already held. */
static void queue_onoff_locked(const uint8_t eui64[8], bool on)
{
    zb_device_t *t = device_find_by_eui_locked(eui64);
    if (!t || !t->used) {
        return;
    }
    t->onoff_on = on;
    t->has_onoff = true;
    if (!on) {
        t->level = 0;
    }
    int slot = -1;
    for (uint8_t i = 0; i < s_onoff_q_len; i++) {
        if (memcmp(s_onoff_q[i].eui64, eui64, 8) == 0) {
            slot = (int)i;
            break;
        }
    }
    if (slot < 0 && s_onoff_q_len < ZB_CMD_QUEUE_MAX) {
        slot = (int)s_onoff_q_len++;
    }
    if (slot >= 0) {
        memcpy(s_onoff_q[slot].eui64, eui64, 8);
        s_onoff_q[slot].on = on;
    }
}

/** Enqueue brightness (0–100) for target while status_lock is already held. */
static void queue_brightness_locked(const uint8_t eui64[8], uint8_t brightness_pct)
{
    zb_device_t *t = device_find_by_eui_locked(eui64);
    if (!t || !t->used) {
        return;
    }
    if (brightness_pct > 100) {
        brightness_pct = 100;
    }
    uint8_t level = zigbee_host_brightness_to_level(brightness_pct);
    t->level = level;
    t->has_level = true;
    t->onoff_on = (level > 0);
    t->has_onoff = true;
    int slot = -1;
    for (uint8_t i = 0; i < s_level_q_len; i++) {
        if (memcmp(s_level_q[i].eui64, eui64, 8) == 0) {
            slot = (int)i;
            break;
        }
    }
    if (slot < 0 && s_level_q_len < ZB_CMD_QUEUE_MAX) {
        slot = (int)s_level_q_len++;
    }
    if (slot >= 0) {
        memcpy(s_level_q[slot].eui64, eui64, 8);
        s_level_q[slot].level = level;
    }
}

static void migrate_remote_targets_locked(zb_device_t *d)
{
    if (!d) {
        return;
    }
    if (d->target_count == 0 && !eui_is_zero(d->target_eui64)) {
        memcpy(d->targets[0], d->target_eui64, 8);
        d->target_count = 1;
    }
    if (d->target_count > ZB_REMOTE_MAX_TARGETS) {
        d->target_count = ZB_REMOTE_MAX_TARGETS;
    }
    if (d->target_group_count > ZB_REMOTE_MAX_TARGETS) {
        d->target_group_count = ZB_REMOTE_MAX_TARGETS;
    }
    if (d->target_count > 0) {
        memcpy(d->target_eui64, d->targets[0], 8);
    } else {
        memset(d->target_eui64, 0, 8);
    }
}

/**
 * One-time: old E1810 order was Brighter@1 / Dimmer@2; now Dimmer@1 / Brighter@2
 * (Level DOWN→1, UP→2). Swap per-button name/mode/on so physical labels stay correct.
 */
static void migrate_e1810_btn_order_locked(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    uint8_t done = 0;
    if (nvs_get_u8(h, "btn_ord", &done) == ESP_OK && done == 1) {
        nvs_close(h);
        return;
    }
    bool changed = false;
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        zb_device_t *d = &s_status.devices[i];
        if (!d->used || zigbee_host_remote_button_count(d) < 5) {
            continue;
        }
        char tmp_name[24];
        memcpy(tmp_name, d->btn_name[1], sizeof(tmp_name));
        memcpy(d->btn_name[1], d->btn_name[2], sizeof(d->btn_name[1]));
        memcpy(d->btn_name[2], tmp_name, sizeof(d->btn_name[2]));
        uint8_t tmp_mode = d->btn_mode[1];
        d->btn_mode[1] = d->btn_mode[2];
        d->btn_mode[2] = tmp_mode;
        bool tmp_on = d->btn_on[1];
        d->btn_on[1] = d->btn_on[2];
        d->btn_on[2] = tmp_on;
        /* Drop stored names that now match the new defaults. */
        if (strcmp(d->btn_name[1], "Dimmer") == 0) {
            d->btn_name[1][0] = '\0';
        }
        if (strcmp(d->btn_name[2], "Brighter") == 0) {
            d->btn_name[2][0] = '\0';
        }
        changed = true;
    }
    nvs_set_u8(h, "btn_ord", 1);
    nvs_commit(h);
    nvs_close(h);
    if (changed) {
        s_devices_nvs_dirty = true;
        ESP_LOGI(TAG, "Migrated E1810 button order to Dimmer/Brighter");
    }
}

/** Compact CONTROL snapshot — full zb_device_t[32] BSS copies exhausted DRAM. */
typedef struct {
    uint16_t node_id;
    uint16_t remote_group_id;
    uint8_t target_count;
    uint8_t targets[ZB_REMOTE_MAX_TARGETS][8];
} zb_ctrl_snap_t;

#define ZB_CTRL_SNAP_MAX 8
static zb_ctrl_snap_t s_ctrl_snaps[ZB_CTRL_SNAP_MAX];
static uint8_t s_boot_ctrl_n;
static uint8_t s_boot_ctrl_i;

/** Deferred CONTROL group sync — never run EZSP RemoveGroup on the httpd task. */
static struct {
    bool pending;
    bool do_mgmt_bind;
    zb_ctrl_snap_t snap;
    uint8_t old_n;
    uint8_t old_targets[ZB_REMOTE_MAX_TARGETS][8];
} s_ctrl_sync_job;

static void sync_control_snap_unlocked(const zb_ctrl_snap_t *snap, const uint8_t old_targets[][8],
                                       uint8_t old_n);

static void queue_ctrl_sync(const zb_ctrl_snap_t *snap, const uint8_t old_targets[][8],
                            uint8_t old_n, bool do_mgmt_bind)
{
    if (!snap) {
        return;
    }
    s_ctrl_sync_job.snap = *snap;
    s_ctrl_sync_job.old_n = old_n > ZB_REMOTE_MAX_TARGETS ? ZB_REMOTE_MAX_TARGETS : old_n;
    memset(s_ctrl_sync_job.old_targets, 0, sizeof(s_ctrl_sync_job.old_targets));
    if (old_targets && s_ctrl_sync_job.old_n) {
        memcpy(s_ctrl_sync_job.old_targets, old_targets,
               (size_t)s_ctrl_sync_job.old_n * sizeof(old_targets[0]));
    }
    s_ctrl_sync_job.do_mgmt_bind = do_mgmt_bind;
    s_ctrl_sync_job.pending = true;
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

static void process_ctrl_sync_job_unlocked(void)
{
    if (!s_ctrl_sync_job.pending) {
        return;
    }
    zb_ctrl_snap_t snap = s_ctrl_sync_job.snap;
    uint8_t old_n = s_ctrl_sync_job.old_n;
    uint8_t old_targets[ZB_REMOTE_MAX_TARGETS][8];
    memcpy(old_targets, s_ctrl_sync_job.old_targets, sizeof(old_targets));
    bool do_mgmt_bind = s_ctrl_sync_job.do_mgmt_bind;
    s_ctrl_sync_job.pending = false;

    sync_control_snap_unlocked(&snap, old_targets, old_n);
    if (snap.node_id) {
        ezsp_hint_button_remote(snap.node_id);
        if (snap.remote_group_id) {
            ezsp_note_group_owner(snap.remote_group_id, snap.node_id);
        }
        if (do_mgmt_bind) {
            (void)ezsp_zdo_mgmt_bind_req(snap.node_id, 0);
        }
    }
}

static void ctrl_snap_from_device(const zb_device_t *d, zb_ctrl_snap_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!d) {
        return;
    }
    out->node_id = d->node_id;
    out->remote_group_id = d->remote_group_id;
    out->target_count = d->target_count;
    if (out->target_count > ZB_REMOTE_MAX_TARGETS) {
        out->target_count = ZB_REMOTE_MAX_TARGETS;
    }
    memcpy(out->targets, d->targets, sizeof(out->targets));
    if (out->target_count == 0 && !eui_is_zero(d->target_eui64)) {
        memcpy(out->targets[0], d->target_eui64, 8);
        out->target_count = 1;
    }
}

static void add_remove_device_group_unlocked(const uint8_t eui64[8], uint16_t gid, bool add)
{
    if (!eui64 || gid == 0 || eui_is_zero(eui64)) {
        return;
    }
    status_lock();
    zb_device_t *t = device_find_by_eui_locked(eui64);
    uint16_t node = 0;
    uint8_t ep = 1;
    if (t && t->used) {
        node = t->node_id;
        ep = t->onoff_ep ? t->onoff_ep : (t->level_ep ? t->level_ep : 1);
    }
    status_unlock();
    if (!node) {
        return;
    }
    if (add) {
        ESP_LOGI(TAG, "CONTROL: AddGroup %u → 0x%04X ep%u", (unsigned)gid, node, (unsigned)ep);
        (void)ezsp_zcl_groups_add_group(node, ep, gid);
        if (ep != 1) {
            (void)ezsp_zcl_groups_add_group(node, 1, gid);
        }
    } else {
        ESP_LOGI(TAG, "CONTROL: RemoveGroup %u → 0x%04X ep%u", (unsigned)gid, node, (unsigned)ep);
        (void)ezsp_zcl_groups_remove_group(node, ep, gid);
        if (ep != 1) {
            (void)ezsp_zcl_groups_remove_group(node, 1, gid);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}

/**
 * Put CONTROL targets IN the remote Touchlink group so IKEA Level Move/Step
 * dimming works natively. Power still goes through the GW: we hear Toggle and
 * force absolute MoveToLevel so members stay in sync.
 * Idempotent AddGroup (skip if already done this boot for node+gid).
 */
static void sync_control_snap_unlocked(const zb_ctrl_snap_t *snap, const uint8_t old_targets[][8],
                                       uint8_t old_n)
{
    if (!snap) {
        return;
    }
    uint16_t gid = snap->remote_group_id;
    if (gid == 0) {
        ESP_LOGW(TAG, "CONTROL targets set but remote group unknown — press remote once, then Save");
        return;
    }
    uint8_t new_n = snap->target_count;
    if (new_n > ZB_REMOTE_MAX_TARGETS) {
        new_n = ZB_REMOTE_MAX_TARGETS;
    }

    typedef struct {
        uint16_t gid;
        uint16_t node;
    } member_key_t;
    static member_key_t s_in_group[ZB_HOST_MAX_DEVICES];
    static uint8_t s_in_n;

    /* Remove deselected targets from the Touchlink group. */
    for (uint8_t i = 0; i < old_n && i < ZB_REMOTE_MAX_TARGETS; i++) {
        bool still = false;
        for (uint8_t j = 0; j < new_n; j++) {
            if (memcmp(old_targets[i], snap->targets[j], 8) == 0) {
                still = true;
                break;
            }
        }
        if (!still) {
            add_remove_device_group_unlocked(old_targets[i], gid, false);
            status_lock();
            zb_device_t *t = device_find_by_eui_locked(old_targets[i]);
            uint16_t node = (t && t->used) ? t->node_id : 0;
            status_unlock();
            for (uint8_t k = 0; k < s_in_n; k++) {
                if (s_in_group[k].gid == gid && s_in_group[k].node == node) {
                    s_in_group[k] = s_in_group[s_in_n - 1];
                    s_in_n--;
                    break;
                }
            }
        }
    }

    for (uint8_t i = 0; i < new_n; i++) {
        const uint8_t *eui = snap->targets[i];
        status_lock();
        zb_device_t *t = device_find_by_eui_locked(eui);
        uint16_t node = (t && t->used) ? t->node_id : 0;
        status_unlock();
        bool already = false;
        if (node) {
            for (uint8_t k = 0; k < s_in_n; k++) {
                if (s_in_group[k].gid == gid && s_in_group[k].node == node) {
                    already = true;
                    break;
                }
            }
        }
        if (already) {
            continue;
        }
        add_remove_device_group_unlocked(eui, gid, true);
        if (node && s_in_n < ZB_HOST_MAX_DEVICES) {
            s_in_group[s_in_n].gid = gid;
            s_in_group[s_in_n].node = node;
            s_in_n++;
        }
    }
    if (snap->node_id) {
        ezsp_hint_button_remote(snap->node_id);
        ezsp_note_group_owner(gid, snap->node_id);
    }
    ESP_LOGI(TAG, "CONTROL: %u target(s) in group %u (native dim; GW syncs power)",
             (unsigned)new_n, (unsigned)gid);
}

/** Ensure CONTROL targets are detached (e.g. after group learn / boot). */
static void attach_control_snap_unlocked(const zb_ctrl_snap_t *snap)
{
    sync_control_snap_unlocked(snap, NULL, 0);
}

/** Collect CONTROL target pointers (status_lock held). */
static uint8_t control_collect_targets_locked(const zb_device_t *remote, zb_device_t **out,
                                              uint8_t out_max)
{
    uint8_t n = 0;
    if (!remote || !out || out_max == 0) {
        return 0;
    }
    uint8_t tn = remote->target_count;
    if (tn == 0 && !eui_is_zero(remote->target_eui64)) {
        tn = 1;
    }
    for (uint8_t ti = 0; ti < tn && ti < ZB_REMOTE_MAX_TARGETS && n < out_max; ti++) {
        const uint8_t *eui = (remote->target_count > 0) ? remote->targets[ti] : remote->target_eui64;
        zb_device_t *t = device_find_by_eui_locked(eui);
        if (!t || !t->used) {
            continue;
        }
        zb_device_kind_t tk = zigbee_host_device_kind(t);
        if (tk != ZB_DEVICE_KIND_LIGHT && tk != ZB_DEVICE_KIND_SWITCH &&
            tk != ZB_DEVICE_KIND_OUTLET && tk != ZB_DEVICE_KIND_IRRIGATION) {
            continue;
        }
        out[n++] = t;
    }
    return n;
}

/**
 * Relay remote button to all CONTROL targets with one shared desired state
 * (must hold status_lock). TRADFRI LED drivers ignore ZCL On — use MoveToLevel
 * with On/Off like group_set_onoff() does for the phone/HomeKit path.
 */
static void relay_remote_to_target_locked(const zb_device_t *remote, uint8_t button_index,
                                          uint8_t event)
{
    if (!remote) {
        return;
    }
    /* Debounce: messageSent synth + real RX can both fire for one press.
     * IKEA Move (hold-to-dim) repeats — allow rate-limited steps, not a full ignore. */
    static int64_t s_last_ms;
    static uint16_t s_last_node;
    static uint8_t s_last_btn;
    static bool s_room_on; /* last commanded room power (not stale device reports) */
    static uint16_t s_room_node;
    int64_t now = now_ms();
    uint32_t debounce_ms = 350;
    if (button_index == 0) {
        debounce_ms = 2500; /* power: synth + delayed real Off/Toggle */
    } else if (button_index == 1 || button_index == 2) {
        debounce_ms = (event == HK_BTN_EVENT_LONG) ? 320 : 200;
    }
    if (remote->node_id == s_last_node && button_index == s_last_btn &&
        (now - s_last_ms) < (int64_t)debounce_ms) {
        return;
    }

    zb_device_t *targets[ZB_REMOTE_MAX_TARGETS];
    uint8_t n = control_collect_targets_locked(remote, targets, ZB_REMOTE_MAX_TARGETS);
    if (n == 0) {
        return;
    }

    if (s_room_node != remote->node_id) {
        s_room_node = remote->node_id;
        /* Seed from targets: any on → room on. */
        s_room_on = false;
        for (uint8_t i = 0; i < n; i++) {
            if (targets[i]->onoff_on ||
                (targets[i]->has_level && targets[i]->level > 0)) {
                s_room_on = true;
                break;
            }
        }
    }

    uint8_t nbtn = zigbee_host_remote_button_count(remote);
    bool do_onoff = false;
    bool desired_on = false;

    if (nbtn >= 5) {
        if (button_index == 0) {
            do_onoff = true;
            desired_on = !s_room_on;
        } else {
            /* Dim buttons: native Level groupcast (targets in Touchlink group). */
            s_last_ms = now;
            s_last_node = remote->node_id;
            s_last_btn = button_index;
            if (button_index == 1) {
                s_room_on = true;
            }
            return;
        }
    } else if (nbtn == 4 || nbtn == 2) {
        if (button_index == 0) {
            do_onoff = true;
            desired_on = true;
        } else if (button_index == 1) {
            do_onoff = true;
            desired_on = false;
        } else {
            return; /* arrows — ignore for CONTROL */
        }
    } else {
        do_onoff = true;
        desired_on = !s_room_on;
    }

    if (!do_onoff) {
        return;
    }

    s_last_ms = now;
    s_last_node = remote->node_id;
    s_last_btn = button_index;
    s_room_on = desired_on;

    /* Targets are in the Touchlink group — native On/Off/Toggle already hit them.
     * Sending MoveToLevel again causes a visible double flash. Mirror desired
     * state into the inventory so HomeKit/portal stay in sync (IKEA drivers
     * often skip OnOff attribute reports after groupcast). */
    if (remote->remote_group_id != 0) {
        ESP_LOGI(TAG, "CONTROL power native group %u → room %s (no unicast)",
                 (unsigned)remote->remote_group_id, desired_on ? "ON" : "OFF");
        for (uint8_t i = 0; i < n; i++) {
            zb_device_t *t = targets[i];
            t->has_onoff = true;
            t->onoff_on = desired_on;
            if (!desired_on) {
                t->level = 0;
            } else if (t->has_level && t->level == 0) {
                t->level = 254;
            }
        }
        return;
    }

    ESP_LOGI(TAG, "CONTROL sync %u target(s): %s", (unsigned)n, desired_on ? "ON" : "OFF");

    for (uint8_t i = 0; i < n; i++) {
        zb_device_t *t = targets[i];
        zb_device_kind_t tk = zigbee_host_device_kind(t);
        bool use_level = (tk == ZB_DEVICE_KIND_LIGHT) || t->has_level;
        if (use_level) {
            uint8_t bri = 0;
            if (desired_on) {
                bri = t->has_level ? zigbee_host_level_to_brightness(t->level) : 100;
                if (bri < 1) {
                    bri = 100;
                }
            }
            queue_brightness_locked(t->eui64, bri);
        } else {
            queue_onoff_locked(t->eui64, desired_on);
        }
    }
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

static void emit_remote_button(const zb_device_t *d, uint8_t button_index, uint8_t event)
{
    if (!d || !d->used) {
        return;
    }
    uint8_t nb = zigbee_host_remote_button_count(d);
    if (nb == 0 || button_index >= nb) {
        return;
    }
    /* messageSent synth + real multicast RX can both fire for one physical press.
     * Power (btn 0): synth Toggle then real On/Off/Toggle often 0.5–2s apart —
     * without a wide window the stateful HK switch flips twice (off→on→off). */
    static int64_t s_emit_ms;
    static uint16_t s_emit_node;
    static uint8_t s_emit_btn;
    int64_t now = now_ms();
    int64_t debounce_ms = (button_index == 0) ? 2500 : 350;
    if (d->node_id == s_emit_node && button_index == s_emit_btn &&
        (now - s_emit_ms) < debounce_ms) {
        ESP_LOGI(TAG, "Remote btn %u debounced (%lld ms)", (unsigned)button_index,
                 (long long)(now - s_emit_ms));
        return;
    }
    s_emit_ms = now;
    s_emit_node = d->node_id;
    s_emit_btn = button_index;

    ESP_LOGI(TAG, "Remote button eui=..%02X%02X btn=%u event=%u type=%u", d->eui64[0], d->eui64[1],
             (unsigned)button_index, (unsigned)event, (unsigned)d->remote_type);
    press_log_push(d->eui64, (uint8_t)(button_index + 1), event);

    if (d->remote_type == ZB_REMOTE_TYPE_CONTROL) {
        relay_remote_to_target_locked(d, button_index, event);
        return;
    }
    /* Defer HomeKit cb — it toggles latch / updates HAP and re-takes our locks. */
    hk_btn_q_push(d->eui64, button_index, event);
}

static void mark_remote_bound_locked(zb_device_t *d)
{
    if (!d || d->remote_bound) {
        return;
    }
    d->remote_bound = true;
    ESP_LOGI(TAG, "Remote bind confirmed (button traffic) node=0x%04X", d->node_id);
}

static bool device_looks_like_temp_sensor(const zb_device_t *d)
{
    return d && (zigbee_host_device_kind(d) == ZB_DEVICE_KIND_SENSOR || d->has_temp ||
                 d->has_humidity || model_looks_like_climate(d->model) ||
                 eui_looks_like_sonoff_sensor(d->eui64));
}

/**
 * Queue exactly one APS frame for a sleepy sensor.
 * Ember buffers it until the next child poll; flooding N frames ⇒ N-1 DELIVERY_FAILED.
 * Returns true if a frame was queued (caller marks tx-pending).
 */
static bool sensor_cfg_one_step_unlocked(zb_device_t *d)
{
    if (!d || !d->used) {
        return false;
    }
    uint8_t our_eui[8];
    if (ezsp_get_eui64(our_eui) != ESP_OK) {
        return false;
    }
    uint8_t ep = d->sensor_ep ? d->sensor_ep : 1;
    /* Identity first — without model we cannot confirm clusters; one Basic read. */
    if (!d->model[0]) {
        uint16_t basic_attrs[] = {ZCL_ATTR_MANUFACTURER_NAME, ZCL_ATTR_MODEL_IDENTIFIER,
                                  ZCL_ATTR_APPLICATION_VERSION, ZCL_ATTR_SW_BUILD_ID};
        ESP_LOGI(TAG, "Sensor 0x%04X: Read Basic (no model yet)", d->node_id);
        return ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_BASIC, basic_attrs, 4) ==
               ESP_OK;
    }
    uint8_t step = d->sensor_cfg_step;
    /* After bind+cfg+first reads, rotate reads only (reporting should push updates). */
    if (d->sensor_reporting && step < SENSOR_STEP_READ_TEMP) {
        step = SENSOR_STEP_READ_TEMP;
        d->sensor_cfg_step = step;
    }
    if (step >= SENSOR_STEP_COUNT) {
        step = SENSOR_STEP_READ_TEMP;
        d->sensor_cfg_step = step;
    }

    uint16_t measured = ZCL_ATTR_MEASURED_VALUE;
    uint16_t batt = ZCL_ATTR_BATTERY_PCT_REMAINING;
    esp_err_t err = ESP_FAIL;

    switch (step) {
    case SENSOR_STEP_BIND_TEMP:
        ESP_LOGI(TAG, "Sensor 0x%04X step%u: Bind Temp ep%u", d->node_id, (unsigned)step,
                 (unsigned)ep);
        err = ezsp_zdo_bind(d->node_id, d->eui64, ep, ZCL_CLUSTER_TEMP_MEASUREMENT, our_eui, 1);
        break;
    case SENSOR_STEP_BIND_HUM:
        ESP_LOGI(TAG, "Sensor 0x%04X step%u: Bind Humidity ep%u", d->node_id, (unsigned)step,
                 (unsigned)ep);
        err = ezsp_zdo_bind(d->node_id, d->eui64, ep, ZCL_CLUSTER_REL_HUMIDITY, our_eui, 1);
        break;
    case SENSOR_STEP_BIND_BATT:
        ESP_LOGI(TAG, "Sensor 0x%04X step%u: Bind PowerCfg ep%u", d->node_id, (unsigned)step,
                 (unsigned)ep);
        err = ezsp_zdo_bind(d->node_id, d->eui64, ep, ZCL_CLUSTER_POWER_CONFIG, our_eui, 1);
        break;
    case SENSOR_STEP_CFG_TEMP: {
        /* Temp int16 (0x29). SNZB-02D resolution 0.2°C (=20). Min 10s, max 5 min. */
        uint8_t temp_chg[2] = {20, 0};
        ESP_LOGI(TAG, "Sensor 0x%04X step%u: CfgReport Temp", d->node_id, (unsigned)step);
        err = ezsp_zcl_configure_reporting(d->node_id, ep, ZCL_CLUSTER_TEMP_MEASUREMENT,
                                           ZCL_ATTR_MEASURED_VALUE, 0x29, 10, 300, temp_chg, 2);
        break;
    }
    case SENSOR_STEP_CFG_HUM: {
        uint8_t hum_chg[2] = {50, 0}; /* 0.5% */
        ESP_LOGI(TAG, "Sensor 0x%04X step%u: CfgReport Humidity", d->node_id, (unsigned)step);
        err = ezsp_zcl_configure_reporting(d->node_id, ep, ZCL_CLUSTER_REL_HUMIDITY,
                                           ZCL_ATTR_MEASURED_VALUE, 0x21, 10, 300, hum_chg, 2);
        break;
    }
    case SENSOR_STEP_CFG_BATT: {
        uint8_t batt_chg[1] = {2}; /* 1% */
        ESP_LOGI(TAG, "Sensor 0x%04X step%u: CfgReport Battery", d->node_id, (unsigned)step);
        err = ezsp_zcl_configure_reporting(d->node_id, ep, ZCL_CLUSTER_POWER_CONFIG,
                                           ZCL_ATTR_BATTERY_PCT_REMAINING, 0x20, 3600, 43200,
                                           batt_chg, 1);
        break;
    }
    case SENSOR_STEP_READ_TEMP:
        ESP_LOGI(TAG, "Sensor 0x%04X step%u: Read Temp", d->node_id, (unsigned)step);
        err = ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_TEMP_MEASUREMENT, &measured, 1);
        break;
    case SENSOR_STEP_READ_HUM:
        ESP_LOGI(TAG, "Sensor 0x%04X step%u: Read Humidity", d->node_id, (unsigned)step);
        err = ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_REL_HUMIDITY, &measured, 1);
        break;
    case SENSOR_STEP_READ_BATT:
        ESP_LOGI(TAG, "Sensor 0x%04X step%u: Read Battery", d->node_id, (unsigned)step);
        err = ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_POWER_CONFIG, &batt, 1);
        break;
    default:
        return false;
    }
    return err == ESP_OK;
}

/** Advance or retry after messageSent for the in-flight sensor frame. */
static void process_sensor_sent_events_unlocked(void)
{
    uint16_t node = 0;
    uint16_t cluster = 0;
    uint8_t st = 0;
    zb_device_t next_copy = {0};
    bool send_next = false;
    while (ezsp_take_sent_event(&node, &cluster, &st)) {
        if (!s_sensor_tx_pending || node != s_sensor_tx_node) {
            continue;
        }
        s_sensor_tx_pending = false;
        status_lock();
        zb_device_t *d = device_find_by_node_locked(node);
        if (d && d->used && device_looks_like_temp_sensor(d)) {
            if (st == EMBER_SUCCESS) {
                if (!d->model[0]) {
                    /* Identity Basic read — do not advance bind/cfg steps. */
                    ESP_LOGI(TAG, "Sensor 0x%04X Basic delivery ok — await model", node);
                } else if (d->sensor_cfg_step < SENSOR_STEP_COUNT - 1) {
                    d->sensor_cfg_step++;
                    ESP_LOGI(TAG, "Sensor 0x%04X delivery ok → step%u", node,
                             (unsigned)d->sensor_cfg_step);
                    /* Queue next setup frame immediately while the sleepy window is warm.
                     * After reporting is confirmed, stop the burst (reports will push). */
                    if (!d->sensor_reporting && d->sensor_cfg_step < SENSOR_STEP_READ_TEMP) {
                        next_copy = *d;
                        send_next = true;
                    }
                } else {
                    /* Rotate reads: temp → hum → batt → temp … */
                    d->sensor_cfg_step = SENSOR_STEP_READ_TEMP;
                    ESP_LOGI(TAG, "Sensor 0x%04X delivery ok → step%u", node,
                             (unsigned)d->sensor_cfg_step);
                }
            } else {
                ESP_LOGW(TAG, "Sensor 0x%04X delivery 0x%02X — keep step%u (retry next poll)",
                         node, st, (unsigned)d->sensor_cfg_step);
            }
        }
        status_unlock();
        (void)cluster;
    }
    /* Stale in-flight (NCP timed out without callback) — free the slot. */
    if (s_sensor_tx_pending && (now_ms() - s_sensor_tx_ms) > 70000) {
        ESP_LOGW(TAG, "Sensor 0x%04X tx slot timeout — retry", s_sensor_tx_node);
        s_sensor_tx_pending = false;
    }
    if (send_next) {
        sensor_try_send_one_unlocked(&next_copy);
    }
}

/** Send one sensor step if the slot is free. Updates live device step via eui. */
static void sensor_try_send_one_unlocked(zb_device_t *d)
{
    if (!d || !d->used || s_sensor_tx_pending) {
        return;
    }
    zb_device_t copy = *d;
    if (!sensor_cfg_one_step_unlocked(&copy)) {
        return;
    }
    s_sensor_tx_pending = true;
    s_sensor_tx_node = copy.node_id;
    s_sensor_tx_ms = now_ms();
    /* Mirror step on live record (copy may be stale pointer-wise). */
    status_lock();
    zb_device_t *live = device_find_by_eui_locked(copy.eui64);
    if (live) {
        live->sensor_cfg_step = copy.sensor_cfg_step;
        live->last_interview_ms = now_ms();
    }
    status_unlock();
}

static void mark_sensor_reporting_locked(zb_device_t *d)
{
    if (!d || d->sensor_reporting) {
        return;
    }
    d->sensor_reporting = true;
    ESP_LOGI(TAG, "Sensor reporting confirmed node=0x%04X", d->node_id);
}

/**
 * Arm multicast + Identify Time so we look like a light target for any classic F&B.
 * Note: IKEA's 10s hold uses Touchlink to physical bulbs — coordinators do not get that.
 */
static void arm_fake_bulb_unlocked(zb_device_t *d)
{
    (void)d;
    ezsp_join_ikea_multicast_groups();
    ezsp_light_start_identify(180);
    ESP_LOGI(TAG, "Fake bulb armed (ep%u) — mash remote buttons to bind OnOff here",
             (unsigned)ZB_FAKE_BULB_ENDPOINT);
}

/**
 * One ZDO bind while remote is awake. Keep group 901 (never unbind) — that groupcast
 * path is what previously delivered OnOff when the remote controlled a real device.
 */
static bool remote_bind_one_step_unlocked(zb_device_t *d)
{
    if (!d || !d->used || d->remote_bound) {
        return false;
    }
    if (device_looks_like_temp_sensor(d) ||
        zigbee_host_device_kind(d) == ZB_DEVICE_KIND_SENSOR) {
        return false;
    }
    if (s_remote_bind_step >= REMOTE_BIND_STEP_DONE) {
        return false;
    }
    uint8_t ep = d->remote_ep ? d->remote_ep : 1;
    uint8_t our_eui[8];
    if (ezsp_get_eui64(our_eui) != ESP_OK) {
        return false;
    }
    ezsp_join_ikea_multicast_groups();

    uint8_t step = s_remote_bind_step;
    switch (step) {
    case REMOTE_BIND_STEP_IEEE_ONOFF:
        ESP_LOGI(TAG, "Remote bind %u/5 0x%04X — OnOff → fake bulb ep%u (press again!)",
                 (unsigned)step + 1, d->node_id, (unsigned)ZB_FAKE_BULB_ENDPOINT);
        (void)ezsp_zdo_bind(d->node_id, d->eui64, ep, ZCL_CLUSTER_ON_OFF, our_eui,
                            ZB_FAKE_BULB_ENDPOINT);
        break;
    case REMOTE_BIND_STEP_IEEE_LEVEL:
        ESP_LOGI(TAG, "Remote bind %u/5 0x%04X — Level → fake bulb ep%u", (unsigned)step + 1,
                 d->node_id, (unsigned)ZB_FAKE_BULB_ENDPOINT);
        (void)ezsp_zdo_bind(d->node_id, d->eui64, ep, ZCL_CLUSTER_LEVEL_CONTROL, our_eui,
                            ZB_FAKE_BULB_ENDPOINT);
        break;
    case REMOTE_BIND_STEP_IEEE_SCENES:
        ESP_LOGI(TAG, "Remote bind %u/5 0x%04X — Scenes → fake bulb ep%u", (unsigned)step + 1,
                 d->node_id, (unsigned)ZB_FAKE_BULB_ENDPOINT);
        (void)ezsp_zdo_bind(d->node_id, d->eui64, ep, ZCL_CLUSTER_SCENES, our_eui,
                            ZB_FAKE_BULB_ENDPOINT);
        break;
    case REMOTE_BIND_STEP_GROUP_ONOFF:
        ESP_LOGI(TAG, "Remote bind %u/5 0x%04X — OnOff → group %u", (unsigned)step + 1,
                 d->node_id, (unsigned)ZB_IKEA_DEFAULT_BIND_GROUP);
        (void)ezsp_zdo_bind_group(d->node_id, d->eui64, ep, ZCL_CLUSTER_ON_OFF,
                                  ZB_IKEA_DEFAULT_BIND_GROUP, false);
        break;
    case REMOTE_BIND_STEP_GROUP_LEVEL:
        ESP_LOGI(TAG, "Remote bind %u/5 0x%04X — Level → group %u", (unsigned)step + 1,
                 d->node_id, (unsigned)ZB_IKEA_DEFAULT_BIND_GROUP);
        (void)ezsp_zdo_bind_group(d->node_id, d->eui64, ep, ZCL_CLUSTER_LEVEL_CONTROL,
                                  ZB_IKEA_DEFAULT_BIND_GROUP, false);
        break;
    default:
        return false;
    }
    s_remote_bind_step = (uint8_t)(step + 1);
    s_last_remote_bind_ms = now_ms();
    return s_remote_bind_step < REMOTE_BIND_STEP_DONE;
}

/** Drain binds only while remote is awake from Identify Query (F&B hold). */
static void __attribute__((unused)) finish_fake_bulb_pair_unlocked(zb_device_t *d)
{
    if (!d || !d->used) {
        return;
    }
    ESP_LOGI(TAG, "F&B awake 0x%04X — bind to fake bulb / group 901 (step %u)", d->node_id,
             (unsigned)s_remote_bind_step);
    for (int i = 0; i < 5 && remote_bind_one_step_unlocked(d); i++) {
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

/** Remove Touchlink group membership from lights/plugs so only the GW hears remotes
 *  (HomeKit button mode). CONTROL-mode targets stay in the group for native dimming. */
static void detach_lights_from_group_unlocked(uint16_t gid, uint16_t remote_node)
{
    if (gid == 0) {
        return;
    }
    static uint16_t s_detached_gids[4];
    static uint8_t s_detached_n;
    for (uint8_t i = 0; i < s_detached_n; i++) {
        if (s_detached_gids[i] == gid) {
            return; /* already stripped this boot */
        }
    }
    typedef struct {
        uint16_t node_id;
        uint8_t ep;
    } detach_t;
    detach_t detach[ZB_HOST_MAX_DEVICES];
    uint16_t ndetach = 0;
    status_lock();
    for (uint16_t di = 0; di < ZB_HOST_MAX_DEVICES; di++) {
        zb_device_t *ld = &s_status.devices[di];
        if (!ld->used || ld->node_id == 0 || ld->node_id == remote_node) {
            continue;
        }
        if (zigbee_host_device_kind(ld) == ZB_DEVICE_KIND_REMOTE ||
            eui_looks_like_ikea_remote(ld->eui64)) {
            continue;
        }
        zb_device_kind_t lk = zigbee_host_device_kind(ld);
        if (lk != ZB_DEVICE_KIND_LIGHT && lk != ZB_DEVICE_KIND_SWITCH && !ld->has_onoff &&
            !ld->has_level) {
            continue;
        }
        /* Keep CONTROL targets — they need native Level groupcasts for dimming. */
        bool is_ctrl_target = false;
        for (uint16_t ri = 0; ri < ZB_HOST_MAX_DEVICES; ri++) {
            zb_device_t *rd = &s_status.devices[ri];
            if (!rd->used || rd->remote_type != ZB_REMOTE_TYPE_CONTROL) {
                continue;
            }
            migrate_remote_targets_locked(rd);
            if (rd->remote_group_id != 0 && rd->remote_group_id != gid) {
                continue;
            }
            uint8_t tn = rd->target_count;
            if (tn == 0 && !eui_is_zero(rd->target_eui64)) {
                tn = 1;
            }
            for (uint8_t ti = 0; ti < tn && ti < ZB_REMOTE_MAX_TARGETS; ti++) {
                const uint8_t *eui = (rd->target_count > 0) ? rd->targets[ti] : rd->target_eui64;
                if (memcmp(eui, ld->eui64, 8) == 0) {
                    is_ctrl_target = true;
                    break;
                }
            }
            if (is_ctrl_target) {
                break;
            }
        }
        if (is_ctrl_target) {
            continue;
        }
        detach[ndetach].node_id = ld->node_id;
        detach[ndetach].ep = ld->onoff_ep ? ld->onoff_ep : (ld->level_ep ? ld->level_ep : 1);
        ndetach++;
    }
    status_unlock();
    for (uint16_t di = 0; di < ndetach; di++) {
        ESP_LOGI(TAG, "Detach 0x%04X from Touchlink group %u (HK remotes)", detach[di].node_id,
                 (unsigned)gid);
        (void)ezsp_zcl_groups_remove_group(detach[di].node_id, detach[di].ep, gid);
        if (detach[di].ep != 1) {
            (void)ezsp_zcl_groups_remove_group(detach[di].node_id, 1, gid);
        }
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    if (s_detached_n < 4) {
        s_detached_gids[s_detached_n++] = gid;
    }
}

/**
 * Map ZCL commands that hit our fake bulb onto HomeKit programmable-switch indices.
 * Layout by button count:
 *   5: Power, Dimmer, Brighter, Left, Right  (E1810)
 *   4: On, Off, Left, Right                  (STYRBAR)
 *   2: On, Off
 *   1: Button
 */
static void handle_remote_cluster_cmd(zb_device_t *d, uint16_t cluster, uint8_t cmd,
                                      uint16_t mfg_code, const uint8_t *payload, size_t plen,
                                      uint8_t source_ep)
{
    if (!d) {
        return;
    }
    if (source_ep) {
        d->remote_ep = source_ep;
    }
    uint8_t nbtn = zigbee_host_remote_button_count(d);
    if (nbtn == 0) {
        return;
    }

    if (cluster == ZCL_CLUSTER_ON_OFF) {
        mark_remote_bound_locked(d);
        ESP_LOGI(TAG, "Remote 0x%04X OnOff cmd=0x%02X → HK button", d->node_id, cmd);
        if (d->remote_type == ZB_REMOTE_TYPE_CONTROL) {
            /* 4/2-btn: On/Off are separate. 5-btn power is usually Toggle. */
            if (cmd == ZCL_CMD_OFF && (nbtn == 4 || nbtn == 2)) {
                emit_remote_button(d, 1, HK_BTN_EVENT_SINGLE);
            } else if (cmd == ZCL_CMD_ON && (nbtn == 4 || nbtn == 2)) {
                emit_remote_button(d, 0, HK_BTN_EVENT_SINGLE);
            } else {
                emit_remote_button(d, 0, HK_BTN_EVENT_SINGLE); /* Toggle / 5-btn power */
            }
            return;
        }
        if (cmd == ZCL_CMD_TOGGLE) {
            emit_remote_button(d, 0, HK_BTN_EVENT_SINGLE);
        } else if (cmd == ZCL_CMD_ON) {
            emit_remote_button(d, 0, HK_BTN_EVENT_SINGLE);
        } else if (cmd == ZCL_CMD_OFF) {
            /* 5-btn IKEA power is one button — map Off to btn0 (same as On/Toggle)
             * so synth Toggle + real Off debounce together instead of double-latch. */
            uint8_t idx = 0;
            if (nbtn == 4 || nbtn == 2) {
                idx = 1;
            }
            emit_remote_button(d, idx, HK_BTN_EVENT_SINGLE);
        }
        return;
    }

    if (cluster == ZCL_CLUSTER_LEVEL_CONTROL) {
        uint8_t dir = (plen >= 1) ? payload[0] : ZCL_LEVEL_DIR_UP;
        bool hold = (cmd == ZCL_CMD_MOVE || cmd == ZCL_CMD_MOVE_WITH_ON_OFF);
        bool step = (cmd == ZCL_CMD_STEP || cmd == ZCL_CMD_STEP_WITH_ON_OFF);
        bool stop = (cmd == ZCL_CMD_STOP || cmd == ZCL_CMD_STOP_WITH_ON_OFF);
        ESP_LOGI(TAG, "Remote 0x%04X Level cmd=0x%02X dir=%u hold=%d step=%d", d->node_id, cmd,
                 (unsigned)dir, (int)hold, (int)step);
        if (stop) {
            return;
        }
        if (!hold && !step) {
            return;
        }
        mark_remote_bound_locked(d);
        if (d->remote_type == ZB_REMOTE_TYPE_CONTROL) {
            /* Native groupcast dims targets; only log for button test UI. */
            uint8_t bri_btn = (dir == ZCL_LEVEL_DIR_DOWN) ? 1 : 2;
            if (nbtn == 4) {
                bri_btn = (dir == ZCL_LEVEL_DIR_DOWN) ? 1 : 0;
            }
            press_log_push(d->eui64, (uint8_t)(bri_btn + 1),
                           hold ? HK_BTN_EVENT_LONG : HK_BTN_EVENT_SINGLE);
            return;
        }
        uint8_t event = hold ? HK_BTN_EVENT_LONG : HK_BTN_EVENT_SINGLE;
        if (nbtn >= 5) {
            /* idx1=Dimmer (down), idx2=Brighter (up) */
            emit_remote_button(d, (dir == ZCL_LEVEL_DIR_DOWN) ? 1 : 2, event);
        } else if (nbtn == 4) {
            emit_remote_button(d, (dir == ZCL_LEVEL_DIR_DOWN) ? 1 : 0, event);
        } else {
            emit_remote_button(d, 0, event);
        }
        return;
    }

    bool ikea_arrow = (mfg_code == ZCL_IKEA_MFG_CODE) || (cluster == ZCL_CLUSTER_IKEA_BUTTON) ||
                      (cluster == ZCL_CLUSTER_SCENES &&
                       (cmd == ZCL_IKEA_CMD_ARROW_CLICK || cmd == ZCL_IKEA_CMD_ARROW_HOLD ||
                        cmd == ZCL_IKEA_CMD_ARROW_RELEASE));
    if (ikea_arrow) {
        ESP_LOGI(TAG, "Remote 0x%04X arrow/Scenes cmd=0x%02X mfg=0x%04X", d->node_id, cmd,
                 mfg_code);
        if (cmd == ZCL_IKEA_CMD_ARROW_RELEASE) {
            return;
        }
        mark_remote_bound_locked(d);
        uint8_t arrow = (plen >= 1) ? payload[0] : ZCL_IKEA_ARROW_LEFT;
        uint8_t event =
            (cmd == ZCL_IKEA_CMD_ARROW_HOLD) ? HK_BTN_EVENT_LONG : HK_BTN_EVENT_SINGLE;
        uint8_t left_idx = (nbtn >= 5) ? 3 : ((nbtn >= 4) ? 2 : 0);
        uint8_t right_idx = (nbtn >= 5) ? 4 : ((nbtn >= 4) ? 3 : 0);
        emit_remote_button(d, (arrow == ZCL_IKEA_ARROW_LEFT) ? left_idx : right_idx, event);
        return;
    }

    if (cluster == ZCL_CLUSTER_SCENES && cmd == ZCL_CMD_RECALL_SCENE) {
        mark_remote_bound_locked(d);
        uint8_t scene = (plen >= 3) ? payload[2] : 0;
        if (nbtn >= 4) {
            emit_remote_button(d, (scene & 1) ? ((nbtn >= 5) ? 4 : 3) : ((nbtn >= 5) ? 3 : 2),
                               HK_BTN_EVENT_SINGLE);
        } else {
            emit_remote_button(d, 0, HK_BTN_EVENT_SINGLE);
        }
    }
}

uint8_t zigbee_host_level_to_brightness(uint8_t level)
{
    if (level >= 254) {
        return 100;
    }
    return (uint8_t)((level * 100u + 127u) / 254u);
}

uint8_t zigbee_host_brightness_to_level(uint8_t brightness_pct)
{
    if (brightness_pct >= 100) {
        return 254;
    }
    if (brightness_pct == 0) {
        return 0;
    }
    return (uint8_t)((brightness_pct * 254u + 50u) / 100u);
}

static void device_upsert_locked(const uint8_t eui64[8], uint16_t node_id, uint8_t node_type)
{
    if (!eui64) {
        return;
    }
    bool blank = true;
    for (int i = 0; i < 8; i++) {
        if (eui64[i]) {
            blank = false;
            break;
        }
    }
    if (blank || node_id == 0 || node_id == 0xFFFF) {
        return;
    }

    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        if (s_status.devices[i].used && memcmp(s_status.devices[i].eui64, eui64, 8) == 0) {
            s_status.devices[i].node_id = node_id;
            s_status.devices[i].node_type = node_type;
            s_status.devices[i].last_seen_ms = now_ms();
            device_refresh_label(&s_status.devices[i]);
            return;
        }
    }
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        if (!s_status.devices[i].used) {
            zb_device_t *d = &s_status.devices[i];
            memset(d, 0, sizeof(*d));
            d->used = true;
            memcpy(d->eui64, eui64, 8);
            d->node_id = node_id;
            d->node_type = node_type;
            d->homekit_expose = true; /* opt-in by default for sensors */
            d->last_seen_ms = now_ms();
            if (eui_looks_like_ikea_remote(eui64) &&
                (node_type == EMBER_SLEEPY_END_DEVICE || node_type == EMBER_END_DEVICE)) {
                snprintf(d->name, sizeof(d->name), "TRADFRI remote");
                snprintf(d->manufacturer, sizeof(d->manufacturer), "IKEA of Sweden");
            }
            device_refresh_label(d);
            char eui[32];
            ezsp_format_eui64(eui64, eui, sizeof(eui));
            ESP_LOGI(TAG, "Device registered: %s (0x%04X)", eui, node_id);
            return;
        }
    }
}

static void device_remove_locked(const uint8_t eui64[8])
{
    if (!eui64) {
        return;
    }
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        if (s_status.devices[i].used && memcmp(s_status.devices[i].eui64, eui64, 8) == 0) {
            memset(&s_status.devices[i], 0, sizeof(s_status.devices[i]));
            if (s_status.device_count > 0) {
                s_status.device_count--;
            }
            return;
        }
    }
}

/** Pack used devices into scratch (caller holds status_lock). */
static uint16_t nvs_pack_devices_locked(void)
{
    if (!s_nvs_scratch) {
        return 0;
    }
    uint16_t n = 0;
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        if (s_status.devices[i].used) {
            s_nvs_scratch[n++] = s_status.devices[i];
        }
    }
    return n;
}

static zb_device_t *nvs_scratch_acquire(void)
{
    if (!s_nvs_scratch) {
        s_nvs_scratch = calloc(ZB_HOST_MAX_DEVICES, sizeof(zb_device_t));
        if (!s_nvs_scratch) {
            ESP_LOGE(TAG, "NVS scratch alloc failed (%u bytes)",
                     (unsigned)(ZB_HOST_MAX_DEVICES * sizeof(zb_device_t)));
        }
    }
    return s_nvs_scratch;
}

static void nvs_scratch_release(void)
{
    free(s_nvs_scratch);
    s_nvs_scratch = NULL;
}

/** Drop legacy device blobs that fragment the 24 KiB NVS partition. */
static void nvs_erase_legacy_device_keys(nvs_handle_t h)
{
    nvs_erase_key(h, "devs4");
    nvs_erase_key(h, "devs3");
    nvs_erase_key(h, "devs2");
    nvs_erase_key(h, "devs");
}

/** Flash write from scratch — must NOT hold status_lock (blocks web/HAP). */
static void nvs_commit_scratch(uint16_t n)
{
    if (!s_nvs_scratch) {
        ESP_LOGE(TAG, "NVS commit without scratch");
        s_devices_nvs_dirty = true;
        return;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed — device table not saved");
        s_devices_nvs_dirty = true;
        return;
    }

    /* Always drop prior dens5 first — 24 KiB NVS cannot hold two 4 KiB copies, and the
     * erase+retry path was hammering flash (unicore silent STA). */
    nvs_erase_key(h, "devs5");
    nvs_erase_legacy_device_keys(h);
    (void)nvs_commit(h);

    esp_err_t err = nvs_set_blob(h, "devs5", s_nvs_scratch, n * sizeof(zb_device_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS set_blob(devs5) failed: %s (n=%u bytes=%u)", esp_err_to_name(err),
                 (unsigned)n, (unsigned)(n * sizeof(zb_device_t)));
        nvs_close(h);
        s_devices_nvs_dirty = true;
        return;
    }
    nvs_set_u16(h, "dev_n", n);
    err = nvs_commit(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
        nvs_close(h);
        s_devices_nvs_dirty = true;
        return;
    }
    nvs_close(h);
    ESP_LOGI(TAG, "Saved %u device(s) to NVS (name0='%s' hk0=%d)", (unsigned)n,
             n ? s_nvs_scratch[0].name : "", n ? (int)s_nvs_scratch[0].homekit_expose : 0);
}

static void nvs_save_devices_locked(void)
{
    /* Rare paths (device edit / join). Button latch uses dirty + unlocked flush. */
    if (!nvs_scratch_acquire()) {
        s_devices_nvs_dirty = true;
        return;
    }
    uint16_t n = nvs_pack_devices_locked();
    if (n == 0) {
        /* Never wipe a populated table with an empty pack (e.g. save-before-load race). */
        ESP_LOGW(TAG, "Skip NVS device save — inventory empty");
        nvs_scratch_release();
        return;
    }
    s_devices_nvs_dirty = false;
    nvs_commit_scratch(n);
    nvs_scratch_release();
}

/** Deferred flush — must not spam flash on unicore (NVS erase starves Wi‑Fi/httpd). */
static void nvs_flush_devices_if_dirty(bool force)
{
    if (!s_devices_nvs_dirty) {
        return;
    }
    int64_t now = now_ms();
    /* Skip during bring-up — EZSP + NVS together kill STA. */
    if (now < 30000) {
        return;
    }
    /* Erase+rewrite dens5 stalls Wi‑Fi on C3 unicore — coalesce aggressively. */
    if (!force && (now - s_devices_nvs_last_flush_ms) < 60000) {
        return;
    }
    /* Serialize with device-update / host EZSP paths that also touch scratch. */
    if (xSemaphoreTake(s_op_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    if (!s_devices_nvs_dirty) {
        xSemaphoreGive(s_op_mutex);
        return;
    }
    status_lock();
    if (!nvs_scratch_acquire()) {
        status_unlock();
        xSemaphoreGive(s_op_mutex);
        return;
    }
    uint16_t n = nvs_pack_devices_locked();
    if (n == 0) {
        status_unlock();
        nvs_scratch_release();
        xSemaphoreGive(s_op_mutex);
        ESP_LOGW(TAG, "Skip deferred NVS device flush — inventory empty");
        return;
    }
    s_devices_nvs_dirty = false;
    status_unlock();
    nvs_commit_scratch(n);
    nvs_scratch_release();
    s_devices_nvs_last_flush_ms = now_ms();
    xSemaphoreGive(s_op_mutex);
    /* Let Wi-Fi/lwIP run after flash cache was disabled. */
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void nvs_load_devices_locked(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }

    memset(s_status.devices, 0, sizeof(s_status.devices));
    uint16_t n = 0;
    bool loaded = false;
    if (nvs_get_u16(h, "dev_n", &n) == ESP_OK && n > 0 && n <= ZB_HOST_MAX_DEVICES) {
        size_t sz = 0;
        if (nvs_get_blob(h, "devs5", NULL, &sz) == ESP_OK && sz > 0 && (sz % n) == 0) {
            size_t old_sz = sz / n;
            uint8_t *raw = calloc(1, sz);
            if (raw && nvs_get_blob(h, "devs5", raw, &sz) == ESP_OK) {
                size_t copy = old_sz < sizeof(zb_device_t) ? old_sz : sizeof(zb_device_t);
                for (uint16_t i = 0; i < n; i++) {
                    memset(&s_status.devices[i], 0, sizeof(zb_device_t));
                    memcpy(&s_status.devices[i], raw + (size_t)i * old_sz, copy);
                    s_status.devices[i].used = true;
                }
                loaded = true;
                ESP_LOGI(TAG, "Loaded %u device(s) from NVS (rec=%uB cur=%uB name0='%s' hk0=%d)",
                         (unsigned)n, (unsigned)old_sz, (unsigned)sizeof(zb_device_t),
                         s_status.devices[0].name, (int)s_status.devices[0].homekit_expose);
            }
            free(raw);
        }
    }

    if (!loaded) {
        /* Recover a single known device from older full-table blobs if present. */
        size_t full = sizeof(s_status.devices);
        zb_device_t *s_full = nvs_scratch_acquire();
        if (!s_full) {
            nvs_close(h);
            return;
        }
        const char *keys[] = {"devs4", "devs3", "devs2"};
        for (size_t k = 0; k < sizeof(keys) / sizeof(keys[0]) && !loaded; k++) {
            size_t sz = full;
            memset(s_full, 0, sizeof(zb_device_t) * ZB_HOST_MAX_DEVICES);
            if (nvs_get_blob(h, keys[k], s_full, &sz) != ESP_OK) {
                continue;
            }
            uint16_t count = 0;
            memset(s_status.devices, 0, sizeof(s_status.devices));
            for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
                if (!s_full[i].used) {
                    continue;
                }
                s_status.devices[count++] = s_full[i];
            }
            if (count) {
                loaded = true;
                ESP_LOGI(TAG, "Migrated %u device(s) from %s", (unsigned)count, keys[k]);
            }
        }
        nvs_scratch_release();
    }

    uint16_t count = 0;
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        if (s_status.devices[i].used) {
            /* Force re-confirm reporting after boot — older firmware set this
             * optimistically without waiting for CfgReportRsp / Attribute Report.
             * last_interview_ms is tick-based (not wall clock), so reset it too. */
            if (device_looks_like_temp_sensor(&s_status.devices[i])) {
                s_status.devices[i].sensor_reporting = false;
                s_status.devices[i].sensor_cfg_step = 0;
                s_status.devices[i].last_interview_ms = 0;
            }
            migrate_remote_targets_locked(&s_status.devices[i]);
            count++;
        }
    }
    s_status.device_count = count;
    nvs_close(h);
    migrate_e1810_btn_order_locked();
}

static zb_device_t *device_find_by_node_locked(uint16_t node_id)
{
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        if (s_status.devices[i].used && s_status.devices[i].node_id == node_id) {
            return &s_status.devices[i];
        }
    }
    return NULL;
}

static zb_device_t *device_find_by_eui_locked(const uint8_t eui64[8])
{
    if (!eui64) {
        return NULL;
    }
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        if (s_status.devices[i].used && memcmp(s_status.devices[i].eui64, eui64, 8) == 0) {
            return &s_status.devices[i];
        }
    }
    return NULL;
}

static void zcl_copy_string(char *dst, size_t dst_len, const uint8_t *src, uint8_t slen)
{
    if (!dst || dst_len == 0) {
        return;
    }
    size_t n = slen;
    if (n >= dst_len) {
        n = dst_len - 1;
    }
    memcpy(dst, src, n);
    dst[n] = '\0';
    /* Strip non-printable */
    for (size_t i = 0; i < n; i++) {
        if ((unsigned char)dst[i] < 0x20 || (unsigned char)dst[i] > 0x7E) {
            dst[i] = '?';
        }
    }
}

static size_t zcl_skip_value(uint8_t dtype, const uint8_t *p, size_t remain)
{
    switch (dtype) {
    case 0x08: /* data8 */
    case 0x10: /* bool */
    case 0x18: /* map8 */
    case 0x20: /* uint8 */
    case 0x28: /* int8 */
    case 0x30: /* enum8 */
        return remain >= 1 ? 1 : remain;
    case 0x09:
    case 0x19:
    case 0x21:
    case 0x29:
    case 0x31:
        return remain >= 2 ? 2 : remain;
    case 0x0A:
    case 0x1A:
    case 0x22:
    case 0x2A:
        return remain >= 3 ? 3 : remain;
    case 0x0B:
    case 0x1B:
    case 0x23:
    case 0x2B:
    case 0x39: /* single */
        return remain >= 4 ? 4 : remain;
    case 0x0C:
    case 0x1C:
    case 0x24:
    case 0x2C:
        return remain >= 5 ? 5 : remain;
    case 0x0D:
    case 0x1D:
    case 0x25:
    case 0x2D:
        return remain >= 6 ? 6 : remain;
    case 0x0E:
    case 0x1E:
    case 0x26:
    case 0x2E:
        return remain >= 7 ? 7 : remain;
    case 0x0F:
    case 0x1F:
    case 0x27:
    case 0x2F:
    case 0x3A: /* double */
        return remain >= 8 ? 8 : remain;
    case 0x41: /* octet string */
    case 0x42: /* char string */
        if (remain < 1) {
            return remain;
        }
        return 1u + (size_t)p[0] <= remain ? 1u + (size_t)p[0] : remain;
    default:
        return remain;
    }
}

static bool apply_zcl_attr_locked(zb_device_t *d, uint16_t cluster, uint16_t attr, uint8_t dtype,
                                  const uint8_t *val, size_t vlen, uint8_t source_ep)
{
    if (!d || !val) {
        return false;
    }
    bool changed = false;
    if (cluster == ZCL_CLUSTER_BASIC) {
        if (attr == ZCL_ATTR_MANUFACTURER_NAME && dtype == 0x42 && vlen >= 1) {
            char tmp[sizeof(d->manufacturer)];
            zcl_copy_string(tmp, sizeof(tmp), val + 1, val[0]);
            if (strcmp(tmp, d->manufacturer) != 0) {
                snprintf(d->manufacturer, sizeof(d->manufacturer), "%s", tmp);
                changed = true;
            }
        } else if (attr == ZCL_ATTR_MODEL_IDENTIFIER && dtype == 0x42 && vlen >= 1) {
            char tmp[sizeof(d->model)];
            zcl_copy_string(tmp, sizeof(tmp), val + 1, val[0]);
            if (strcmp(tmp, d->model) != 0) {
                snprintf(d->model, sizeof(d->model), "%s", tmp);
                changed = true;
            }
            if (!d->name[0] && d->model[0]) {
                char ntmp[sizeof(d->name)];
                snprintf(ntmp, sizeof(ntmp), "%s", d->model);
                snprintf(d->name, sizeof(d->name), "%s", ntmp);
                changed = true;
            }
        } else if (attr == ZCL_ATTR_SW_BUILD_ID && dtype == 0x42 && vlen >= 1) {
            char fw[sizeof(d->firmware)];
            zcl_copy_string(fw, sizeof(fw), val + 1, val[0]);
            /* Ignore build strings that are just the model name echoed back. */
            if (fw[0] && !(d->model[0] && strcmp(fw, d->model) == 0) &&
                strcmp(fw, d->firmware) != 0) {
                snprintf(d->firmware, sizeof(d->firmware), "%s", fw);
                changed = true;
            }
        } else if (attr == ZCL_ATTR_APPLICATION_VERSION && dtype == 0x20 && vlen >= 1) {
            if (!d->firmware[0] || (d->model[0] && strcmp(d->firmware, d->model) == 0)) {
                char fw[16];
                snprintf(fw, sizeof(fw), "%u", (unsigned)val[0]);
                if (strcmp(fw, d->firmware) != 0) {
                    snprintf(d->firmware, sizeof(d->firmware), "%s", fw);
                    changed = true;
                }
            }
        }
        if (changed) {
            device_refresh_label(d);
            /* Identity must survive reboot — temp reports are RAM-only. */
            s_devices_nvs_dirty = true;
        }
    } else if (cluster == ZCL_CLUSTER_TEMP_MEASUREMENT && attr == ZCL_ATTR_MEASURED_VALUE &&
               vlen >= 2 && (dtype == 0x29 || dtype == 0x21)) {
        int16_t raw = (int16_t)((uint16_t)val[0] | ((uint16_t)val[1] << 8));
        if (raw != (int16_t)0x8000) {
            float t = (float)raw / 100.0f;
            if (!d->has_temp || d->temperature_c != t) {
                d->temperature_c = t;
                d->has_temp = true;
                changed = true;
                sensor_upd_q_push(d->eui64);
            }
            if (source_ep && d->sensor_ep != source_ep) {
                d->sensor_ep = source_ep;
                changed = true;
            }
        }
    } else if (cluster == ZCL_CLUSTER_REL_HUMIDITY && attr == ZCL_ATTR_MEASURED_VALUE &&
               vlen >= 2 && (dtype == 0x21 || dtype == 0x29)) {
        uint16_t raw = (uint16_t)val[0] | ((uint16_t)val[1] << 8);
        if (raw != 0xFFFF && raw != 0x8000) {
            float h = (float)raw / 100.0f;
            if (h > 100.0f) {
                h = 100.0f;
            }
            if (!d->has_humidity || d->humidity_pct != h) {
                d->humidity_pct = h;
                d->has_humidity = true;
                changed = true;
                sensor_upd_q_push(d->eui64);
            }
            if (source_ep && d->sensor_ep != source_ep) {
                d->sensor_ep = source_ep;
                changed = true;
            }
        }
    } else if (cluster == ZCL_CLUSTER_POWER_CONFIG && attr == ZCL_ATTR_BATTERY_PCT_REMAINING &&
               vlen >= 1 && dtype == 0x20) {
        /* Zigbee: units of 0.5%. Clamp to 100. */
        uint8_t pct = (uint8_t)(val[0] / 2);
        if (pct > 100) {
            pct = 100;
        }
        if (!d->has_battery || d->battery_pct != pct) {
            d->battery_pct = pct;
            d->has_battery = true;
            changed = true;
            sensor_upd_q_push(d->eui64);
        }
    } else if (cluster == ZCL_CLUSTER_ON_OFF && attr == ZCL_ATTR_ON_OFF && vlen >= 1 &&
               (dtype == 0x10 || dtype == 0x20)) {
        bool on = (val[0] != 0);
        if (!d->has_onoff || d->onoff_on != on) {
            d->onoff_on = on;
            d->has_onoff = true;
            changed = true;
            refresh_binary_on_locked(d);
            sensor_upd_q_push(d->eui64);
        }
        if (source_ep && d->onoff_ep != source_ep) {
            d->onoff_ep = source_ep;
            changed = true;
        }
    } else if (cluster == ZCL_CLUSTER_LEVEL_CONTROL && attr == ZCL_ATTR_CURRENT_LEVEL &&
               vlen >= 1 && dtype == 0x20) {
        uint8_t lvl = val[0];
        if (!d->has_level || d->level != lvl) {
            d->level = lvl;
            d->has_level = true;
            changed = true;
        }
        if (source_ep && d->level_ep != source_ep) {
            d->level_ep = source_ep;
            changed = true;
        }
        if (!d->onoff_ep && source_ep) {
            d->onoff_ep = source_ep;
        }
    } else if (cluster == ZCL_CLUSTER_IAS_ZONE && attr == ZCL_ATTR_IAS_ZONE_TYPE && vlen >= 2 &&
               (dtype == 0x31 || dtype == 0x21)) {
        uint16_t zt = (uint16_t)val[0] | ((uint16_t)val[1] << 8);
        if (!d->has_ias_zone || d->ias_zone_type != zt) {
            d->ias_zone_type = zt;
            d->has_ias_zone = true;
            changed = true;
            s_devices_nvs_dirty = true;
        }
        if (source_ep && d->ias_zone_ep != source_ep) {
            d->ias_zone_ep = source_ep;
            changed = true;
        }
    } else if (cluster == ZCL_CLUSTER_IAS_ZONE && attr == ZCL_ATTR_IAS_ZONE_STATUS && vlen >= 2 &&
               (dtype == 0x19 || dtype == 0x21)) {
        uint16_t zs = (uint16_t)val[0] | ((uint16_t)val[1] << 8);
        if (!d->has_ias_zone || d->ias_zone_status != zs) {
            d->ias_zone_status = zs;
            d->has_ias_zone = true;
            refresh_binary_on_locked(d);
            changed = true;
            sensor_upd_q_push(d->eui64);
        }
        if (source_ep && d->ias_zone_ep != source_ep) {
            d->ias_zone_ep = source_ep;
            changed = true;
        }
    } else if (cluster == ZCL_CLUSTER_OCCUPANCY && attr == ZCL_ATTR_OCCUPANCY && vlen >= 1 &&
               (dtype == 0x18 || dtype == 0x20)) {
        bool occ = (val[0] & 0x01) != 0;
        if (!d->has_occupancy || d->occupancy != occ) {
            d->occupancy = occ;
            d->has_occupancy = true;
            refresh_binary_on_locked(d);
            changed = true;
            sensor_upd_q_push(d->eui64);
        }
        if (source_ep && d->occupancy_ep != source_ep) {
            d->occupancy_ep = source_ep;
            changed = true;
        }
    }
    return changed;
}

static bool parse_zcl_payload_locked(zb_device_t *d, uint16_t cluster, const uint8_t *zcl,
                                     size_t len, uint8_t source_ep, uint8_t dest_ep)
{
    if (!d || !zcl || len < 3) {
        return false;
    }
    uint8_t fc = zcl[0];
    bool manuf = (fc & 0x04) != 0;
    bool cluster_specific = (fc & 0x01) != 0;
    size_t off = 1;
    uint16_t mfg_code = 0;
    if (manuf) {
        if (len < 5) {
            return false;
        }
        mfg_code = (uint16_t)zcl[off] | ((uint16_t)zcl[off + 1] << 8);
        off += 2;
    }
    if (off + 1 >= len) {
        return false;
    }
    uint8_t zcl_seq = zcl[off++];
    uint8_t cmd = zcl[off++];

    /* Cluster-specific commands from remotes (On/Off, Level, Scenes/IKEA arrows). */
    if (cluster_specific) {
        if (cluster == ZCL_CLUSTER_IAS_ZONE) {
            if (cmd == ZCL_CMD_IAS_ZONE_STATUS_CHANGE && off + 2 <= len) {
                uint16_t zs = (uint16_t)zcl[off] | ((uint16_t)zcl[off + 1] << 8);
                if (!d->has_ias_zone || d->ias_zone_status != zs) {
                    d->ias_zone_status = zs;
                    d->has_ias_zone = true;
                    if (source_ep) {
                        d->ias_zone_ep = source_ep;
                    }
                    refresh_binary_on_locked(d);
                    sensor_upd_q_push(d->eui64);
                    ESP_LOGI(TAG, "IAS ZoneStatus 0x%04X node=0x%04X alarm=%d", zs, d->node_id,
                             (int)d->binary_on);
                    return true;
                }
                return false;
            }
            if (cmd == ZCL_CMD_IAS_ZONE_ENROLL_REQ && off + 2 <= len) {
                uint16_t zt = (uint16_t)zcl[off] | ((uint16_t)zcl[off + 1] << 8);
                d->ias_zone_type = zt;
                d->has_ias_zone = true;
                if (source_ep) {
                    d->ias_zone_ep = source_ep;
                }
                s_devices_nvs_dirty = true;
                ESP_LOGI(TAG, "IAS ZoneEnrollReq node=0x%04X type=0x%04X — accepting", d->node_id,
                         zt);
                (void)ezsp_zcl_ias_zone_enroll_response(d->node_id, source_ep ? source_ep : 1, 1,
                                                        zcl_seq, 0 /* success */, 1);
                return true;
            }
            return false;
        }
        if (cluster == ZCL_CLUSTER_ON_OFF || cluster == ZCL_CLUSTER_LEVEL_CONTROL ||
            cluster == ZCL_CLUSTER_SCENES || cluster == ZCL_CLUSTER_IKEA_BUTTON) {
            bool known_remote = zigbee_host_device_kind(d) == ZB_DEVICE_KIND_REMOTE ||
                                model_looks_like_remote(d->model) ||
                                model_looks_like_remote(d->name) ||
                                eui_looks_like_ikea_remote(d->eui64);
            /* Fake bulb / broadcast / groupcast destinations. */
            bool to_light = (dest_ep == ZB_FAKE_BULB_ENDPOINT || dest_ep == 10 || dest_ep == 1 ||
                             dest_ep == 0xFF || dest_ep == 0);
            bool sleepy = d->node_type == EMBER_SLEEPY_END_DEVICE ||
                          d->node_type == EMBER_END_DEVICE;
            if (known_remote || (to_light && sleepy) || d->remote_bound) {
                bool was_bound = d->remote_bound;
                handle_remote_cluster_cmd(d, cluster, cmd, mfg_code, zcl + off, len - off,
                                          source_ep);
                return !was_bound && d->remote_bound; /* persist once bind is confirmed */
            }
        }
        return false;
    }

    /* 0x07 Configure Reporting Response — confirm bind/cfg stuck. */
    if (cmd == 0x07) {
        bool ok = false;
        bool any = false;
        while (off + 4 <= len) {
            uint8_t status = zcl[off++];
            uint8_t direction = zcl[off++];
            uint16_t attr = (uint16_t)zcl[off] | ((uint16_t)zcl[off + 1] << 8);
            off += 2;
            any = true;
            ESP_LOGI(TAG, "CfgReportRsp 0x%04X cluster=0x%04X attr=0x%04X dir=%u status=0x%02X",
                     d->node_id, cluster, attr, (unsigned)direction, status);
            if (status == 0x00 &&
                (cluster == ZCL_CLUSTER_TEMP_MEASUREMENT || cluster == ZCL_CLUSTER_REL_HUMIDITY ||
                 cluster == ZCL_CLUSTER_POWER_CONFIG)) {
                ok = true;
            }
        }
        /* Empty success payload = all records OK (ZCL). */
        if (!any || ok) {
            mark_sensor_reporting_locked(d);
            return true;
        }
        return false;
    }

    /* 0x01 Read Attributes Response; 0x0A Report Attributes */
    if (cmd != 0x01 && cmd != 0x0A) {
        return false;
    }
    bool changed = false;
    bool got_sensor_report = false;
    while (off < len) {
        if (off + 2 > len) {
            break;
        }
        uint16_t attr = (uint16_t)zcl[off] | ((uint16_t)zcl[off + 1] << 8);
        off += 2;
        if (cmd == 0x01) {
            if (off >= len) {
                break;
            }
            uint8_t status = zcl[off++];
            if (status != 0x00) {
                continue;
            }
        }
        if (off >= len) {
            break;
        }
        uint8_t dtype = zcl[off++];
        size_t remain = len - off;
        size_t vlen = zcl_skip_value(dtype, zcl + off, remain);
        if (vlen > remain) {
            break;
        }
        if (apply_zcl_attr_locked(d, cluster, attr, dtype, zcl + off, vlen, source_ep)) {
            changed = true;
        }
        if (cmd == 0x0A &&
            ((cluster == ZCL_CLUSTER_TEMP_MEASUREMENT || cluster == ZCL_CLUSTER_REL_HUMIDITY) &&
             attr == ZCL_ATTR_MEASURED_VALUE)) {
            got_sensor_report = true;
        }
        off += vlen;
    }
    if (got_sensor_report) {
        mark_sensor_reporting_locked(d);
        changed = true;
    }
    return changed;
}

static bool any_unbound_remote_locked(void);

static void process_zcl_messages_locked(void)
{
    ezsp_zcl_message_t msg;
    bool persist = false;
    while (ezsp_pop_zcl_message(&msg)) {
        /* ZDO Bind_rsp — status after our Bind_req */
        if (msg.profile_id == ZDO_PROFILE_ID && msg.cluster_id == ZDO_CLUSTER_BIND_RSP &&
            msg.len >= 2) {
            uint8_t st = msg.data[1];
            const char *why = st == ZDO_STATUS_SUCCESS         ? "SUCCESS"
                              : st == ZDO_STATUS_NOT_SUPPORTED ? "NOT_SUPPORTED"
                              : st == ZDO_STATUS_TABLE_FULL    ? "TABLE_FULL"
                              : st == ZDO_STATUS_NOT_PERMITTED ? "NOT_PERMITTED"
                                                               : "FAILED";
            ESP_LOGI(TAG, "ZDO Bind_rsp from 0x%04X status=0x%02X %s", msg.sender, st, why);
            zb_device_t *bd = device_find_by_node_locked(msg.sender);
            if (bd && bd->used) {
                if (st == ZDO_STATUS_NOT_SUPPORTED) {
                    bd->remote_no_zdo_bind = true;
                    s_remote_bind_step = REMOTE_BIND_STEP_DONE;
                    ESP_LOGW(TAG,
                             "0x%04X rejects ZDO Bind (0x84) — stopping binds; wait for F&B "
                             "OnOff/Groups after Identify Query",
                             msg.sender);
                } else if (st == ZDO_STATUS_SUCCESS && s_remote_bind_step < REMOTE_BIND_STEP_DONE &&
                           (now_ms() - s_last_remote_bind_ms) > 200) {
                    memcpy(s_pending_remote_bind_eui, bd->eui64, 8);
                    s_pending_remote_bind = true;
                }
            }
            continue;
        }
        /* ZDO Mgmt_Bind_rsp — dump remote binding table (group 901 / coordinator / none). */
        if (msg.profile_id == ZDO_PROFILE_ID && msg.cluster_id == ZDO_CLUSTER_MGMT_BIND_RSP &&
            msg.len >= 5) {
            uint8_t st = msg.data[1];
            uint8_t total = msg.data[2];
            uint8_t start = msg.data[3];
            uint8_t count = msg.data[4];
            ESP_LOGI(TAG, "MgmtBind_rsp 0x%04X status=0x%02X total=%u start=%u count=%u",
                     msg.sender, st, (unsigned)total, (unsigned)start, (unsigned)count);
            size_t off = 5;
            for (uint8_t i = 0; i < count && off + 11 <= msg.len; i++) {
                const uint8_t *src = msg.data + off;
                uint8_t sep = msg.data[off + 8];
                uint16_t cluster = (uint16_t)msg.data[off + 9] | ((uint16_t)msg.data[off + 10] << 8);
                uint8_t mode = msg.data[off + 11];
                off += 12;
                if (mode == 0x01 && off + 2 <= msg.len) {
                    uint16_t gid = (uint16_t)msg.data[off] | ((uint16_t)msg.data[off + 1] << 8);
                    off += 2;
                    ESP_LOGI(TAG,
                             "  bind[%u] ep=%u cluster=0x%04X → group %u (0x%04X) src=%02X%02X…",
                             (unsigned)i, (unsigned)sep, cluster, (unsigned)gid, gid, src[0],
                             src[1]);
                    if (gid != 0) {
                        zb_device_t *owner_d = device_find_by_node_locked(msg.sender);
                        if (owner_d) {
                            owner_d->remote_group_id = gid;
                            nvs_save_devices_locked();
                        }
                        status_unlock();
                        ezsp_note_group_owner(gid, msg.sender);
                        ezsp_hint_button_remote(msg.sender);
                        (void)ezsp_ensure_multicast_group(gid);
                        detach_lights_from_group_unlocked(gid, msg.sender);
                        /* Re-detach CONTROL targets after detach sweep. */
                        status_lock();
                        uint8_t nsnap = 0;
                        for (uint16_t ri = 0; ri < ZB_HOST_MAX_DEVICES && nsnap < ZB_CTRL_SNAP_MAX;
                             ri++) {
                            zb_device_t *rd = &s_status.devices[ri];
                            if (rd->used && rd->remote_type == ZB_REMOTE_TYPE_CONTROL &&
                                (rd->remote_group_id == gid || rd->node_id == msg.sender)) {
                                rd->remote_group_id = gid;
                                ctrl_snap_from_device(rd, &s_ctrl_snaps[nsnap++]);
                            }
                        }
                        status_unlock();
                        for (uint8_t si = 0; si < nsnap; si++) {
                            attach_control_snap_unlocked(&s_ctrl_snaps[si]);
                        }
                        status_lock();
                    }
                } else if (mode == 0x03 && off + 9 <= msg.len) {
                    const uint8_t *dst = msg.data + off;
                    uint8_t dep = msg.data[off + 8];
                    off += 9;
                    ESP_LOGI(TAG,
                             "  bind[%u] ep=%u cluster=0x%04X → IEEE %02X%02X…%02X%02X ep%u",
                             (unsigned)i, (unsigned)sep, cluster, dst[0], dst[1], dst[6], dst[7],
                             (unsigned)dep);
                } else {
                    ESP_LOGW(TAG, "  bind[%u] mode=0x%02X (parse stop)", (unsigned)i, mode);
                    break;
                }
                (void)src;
            }
            if (st == ZDO_STATUS_SUCCESS && total == 0) {
                ESP_LOGW(TAG,
                         "MgmtBind empty 0x%04X — remote did not F&B to us. "
                         "Touchlink it to a real bulb (≤5cm, bulb LED flashes), then press; "
                         "we catch groupcast OnOff on group 901.",
                         msg.sender);
                /* Stay ready for groupcast after user Touchlinks a physical light. */
                status_unlock();
                ezsp_join_ikea_multicast_groups();
                ezsp_light_start_identify(180);
                status_lock();
                s_remote_fb_quiet_until_ms = now_ms() + 180000;
            }
            continue;
        }
        /* F&B group binding: remote may Add Group / Add Group If Identifying to our light. */
        if (msg.profile_id == ZCL_PROFILE_HA && msg.cluster_id == ZCL_CLUSTER_GROUPS &&
            msg.len >= 5) {
            uint8_t fc = msg.data[0];
            bool manuf = (fc & 0x04) != 0;
            bool cluster_specific = (fc & 0x01) != 0;
            bool direction_server = (fc & 0x08) != 0;
            size_t seq_off = 1 + (manuf ? 2 : 0);
            size_t cmd_off = seq_off + 1;
            if (cluster_specific && !direction_server && cmd_off + 2 < msg.len) {
                uint8_t cmd = msg.data[cmd_off];
                /* 0x00 Add Group, 0x05 Add Group If Identifying */
                if (cmd == 0x00 || cmd == 0x05) {
                    if (cmd == 0x05 && !ezsp_light_is_identifying()) {
                        ezsp_light_start_identify(180);
                    }
                    uint8_t zcl_seq = msg.data[seq_off];
                    uint16_t gid = (uint16_t)msg.data[cmd_off + 1] |
                                   ((uint16_t)msg.data[cmd_off + 2] << 8);
                    uint8_t our_ep =
                        msg.destination_endpoint ? msg.destination_endpoint : ZB_FAKE_BULB_ENDPOINT;
                    if (our_ep != 1 && our_ep != 10 && our_ep != ZB_FAKE_BULB_ENDPOINT) {
                        our_ep = ZB_FAKE_BULB_ENDPOINT;
                    }
                    uint8_t remote_ep = msg.source_endpoint ? msg.source_endpoint : 1;
                    /* Dynamic slots 6–7 for groups learned during F&B. */
                    static uint8_t s_dyn_slot = 6;
                    uint8_t slot_a = s_dyn_slot;
                    s_dyn_slot = (s_dyn_slot == 6) ? 7 : 6;
                    uint8_t slot_b = s_dyn_slot;
                    bool also_bulb = (our_ep != ZB_FAKE_BULB_ENDPOINT);
                    if (also_bulb) {
                        s_dyn_slot = (s_dyn_slot == 6) ? 7 : 6;
                    }
                    status_unlock();
                    ezsp_set_multicast_group(slot_a, gid, our_ep);
                    if (also_bulb) {
                        ezsp_set_multicast_group(slot_b, gid, ZB_FAKE_BULB_ENDPOINT);
                    }
                    ezsp_zcl_groups_add_group_response(msg.sender, remote_ep, our_ep, zcl_seq, 0x00,
                                                       gid);
                    status_lock();
                    ESP_LOGI(TAG, "Joined group %u (0x%04X) on ep%u for remote 0x%04X",
                             (unsigned)gid, gid, (unsigned)our_ep, msg.sender);
                    continue;
                }
            }
        }
        /* Light-target ZCL server: Identify + stub OnOff/Level reads (fake bulb probe). */
        if (msg.profile_id == ZCL_PROFILE_HA && msg.len >= 3 &&
            (msg.destination_endpoint == 1 || msg.destination_endpoint == 10 ||
             msg.destination_endpoint == ZB_FAKE_BULB_ENDPOINT ||
             msg.destination_endpoint == 0xFF || msg.destination_endpoint == 0 ||
             msg.group_id != 0)) {
            uint8_t fc = msg.data[0];
            bool manuf = (fc & 0x04) != 0;
            bool cluster_specific = (fc & 0x01) != 0;
            bool direction_server = (fc & 0x08) != 0;
            size_t seq_off = 1 + (manuf ? 2 : 0);
            size_t cmd_off = seq_off + 1;
            if (cmd_off < msg.len && !direction_server) {
                uint8_t zcl_seq = msg.data[seq_off];
                uint8_t cmd = msg.data[cmd_off];
                    uint8_t our_ep = msg.destination_endpoint;
                    if (our_ep != 1 && our_ep != 10 && our_ep != ZB_FAKE_BULB_ENDPOINT) {
                        our_ep = ZB_FAKE_BULB_ENDPOINT;
                    }
                uint8_t remote_ep = msg.source_endpoint ? msg.source_endpoint : 1;

                if (msg.cluster_id == ZCL_CLUSTER_IDENTIFY && cluster_specific &&
                    cmd == ZCL_CMD_IDENTIFY_CMD && cmd_off + 2 < msg.len) {
                    uint16_t t = (uint16_t)msg.data[cmd_off + 1] |
                                 ((uint16_t)msg.data[cmd_off + 2] << 8);
                    status_unlock();
                    ezsp_light_start_identify(t ? t : 180);
                    status_lock();
                    ESP_LOGI(TAG, "Identify cmd from 0x%04X time=%u — light target active",
                             msg.sender, (unsigned)(t ? t : 180));
                    continue;
                }
                if (!cluster_specific && cmd == 0x00 /* Read Attributes */) {
                    size_t aoff = cmd_off + 1;
                    if (aoff + 1 < msg.len) {
                        uint16_t attr =
                            (uint16_t)msg.data[aoff] | ((uint16_t)msg.data[aoff + 1] << 8);
                        bool handled = false;
                        status_unlock();
                        if (msg.cluster_id == ZCL_CLUSTER_IDENTIFY &&
                            attr == ZCL_ATTR_IDENTIFY_TIME) {
                            uint16_t t = ezsp_light_identify_time_s();
                            ezsp_zcl_read_attr_response_u16(msg.sender, remote_ep, our_ep, zcl_seq,
                                                            ZCL_CLUSTER_IDENTIFY, attr, 0x21, t);
                            handled = true;
                        } else if (msg.cluster_id == ZCL_CLUSTER_ON_OFF &&
                                   attr == ZCL_ATTR_ON_OFF) {
                            ezsp_zcl_read_attr_response_u8(msg.sender, remote_ep, our_ep, zcl_seq,
                                                           ZCL_CLUSTER_ON_OFF, attr, 0x10, 1);
                            handled = true;
                        } else if (msg.cluster_id == ZCL_CLUSTER_LEVEL_CONTROL &&
                                   attr == ZCL_ATTR_CURRENT_LEVEL) {
                            ezsp_zcl_read_attr_response_u8(msg.sender, remote_ep, our_ep, zcl_seq,
                                                           ZCL_CLUSTER_LEVEL_CONTROL, attr, 0x20,
                                                           254);
                            handled = true;
                        }
                        status_lock();
                        if (handled) {
                            continue;
                        }
                    }
                }
            }
        }
        zb_device_t *d = device_find_by_node_locked(msg.sender);
        if (!d) {
            continue;
        }
        d->last_seen_ms = now_ms();
        d->last_rssi = msg.last_hop_rssi;
        d->last_lqi = msg.last_hop_lqi;
        if (parse_zcl_payload_locked(d, msg.cluster_id, msg.data, msg.len, msg.source_endpoint,
                                     msg.destination_endpoint)) {
            persist = true;
        }
        /* Light OnOff/Level reports while a remote is unpaired → dump remote binds
         * (Touchlink often writes a non-901 group we must join to hear buttons). */
        if ((msg.cluster_id == ZCL_CLUSTER_ON_OFF || msg.cluster_id == ZCL_CLUSTER_LEVEL_CONTROL) &&
            zigbee_host_device_kind(d) != ZB_DEVICE_KIND_REMOTE &&
            !eui_looks_like_ikea_remote(d->eui64) && any_unbound_remote_locked()) {
            static int64_t s_last_touchlink_mgmt_ms;
            if ((now_ms() - s_last_touchlink_mgmt_ms) > 8000) {
                s_last_touchlink_mgmt_ms = now_ms();
                for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
                    zb_device_t *r = &s_status.devices[i];
                    if (!r->used || r->remote_bound) {
                        continue;
                    }
                    if (zigbee_host_device_kind(r) != ZB_DEVICE_KIND_REMOTE &&
                        !eui_looks_like_ikea_remote(r->eui64)) {
                        continue;
                    }
                    uint16_t rn = r->node_id;
                    status_unlock();
                    ESP_LOGI(TAG, "Light activity — MgmtBindReq remote 0x%04X (find Touchlink group)",
                             rn);
                    (void)ezsp_zdo_mgmt_bind_req(rn, 0);
                    status_lock();
                }
            }
        }
        /* Sensor just replied (awake window) — queue one setup/read if still configuring.
         * Do not re-bind every report once reporting is confirmed (starves Wi‑Fi on C3). */
        if (device_looks_like_temp_sensor(d) &&
            (!d->sensor_reporting || (now_ms() - d->last_interview_ms) > 300000)) {
            int64_t now = now_ms();
            if ((now - s_last_sensor_cfg_ms) > 400) {
                memcpy(s_pending_sensor_cfg_eui, d->eui64, 8);
                s_pending_sensor_cfg = true;
                s_last_sensor_cfg_ms = now;
            }
        }
        /* Unbound remote traffic: remember ep; next poll continues bind steps. */
        if ((zigbee_host_device_kind(d) == ZB_DEVICE_KIND_REMOTE ||
             eui_looks_like_ikea_remote(d->eui64)) &&
            !d->remote_bound) {
            if (msg.source_endpoint) {
                d->remote_ep = msg.source_endpoint;
            }
            if (s_remote_bind_step < REMOTE_BIND_STEP_DONE &&
                (now_ms() - s_last_remote_bind_ms) > 400) {
                memcpy(s_pending_remote_bind_eui, d->eui64, 8);
                s_pending_remote_bind = true;
            }
        }
    }
    if (persist) {
        /* Do not dirty NVS on every attribute report — flash erase on unicore
         * stalls Wi-Fi/lwIP long enough for the radio to go silent. Latch/name
         * and explicit device edits persist instead. */
        (void)persist;
    }
}

static bool any_unbound_remote_locked(void)
{
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        zb_device_t *d = &s_status.devices[i];
        if (!d->used || d->remote_bound) {
            continue;
        }
        if (zigbee_host_device_kind(d) == ZB_DEVICE_KIND_REMOTE ||
            eui_looks_like_ikea_remote(d->eui64)) {
            return true;
        }
    }
    return false;
}

static void process_pending_sensor_cfg_unlocked(void)
{
    if (!s_pending_sensor_cfg) {
        return;
    }
    uint8_t eui[8];
    memcpy(eui, s_pending_sensor_cfg_eui, 8);
    s_pending_sensor_cfg = false;

    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui);
    zb_device_t copy = {0};
    bool do_it = false;
    if (d && d->used && device_looks_like_temp_sensor(d) &&
        (!d->sensor_reporting || (now_ms() - d->last_interview_ms) > 300000)) {
        copy = *d;
        do_it = true;
    }
    status_unlock();
    if (!do_it) {
        return;
    }
    ESP_LOGI(TAG, "Sensor 0x%04X traffic — one setup/read frame now", copy.node_id);
    sensor_try_send_one_unlocked(&copy);
}

static esp_err_t interview_device_unlocked(zb_device_t *d);

static void process_pending_interview_unlocked(void)
{
    if (!s_pending_interview) {
        return;
    }
    if (now_ms() < s_remote_fb_quiet_until_ms) {
        return; /* keep pending — retry after F&B quiet window */
    }
    uint8_t eui[8];
    memcpy(eui, s_pending_interview_eui, 8);
    s_pending_interview = false;

    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui);
    zb_device_t copy = {0};
    if (d && d->used) {
        copy = *d;
    }
    status_unlock();
    if (!copy.used) {
        return;
    }
    if (zigbee_host_device_kind(&copy) == ZB_DEVICE_KIND_REMOTE ||
        eui_looks_like_ikea_remote(copy.eui64)) {
        arm_fake_bulb_unlocked(&copy);
        ESP_LOGI(TAG, "Skip interview for remote 0x%04X — arm fake bulb only", copy.node_id);
        return;
    }
    ESP_LOGI(TAG, "Interview while awake node=0x%04X (model='%s')", copy.node_id, copy.model);
    (void)interview_device_unlocked(&copy);
}

static void process_pending_remote_bind_unlocked(void)
{
    if (!s_pending_remote_bind) {
        return;
    }
    uint8_t eui[8];
    memcpy(eui, s_pending_remote_bind_eui, 8);
    s_pending_remote_bind = false;

    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui);
    zb_device_t copy = {0};
    if (d && d->used && !d->remote_bound) {
        copy = *d;
    }
    status_unlock();
    if (!copy.used || copy.remote_no_zdo_bind) {
        if (copy.used && copy.remote_no_zdo_bind) {
            arm_fake_bulb_unlocked(&copy);
        }
        return;
    }
    /* Hub path: remote wakes on button/poll → one ZDO bind to fake bulb / group 901.
     * (IKEA 10s hold is Touchlink to a physical light — not Identify Query on a coordinator.) */
    arm_fake_bulb_unlocked(&copy);
    if (s_remote_bind_step < REMOTE_BIND_STEP_DONE) {
        (void)remote_bind_one_step_unlocked(&copy);
    }
}

static esp_err_t interview_device_unlocked(zb_device_t *d)
{
    if (!d || !d->used) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t basic_attrs[] = {ZCL_ATTR_MANUFACTURER_NAME, ZCL_ATTR_MODEL_IDENTIFIER,
                              ZCL_ATTR_APPLICATION_VERSION, ZCL_ATTR_SW_BUILD_ID};
    uint16_t measured[] = {ZCL_ATTR_MEASURED_VALUE};
    uint16_t batt[] = {ZCL_ATTR_BATTERY_PCT_REMAINING};
    uint16_t onoff[] = {ZCL_ATTR_ON_OFF};
    uint16_t level[] = {ZCL_ATTR_CURRENT_LEVEL};
    uint16_t ias_attrs[] = {ZCL_ATTR_IAS_ZONE_TYPE, ZCL_ATTR_IAS_ZONE_STATUS};
    uint16_t occ_attrs[] = {ZCL_ATTR_OCCUPANCY};

    bool is_remote = zigbee_host_device_kind(d) == ZB_DEVICE_KIND_REMOTE ||
                     model_looks_like_remote(d->model) || eui_looks_like_ikea_remote(d->eui64);
    bool sleepy = d->node_type == EMBER_SLEEPY_END_DEVICE || d->node_type == EMBER_END_DEVICE;

    /* Never auto-interview remotes — Basic/PowerCfg fails with 0x66 and steals the wake window. */
    if (is_remote) {
        d->last_interview_ms = now_ms();
        if (!d->manufacturer[0]) {
            snprintf(d->manufacturer, sizeof(d->manufacturer), "IKEA of Sweden");
        }
        if (!d->name[0]) {
            snprintf(d->name, sizeof(d->name), "Tradfri remote");
        }
        ESP_LOGI(TAG, "Remote 0x%04X — skip interview (press buttons to bind to fake bulb)",
                 d->node_id);
        return ESP_OK;
    }
    bool is_sensor = device_looks_like_temp_sensor(d);
    bool is_binary = zigbee_host_is_binary_sensor(d) || model_looks_like_contact(d->model) ||
                     model_looks_like_motion(d->model) || model_looks_like_leak(d->model) ||
                     model_looks_like_smoke(d->model);

    /* Sensors / Sonoff TI OUI / unknown sleepy: one APS frame only.
     * Ember keeps it until the next child poll (button or periodic). */
    if ((is_sensor || is_binary || (sleepy && !d->model[0])) && !d->has_onoff) {
        (void)sensor_cfg_one_step_unlocked(d);
        s_sensor_tx_pending = true;
        s_sensor_tx_node = d->node_id;
        s_sensor_tx_ms = now_ms();
        d->last_interview_ms = now_ms();
        (void)measured;
        (void)batt;
        (void)basic_attrs;
        (void)sleepy;
        return ESP_OK;
    }

    /* Try common endpoints 1 and 2 (Sonoff often uses 1). */
    for (uint8_t ep = 1; ep <= 2; ep++) {
        ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_BASIC, basic_attrs, 4);
        vTaskDelay(pdMS_TO_TICKS(80));
        ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_ON_OFF, onoff, 1);
        vTaskDelay(pdMS_TO_TICKS(80));
        ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_LEVEL_CONTROL, level, 1);
        vTaskDelay(pdMS_TO_TICKS(80));
        ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_TEMP_MEASUREMENT, measured, 1);
        vTaskDelay(pdMS_TO_TICKS(80));
        ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_REL_HUMIDITY, measured, 1);
        vTaskDelay(pdMS_TO_TICKS(80));
        ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_POWER_CONFIG, batt, 1);
        vTaskDelay(pdMS_TO_TICKS(80));
        ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_IAS_ZONE, ias_attrs, 2);
        vTaskDelay(pdMS_TO_TICKS(80));
        ezsp_zcl_read_attributes(d->node_id, ep, ZCL_CLUSTER_OCCUPANCY, occ_attrs, 1);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    d->last_interview_ms = now_ms();
    return ESP_OK;
}

static void process_join_events_locked(void)
{
    ezsp_join_event_t ev;
    bool changed = false;
    while (ezsp_pop_join_event(&ev)) {
        changed = true;
        if (ev.joining) {
            device_upsert_locked(ev.eui64, ev.node_id, ev.node_type);
            zb_device_t *d = device_find_by_eui_locked(ev.eui64);
            /* IKEA remotes: F&B only (ZDO Bind returns 0x84). Arm light; wait for Identify Query. */
            if (d && (zigbee_host_device_kind(d) == ZB_DEVICE_KIND_REMOTE ||
                      eui_looks_like_ikea_remote(d->eui64))) {
                d->remote_bound = false;
                d->remote_no_zdo_bind = true; /* never ZDO-bind these */
                s_remote_bind_step = REMOTE_BIND_STEP_DONE;
                s_pending_remote_bind = false;
                s_last_remote_bind_ms = 0;
                s_remote_fb_quiet_until_ms = now_ms() + 120000;
                ESP_LOGI(TAG,
                         "Join → remote 0x%04X — Touchlink a real bulb (≤5cm, bulb LED), "
                         "then press; GW listens on group 901 (ZDO Bind unsupported)",
                         d->node_id);
            }
        } else {
            /* Do NOT wipe inventory on leave — sleepy end devices and re-joins would
             * clear name / HomeKit flags and make Home forget room assignments. */
            char eui[32];
            ezsp_format_eui64(ev.eui64, eui, sizeof(eui));
            ESP_LOGI(TAG, "Device left network (kept in inventory): %s", eui);
            zb_device_t *d = device_find_by_eui_locked(ev.eui64);
            if (d) {
                d->last_seen_ms = now_ms();
            }
        }
    }
    if (changed) {
        uint16_t count = 0;
        for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
            if (s_status.devices[i].used) {
                count++;
            }
        }
        s_status.device_count = count;
        nvs_save_devices_locked();
    }
}

static void refresh_devices_locked(void)
{
    process_join_events_locked();

    bool changed = false;
    /* Merge any NCP child-table entries (may be empty on some NCPs). */
    uint8_t empty_streak = 0;
    for (uint8_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        ezsp_child_t child;
        if (ezsp_get_child_data(i, &child) != ESP_OK) {
            continue;
        }
        if (!child.valid) {
            if (++empty_streak >= 3) {
                break;
            }
            continue;
        }
        empty_streak = 0;
        /* Detect new registrations so we don't rewrite NVS every poll. */
        bool existed = device_find_by_eui_locked(child.eui64) != NULL;
        device_upsert_locked(child.eui64, child.node_id, child.node_type);
        if (!existed && device_find_by_eui_locked(child.eui64)) {
            changed = true;
        }
    }

    uint16_t count = 0;
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        if (s_status.devices[i].used) {
            count++;
        }
    }
    if (count != s_status.device_count) {
        changed = true;
    }
    s_status.device_count = count;
    if (changed) {
        nvs_save_devices_locked();
    }
}

static void print_banner_connected(const zigbee_host_status_t *st)
{
    char ver[32];
    char eui[32];
    ezsp_format_stack_version(st->ncp.stack_version, ver, sizeof(ver));
    if (st->ncp.eui64_valid) {
        ezsp_format_eui64(st->ncp.eui64, eui, sizeof(eui));
    } else {
        snprintf(eui, sizeof(eui), "(unavailable)");
    }

    ESP_LOGI(TAG, "------------------------------------------------");
    ESP_LOGI(TAG, "ICC-1 Zigbee NCP");
    ESP_LOGI(TAG, "UART: GPIO%d TX / GPIO%d RX @ %d", CONFIG_ICC_UART_TX_GPIO,
             CONFIG_ICC_UART_RX_GPIO, CONFIG_ICC_UART_BAUD);
    ESP_LOGI(TAG, "EZSP: protocol version %u", (unsigned)st->ncp.protocol_version);
    ESP_LOGI(TAG, "EmberZNet: %s", ver);
    ESP_LOGI(TAG, "EUI64: %s", eui);
    if (st->ncp.network_state_valid) {
        ESP_LOGI(TAG, "Network: %s", zigbee_host_network_state_str(st->ncp.network_state));
    }
    if (st->ncp.net_params_valid) {
        ESP_LOGI(TAG, "Channel: %u  PAN: 0x%04X", st->ncp.net.radio_channel, st->ncp.net.pan_id);
    }
    ESP_LOGI(TAG, "ICC status: CONNECTED");
    ESP_LOGI(TAG, "------------------------------------------------");
}

static esp_err_t probe_ncp(void)
{
    status_lock();
    s_status.connect_attempts++;
    s_status.icc_status = ICC_STATUS_NOT_CONNECTED;
    s_status.version_ok = false;
    s_status.ncp.eui64_valid = false;
    s_status.ncp.network_state_valid = false;
    s_status.ncp.net_params_valid = false;
    status_unlock();

    esp_err_t err = ash_connect(CONFIG_ASH_CONNECT_TIMEOUT_MS);
    status_lock();
    s_status.ash_state = ash_get_state();
    s_status.ash_reset_code = ash_last_reset_code();
    ash_get_stats(&s_status.ash_stats);
    status_unlock();

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ASH connect failed (RX=%lu)", (unsigned long)s_status.ash_stats.rx_bytes);
        return err;
    }

    ezsp_ncp_info_t info;
    memset(&info, 0, sizeof(info));
    err = ezsp_negotiate_version(&info);
    if (err != ESP_OK) {
        set_last_error("EZSP version negotiation failed");
        return err;
    }

    uint8_t eui[8];
    if (ezsp_get_eui64(eui) == ESP_OK) {
        memcpy(info.eui64, eui, 8);
        info.eui64_valid = true;
    }

    /* Stack profile / security / TC policies required for ZB3 device joins. */
    ezsp_configure_for_coordinator();

    ezsp_network_status_t netst = EZSP_NO_NETWORK;
    if (ezsp_get_network_state(&netst) == ESP_OK && netst == EZSP_NO_NETWORK) {
        uint8_t st = 0;
        if (ezsp_network_init(&st) == ESP_OK) {
            ESP_LOGI(TAG, "Restored network from NCP tokens");
            vTaskDelay(pdMS_TO_TICKS(400));
        }
    }

    /* Multicast table is only valid once the stack is on a network. */
    if (ezsp_get_network_state(&netst) == ESP_OK && netst == EZSP_JOINED_NETWORK) {
        ezsp_join_ikea_multicast_groups();
    }

    status_lock();
    s_status.ncp = info;
    s_status.version_ok = true;
    s_status.icc_status = ICC_STATUS_CONNECTED;
    s_status.connect_successes++;
    nvs_load_devices_locked();
    /* Re-register Touchlink group owners so messageSent synth works after reboot.
     * Do NOT call ezsp_ensure_multicast_group under status_lock — it blocks the
     * portal/HomeKit for seconds (multiple EZSP cmds) and can look like a hang. */
    uint16_t restore_gids[4];
    uint8_t restore_gid_n = 0;
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        zb_device_t *d = &s_status.devices[i];
        if (!d->used || d->node_id == 0) {
            continue;
        }
        if (zigbee_host_device_kind(d) == ZB_DEVICE_KIND_REMOTE ||
            eui_looks_like_ikea_remote(d->eui64)) {
            ezsp_hint_button_remote(d->node_id);
            if (d->remote_group_id) {
                ezsp_note_group_owner(d->remote_group_id, d->node_id);
                if (restore_gid_n < 4) {
                    bool seen = false;
                    for (uint8_t j = 0; j < restore_gid_n; j++) {
                        if (restore_gids[j] == d->remote_group_id) {
                            seen = true;
                            break;
                        }
                    }
                    if (!seen) {
                        restore_gids[restore_gid_n++] = d->remote_group_id;
                    }
                }
            }
        }
    }
    refresh_network_locked();
    if (s_status.ncp.network_state == EZSP_JOINED_NETWORK) {
        refresh_devices_locked();
    }
    ash_get_stats(&s_status.ash_stats);
    ezsp_get_stats(&s_status.ezsp_stats);
    print_banner_connected(&s_status);
    /* Queue CONTROL detach — do not block connect / race HomeKit with EZSP here. */
    uint8_t n_boot_ctrl = 0;
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES && n_boot_ctrl < ZB_CTRL_SNAP_MAX; i++) {
        zb_device_t *d = &s_status.devices[i];
        if (!d->used || d->remote_type != ZB_REMOTE_TYPE_CONTROL) {
            continue;
        }
        migrate_remote_targets_locked(d);
        if (d->remote_group_id && d->target_count > 0) {
            ctrl_snap_from_device(d, &s_ctrl_snaps[n_boot_ctrl++]);
        }
    }
    status_unlock();

    /* Stagger Touchlink joins — back-to-back EZSP during HAP bring-up starved Wi-Fi. */
    for (uint8_t gi = 0; gi < restore_gid_n; gi++) {
        (void)ezsp_ensure_multicast_group(restore_gids[gi]);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    s_boot_ctrl_n = n_boot_ctrl;
    s_boot_ctrl_i = 0;
    /* Defer CONTROL AddGroup until host loop — don't race HAP Pair Verify. */
    if (n_boot_ctrl > 0) {
        ESP_LOGI(TAG, "Queued %u CONTROL remote(s) for deferred group sync",
                 (unsigned)n_boot_ctrl);
    }

    return ESP_OK;
}

esp_err_t zigbee_host_refresh_network(void)
{
    if (!ash_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_op_mutex, portMAX_DELAY);
    status_lock();
    refresh_network_locked();
    status_unlock();
    xSemaphoreGive(s_op_mutex);
    return ESP_OK;
}

esp_err_t zigbee_host_refresh_devices(void)
{
    if (!ash_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_op_mutex, portMAX_DELAY);
    status_lock();
    refresh_devices_locked();
    process_zcl_messages_locked();
    status_unlock();
    xSemaphoreGive(s_op_mutex);
    return ESP_OK;
}

esp_err_t zigbee_host_interview_device(const uint8_t eui64[8])
{
    if (!eui64 || !ash_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_op_mutex, portMAX_DELAY);
    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui64);
    zb_device_t copy = {0};
    if (d) {
        copy = *d;
    }
    status_unlock();
    esp_err_t err = ESP_ERR_NOT_FOUND;
    if (copy.used) {
        /* Manual interview: restart sleepy setup while user wakes the sensor.
         * sensor_reporting is only set when CfgReportRsp or Attribute Report arrives. */
        if (device_looks_like_temp_sensor(&copy)) {
            copy.sensor_reporting = false;
            copy.sensor_cfg_step = 0;
            status_lock();
            zb_device_t *live_s = device_find_by_eui_locked(eui64);
            if (live_s) {
                live_s->sensor_reporting = false;
                live_s->sensor_cfg_step = 0;
            }
            status_unlock();
            s_sensor_tx_pending = false;
        }
        /* Remotes: arm + start bind-on-wake (no Basic interview). */
        if (zigbee_host_device_kind(&copy) == ZB_DEVICE_KIND_REMOTE ||
            eui_looks_like_ikea_remote(copy.eui64)) {
            arm_fake_bulb_unlocked(&copy);
            s_remote_bind_step = REMOTE_BIND_STEP_IEEE_ONOFF;
            memcpy(s_pending_remote_bind_eui, copy.eui64, 8);
            s_pending_remote_bind = true;
            s_remote_fb_quiet_until_ms = now_ms() + 90000;
            status_lock();
            zb_device_t *live = device_find_by_eui_locked(eui64);
            if (live) {
                live->remote_bound = false;
                live->remote_no_zdo_bind = false;
                live->last_interview_ms = now_ms();
            }
            status_unlock();
            xSemaphoreGive(s_op_mutex);
            return ESP_OK;
        }
        err = interview_device_unlocked(&copy);
        /* Drain any immediate responses */
        vTaskDelay(pdMS_TO_TICKS(300));
        status_lock();
        process_zcl_messages_locked();
        zb_device_t *live = device_find_by_eui_locked(eui64);
        if (live) {
            live->last_interview_ms = now_ms();
            nvs_save_devices_locked();
        }
        status_unlock();
    }
    xSemaphoreGive(s_op_mutex);
    return err;
}

esp_err_t zigbee_host_set_onoff(const uint8_t eui64[8], bool on)
{
    if (!eui64) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Optimistic + queued: never block the HomeKit write path on EZSP/NVS. */
    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui64);
    if (!d || !d->used) {
        status_unlock();
        return ESP_ERR_NOT_FOUND;
    }
    d->onoff_on = on;
    d->has_onoff = true;
    if (!on) {
        d->level = 0;
    }
    /* Coalesce existing queue entry for this EUI, else append. */
    int slot = -1;
    for (uint8_t i = 0; i < s_onoff_q_len; i++) {
        if (memcmp(s_onoff_q[i].eui64, eui64, 8) == 0) {
            slot = (int)i;
            break;
        }
    }
    if (slot < 0 && s_onoff_q_len < ZB_CMD_QUEUE_MAX) {
        slot = (int)s_onoff_q_len++;
    }
    if (slot >= 0) {
        memcpy(s_onoff_q[slot].eui64, eui64, 8);
        s_onoff_q[slot].on = on;
    } else {
        ESP_LOGW(TAG, "On/Off queue full — dropping cmd");
    }
    status_unlock();
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
    return ESP_OK;
}

static void process_pending_onoff_unlocked(void)
{
    onoff_req_t req = {0};
    uint16_t node_id = 0;
    uint8_t ep = 1;

    status_lock();
    if (s_onoff_q_len > 0) {
        req = s_onoff_q[0];
        memmove(&s_onoff_q[0], &s_onoff_q[1], (size_t)(s_onoff_q_len - 1) * sizeof(s_onoff_q[0]));
        s_onoff_q_len--;
        zb_device_t *d = device_find_by_eui_locked(req.eui64);
        if (d && d->used) {
            node_id = d->node_id;
            ep = d->onoff_ep ? d->onoff_ep : 1;
            req.on = req.on; /* keep */
        } else {
            node_id = 0;
        }
    }
    status_unlock();

    if (!node_id) {
        return;
    }

    esp_err_t err = ezsp_zcl_on_off_command(node_id, ep, req.on ? ZCL_CMD_ON : ZCL_CMD_OFF);
    if (err != ESP_OK && ep == 1) {
        err = ezsp_zcl_on_off_command(node_id, 2, req.on ? ZCL_CMD_ON : ZCL_CMD_OFF);
        if (err == ESP_OK) {
            status_lock();
            zb_device_t *live = device_find_by_eui_locked(req.eui64);
            if (live) {
                live->onoff_ep = 2;
            }
            status_unlock();
        }
    } else if (err == ESP_OK) {
        status_lock();
        zb_device_t *live = device_find_by_eui_locked(req.eui64);
        if (live && !live->onoff_ep) {
            live->onoff_ep = ep;
        }
        status_unlock();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "On/Off cmd failed for 0x%04X: %s", node_id, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "On/Off %s → node=0x%04X ep%u (from queue)", req.on ? "ON" : "OFF", node_id,
                 (unsigned)ep);
    }
}

esp_err_t zigbee_host_set_brightness(const uint8_t eui64[8], uint8_t brightness_pct)
{
    if (!eui64) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t level = zigbee_host_brightness_to_level(brightness_pct);
    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui64);
    if (!d || !d->used) {
        status_unlock();
        return ESP_ERR_NOT_FOUND;
    }
    d->level = level;
    d->has_level = true;
    d->onoff_on = (level > 0);
    d->has_onoff = true;
    int slot = -1;
    for (uint8_t i = 0; i < s_level_q_len; i++) {
        if (memcmp(s_level_q[i].eui64, eui64, 8) == 0) {
            slot = (int)i;
            break;
        }
    }
    if (slot < 0 && s_level_q_len < ZB_CMD_QUEUE_MAX) {
        slot = (int)s_level_q_len++;
    }
    if (slot >= 0) {
        memcpy(s_level_q[slot].eui64, eui64, 8);
        s_level_q[slot].level = level;
    } else {
        ESP_LOGW(TAG, "Level queue full — dropping cmd");
    }
    status_unlock();
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
    return ESP_OK;
}

static void process_pending_level_unlocked(void)
{
    level_req_t req = {0};
    uint16_t node_id = 0;
    uint8_t ep = 1;

    status_lock();
    if (s_level_q_len > 0) {
        req = s_level_q[0];
        memmove(&s_level_q[0], &s_level_q[1], (size_t)(s_level_q_len - 1) * sizeof(s_level_q[0]));
        s_level_q_len--;
        zb_device_t *d = device_find_by_eui_locked(req.eui64);
        if (d && d->used) {
            node_id = d->node_id;
            ep = d->level_ep ? d->level_ep : (d->onoff_ep ? d->onoff_ep : 1);
        } else {
            node_id = 0;
        }
    }
    status_unlock();

    if (!node_id) {
        return;
    }

    esp_err_t err = ezsp_zcl_move_to_level(node_id, ep, req.level, true);
    if (err != ESP_OK && ep == 1) {
        err = ezsp_zcl_move_to_level(node_id, 2, req.level, true);
        if (err == ESP_OK) {
            status_lock();
            zb_device_t *live = device_find_by_eui_locked(req.eui64);
            if (live) {
                live->level_ep = 2;
                if (!live->onoff_ep) {
                    live->onoff_ep = 2;
                }
            }
            status_unlock();
        }
    } else if (err == ESP_OK) {
        status_lock();
        zb_device_t *live = device_find_by_eui_locked(req.eui64);
        if (live && !live->level_ep) {
            live->level_ep = ep;
        }
        status_unlock();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MoveToLevel failed for 0x%04X: %s", node_id, esp_err_to_name(err));
    }
}

esp_err_t zigbee_host_update_device(const uint8_t eui64[8], const zb_device_update_t *upd)
{
    if (!eui64 || !upd) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_op_mutex, portMAX_DELAY);
    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui64);
    esp_err_t err = ESP_ERR_NOT_FOUND;
    if (d) {
        if (upd->set_name) {
            snprintf(d->name, sizeof(d->name), "%s", upd->name);
            device_refresh_label(d);
        }
        if (upd->set_homekit) {
            d->homekit_expose = upd->homekit_expose;
        }
        if (upd->set_btn_modes) {
            uint8_t n = zigbee_host_remote_button_count(d);
            if (n == 0) {
                n = ZB_REMOTE_MAX_BUTTONS;
            }
            uint8_t lim = upd->btn_mode_count < n ? upd->btn_mode_count : n;
            for (uint8_t i = 0; i < lim && i < ZB_REMOTE_MAX_BUTTONS; i++) {
                uint8_t m = upd->btn_modes[i];
                d->btn_mode[i] = (m == ZB_BTN_MODE_STATEFUL) ? ZB_BTN_MODE_STATEFUL
                                                             : ZB_BTN_MODE_STATELESS;
            }
        }
        if (upd->set_btn_names) {
            uint8_t n = zigbee_host_remote_button_count(d);
            if (n == 0) {
                n = ZB_REMOTE_MAX_BUTTONS;
            }
            uint8_t lim = upd->btn_name_count < n ? upd->btn_name_count : n;
            for (uint8_t i = 0; i < lim && i < ZB_REMOTE_MAX_BUTTONS; i++) {
                snprintf(d->btn_name[i], sizeof(d->btn_name[i]), "%s", upd->btn_names[i]);
            }
        }
        if (upd->set_remote_type) {
            d->remote_type = (upd->remote_type == ZB_REMOTE_TYPE_CONTROL) ? ZB_REMOTE_TYPE_CONTROL
                                                                          : ZB_REMOTE_TYPE_HOMEKIT;
        }
        uint8_t old_targets[ZB_REMOTE_MAX_TARGETS][8];
        uint8_t old_n = 0;
        migrate_remote_targets_locked(d);
        old_n = d->target_count;
        if (old_n > ZB_REMOTE_MAX_TARGETS) {
            old_n = ZB_REMOTE_MAX_TARGETS;
        }
        memcpy(old_targets, d->targets, sizeof(old_targets));

        if (upd->set_targets) {
            d->target_count = upd->target_count;
            if (d->target_count > ZB_REMOTE_MAX_TARGETS) {
                d->target_count = ZB_REMOTE_MAX_TARGETS;
            }
            memset(d->targets, 0, sizeof(d->targets));
            for (uint8_t i = 0; i < d->target_count; i++) {
                memcpy(d->targets[i], upd->targets[i], 8);
            }
            d->target_group_count = upd->target_group_count;
            if (d->target_group_count > ZB_REMOTE_MAX_TARGETS) {
                d->target_group_count = ZB_REMOTE_MAX_TARGETS;
            }
            memset(d->target_groups, 0, sizeof(d->target_groups));
            for (uint8_t i = 0; i < d->target_group_count; i++) {
                d->target_groups[i] = upd->target_groups[i];
            }
            migrate_remote_targets_locked(d);
        }

        zb_ctrl_snap_t sync_snap = {0};
        uint8_t old_targets_copy[ZB_REMOTE_MAX_TARGETS][8];
        uint8_t old_n_copy = old_n;
        bool do_sync = false;
        if (d->remote_type == ZB_REMOTE_TYPE_CONTROL &&
            (upd->set_targets || upd->set_remote_type)) {
            ctrl_snap_from_device(d, &sync_snap);
            memcpy(old_targets_copy, old_targets, sizeof(old_targets_copy));
            do_sync = true;
        } else if (d->remote_type != ZB_REMOTE_TYPE_CONTROL && old_n > 0 && d->remote_group_id) {
            /* Leaving CONTROL — detach previous targets from the Touchlink group. */
            sync_snap.node_id = d->node_id;
            sync_snap.remote_group_id = d->remote_group_id;
            sync_snap.target_count = 0;
            memcpy(old_targets_copy, old_targets, sizeof(old_targets_copy));
            do_sync = true;
        }
        nvs_save_devices_locked();
        err = ESP_OK;
        status_unlock();
        if (do_sync) {
            queue_ctrl_sync(&sync_snap, old_targets_copy, old_n_copy, true);
        }
        xSemaphoreGive(s_op_mutex);
        return err;
    }
    status_unlock();
    xSemaphoreGive(s_op_mutex);
    return err;
}

esp_err_t zigbee_host_remove_device(const uint8_t eui64[8])
{
    if (!eui64) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_op_mutex, portMAX_DELAY);
    status_lock();
    device_remove_locked(eui64);
    uint16_t count = 0;
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        if (s_status.devices[i].used) {
            count++;
        }
    }
    s_status.device_count = count;
    nvs_save_devices_locked();
    status_unlock();
    xSemaphoreGive(s_op_mutex);
    return ESP_OK;
}

bool zigbee_host_get_device(const uint8_t eui64[8], zb_device_t *out)
{
    if (!eui64 || !out) {
        return false;
    }
    bool ok = false;
    status_lock();
    zb_device_t *d = device_find_by_eui_locked(eui64);
    if (d) {
        *out = *d;
        ok = true;
    }
    status_unlock();
    return ok;
}

bool zigbee_host_get_device_at(uint16_t index, zb_device_t *out)
{
    if (!out || index >= ZB_HOST_MAX_DEVICES) {
        return false;
    }
    bool ok = false;
    status_lock();
    if (s_status.devices[index].used) {
        *out = s_status.devices[index];
        ok = true;
    }
    status_unlock();
    return ok;
}

uint16_t zigbee_host_count_homekit_exposed(void)
{
    uint16_t n = 0;
    status_lock();
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        zb_device_t *d = &s_status.devices[i];
        if (!d->used || !d->homekit_expose) {
            continue;
        }
        zb_device_kind_t k = zigbee_host_device_kind(d);
        if (k == ZB_DEVICE_KIND_LIGHT || k == ZB_DEVICE_KIND_SWITCH ||
            k == ZB_DEVICE_KIND_SENSOR || k == ZB_DEVICE_KIND_REMOTE) {
            n++;
        }
    }
    status_unlock();
    return n;
}

esp_err_t zigbee_host_register_device(const uint8_t eui64[8], uint16_t node_id, uint8_t node_type)
{
    if (!eui64) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_op_mutex, portMAX_DELAY);
    status_lock();
    device_upsert_locked(eui64, node_id, node_type);
    uint16_t count = 0;
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        if (s_status.devices[i].used) {
            count++;
        }
    }
    s_status.device_count = count;
    nvs_save_devices_locked();
    status_unlock();
    xSemaphoreGive(s_op_mutex);
    return ESP_OK;
}

esp_err_t zigbee_host_form_network(const zigbee_form_options_t *opt)
{
    if (!ash_is_connected()) {
        set_last_error("ICC not connected");
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_op_mutex, portMAX_DELAY);

    ezsp_configure_for_coordinator();

    uint8_t st = 0;
    ezsp_network_status_t netst;
    if (ezsp_get_network_state(&netst) == ESP_OK && netst == EZSP_JOINED_NETWORK) {
        ESP_LOGI(TAG, "Already on a network — leaving before form");
        ezsp_leave_network(&st);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    uint8_t nwk_key[16];
    esp_fill_random(nwk_key, sizeof(nwk_key));

    esp_err_t err = ezsp_set_initial_security_state(nwk_key, &st);
    if (err != ESP_OK) {
        set_last_error("setInitialSecurityState failed");
        xSemaphoreGive(s_op_mutex);
        return err;
    }

    ezsp_network_params_t params;
    memset(&params, 0, sizeof(params));
    esp_fill_random(params.extended_pan_id, sizeof(params.extended_pan_id));
    uint16_t pan = (opt && opt->pan_id) ? opt->pan_id : (uint16_t)(esp_random() & 0xFFFF);
    if (pan == 0 || pan == 0xFFFF) {
        pan = 0x1A2B;
    }
    params.pan_id = pan;
    params.radio_tx_power = (opt && opt->tx_power) ? opt->tx_power : 8;
    uint8_t ch = (opt && opt->channel >= 11 && opt->channel <= 26) ? opt->channel : 15;
    params.radio_channel = ch;
    params.join_method = EMBER_USE_MAC_ASSOCIATION;
    params.nwk_manager_id = 0;
    params.nwk_update_id = 0;
    params.channels = (1u << ch);

    err = ezsp_form_network(&params, &st);
    if (err != ESP_OK) {
        char msg[64];
        snprintf(msg, sizeof(msg), "formNetwork failed: %s (0x%02X)", ezsp_ember_status_str(st), st);
        set_last_error(msg);
        xSemaphoreGive(s_op_mutex);
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(300));
    ezsp_join_ikea_multicast_groups();
    status_lock();
    refresh_network_locked();
    nvs_save_network(&s_status.ncp.net);
    s_status.device_count = 0;
    memset(s_status.devices, 0, sizeof(s_status.devices));
    nvs_save_devices_locked();
    s_status.last_error[0] = '\0';
    status_unlock();

    xSemaphoreGive(s_op_mutex);
    ESP_LOGI(TAG, "Zigbee network formed on channel %u", ch);
    return ESP_OK;
}

esp_err_t zigbee_host_leave_network(void)
{
    if (!ash_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_op_mutex, portMAX_DELAY);
    uint8_t st = 0;
    esp_err_t err = ezsp_leave_network(&st);
    nvs_clear_network();
    s_permit_deadline_ms = 0;
    status_lock();
    refresh_network_locked();
    s_status.device_count = 0;
    memset(s_status.devices, 0, sizeof(s_status.devices));
    nvs_save_devices_locked();
    s_status.permit_join_remaining = 0;
    if (err != ESP_OK) {
        snprintf(s_status.last_error, sizeof(s_status.last_error), "leaveNetwork: %s",
                 ezsp_ember_status_str(st));
    } else {
        s_status.last_error[0] = '\0';
    }
    status_unlock();
    xSemaphoreGive(s_op_mutex);
    return err;
}

esp_err_t zigbee_host_permit_join(uint8_t duration_sec)
{
    if (!ash_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    status_lock();
    bool joined = (s_status.ncp.network_state == EZSP_JOINED_NETWORK);
    status_unlock();
    if (!joined) {
        set_last_error("No Zigbee network — create one first");
        return ESP_ERR_INVALID_STATE;
    }
    /* Must not run EZSP/ZLL from the httpd task — ~11KB status + Touchlink arm
     * overflow the 12KB httpd stack (Stack protection fault). */
    s_pending_permit_dur = duration_sec;
    s_pending_permit = true;
    if (duration_sec == 0) {
        s_permit_deadline_ms = 0;
    } else if (duration_sec == 0xFF) {
        s_permit_deadline_ms = now_ms() + (int64_t)24 * 3600 * 1000;
    } else {
        s_permit_deadline_ms = now_ms() + (int64_t)duration_sec * 1000;
    }
    status_lock();
    update_permit_remaining();
    status_unlock();
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
    return ESP_OK;
}

static void process_pending_permit_unlocked(void)
{
    if (!s_pending_permit) {
        return;
    }
    uint8_t duration_sec = s_pending_permit_dur;
    s_pending_permit = false;

    ezsp_network_status_t netst = EZSP_NO_NETWORK;
    if (ezsp_get_network_state(&netst) != ESP_OK || netst != EZSP_JOINED_NETWORK) {
        set_last_error("No Zigbee network — create one first");
        s_permit_deadline_ms = 0;
        status_lock();
        update_permit_remaining();
        status_unlock();
        return;
    }

    esp_err_t perr = ezsp_set_join_policy(duration_sec > 0);
    if (perr != ESP_OK) {
        ESP_LOGW(TAG, "set join policy failed — continuing with permitJoining");
    }

    uint8_t st = 0;
    esp_err_t err = ezsp_permit_joining(duration_sec, &st);
    if (err == ESP_OK) {
        status_lock();
        s_status.last_error[0] = '\0';
        update_permit_remaining();
        status_unlock();
        ESP_LOGI(TAG, "Permit join %s (%u s)", duration_sec ? "OPEN" : "CLOSED",
                 (unsigned)duration_sec);
        if (duration_sec > 0) {
            ezsp_join_ikea_multicast_groups();
            ezsp_light_start_identify(duration_sec > 180 ? 180 : duration_sec);
            uint32_t tl_ms = (duration_sec == 0xFF) ? 180000u : (uint32_t)duration_sec * 1000u;
            if (tl_ms > 300000u) {
                tl_ms = 300000u;
            }
            if (ezsp_zll_supported()) {
                (void)ezsp_zll_arm_touchlink_target(tl_ms);
            }
            status_lock();
            for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
                zb_device_t *d = &s_status.devices[i];
                if (!d->used || d->remote_bound) {
                    continue;
                }
                if (zigbee_host_device_kind(d) != ZB_DEVICE_KIND_REMOTE &&
                    !eui_looks_like_ikea_remote(d->eui64)) {
                    continue;
                }
                s_remote_bind_step = REMOTE_BIND_STEP_IEEE_ONOFF;
                memcpy(s_pending_remote_bind_eui, d->eui64, 8);
                s_pending_remote_bind = true;
                s_last_remote_bind_ms = 0;
                s_remote_fb_quiet_until_ms = now_ms() + 90000;
                ESP_LOGI(TAG, "Permit → remote 0x%04X — mash buttons ~15s to bind fake bulb",
                         d->node_id);
                break;
            }
            status_unlock();
        }
    } else {
        char msg[64];
        snprintf(msg, sizeof(msg), "permitJoining failed: %s", ezsp_ember_status_str(st));
        set_last_error(msg);
        s_permit_deadline_ms = 0;
        status_lock();
        update_permit_remaining();
        status_unlock();
    }
}

static void host_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Zigbee host task running");

    if (ezsp_init() != ESP_OK) {
        ESP_LOGE(TAG, "ezsp_init failed");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        if (!ash_is_connected() || s_status.icc_status != ICC_STATUS_CONNECTED) {
            probe_ncp();
        } else {
            status_lock();
            ash_get_stats(&s_status.ash_stats);
            ezsp_get_stats(&s_status.ezsp_stats);
            s_status.ash_state = ash_get_state();
            update_permit_remaining();
            status_unlock();

            if (ezsp_nop() != ESP_OK) {
                ESP_LOGW(TAG, "EZSP nop failed — marking ICC disconnected");
                status_lock();
                s_status.icc_status = ICC_STATUS_NOT_CONNECTED;
                status_unlock();
            } else {
                /* Answer Identify Query ASAP (queued from RX callback — must not block ASH). */
                ezsp_flush_identify_query_responses();
                /* Drain button synth / ZCL before slow multicast join so HomeKit stays snappy. */
                xSemaphoreTake(s_op_mutex, portMAX_DELAY);
                status_lock();
                process_zcl_messages_locked();
                status_unlock();
                xSemaphoreGive(s_op_mutex);
                bool had_btn = s_hk_btn_q_len > 0;
                flush_hk_btn_callbacks();
                flush_sensor_upd_callbacks();
                /* Force flush only after a button latch; otherwise debounce (≤5s). */
                nvs_flush_devices_if_dirty(had_btn);
                /* Join Touchlink groups overheard while relaying remote→bulb groupcasts. */
                {
                    uint16_t learn_gid = 0;
                    while (ezsp_take_learned_group(&learn_gid)) {
                        xSemaphoreTake(s_op_mutex, portMAX_DELAY);
                        ESP_LOGI(TAG, "Overheard button groupcast → join group %u (0x%04X)",
                                 (unsigned)learn_gid, learn_gid);
                        (void)ezsp_ensure_multicast_group(learn_gid);
                        /* Attribute this group to a remote (multi-remote safe). */
                        status_lock();
                        zb_device_t *owner_d = NULL;
                        zb_device_t *unassigned = NULL;
                        uint16_t nunassigned = 0;
                        uint16_t nrem = 0;
                        for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
                            zb_device_t *d = &s_status.devices[i];
                            if (!d->used) {
                                continue;
                            }
                            if (zigbee_host_device_kind(d) != ZB_DEVICE_KIND_REMOTE &&
                                !eui_looks_like_ikea_remote(d->eui64)) {
                                continue;
                            }
                            nrem++;
                            if (d->remote_group_id == learn_gid) {
                                owner_d = d;
                                break;
                            }
                            if (d->remote_group_id == 0) {
                                unassigned = d;
                                nunassigned++;
                            }
                        }
                        if (!owner_d && nunassigned == 1) {
                            owner_d = unassigned;
                        }
                        if (!owner_d && nrem == 1) {
                            owner_d = unassigned; /* only remote present */
                            for (uint16_t i = 0; !owner_d && i < ZB_HOST_MAX_DEVICES; i++) {
                                zb_device_t *d = &s_status.devices[i];
                                if (d->used && (zigbee_host_device_kind(d) == ZB_DEVICE_KIND_REMOTE ||
                                                eui_looks_like_ikea_remote(d->eui64))) {
                                    owner_d = d;
                                }
                            }
                        }
                        /* Prefer HOMEKIT remotes still missing a group over CONTROL ones. */
                        if (!owner_d) {
                            for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
                                zb_device_t *d = &s_status.devices[i];
                                if (!d->used || d->remote_group_id != 0) {
                                    continue;
                                }
                                if (zigbee_host_device_kind(d) != ZB_DEVICE_KIND_REMOTE &&
                                    !eui_looks_like_ikea_remote(d->eui64)) {
                                    continue;
                                }
                                if (d->remote_type == ZB_REMOTE_TYPE_HOMEKIT) {
                                    owner_d = d;
                                    break;
                                }
                                if (!owner_d) {
                                    owner_d = d;
                                }
                            }
                        }
                        if (owner_d) {
                            if (owner_d->remote_group_id != learn_gid) {
                                owner_d->remote_group_id = learn_gid;
                                nvs_save_devices_locked();
                            }
                            ezsp_hint_button_remote(owner_d->node_id);
                            ezsp_note_group_owner(learn_gid, owner_d->node_id);
                            ESP_LOGI(TAG, "Group %u → remote 0x%04X (%s)", (unsigned)learn_gid,
                                     owner_d->node_id,
                                     owner_d->name[0] ? owner_d->name : owner_d->model);
                        }
                        status_unlock();
                        /* Detach unknown lights once; do NOT re-RemoveGroup CONTROL targets
                         * on every Level/OnOff overhear (caused Groups 0x8B storms). */
                        detach_lights_from_group_unlocked(learn_gid, 0);
                        xSemaphoreGive(s_op_mutex);
                        s_remote_fb_quiet_until_ms = now_ms() + 120000;
                    }
                }
                /* After IdentifyQueryRsp: act as a bulb — join multicast / stay identifying.
                 * Do NOT ZDO-bind the remote (factory Tradfri returns 0x84). */
                uint16_t id_node = 0;
                while (ezsp_take_identify_bind_node(&id_node)) {
                    status_lock();
                    zb_device_t *d = device_find_by_node_locked(id_node);
                    zb_device_t copy = {0};
                    if (d && d->used) {
                        copy = *d;
                    }
                    status_unlock();
                    xSemaphoreTake(s_op_mutex, portMAX_DELAY);
                    ESP_LOGI(TAG,
                             "Identify Query from 0x%04X — QueryRsp done; wait for OnOff/Groups "
                             "(dumping remote binding table in 2s)",
                             id_node);
                    arm_fake_bulb_unlocked(copy.used ? &copy : NULL);
                    if (copy.used) {
                        s_remote_fb_quiet_until_ms = now_ms() + 120000;
                        status_lock();
                        zb_device_t *live = device_find_by_node_locked(id_node);
                        if (live) {
                            live->remote_no_zdo_bind = true;
                        }
                        status_unlock();
                        s_remote_bind_step = REMOTE_BIND_STEP_DONE;
                        s_pending_remote_bind = false;
                        xSemaphoreGive(s_op_mutex);
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        xSemaphoreTake(s_op_mutex, portMAX_DELAY);
                        ESP_LOGI(TAG, "MgmtBindReq → 0x%04X (see if F&B created binds)", id_node);
                        (void)ezsp_zdo_mgmt_bind_req(id_node, 0);
                    }
                    xSemaphoreGive(s_op_mutex);
                }
                /* Sleepy polled — queue ONE sensor frame for next check-in; interview unknowns. */
                uint16_t poll_node = 0;
                while (ezsp_take_poll_event(&poll_node)) {
                    status_lock();
                    zb_device_t *pd = device_find_by_node_locked(poll_node);
                    zb_device_t pcopy = {0};
                    bool is_sensor = false;
                    bool need_interview = false;
                    if (pd && pd->used) {
                        if (device_looks_like_temp_sensor(pd)) {
                            pcopy = *pd;
                            is_sensor = true;
                        } else if (zigbee_host_device_kind(pd) == ZB_DEVICE_KIND_REMOTE ||
                                   eui_looks_like_ikea_remote(pd->eui64)) {
                            /* F&B only — never ZDO-bind on poll. */
                            if (!pd->remote_bound) {
                                s_remote_fb_quiet_until_ms = now_ms() + 120000;
                            }
                        } else if (!pd->model[0]) {
                            /* Unknown sleepy — re-queue Basic on poll (Sonoff TI more often). */
                            int64_t gap =
                                eui_looks_like_sonoff_sensor(pd->eui64) ? 15000 : 120000;
                            if ((now_ms() - s_last_poll_interview_ms) > 3000 &&
                                (pd->last_interview_ms == 0 ||
                                 (now_ms() - pd->last_interview_ms) > gap)) {
                                memcpy(s_pending_interview_eui, pd->eui64, 8);
                                s_pending_interview = true;
                                need_interview = true;
                                s_last_poll_interview_ms = now_ms();
                                pd->last_interview_ms = now_ms();
                            }
                        }
                    }
                    status_unlock();
                    if (is_sensor &&
                        (!pcopy.sensor_reporting ||
                         (now_ms() - pcopy.last_interview_ms) > 300000)) {
                        xSemaphoreTake(s_op_mutex, portMAX_DELAY);
                        sensor_try_send_one_unlocked(&pcopy);
                        xSemaphoreGive(s_op_mutex);
                    }
                    (void)need_interview;
                }
                bool join_evt = ezsp_consume_join_event();
                if (ezsp_take_network_up_event()) {
                    ezsp_join_ikea_multicast_groups();
                }
                static int64_t s_last_child_scan_ms;
                static int64_t s_last_interview_ms;
                int64_t now = now_ms();
                xSemaphoreTake(s_op_mutex, portMAX_DELAY);
                process_sensor_sent_events_unlocked();
                process_pending_permit_unlocked();
                process_ctrl_sync_job_unlocked();
                if (!s_ctrl_sync_job.pending && s_boot_ctrl_i < s_boot_ctrl_n &&
                    now_ms() > 60000) {
                    queue_ctrl_sync(&s_ctrl_snaps[s_boot_ctrl_i], NULL, 0, false);
                    s_boot_ctrl_i++;
                    process_ctrl_sync_job_unlocked();
                }
                for (int qi = 0; qi < 4; qi++) {
                    bool more = false;
                    status_lock();
                    more = (s_onoff_q_len > 0) || (s_level_q_len > 0);
                    status_unlock();
                    if (!more) {
                        break;
                    }
                    process_pending_onoff_unlocked();
                    process_pending_level_unlocked();
                    if (qi + 1 < 4) {
                        /* Don't hold the op mutex across Zigbee inter-command gaps. */
                        xSemaphoreGive(s_op_mutex);
                        vTaskDelay(pdMS_TO_TICKS(30));
                        xSemaphoreTake(s_op_mutex, portMAX_DELAY);
                    }
                }
                /* EZSP under status_lock freezes the web portal + HomeKit for up to
                 * N×3s on timeouts — fetch network state with the lock released. */
                xSemaphoreGive(s_op_mutex);
                {
                    /* Network params are slow EZSP round-trips — every loop starved Wi‑Fi. */
                    static int64_t s_last_net_poll_ms;
                    bool poll_net = (now_ms() - s_last_net_poll_ms) > 5000;
                    ezsp_network_status_t st = 0;
                    uint8_t node_type = 0;
                    ezsp_network_params_t params;
                    memset(&params, 0, sizeof(params));
                    bool got_st = false;
                    bool got_params = false;
                    if (poll_net) {
                        s_last_net_poll_ms = now_ms();
                        got_st = (ezsp_get_network_state(&st) == ESP_OK);
                        got_params =
                            (ezsp_get_network_parameters(&node_type, &params) == ESP_OK);
                    }
                    status_lock();
                    if (got_st) {
                        s_status.ncp.network_state = st;
                        s_status.ncp.network_state_valid = true;
                    }
                    if (got_params) {
                        s_status.ncp.node_type = node_type;
                        s_status.ncp.net = params;
                        s_status.ncp.net_params_valid = true;
                    }
                }
                update_permit_remaining();
                process_join_events_locked();
                process_zcl_messages_locked();
                bool need_remote_bind = s_pending_remote_bind;
                bool need_sensor_cfg = s_pending_sensor_cfg;
                bool need_interview = s_pending_interview;
                bool permit_open = s_status.permit_join_remaining > 0 &&
                                   s_status.ncp.network_state == EZSP_JOINED_NETWORK;
                bool due = (now - s_last_child_scan_ms) > 4000;
                bool do_child_scan = false;
                if (join_evt || (permit_open && due)) {
                    do_child_scan = true;
                    s_last_child_scan_ms = now;
                }
                /* Skip all remotes for periodic interview (Basic once at join is enough). */
                bool interview_due = (now - s_last_interview_ms) > 30000;
                zb_device_t interview_target = {0};
                zb_device_t sensor_cfg_target = {0};
                bool have_sensor_cfg = false;
                if (interview_due && s_status.ncp.network_state == EZSP_JOINED_NETWORK) {
                    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
                        zb_device_t *d = &s_status.devices[i];
                        if (!d->used) {
                            continue;
                        }
                        if (zigbee_host_device_kind(d) == ZB_DEVICE_KIND_REMOTE ||
                            model_looks_like_remote(d->model) ||
                            eui_looks_like_ikea_remote(d->eui64)) {
                            continue;
                        }
                        /* Keep one frame queued for sleepy sensors awaiting setup/refresh. */
                        if (!have_sensor_cfg && device_looks_like_temp_sensor(d) &&
                            !s_sensor_tx_pending &&
                            (d->last_interview_ms == 0 ||
                             (now - d->last_interview_ms) >
                                 (d->sensor_reporting ? 300000 : 90000))) {
                            sensor_cfg_target = *d;
                            have_sensor_cfg = true;
                        }
                        if (d->model[0] &&
                            !((zigbee_host_device_kind(d) == ZB_DEVICE_KIND_LIGHT &&
                               !d->has_level) ||
                              (zigbee_host_device_kind(d) == ZB_DEVICE_KIND_SWITCH &&
                               !d->has_onoff) ||
                              (zigbee_host_device_kind(d) == ZB_DEVICE_KIND_SENSOR &&
                               !d->has_temp && !d->has_humidity))) {
                            continue;
                        }
                        if (d->last_interview_ms == 0 || (now - d->last_interview_ms) > 60000) {
                            interview_target = *d;
                            break;
                        }
                    }
                    s_last_interview_ms = now;
                }
                status_unlock();
                if (do_child_scan) {
                    xSemaphoreTake(s_op_mutex, portMAX_DELAY);
                    status_lock();
                    refresh_devices_locked();
                    status_unlock();
                    xSemaphoreGive(s_op_mutex);
                }
                xSemaphoreTake(s_op_mutex, portMAX_DELAY);
                if (need_interview) {
                    process_pending_interview_unlocked();
                }
                if (need_sensor_cfg) {
                    process_pending_sensor_cfg_unlocked();
                }
                if (have_sensor_cfg) {
                    ESP_LOGI(TAG, "Sensor 0x%04X: periodic one-frame step%u (awaiting confirm)",
                             sensor_cfg_target.node_id, (unsigned)sensor_cfg_target.sensor_cfg_step);
                    sensor_try_send_one_unlocked(&sensor_cfg_target);
                }
                if (need_remote_bind) {
                    process_pending_remote_bind_unlocked();
                }
                if (interview_target.used) {
                    interview_device_unlocked(&interview_target);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    status_lock();
                    process_zcl_messages_locked();
                    zb_device_t *live = device_find_by_eui_locked(interview_target.eui64);
                    if (live) {
                        live->last_interview_ms = now_ms();
                    }
                    status_unlock();
                }
                /* Drain queued On/Off / Level (groups enqueue multiple). */
                for (int qi = 0; qi < ZB_CMD_QUEUE_MAX; qi++) {
                    bool more = false;
                    status_lock();
                    more = (s_onoff_q_len > 0) || (s_level_q_len > 0);
                    status_unlock();
                    if (!more) {
                        break;
                    }
                    process_pending_onoff_unlocked();
                    process_pending_level_unlocked();
                    xSemaphoreGive(s_op_mutex);
                    vTaskDelay(pdMS_TO_TICKS(40));
                    xSemaphoreTake(s_op_mutex, portMAX_DELAY);
                }
                xSemaphoreGive(s_op_mutex);
                bool btn_pending = s_hk_btn_q_len > 0;
                flush_hk_btn_callbacks();
                flush_sensor_upd_callbacks();
                nvs_flush_devices_if_dirty(btn_pending);
            }
        }
        /* Poll faster while waiting for joins; wake early for On/Off / Identify. */
        uint32_t delay = 400; /* was CONFIG_ASH_RETRY (5s) — too slow for F&B */
        status_lock();
        if (s_status.permit_join_remaining > 0) {
            delay = 200;
        }
        if (s_onoff_q_len > 0 || s_level_q_len > 0) {
            delay = 20;
        }
        status_unlock();
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(delay));
    }
}

static void host_wake_from_ezsp(void)
{
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

esp_err_t zigbee_host_start(void)
{
    if (!s_status_mutex) {
        s_status_mutex = xSemaphoreCreateMutex();
    }
    if (!s_op_mutex) {
        s_op_mutex = xSemaphoreCreateMutex();
    }
    if (!s_status_mutex || !s_op_mutex) {
        return ESP_ERR_NO_MEM;
    }

    memset(&s_status, 0, sizeof(s_status));
    s_status.icc_status = ICC_STATUS_UNKNOWN;
    ezsp_set_host_wake(host_wake_from_ezsp);

    if (!s_task) {
        if (xTaskCreate(host_task, "zigbee_host", 16384, NULL, 5, &s_task) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

void zigbee_host_get_status(zigbee_host_status_t *out)
{
    if (!out) {
        return;
    }
    /* Timed lock — never block the HTTP/HAP task for EZSP-length waits. */
    if (!s_status_mutex ||
        xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        memset(out, 0, sizeof(*out));
        out->icc_status = ICC_STATUS_UNKNOWN;
        snprintf(out->last_error, sizeof(out->last_error), "status busy");
        return;
    }
    ash_get_stats(&s_status.ash_stats);
    ezsp_get_stats(&s_status.ezsp_stats);
    s_status.ash_state = ash_get_state();
    update_permit_remaining();
    /* Copy header + used slots only — a full 32-device struct assign (~14KB)
     * under the lock starved the portal httpd on the unicore C3. */
    size_t hdr = offsetof(zigbee_host_status_t, devices);
    memcpy(out, &s_status, hdr);
    memset(out->devices, 0, sizeof(out->devices));
    for (uint16_t i = 0; i < ZB_HOST_MAX_DEVICES; i++) {
        if (s_status.devices[i].used) {
            out->devices[i] = s_status.devices[i];
        }
    }
    memcpy(out->last_error, s_status.last_error, sizeof(out->last_error));
    xSemaphoreGive(s_status_mutex);
}

bool zigbee_host_is_ready(void)
{
    if (!s_status_mutex ||
        xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return false;
    }
    bool ok = (s_status.icc_status == ICC_STATUS_CONNECTED) || (s_status.device_count > 0);
    xSemaphoreGive(s_status_mutex);
    return ok;
}

/** Flush deferred device NVS now (portal/HomeKit names, modes, expose flags). */
esp_err_t zigbee_host_save_devices_now(void)
{
    if (!s_op_mutex || !s_status_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    nvs_flush_devices_if_dirty(true);
    /* Always pack+commit so a backup/reboot cannot lose the last portal edits. */
    if (xSemaphoreTake(s_op_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    status_lock();
    uint16_t n = nvs_pack_devices_locked();
    if (n == 0) {
        status_unlock();
        xSemaphoreGive(s_op_mutex);
        ESP_LOGW(TAG, "Skip force NVS device save — inventory empty");
        return ESP_ERR_INVALID_STATE;
    }
    s_devices_nvs_dirty = false;
    status_unlock();
    nvs_commit_scratch(n);
    xSemaphoreGive(s_op_mutex);
    return ESP_OK;
}
