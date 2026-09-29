/**
 * @file homekit_bridge.c
 * @brief HomeKit bridge exposing Zigbee sensors and switches selected in the portal.
 */

#include "homekit_bridge.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "hap.h"
#include "hap_apple_chars.h"
#include "hap_apple_servs.h"

#include "zigbee_host.h"
#include "thermostat.h"
#include "wifi_manager.h"

static const char *TAG = "homekit";

#ifndef CONFIG_HK_SETUP_CODE
#define CONFIG_HK_SETUP_CODE "111-22-333"
#endif
#ifndef CONFIG_HK_SETUP_ID
#define CONFIG_HK_SETUP_ID "ICC1"
#endif

static homekit_bridge_status_t s_st;
static bool s_started;
static bool s_allow_config_bump; /**< false during boot attach; true for live add/remove */
static hap_acc_t *s_bridge_acc;
static TaskHandle_t s_sync_task;
static SemaphoreHandle_t s_sync_mu;
static uint32_t s_inv_sig_cached;
static bool s_inv_sig_cached_valid;

/** Bump when remote accessory service layout changes (forces rebuild). */
#define REMOTE_LAYOUT_VER 30
/**
 * One bridged accessory per remote, with each physical button as its own
 * Switch / Stateless Programmable Switch service (linked). Sub-tile names
 * come from the web UI (Name + Configured Name).
 */

/** Mark inventory dirty so announce_inventory_if_changed() bumps c# once (not per add). */
static void inventory_mark_dirty(void)
{
    s_inv_sig_cached_valid = false;
}

/** After live add/remove, bump c# + mDNS immediately so Home discovers new tiles
 * without waiting for the next sync cycle (which could miss if sig races). */
static void inventory_notify_changed(void)
{
    inventory_mark_dirty();
    if (s_started && s_allow_config_bump) {
        hap_update_config_number();
    }
}

typedef struct hk_bridged hk_bridged_t;

/** Per-button write context — serv_priv for remote Switch services. */
typedef struct {
    hk_bridged_t *parent;
    uint8_t button_index;
} hk_btn_priv_t;

struct hk_bridged {
    bool used;
    uint8_t eui64[8];
    zb_device_kind_t kind;
    hap_acc_t *acc;
    hap_char_t *temp_char;
    hap_char_t *hum_char;
    hap_char_t *batt_char;
    hap_char_t *low_batt_char;
    hap_char_t *on_char;
    hap_char_t *brightness_char;
    /** Remote: button services under a single accessory. */
    hap_char_t *btn_event_char[ZB_REMOTE_MAX_BUTTONS];
    hap_char_t *btn_on_char[ZB_REMOTE_MAX_BUTTONS];
    hap_char_t *btn_name_char[ZB_REMOTE_MAX_BUTTONS];
    hap_char_t *btn_cfg_name_char[ZB_REMOTE_MAX_BUTTONS];
    uint8_t btn_mode_snap[ZB_REMOTE_MAX_BUTTONS];
    char btn_name_snap[ZB_REMOTE_MAX_BUTTONS][24];
    hk_btn_priv_t btn_priv[ZB_REMOTE_MAX_BUTTONS];
    uint8_t button_count;
    uint8_t layout_ver; /**< REMOTE_LAYOUT_VER when created */
    char acc_name[40]; /**< Last pushed accessory Name */
    uint8_t missing_ticks;
};

typedef struct {
    bool used;
    uint8_t group_id;
    uint8_t group_type; /**< GROUP_TYPE_* */
    hap_acc_t *acc;
    hap_char_t *curr_temp_char;
    hap_char_t *targ_temp_char;
    hap_char_t *curr_state_char;
    hap_char_t *targ_state_char;
    hap_char_t *on_char;
    hap_char_t *brightness_char;
    uint8_t missing_ticks;
} hk_group_bridged_t;

#define HK_MAX_BRIDGED ZB_HOST_MAX_DEVICES
static hk_bridged_t s_bridged[HK_MAX_BRIDGED];
static hk_group_bridged_t s_groups[GROUP_MAX];

static int bridge_identify(hap_acc_t *ha)
{
    (void)ha;
    ESP_LOGI(TAG, "Bridge identify");
    return HAP_SUCCESS;
}

static int accessory_identify(hap_acc_t *ha)
{
    (void)ha;
    ESP_LOGI(TAG, "Bridged accessory identify");
    return HAP_SUCCESS;
}

static hk_bridged_t *find_bridged(const uint8_t eui64[8])
{
    for (int i = 0; i < HK_MAX_BRIDGED; i++) {
        if (s_bridged[i].used && memcmp(s_bridged[i].eui64, eui64, 8) == 0) {
            return &s_bridged[i];
        }
    }
    return NULL;
}

static hk_bridged_t *alloc_bridged(void)
{
    for (int i = 0; i < HK_MAX_BRIDGED; i++) {
        if (!s_bridged[i].used) {
            memset(&s_bridged[i], 0, sizeof(s_bridged[i]));
            return &s_bridged[i];
        }
    }
    return NULL;
}

static void remove_bridged(hk_bridged_t *slot, const char *reason);

static void remove_all_bridged_for_eui(const uint8_t eui64[8], const char *reason)
{
    for (int i = 0; i < HK_MAX_BRIDGED; i++) {
        if (s_bridged[i].used && memcmp(s_bridged[i].eui64, eui64, 8) == 0) {
            remove_bridged(&s_bridged[i], reason);
        }
    }
}

static void update_accessory_name(hk_bridged_t *slot, const char *name)
{
    if (!slot || !slot->acc || !name || !name[0]) {
        return;
    }
    if (strncmp(slot->acc_name, name, sizeof(slot->acc_name)) == 0) {
        return;
    }
    hap_serv_t *info = hap_acc_get_serv_by_uuid(slot->acc, HAP_SERV_UUID_ACCESSORY_INFORMATION);
    if (!info) {
        return;
    }
    hap_char_t *hc = hap_serv_get_char_by_uuid(info, HAP_CHAR_UUID_NAME);
    if (!hc) {
        return;
    }
    hap_val_t val = {.s = (char *)name};
    if (hap_char_update_val_silent(hc, &val) == HAP_SUCCESS) {
        snprintf(slot->acc_name, sizeof(slot->acc_name), "%s", name);
    }
}

static void update_char_float(hap_char_t *hc, float v, float lo, float hi)
{
    if (!hc) {
        return;
    }
    if (v < lo) {
        v = lo;
    } else if (v > hi) {
        v = hi;
    }
    const hap_val_t *cur = hap_char_get_val(hc);
    /* Skip tiny sensor jitter — floods HAP's event queue and drops button notifs. */
    if (cur && fabsf(cur->f - v) < 0.05f) {
        return;
    }
    hap_val_t val = {.f = v};
    hap_char_update_val(hc, &val);
}

/** Live sensor/thermostat push — re-arm EV so Home tiles update without opening detail. */
static void update_char_float_notify(hap_char_t *hc, float v, float lo, float hi)
{
    if (!hc) {
        return;
    }
    if (v < lo) {
        v = lo;
    } else if (v > hi) {
        v = hi;
    }
    const hap_val_t *cur = hap_char_get_val(hc);
    if (cur && fabsf(cur->f - v) < 0.05f) {
        return;
    }
    hap_char_enable_notif_all_sessions(hc);
    hap_val_t val = {.f = v};
    hap_char_update_val(hc, &val);
}

static void update_char_uint8(hap_char_t *hc, uint8_t v, uint8_t lo, uint8_t hi)
{
    if (!hc) {
        return;
    }
    if (v < lo) {
        v = lo;
    } else if (v > hi) {
        v = hi;
    }
    const hap_val_t *cur = hap_char_get_val(hc);
    if (cur && (uint8_t)cur->i == v) {
        return;
    }
    hap_val_t val = {.u = v};
    hap_char_update_val(hc, &val);
}

static void update_char_bool(hap_char_t *hc, bool v)
{
    if (!hc) {
        return;
    }
    const hap_val_t *cur = hap_char_get_val(hc);
    if (cur && cur->b == v) {
        return;
    }
    hap_val_t val = {.b = v};
    hap_char_update_val(hc, &val);
}

/**
 * Push an On-state change that Home must see. Re-arms EV for all live sessions
 * and forces a value edge if needed so EVENT/1.0 is queued. Do NOT close the
 * session here — that raced the EVENT write and made Home miss updates until
 * lock→unlock. Undelivered EVENTs trigger provoke from the HAP notify path.
 */
static void update_char_bool_notify(hap_char_t *hc, bool v)
{
    if (!hc) {
        return;
    }
    hap_char_enable_notif_all_sessions(hc);
    const hap_val_t *cur = hap_char_get_val(hc);
    if (cur && cur->b == v) {
        hap_val_t opp = {.b = !v};
        hap_char_update_val(hc, &opp);
    }
    hap_val_t val = {.b = v};
    hap_char_update_val(hc, &val);
}

static void update_char_int(hap_char_t *hc, int v, int lo, int hi)
{
    if (!hc) {
        return;
    }
    if (v < lo) {
        v = lo;
    } else if (v > hi) {
        v = hi;
    }
    hap_val_t val = {.i = v};
    hap_char_update_val(hc, &val);
}

/** Refresh sensor chars from Zigbee cache before Home GETs (opens detail / tile refresh). */
static int sensor_bulk_read(hap_read_data_t read_data[], int count, void *serv_priv, void *read_priv)
{
    (void)read_priv;
    hk_bridged_t *slot = (hk_bridged_t *)serv_priv;
    if (slot && slot->used) {
        zb_device_t d;
        if (zigbee_host_get_device(slot->eui64, &d)) {
            if (d.has_temp) {
                update_char_float(slot->temp_char, d.temperature_c, 0.0f, 100.0f);
            }
            if (d.has_humidity) {
                update_char_float(slot->hum_char, d.humidity_pct, 0.0f, 100.0f);
            }
            if (d.has_battery) {
                update_char_uint8(slot->batt_char, d.battery_pct, 0, 100);
                update_char_uint8(slot->low_batt_char, d.battery_pct < 20 ? 1 : 0, 0, 1);
            }
        }
    }
    for (int i = 0; i < count; i++) {
        if (read_data[i].status) {
            *(read_data[i].status) = HAP_STATUS_SUCCESS;
        }
    }
    return HAP_SUCCESS;
}

static int light_bulk_read(hap_read_data_t read_data[], int count, void *serv_priv, void *read_priv)
{
    (void)read_priv;
    hk_bridged_t *slot = (hk_bridged_t *)serv_priv;
    if (slot && slot->used) {
        zb_device_t d;
        if (zigbee_host_get_device(slot->eui64, &d)) {
            if (d.has_onoff) {
                update_char_bool(slot->on_char, d.onoff_on);
            }
            if (d.has_level) {
                update_char_int(slot->brightness_char,
                                (int)zigbee_host_level_to_brightness(d.level), 0, 100);
            }
        }
    }
    for (int i = 0; i < count; i++) {
        if (read_data[i].status) {
            *(read_data[i].status) = HAP_STATUS_SUCCESS;
        }
    }
    return HAP_SUCCESS;
}

static int switch_bulk_read(hap_read_data_t read_data[], int count, void *serv_priv, void *read_priv)
{
    (void)read_priv;
    hk_bridged_t *slot = (hk_bridged_t *)serv_priv;
    if (slot && slot->used) {
        zb_device_t d;
        if (zigbee_host_get_device(slot->eui64, &d) && d.has_onoff) {
            update_char_bool(slot->on_char, d.onoff_on);
        }
    }
    for (int i = 0; i < count; i++) {
        if (read_data[i].status) {
            *(read_data[i].status) = HAP_STATUS_SUCCESS;
        }
    }
    return HAP_SUCCESS;
}

static zb_device_kind_t effective_kind(const zb_device_t *d)
{
    return zigbee_host_device_kind(d);
}

static void format_serial(const uint8_t eui64[8], zb_device_kind_t kind, char *serial,
                          size_t serial_len)
{
    /* hap_get_unique_aid() stores this string as an NVS key (max 15 chars).
     * Zigbee EUIs are stored little-endian: unique node bytes are at the front. */
    const char *pfx = "th";
    if (kind == ZB_DEVICE_KIND_LIGHT) {
        pfx = "lb";
    } else if (kind == ZB_DEVICE_KIND_SWITCH) {
        pfx = "sw";
    } else if (kind == ZB_DEVICE_KIND_REMOTE) {
        pfx = "rd";
    }
    snprintf(serial, serial_len, "%s%02x%02x%02x%02x%02x%02x", pfx, eui64[0], eui64[1], eui64[2],
             eui64[3], eui64[4], eui64[5]);
}

static void fill_identity(const zb_device_t *d, zb_device_kind_t kind, char *name, size_t name_len,
                          char *manufacturer, size_t mfr_len, char *model, size_t model_len,
                          char *serial, size_t serial_len)
{
    if (d->name[0]) {
        snprintf(name, name_len, "%s", d->name);
    } else if (d->model[0]) {
        snprintf(name, name_len, "%s", d->model);
    } else {
        char eui[28];
        ezsp_format_eui64(d->eui64, eui, sizeof(eui));
        snprintf(name, name_len, "Zigbee %s", eui + 12);
    }
    snprintf(manufacturer, mfr_len, "%s", d->manufacturer[0] ? d->manufacturer : "Unknown");
    if (d->model[0]) {
        snprintf(model, model_len, "%s", d->model);
    } else if (kind == ZB_DEVICE_KIND_LIGHT) {
        snprintf(model, model_len, "%s", "Zigbee Light");
    } else if (kind == ZB_DEVICE_KIND_SWITCH) {
        snprintf(model, model_len, "%s", "Zigbee Switch");
    } else if (kind == ZB_DEVICE_KIND_REMOTE) {
        snprintf(model, model_len, "%s", "Zigbee Remote");
    } else {
        snprintf(model, model_len, "%s", "Zigbee Sensor");
    }
    format_serial(d->eui64, kind, serial, serial_len);
}

static int switch_write(hap_write_data_t write_data[], int count, void *serv_priv, void *write_priv)
{
    (void)write_priv;
    hk_bridged_t *slot = (hk_bridged_t *)serv_priv;
    int ret = HAP_SUCCESS;
    for (int i = 0; i < count; i++) {
        hap_write_data_t *w = &write_data[i];
        if (!strcmp(hap_char_get_type_uuid(w->hc), HAP_CHAR_UUID_ON)) {
            bool on = w->val.b;
            if (!slot) {
                *(w->status) = HAP_STATUS_OO_RES;
                ret = HAP_FAIL;
                continue;
            }
            /* Queue Zigbee command — never block HAP on EZSP/NVS or Home marks
             * the accessory unresponsive and it can vanish from the Home app. */
            esp_err_t err = zigbee_host_set_onoff(slot->eui64, on);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "On/Off queue failed: %s", esp_err_to_name(err));
                *(w->status) = HAP_STATUS_OO_RES;
                ret = HAP_FAIL;
                continue;
            }
            hap_char_update_val(w->hc, &w->val);
            *(w->status) = HAP_STATUS_SUCCESS;
            ESP_LOGI(TAG, "HomeKit On/Off -> %s (queued)", on ? "ON" : "OFF");
        } else {
            *(w->status) = HAP_STATUS_RES_ABSENT;
            ret = HAP_FAIL;
        }
    }
    return ret;
}

/** Home app toggled a stateful remote-button Switch — persist latch only (no Zigbee On/Off). */
static int remote_btn_switch_write(hap_write_data_t write_data[], int count, void *serv_priv,
                                   void *write_priv)
{
    (void)write_priv;
    hk_btn_priv_t *ctx = (hk_btn_priv_t *)serv_priv;
    int ret = HAP_SUCCESS;
    if (!ctx || !ctx->parent || !ctx->parent->used) {
        for (int i = 0; i < count; i++) {
            *(write_data[i].status) = HAP_STATUS_OO_RES;
        }
        return HAP_FAIL;
    }
    hk_bridged_t *slot = ctx->parent;
    uint8_t btn = ctx->button_index;
    for (int i = 0; i < count; i++) {
        hap_write_data_t *w = &write_data[i];
        const char *uuid = hap_char_get_type_uuid(w->hc);
        if (!strcmp(uuid, HAP_CHAR_UUID_ON)) {
            if (btn >= ZB_REMOTE_MAX_BUTTONS) {
                *(w->status) = HAP_STATUS_OO_RES;
                ret = HAP_FAIL;
                continue;
            }
            esp_err_t err = zigbee_host_btn_set_on(slot->eui64, btn, w->val.b);
            if (err != ESP_OK) {
                *(w->status) = HAP_STATUS_OO_RES;
                ret = HAP_FAIL;
                continue;
            }
            hap_char_update_val(w->hc, &w->val);
            *(w->status) = HAP_STATUS_SUCCESS;
            ESP_LOGI(TAG, "HomeKit remote btn %u latch -> %s", (unsigned)btn,
                     w->val.b ? "ON" : "OFF");
        } else if (!strcmp(uuid, HAP_CHAR_UUID_CONFIGURED_NAME)) {
            /* Allow Home to ack rename; portal names remain the source of truth. */
            hap_char_update_val(w->hc, &w->val);
            *(w->status) = HAP_STATUS_SUCCESS;
        } else {
            *(w->status) = HAP_STATUS_RES_ABSENT;
            ret = HAP_FAIL;
        }
    }
    return ret;
}

static int light_write(hap_write_data_t write_data[], int count, void *serv_priv, void *write_priv)
{
    (void)write_priv;
    hk_bridged_t *slot = (hk_bridged_t *)serv_priv;
    int ret = HAP_SUCCESS;
    if (!slot) {
        for (int i = 0; i < count; i++) {
            *(write_data[i].status) = HAP_STATUS_OO_RES;
        }
        return HAP_FAIL;
    }
    for (int i = 0; i < count; i++) {
        hap_write_data_t *w = &write_data[i];
        const char *uuid = hap_char_get_type_uuid(w->hc);
        if (!strcmp(uuid, HAP_CHAR_UUID_ON)) {
            esp_err_t err = zigbee_host_set_onoff(slot->eui64, w->val.b);
            if (err != ESP_OK) {
                *(w->status) = HAP_STATUS_OO_RES;
                ret = HAP_FAIL;
                continue;
            }
            hap_char_update_val(w->hc, &w->val);
            *(w->status) = HAP_STATUS_SUCCESS;
        } else if (!strcmp(uuid, HAP_CHAR_UUID_BRIGHTNESS)) {
            int pct = w->val.i;
            if (pct < 0) {
                pct = 0;
            }
            if (pct > 100) {
                pct = 100;
            }
            esp_err_t err = zigbee_host_set_brightness(slot->eui64, (uint8_t)pct);
            if (err != ESP_OK) {
                *(w->status) = HAP_STATUS_OO_RES;
                ret = HAP_FAIL;
                continue;
            }
            hap_char_update_val(w->hc, &w->val);
            *(w->status) = HAP_STATUS_SUCCESS;
            ESP_LOGI(TAG, "HomeKit brightness -> %d%% (queued)", pct);
        } else {
            *(w->status) = HAP_STATUS_RES_ABSENT;
            ret = HAP_FAIL;
        }
    }
    return ret;
}

static esp_err_t add_bridged_light(const zb_device_t *d)
{
    if (!d || !d->used || !d->homekit_expose) {
        return ESP_ERR_INVALID_ARG;
    }
    if (find_bridged(d->eui64)) {
        return ESP_OK;
    }
    hk_bridged_t *slot = alloc_bridged();
    if (!slot) {
        return ESP_ERR_NO_MEM;
    }

    char name[40], serial[20], manufacturer[32], model[32];
    fill_identity(d, ZB_DEVICE_KIND_LIGHT, name, sizeof(name), manufacturer, sizeof(manufacturer),
                  model, sizeof(model), serial, sizeof(serial));
    const char *fw_rev = "1.0.0";

    hap_acc_cfg_t cfg = {
        .name = name,
        .manufacturer = manufacturer,
        .model = model,
        .serial_num = serial,
        .fw_rev = (char *)fw_rev,
        .hw_rev = "1.0",
        .pv = "1.1.0",
        .cid = HAP_CID_LIGHTING,
        .identify_routine = accessory_identify,
    };

    hap_acc_t *acc = hap_acc_create(&cfg);
    if (!acc) {
        return ESP_ERR_NO_MEM;
    }

    bool on = d->has_onoff ? d->onoff_on : (d->has_level && d->level > 0);
    int bright = d->has_level ? (int)zigbee_host_level_to_brightness(d->level) : 100;
    if (bright < 1 && on) {
        bright = 100;
    }

    hap_serv_t *lb = hap_serv_lightbulb_create(on);
    if (!lb) {
        hap_acc_delete(acc);
        return ESP_ERR_NO_MEM;
    }
    hap_serv_add_char(lb, hap_char_name_create("Light"));
    hap_serv_add_char(lb, hap_char_brightness_create(bright));
    hap_serv_set_write_cb(lb, light_write);
    hap_serv_set_bulk_read_cb(lb, light_bulk_read);
    hap_serv_set_priv(lb, slot);
    hap_acc_add_serv(acc, lb);
    slot->on_char = hap_serv_get_char_by_uuid(lb, HAP_CHAR_UUID_ON);
    slot->brightness_char = hap_serv_get_char_by_uuid(lb, HAP_CHAR_UUID_BRIGHTNESS);

    int aid = hap_get_unique_aid(serial);
    hap_add_bridged_accessory(acc, aid);
    inventory_notify_changed();

    slot->used = true;
    slot->kind = ZB_DEVICE_KIND_LIGHT;
    slot->missing_ticks = 0;
    memcpy(slot->eui64, d->eui64, 8);
    slot->acc = acc;
    snprintf(slot->acc_name, sizeof(slot->acc_name), "%s", name);
    s_st.accessory_count++;
    ESP_LOGI(TAG, "HomeKit exposed light: %s mfr=%s model=%s aid=%d serial=%s bright=%d", name,
             manufacturer, model, aid, serial, bright);
    return ESP_OK;
}

static esp_err_t add_bridged_switch(const zb_device_t *d)
{
    if (!d || !d->used || !d->homekit_expose) {
        return ESP_ERR_INVALID_ARG;
    }
    if (find_bridged(d->eui64)) {
        return ESP_OK;
    }
    hk_bridged_t *slot = alloc_bridged();
    if (!slot) {
        return ESP_ERR_NO_MEM;
    }

    char name[40], serial[20], manufacturer[32], model[32];
    fill_identity(d, ZB_DEVICE_KIND_SWITCH, name, sizeof(name), manufacturer, sizeof(manufacturer),
                  model, sizeof(model), serial, sizeof(serial));
    const char *fw_rev = "1.0.0";

    hap_acc_cfg_t cfg = {
        .name = name,
        .manufacturer = manufacturer,
        .model = model,
        .serial_num = serial,
        .fw_rev = (char *)fw_rev,
        .hw_rev = "1.0",
        .pv = "1.1.0",
        .cid = HAP_CID_SWITCH,
        .identify_routine = accessory_identify,
    };

    hap_acc_t *acc = hap_acc_create(&cfg);
    if (!acc) {
        return ESP_ERR_NO_MEM;
    }

    bool on = d->has_onoff ? d->onoff_on : false;
    hap_serv_t *sw_serv = hap_serv_switch_create(on);
    if (!sw_serv) {
        hap_acc_delete(acc);
        return ESP_ERR_NO_MEM;
    }
    hap_serv_add_char(sw_serv, hap_char_name_create("Switch"));
    hap_serv_set_write_cb(sw_serv, switch_write);
    hap_serv_set_bulk_read_cb(sw_serv, switch_bulk_read);
    hap_serv_set_priv(sw_serv, slot);
    hap_acc_add_serv(acc, sw_serv);
    slot->on_char = hap_serv_get_char_by_uuid(sw_serv, HAP_CHAR_UUID_ON);

    int aid = hap_get_unique_aid(serial);
    hap_add_bridged_accessory(acc, aid);
    inventory_notify_changed();

    slot->used = true;
    slot->kind = ZB_DEVICE_KIND_SWITCH;
    slot->missing_ticks = 0;
    memcpy(slot->eui64, d->eui64, 8);
    slot->acc = acc;
    snprintf(slot->acc_name, sizeof(slot->acc_name), "%s", name);
    s_st.accessory_count++;
    ESP_LOGI(TAG, "HomeKit exposed switch: %s mfr=%s model=%s aid=%d serial=%s", name, manufacturer,
             model, aid, serial);
    return ESP_OK;
}

static esp_err_t add_bridged_sensor(const zb_device_t *d)
{
    if (!d || !d->used || !d->homekit_expose) {
        return ESP_ERR_INVALID_ARG;
    }
    if (find_bridged(d->eui64)) {
        return ESP_OK;
    }
    hk_bridged_t *slot = alloc_bridged();
    if (!slot) {
        return ESP_ERR_NO_MEM;
    }

    char name[40], serial[20], manufacturer[32], model[32];
    fill_identity(d, ZB_DEVICE_KIND_SENSOR, name, sizeof(name), manufacturer, sizeof(manufacturer),
                  model, sizeof(model), serial, sizeof(serial));
    /* Keep HAP firmware revision stable. Changing it (e.g. Zigbee SW build) makes Home
     * reconfigure the accessory and can show "Unsupported" / drop sensor services. */
    const char *fw_rev = "1.0.0";

    hap_acc_cfg_t cfg = {
        .name = name,
        .manufacturer = manufacturer,
        .model = model,
        .serial_num = serial,
        .fw_rev = (char *)fw_rev,
        .hw_rev = "1.0",
        .pv = "1.1.0",
        .cid = HAP_CID_SENSOR,
        .identify_routine = accessory_identify,
    };

    hap_acc_t *acc = hap_acc_create(&cfg);
    if (!acc) {
        return ESP_ERR_NO_MEM;
    }

    float temp = d->has_temp ? d->temperature_c : 20.0f;
    float hum = d->has_humidity ? d->humidity_pct : 50.0f;
    uint8_t batt = d->has_battery ? d->battery_pct : 100;
    uint8_t low = (d->has_battery && d->battery_pct < 20) ? 1 : 0;

    /* Always expose the same services in the same order so IIDs stay stable across
     * boots. Conditional services make Home show "Unsupported" and drop sensors. */
    hap_serv_t *temp_serv = hap_serv_temperature_sensor_create(temp);
    if (temp_serv) {
        hap_serv_add_char(temp_serv, hap_char_name_create("Temperature"));
        hap_serv_set_bulk_read_cb(temp_serv, sensor_bulk_read);
        hap_serv_set_priv(temp_serv, slot);
        hap_acc_add_serv(acc, temp_serv);
        slot->temp_char = hap_serv_get_char_by_uuid(temp_serv, HAP_CHAR_UUID_CURRENT_TEMPERATURE);
    }

    hap_serv_t *hum_serv = hap_serv_humidity_sensor_create(hum);
    if (hum_serv) {
        hap_serv_add_char(hum_serv, hap_char_name_create("Humidity"));
        hap_serv_set_bulk_read_cb(hum_serv, sensor_bulk_read);
        hap_serv_set_priv(hum_serv, slot);
        hap_acc_add_serv(acc, hum_serv);
        slot->hum_char =
            hap_serv_get_char_by_uuid(hum_serv, HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY);
    }

    hap_serv_t *batt_serv = hap_serv_battery_service_create(batt, 0 /* not charging */, low);
    if (batt_serv) {
        hap_serv_set_bulk_read_cb(batt_serv, sensor_bulk_read);
        hap_serv_set_priv(batt_serv, slot);
        hap_acc_add_serv(acc, batt_serv);
        slot->batt_char = hap_serv_get_char_by_uuid(batt_serv, HAP_CHAR_UUID_BATTERY_LEVEL);
        slot->low_batt_char =
            hap_serv_get_char_by_uuid(batt_serv, HAP_CHAR_UUID_STATUS_LOW_BATTERY);
    }

    int aid = hap_get_unique_aid(serial);
    hap_add_bridged_accessory(acc, aid);
    inventory_notify_changed();

    slot->used = true;
    slot->kind = ZB_DEVICE_KIND_SENSOR;
    slot->missing_ticks = 0;
    memcpy(slot->eui64, d->eui64, 8);
    slot->acc = acc;
    snprintf(slot->acc_name, sizeof(slot->acc_name), "%s", name);
    s_st.accessory_count++;
    ESP_LOGI(TAG, "HomeKit exposed sensor: %s mfr=%s model=%s aid=%d serial=%s (temp+humidity+batt)",
             name, manufacturer, model, aid, serial);
    return ESP_OK;
}

static void update_char_string(hap_char_t *hc, const char *s)
{
    if (!hc || !s) {
        return;
    }
    const hap_val_t *cur = hap_char_get_val(hc);
    if (cur && cur->s && strcmp(cur->s, s) == 0) {
        return;
    }
    hap_val_t val = {.s = (char *)s};
    /* Silent — name EVENTs provoked session closes and wedged Wi‑Fi/httpd. */
    hap_char_update_val_silent(hc, &val);
}

static const char *remote_button_name(uint8_t nbtn, uint8_t idx)
{
    return zigbee_host_remote_button_default_name(nbtn, idx);
}

static void fill_btn_service_name(const zb_device_t *d, uint8_t nbtn, uint8_t idx, char *out,
                                  size_t out_len)
{
    if (!out || out_len == 0) {
        return;
    }
    const char *nm = d ? zigbee_host_remote_button_name(d, idx) : remote_button_name(nbtn, idx);
    snprintf(out, out_len, "%s", nm && nm[0] ? nm : remote_button_name(nbtn, idx));
}

static int count_bridged_for_eui(const uint8_t eui64[8])
{
    int n = 0;
    for (int i = 0; i < HK_MAX_BRIDGED; i++) {
        if (s_bridged[i].used && memcmp(s_bridged[i].eui64, eui64, 8) == 0) {
            n++;
        }
    }
    return n;
}

static hk_bridged_t *find_bridged_remote(const uint8_t eui64[8])
{
    for (int i = 0; i < HK_MAX_BRIDGED; i++) {
        if (s_bridged[i].used && s_bridged[i].kind == ZB_DEVICE_KIND_REMOTE &&
            memcmp(s_bridged[i].eui64, eui64, 8) == 0) {
            return &s_bridged[i];
        }
    }
    return NULL;
}

static bool remote_btn_layout_ok(const zb_device_t *d, uint8_t nbtn)
{
    if (count_bridged_for_eui(d->eui64) != 1) {
        return false;
    }
    hk_bridged_t *slot = find_bridged_remote(d->eui64);
    if (!slot || slot->layout_ver != REMOTE_LAYOUT_VER || slot->button_count != nbtn) {
        return false;
    }
    for (uint8_t bi = 0; bi < nbtn; bi++) {
        uint8_t want = (d->btn_mode[bi] == ZB_BTN_MODE_STATEFUL) ? ZB_BTN_MODE_STATEFUL
                                                                 : ZB_BTN_MODE_STATELESS;
        if (slot->btn_mode_snap[bi] != want) {
            return false;
        }
    }
    return true;
}

/**
 * One HomeKit accessory for the remote. Each button is a Switch (stateful) or
 * Stateless Programmable Switch service under it; Home shows them as sub-tiles
 * named from the web UI (Name + Configured Name).
 */
static esp_err_t add_bridged_remote(const zb_device_t *d)
{
    if (!d || !d->used || !d->homekit_expose) {
        return ESP_ERR_INVALID_ARG;
    }
    if (d->remote_type == ZB_REMOTE_TYPE_CONTROL) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    uint8_t nbtn = zigbee_host_remote_button_count(d);
    if (nbtn == 0 || nbtn > ZB_REMOTE_MAX_BUTTONS) {
        nbtn = 5;
    }

    if (remote_btn_layout_ok(d, nbtn)) {
        return ESP_OK;
    }
    remove_all_bridged_for_eui(d->eui64, "remote multi-button layout refresh");

    hk_bridged_t *slot = alloc_bridged();
    if (!slot) {
        return ESP_ERR_NO_MEM;
    }

    char name[40], serial[20], manufacturer[32], model[32];
    fill_identity(d, ZB_DEVICE_KIND_REMOTE, name, sizeof(name), manufacturer, sizeof(manufacturer),
                  model, sizeof(model), serial, sizeof(serial));
    const char *fw_rev = "1.2.0";

    bool any_stateful = false;
    for (uint8_t i = 0; i < nbtn; i++) {
        if (d->btn_mode[i] == ZB_BTN_MODE_STATEFUL) {
            any_stateful = true;
            break;
        }
    }

    hap_acc_cfg_t cfg = {
        .name = name,
        .manufacturer = manufacturer,
        .model = model,
        .serial_num = serial,
        .fw_rev = (char *)fw_rev,
        .hw_rev = "1.0",
        .pv = "1.1.0",
        .cid = any_stateful ? HAP_CID_SWITCH : HAP_CID_PROGRAMMABLE_SWITCH,
        .identify_routine = accessory_identify,
    };

    hap_acc_t *acc = hap_acc_create(&cfg);
    if (!acc) {
        return ESP_ERR_NO_MEM;
    }

    hap_serv_t *primary = NULL;
    hap_serv_t *btn_servs[ZB_REMOTE_MAX_BUTTONS] = {0};

    for (uint8_t i = 0; i < nbtn; i++) {
        uint8_t mode = (d->btn_mode[i] == ZB_BTN_MODE_STATEFUL) ? ZB_BTN_MODE_STATEFUL
                                                                : ZB_BTN_MODE_STATELESS;
        char bname[24];
        fill_btn_service_name(d, nbtn, i, bname, sizeof(bname));

        slot->btn_priv[i].parent = slot;
        slot->btn_priv[i].button_index = i;
        slot->btn_mode_snap[i] = mode;
        snprintf(slot->btn_name_snap[i], sizeof(slot->btn_name_snap[i]), "%s", bname);
        slot->btn_event_char[i] = NULL;
        slot->btn_on_char[i] = NULL;
        slot->btn_name_char[i] = NULL;
        slot->btn_cfg_name_char[i] = NULL;

        hap_char_t *name_ch = hap_char_name_create(bname);
        hap_char_t *cfg_ch = hap_char_configured_name_create(bname);
        if (!name_ch || !cfg_ch) {
            if (name_ch) {
                hap_char_delete(name_ch);
            }
            if (cfg_ch) {
                hap_char_delete(cfg_ch);
            }
            hap_acc_delete(acc);
            return ESP_ERR_NO_MEM;
        }
        hap_char_string_set_maxlen(name_ch, 23);
        hap_char_string_set_maxlen(cfg_ch, 23);

        hap_serv_t *serv = NULL;
        if (mode == ZB_BTN_MODE_STATEFUL) {
            serv = hap_serv_switch_create(d->btn_on[i]);
            if (!serv) {
                hap_char_delete(name_ch);
                hap_char_delete(cfg_ch);
                hap_acc_delete(acc);
                return ESP_ERR_NO_MEM;
            }
            hap_serv_add_char(serv, name_ch);
            hap_serv_add_char(serv, cfg_ch);
            hap_serv_set_write_cb(serv, remote_btn_switch_write);
            hap_serv_set_priv(serv, &slot->btn_priv[i]);
            slot->btn_on_char[i] = hap_serv_get_char_by_uuid(serv, HAP_CHAR_UUID_ON);
        } else {
            serv = hap_serv_stateless_programmable_switch_create(0);
            if (!serv) {
                hap_char_delete(name_ch);
                hap_char_delete(cfg_ch);
                hap_acc_delete(acc);
                return ESP_ERR_NO_MEM;
            }
            hap_serv_add_char(serv, name_ch);
            hap_serv_add_char(serv, cfg_ch);
            /* Service Label Index is for SPS / multi-instance; not on Switch. */
            hap_serv_add_char(serv, hap_char_service_label_index_create((uint8_t)(i + 1)));
            hap_serv_set_write_cb(serv, remote_btn_switch_write);
            hap_serv_set_priv(serv, &slot->btn_priv[i]);
            hap_char_t *ev =
                hap_serv_get_char_by_uuid(serv, HAP_CHAR_UUID_PROGRAMMABLE_SWITCH_EVENT);
            static const uint8_t k_events[] = {0, 1, 2};
            if (ev) {
                hap_char_add_valid_vals(ev, k_events, sizeof(k_events));
            }
            slot->btn_event_char[i] = ev;
        }
        slot->btn_name_char[i] = name_ch;
        slot->btn_cfg_name_char[i] = cfg_ch;
        btn_servs[i] = serv;
        hap_acc_add_serv(acc, serv);
        if (!primary) {
            primary = serv;
            hap_serv_mark_primary(serv);
        }
    }

    if (primary) {
        for (uint8_t i = 1; i < nbtn; i++) {
            if (btn_servs[i]) {
                hap_serv_link_serv(primary, btn_servs[i]);
            }
        }
    }

    /* Only needed when any button is a stateless programmable switch. */
    bool any_sps = false;
    for (uint8_t i = 0; i < nbtn; i++) {
        if (d->btn_mode[i] != ZB_BTN_MODE_STATEFUL) {
            any_sps = true;
            break;
        }
    }
    if (any_sps) {
        hap_serv_t *label_serv = hap_serv_service_label_create(1); /* dots */
        if (label_serv) {
            hap_acc_add_serv(acc, label_serv);
        }
    }

    uint8_t batt = d->has_battery ? d->battery_pct : 100;
    uint8_t low = (d->has_battery && d->battery_pct < 20) ? 1 : 0;
    hap_serv_t *batt_serv = hap_serv_battery_service_create(batt, 0, low);
    if (batt_serv) {
        hap_acc_add_serv(acc, batt_serv);
        slot->batt_char = hap_serv_get_char_by_uuid(batt_serv, HAP_CHAR_UUID_BATTERY_LEVEL);
        slot->low_batt_char =
            hap_serv_get_char_by_uuid(batt_serv, HAP_CHAR_UUID_STATUS_LOW_BATTERY);
    }

    int aid = hap_get_unique_aid(serial);
    hap_add_bridged_accessory(acc, aid);
    inventory_notify_changed();

    slot->used = true;
    slot->kind = ZB_DEVICE_KIND_REMOTE;
    slot->button_count = nbtn;
    slot->layout_ver = REMOTE_LAYOUT_VER;
    slot->missing_ticks = 0;
    slot->acc = acc;
    slot->on_char = NULL;
    memcpy(slot->eui64, d->eui64, 8);
    snprintf(slot->acc_name, sizeof(slot->acc_name), "%s", name);
    s_st.accessory_count++;
    ESP_LOGI(TAG, "HomeKit remote '%s' (%u buttons) aid=%d serial=%s", name, (unsigned)nbtn, aid,
             serial);
    return ESP_OK;
}

static void on_remote_button(const uint8_t eui64[8], uint8_t button_index, uint8_t event)
{
    if (!eui64) {
        return;
    }
    zb_device_t d;
    bool have_d = zigbee_host_get_device(eui64, &d);
    uint8_t mode = ZB_BTN_MODE_STATELESS;
    if (have_d && button_index < ZB_REMOTE_MAX_BUTTONS) {
        mode = (d.btn_mode[button_index] == ZB_BTN_MODE_STATEFUL) ? ZB_BTN_MODE_STATEFUL
                                                                  : ZB_BTN_MODE_STATELESS;
    }

    if (!s_started) {
        if (mode == ZB_BTN_MODE_STATEFUL) {
            (void)zigbee_host_btn_toggle(eui64, button_index);
        }
        return;
    }

    if (s_sync_mu) {
        xSemaphoreTake(s_sync_mu, portMAX_DELAY);
    }
    hk_bridged_t *slot = find_bridged_remote(eui64);
    if (!slot || slot->kind != ZB_DEVICE_KIND_REMOTE || button_index >= slot->button_count ||
        button_index >= ZB_REMOTE_MAX_BUTTONS) {
        if (s_sync_mu) {
            xSemaphoreGive(s_sync_mu);
        }
        if (mode == ZB_BTN_MODE_STATEFUL) {
            (void)zigbee_host_btn_toggle(eui64, button_index);
        }
        ESP_LOGW(TAG, "Remote btn %u: no HomeKit accessory for this eui", (unsigned)button_index);
        return;
    }

    if (mode == ZB_BTN_MODE_STATEFUL) {
        bool on = zigbee_host_btn_toggle(eui64, button_index);
        hap_char_t *hc = slot->btn_on_char[button_index];
        /* Release before EVENT I/O — holding s_sync_mu during hap_http_send_notif
         * blocked the HK sync task and stalled controller sessions. */
        if (s_sync_mu) {
            xSemaphoreGive(s_sync_mu);
        }
        if (!hc) {
            ESP_LOGW(TAG, "Remote btn %u stateful but On char NULL", (unsigned)button_index);
            return;
        }
        update_char_bool_notify(hc, on);
        /* Bridged Switch EVENTs often never reach an idle iPhone; lock→unlock
         * was the only refresh. Provoke the same reconnect after remote presses. */
        hap_provoke_controller_refresh();
        ESP_LOGI(TAG, "HomeKit remote button %u stateful toggle -> %s", (unsigned)button_index,
                 on ? "ON" : "OFF");
        return;
    }
    hap_char_t *ev = slot->btn_event_char[button_index];
    if (s_sync_mu) {
        xSemaphoreGive(s_sync_mu);
    }
    if (!ev) {
        ESP_LOGW(TAG, "Remote btn %u stateless but event char NULL", (unsigned)button_index);
        return;
    }
    if (event > HK_BTN_EVENT_LONG) {
        event = HK_BTN_EVENT_SINGLE;
    }
    hap_char_enable_notif_all_sessions(ev);
    hap_val_t val = {.i = (int)event};
    hap_char_update_val(ev, &val);
    hap_provoke_controller_refresh();
    ESP_LOGI(TAG, "HomeKit remote button %u event %u (single=0 double=1 long=2)",
             (unsigned)button_index, (unsigned)event);
}

/** Push live sensor readings (and thermostats that use this sensor) into HomeKit. */
static void on_sensor_update(const uint8_t eui64[8])
{
    if (!eui64 || !s_started) {
        return;
    }

    /* Thermostat heat decision first — updates group runtime from the sensor. */
    group_on_sensor_updated(eui64);

    zb_device_t d;
    if (!zigbee_host_get_device(eui64, &d)) {
        return;
    }

    hap_char_t *temp_hc = NULL;
    hap_char_t *hum_hc = NULL;
    hap_char_t *batt_hc = NULL;
    hap_char_t *low_hc = NULL;
    hap_char_t *thermo_temp[GROUP_MAX];
    hap_char_t *thermo_heat[GROUP_MAX];
    float thermo_t[GROUP_MAX];
    uint8_t thermo_h[GROUP_MAX];
    uint8_t nthermo = 0;

    if (s_sync_mu) {
        xSemaphoreTake(s_sync_mu, portMAX_DELAY);
    }
    for (int i = 0; i < HK_MAX_BRIDGED; i++) {
        if (s_bridged[i].used && s_bridged[i].kind == ZB_DEVICE_KIND_SENSOR &&
            memcmp(s_bridged[i].eui64, eui64, 8) == 0) {
            temp_hc = s_bridged[i].temp_char;
            hum_hc = s_bridged[i].hum_char;
            batt_hc = s_bridged[i].batt_char;
            low_hc = s_bridged[i].low_batt_char;
            break;
        }
    }
    for (uint16_t ti = 0; ti < GROUP_MAX && nthermo < GROUP_MAX; ti++) {
        if (!s_groups[ti].used || s_groups[ti].group_type != GROUP_TYPE_THERMOSTAT) {
            continue;
        }
        group_t t;
        if (!group_get_by_id(s_groups[ti].group_id, &t)) {
            continue;
        }
        if (memcmp(t.sensor_eui, eui64, 8) != 0) {
            continue;
        }
        thermo_temp[nthermo] = t.has_current_temp ? s_groups[ti].curr_temp_char : NULL;
        thermo_heat[nthermo] = s_groups[ti].curr_state_char;
        thermo_t[nthermo] = t.current_temp_c;
        thermo_h[nthermo] = t.heating ? 1 : 0;
        if (thermo_temp[nthermo] || thermo_heat[nthermo]) {
            nthermo++;
        }
    }
    if (s_sync_mu) {
        xSemaphoreGive(s_sync_mu);
    }

    if (d.has_temp) {
        update_char_float_notify(temp_hc, d.temperature_c, 0.0f, 100.0f);
    }
    if (d.has_humidity) {
        update_char_float_notify(hum_hc, d.humidity_pct, 0.0f, 100.0f);
    }
    if (d.has_battery) {
        if (batt_hc) {
            hap_char_enable_notif_all_sessions(batt_hc);
            update_char_uint8(batt_hc, d.battery_pct, 0, 100);
        }
        if (low_hc) {
            hap_char_enable_notif_all_sessions(low_hc);
            update_char_uint8(low_hc, d.battery_pct < 20 ? 1 : 0, 0, 1);
        }
    }
    for (uint8_t k = 0; k < nthermo; k++) {
        if (thermo_temp[k]) {
            update_char_float_notify(thermo_temp[k], thermo_t[k], 0.0f, 100.0f);
        }
        if (thermo_heat[k]) {
            hap_char_enable_notif_all_sessions(thermo_heat[k]);
            update_char_uint8(thermo_heat[k], thermo_h[k], 0, 2);
        }
    }
}

static esp_err_t add_bridged_device(const zb_device_t *d)
{
    zb_device_kind_t k = effective_kind(d);
    if (k == ZB_DEVICE_KIND_LIGHT) {
        return add_bridged_light(d);
    }
    if (k == ZB_DEVICE_KIND_SWITCH) {
        return add_bridged_switch(d);
    }
    if (k == ZB_DEVICE_KIND_SENSOR) {
        return add_bridged_sensor(d);
    }
    if (k == ZB_DEVICE_KIND_REMOTE) {
        return add_bridged_remote(d);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static hk_group_bridged_t *find_group(uint8_t id)
{
    for (int i = 0; i < GROUP_MAX; i++) {
        if (s_groups[i].used && s_groups[i].group_id == id) {
            return &s_groups[i];
        }
    }
    return NULL;
}

static hk_group_bridged_t *alloc_group(void)
{
    for (int i = 0; i < GROUP_MAX; i++) {
        if (!s_groups[i].used) {
            memset(&s_groups[i], 0, sizeof(s_groups[i]));
            return &s_groups[i];
        }
    }
    return NULL;
}

static int thermo_write(hap_write_data_t write_data[], int count, void *serv_priv, void *write_priv)
{
    (void)write_priv;
    hk_group_bridged_t *slot = (hk_group_bridged_t *)serv_priv;
    int ret = HAP_SUCCESS;
    if (!slot) {
        for (int i = 0; i < count; i++) {
            *(write_data[i].status) = HAP_STATUS_OO_RES;
        }
        return HAP_FAIL;
    }
    for (int i = 0; i < count; i++) {
        hap_write_data_t *w = &write_data[i];
        const char *uuid = hap_char_get_type_uuid(w->hc);
        if (!strcmp(uuid, HAP_CHAR_UUID_TARGET_TEMPERATURE)) {
            esp_err_t err = thermostat_set_target(slot->group_id, w->val.f);
            if (err != ESP_OK) {
                *(w->status) = HAP_STATUS_OO_RES;
                ret = HAP_FAIL;
                continue;
            }
            hap_char_update_val(w->hc, &w->val);
            *(w->status) = HAP_STATUS_SUCCESS;
        } else if (!strcmp(uuid, HAP_CHAR_UUID_TARGET_HEATING_COOLING_STATE)) {
            uint8_t mode = (w->val.u == 1) ? THERMO_MODE_HEAT : THERMO_MODE_OFF;
            esp_err_t err = thermostat_set_mode(slot->group_id, mode);
            if (err != ESP_OK) {
                *(w->status) = HAP_STATUS_OO_RES;
                ret = HAP_FAIL;
                continue;
            }
            hap_char_update_val(w->hc, &w->val);
            *(w->status) = HAP_STATUS_SUCCESS;
        } else if (!strcmp(uuid, HAP_CHAR_UUID_TEMPERATURE_DISPLAY_UNITS)) {
            hap_val_t c = {.u = 0};
            hap_char_update_val(w->hc, &c);
            *(w->status) = HAP_STATUS_SUCCESS;
        } else {
            *(w->status) = HAP_STATUS_RES_ABSENT;
            ret = HAP_FAIL;
        }
    }
    return ret;
}

static int group_general_write(hap_write_data_t write_data[], int count, void *serv_priv,
                               void *write_priv)
{
    (void)write_priv;
    hk_group_bridged_t *slot = (hk_group_bridged_t *)serv_priv;
    int ret = HAP_SUCCESS;
    if (!slot) {
        for (int i = 0; i < count; i++) {
            *(write_data[i].status) = HAP_STATUS_OO_RES;
        }
        return HAP_FAIL;
    }
    for (int i = 0; i < count; i++) {
        hap_write_data_t *w = &write_data[i];
        const char *uuid = hap_char_get_type_uuid(w->hc);
        if (!strcmp(uuid, HAP_CHAR_UUID_ON)) {
            ESP_LOGI(TAG, "HomeKit group %u On -> %s", (unsigned)slot->group_id,
                     w->val.b ? "ON" : "OFF");
            if (group_set_onoff(slot->group_id, w->val.b) != ESP_OK) {
                *(w->status) = HAP_STATUS_OO_RES;
                ret = HAP_FAIL;
                continue;
            }
            hap_char_update_val(w->hc, &w->val);
            *(w->status) = HAP_STATUS_SUCCESS;
        } else if (!strcmp(uuid, HAP_CHAR_UUID_BRIGHTNESS)) {
            int pct = w->val.i;
            if (pct < 0) {
                pct = 0;
            }
            if (pct > 100) {
                pct = 100;
            }
            ESP_LOGI(TAG, "HomeKit group %u Brightness -> %d%%", (unsigned)slot->group_id, pct);
            if (group_set_brightness(slot->group_id, (uint8_t)pct) != ESP_OK) {
                *(w->status) = HAP_STATUS_OO_RES;
                ret = HAP_FAIL;
                continue;
            }
            hap_char_update_val(w->hc, &w->val);
            *(w->status) = HAP_STATUS_SUCCESS;
            /* Keep On in sync with brightness. */
            if (slot->on_char) {
                hap_val_t onv = {.b = pct > 0};
                hap_char_update_val(slot->on_char, &onv);
            }
        } else {
            *(w->status) = HAP_STATUS_RES_ABSENT;
            ret = HAP_FAIL;
        }
    }
    return ret;
}

static void remove_group_acc(hk_group_bridged_t *slot, const char *reason)
{
    if (!slot || !slot->used) {
        return;
    }
    ESP_LOGW(TAG, "HomeKit removing group (%s)", reason ? reason : "unknown");
    if (slot->acc) {
        hap_remove_bridged_accessory(slot->acc);
        slot->acc = NULL;
        inventory_notify_changed();
    }
    memset(slot, 0, sizeof(*slot));
    if (s_st.accessory_count > 0) {
        s_st.accessory_count--;
    }
}

static esp_err_t add_bridged_thermostat(const group_t *t)
{
    if (!t || !t->used || !t->homekit_expose || t->type != GROUP_TYPE_THERMOSTAT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (find_group(t->id)) {
        return ESP_OK;
    }
    hk_group_bridged_t *slot = alloc_group();
    if (!slot) {
        return ESP_ERR_NO_MEM;
    }

    char name[40];
    snprintf(name, sizeof(name), "%s", t->name[0] ? t->name : "Thermostat");
    char serial[16];
    snprintf(serial, sizeof(serial), "tm%02x%02x%02x%02x", t->id, t->sensor_eui[0], t->sensor_eui[1],
             t->sensor_eui[2]);
    const char *fw_rev = "1.0.0";

    hap_acc_cfg_t cfg = {
        .name = name,
        .manufacturer = "ICC",
        .model = "Virtual Thermostat",
        .serial_num = serial,
        .fw_rev = (char *)fw_rev,
        .hw_rev = "1.0",
        .pv = "1.1.0",
        .cid = HAP_CID_THERMOSTAT,
        .identify_routine = accessory_identify,
    };

    hap_acc_t *acc = hap_acc_create(&cfg);
    if (!acc) {
        return ESP_ERR_NO_MEM;
    }

    float cur = t->has_current_temp ? t->current_temp_c : 20.0f;
    float tgt = t->target_c > 0 ? t->target_c : 21.0f;
    uint8_t curr_state = t->heating ? 1 : 0;
    uint8_t targ_state = (t->mode == THERMO_MODE_HEAT) ? 1 : 0;

    hap_serv_t *ts = hap_serv_thermostat_create(curr_state, targ_state, cur, tgt, 0);
    if (!ts) {
        hap_acc_delete(acc);
        return ESP_ERR_NO_MEM;
    }
    hap_char_t *targ_mode =
        hap_serv_get_char_by_uuid(ts, HAP_CHAR_UUID_TARGET_HEATING_COOLING_STATE);
    if (targ_mode) {
        hap_char_int_set_constraints(targ_mode, 0, 1, 1);
    }
    hap_serv_set_write_cb(ts, thermo_write);
    hap_serv_set_priv(ts, slot);
    hap_acc_add_serv(acc, ts);

    slot->curr_temp_char = hap_serv_get_char_by_uuid(ts, HAP_CHAR_UUID_CURRENT_TEMPERATURE);
    slot->targ_temp_char = hap_serv_get_char_by_uuid(ts, HAP_CHAR_UUID_TARGET_TEMPERATURE);
    slot->curr_state_char =
        hap_serv_get_char_by_uuid(ts, HAP_CHAR_UUID_CURRENT_HEATING_COOLING_STATE);
    slot->targ_state_char = targ_mode;

    int aid = hap_get_unique_aid(serial);
    hap_add_bridged_accessory(acc, aid);
    inventory_notify_changed();

    slot->used = true;
    slot->group_id = t->id;
    slot->group_type = GROUP_TYPE_THERMOSTAT;
    slot->missing_ticks = 0;
    slot->acc = acc;
    s_st.accessory_count++;
    ESP_LOGI(TAG, "HomeKit exposed thermostat: %s id=%u aid=%d", name, (unsigned)t->id, aid);
    return ESP_OK;
}

static esp_err_t add_bridged_general_group(const group_t *g)
{
    if (!g || !g->used || !g->homekit_expose || g->type != GROUP_TYPE_GENERAL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (find_group(g->id)) {
        return ESP_OK;
    }
    hk_group_bridged_t *slot = alloc_group();
    if (!slot) {
        return ESP_ERR_NO_MEM;
    }

    char name[40];
    snprintf(name, sizeof(name), "%s", g->name[0] ? g->name : "Grouped device");
    char serial[16];
    snprintf(serial, sizeof(serial), "gg%02x%02x%02x%02x", g->id, g->members[0][0], g->members[0][1],
             g->members[0][2]);
    const char *fw_rev = "1.0.0";
    bool as_light = g->is_light_group;

    hap_acc_cfg_t cfg = {
        .name = name,
        .manufacturer = "ICC",
        .model = as_light ? "Grouped Light" : "Grouped Switch",
        .serial_num = serial,
        .fw_rev = (char *)fw_rev,
        .hw_rev = "1.0",
        .pv = "1.1.0",
        .cid = as_light ? HAP_CID_LIGHTING : HAP_CID_SWITCH,
        .identify_routine = accessory_identify,
    };

    hap_acc_t *acc = hap_acc_create(&cfg);
    if (!acc) {
        return ESP_ERR_NO_MEM;
    }

    if (as_light) {
        int bright = g->brightness_pct ? (int)g->brightness_pct : (g->on ? 100 : 0);
        hap_serv_t *lb = hap_serv_lightbulb_create(g->on);
        if (!lb) {
            hap_acc_delete(acc);
            return ESP_ERR_NO_MEM;
        }
        hap_serv_add_char(lb, hap_char_name_create("Light"));
        hap_serv_add_char(lb, hap_char_brightness_create(bright > 0 ? bright : 100));
        hap_serv_set_write_cb(lb, group_general_write);
        hap_serv_set_priv(lb, slot);
        hap_acc_add_serv(acc, lb);
        slot->on_char = hap_serv_get_char_by_uuid(lb, HAP_CHAR_UUID_ON);
        slot->brightness_char = hap_serv_get_char_by_uuid(lb, HAP_CHAR_UUID_BRIGHTNESS);
    } else {
        hap_serv_t *sw = hap_serv_switch_create(g->on);
        if (!sw) {
            hap_acc_delete(acc);
            return ESP_ERR_NO_MEM;
        }
        hap_serv_add_char(sw, hap_char_name_create("Switch"));
        hap_serv_set_write_cb(sw, group_general_write);
        hap_serv_set_priv(sw, slot);
        hap_acc_add_serv(acc, sw);
        slot->on_char = hap_serv_get_char_by_uuid(sw, HAP_CHAR_UUID_ON);
    }

    int aid = hap_get_unique_aid(serial);
    hap_add_bridged_accessory(acc, aid);
    inventory_notify_changed();

    slot->used = true;
    slot->group_id = g->id;
    slot->group_type = GROUP_TYPE_GENERAL;
    slot->missing_ticks = 0;
    slot->acc = acc;
    s_st.accessory_count++;
    ESP_LOGI(TAG, "HomeKit exposed group: %s id=%u type=%s aid=%d", name, (unsigned)g->id,
             as_light ? "light" : "switch", aid);
    return ESP_OK;
}

static esp_err_t add_bridged_group(const group_t *g)
{
    if (!g || !g->used || !g->homekit_expose) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g->type == GROUP_TYPE_THERMOSTAT) {
        return add_bridged_thermostat(g);
    }
    return add_bridged_general_group(g);
}

static void remove_bridged(hk_bridged_t *slot, const char *reason)
{
    if (!slot || !slot->used) {
        return;
    }
    ESP_LOGW(TAG, "HomeKit removing bridged accessory (%s)", reason ? reason : "unknown");
    if (slot->acc) {
        hap_remove_bridged_accessory(slot->acc);
        slot->acc = NULL;
        inventory_notify_changed();
    }
    memset(slot, 0, sizeof(*slot));
    if (s_st.accessory_count > 0) {
        s_st.accessory_count--;
    }
}

static void sync_from_zigbee(void)
{
    /* Update / prune existing bridged accessories. Never drop an accessory because of a
     * transient lookup miss — Home deletes room/name when the list briefly shrinks. */
    for (uint16_t i = 0; i < HK_MAX_BRIDGED; i++) {
        if (!s_bridged[i].used) {
            continue;
        }
        zb_device_t d;
        if (!zigbee_host_get_device(s_bridged[i].eui64, &d)) {
            if (s_bridged[i].missing_ticks < 255) {
                s_bridged[i].missing_ticks++;
            }
            if (s_bridged[i].missing_ticks >= 6) {
                remove_bridged(&s_bridged[i], "device missing from inventory");
            }
            continue;
        }
        s_bridged[i].missing_ticks = 0;
        if (!d.homekit_expose) {
            remove_bridged(&s_bridged[i], "homekit_expose off");
            continue;
        }
        zb_device_kind_t want = effective_kind(&d);
        if (want == ZB_DEVICE_KIND_UNKNOWN) {
            remove_bridged(&s_bridged[i], "unsupported device kind");
            continue;
        }
        if (s_bridged[i].kind != want) {
            remove_bridged(&s_bridged[i], "device kind changed");
            continue;
        }
        if (want == ZB_DEVICE_KIND_REMOTE && d.remote_type == ZB_REMOTE_TYPE_CONTROL) {
            remove_bridged(&s_bridged[i], "remote switched to control mode");
            continue;
        }
        if (s_bridged[i].kind == ZB_DEVICE_KIND_LIGHT) {
            if (d.name[0]) {
                update_accessory_name(&s_bridged[i], d.name);
            }
            if (d.has_onoff) {
                update_char_bool(s_bridged[i].on_char, d.onoff_on);
            }
            if (d.has_level) {
                update_char_int(s_bridged[i].brightness_char,
                                (int)zigbee_host_level_to_brightness(d.level), 0, 100);
            }
        } else if (s_bridged[i].kind == ZB_DEVICE_KIND_SWITCH) {
            if (d.name[0]) {
                update_accessory_name(&s_bridged[i], d.name);
            }
            if (d.has_onoff) {
                update_char_bool(s_bridged[i].on_char, d.onoff_on);
            }
        } else if (s_bridged[i].kind == ZB_DEVICE_KIND_REMOTE) {
            uint8_t nbtn = zigbee_host_remote_button_count(&d);
            if (nbtn == 0 || nbtn > ZB_REMOTE_MAX_BUTTONS) {
                nbtn = 5;
            }
            if (!remote_btn_layout_ok(&d, nbtn)) {
                remove_all_bridged_for_eui(s_bridged[i].eui64, "remote button layout changed");
                continue;
            }
            if (d.name[0]) {
                update_accessory_name(&s_bridged[i], d.name);
            }
            for (uint8_t bi = 0; bi < nbtn && bi < ZB_REMOTE_MAX_BUTTONS; bi++) {
                if (s_bridged[i].btn_mode_snap[bi] == ZB_BTN_MODE_STATEFUL &&
                    s_bridged[i].btn_on_char[bi]) {
                    /* Silent sync — live presses use update_char_bool_notify. */
                    update_char_bool(s_bridged[i].btn_on_char[bi], d.btn_on[bi]);
                }
                char bname[24];
                fill_btn_service_name(&d, nbtn, bi, bname, sizeof(bname));
                if (strncmp(s_bridged[i].btn_name_snap[bi], bname,
                            sizeof(s_bridged[i].btn_name_snap[bi])) != 0) {
                    if (s_bridged[i].btn_name_char[bi]) {
                        update_char_string(s_bridged[i].btn_name_char[bi], bname);
                    }
                    if (s_bridged[i].btn_cfg_name_char[bi]) {
                        update_char_string(s_bridged[i].btn_cfg_name_char[bi], bname);
                    }
                    snprintf(s_bridged[i].btn_name_snap[bi],
                             sizeof(s_bridged[i].btn_name_snap[bi]), "%s", bname);
                    ESP_LOGI(TAG, "HomeKit remote btn %u name -> '%s'", (unsigned)bi, bname);
                }
            }
            if (d.has_battery && s_bridged[i].batt_char) {
                update_char_uint8(s_bridged[i].batt_char, d.battery_pct, 0, 100);
                update_char_uint8(s_bridged[i].low_batt_char, d.battery_pct < 20 ? 1 : 0, 0, 1);
            }
        } else {
            if (d.name[0]) {
                update_accessory_name(&s_bridged[i], d.name);
            }
            if (d.has_temp) {
                update_char_float(s_bridged[i].temp_char, d.temperature_c, 0.0f, 100.0f);
            }
            if (d.has_humidity) {
                update_char_float(s_bridged[i].hum_char, d.humidity_pct, 0.0f, 100.0f);
            }
            if (d.has_battery) {
                update_char_uint8(s_bridged[i].batt_char, d.battery_pct, 0, 100);
                update_char_uint8(s_bridged[i].low_batt_char, d.battery_pct < 20 ? 1 : 0, 0, 1);
            }
        }
    }

    /* Discover newly exposed devices without allocating the full status blob. */
    for (uint16_t j = 0; j < ZB_HOST_MAX_DEVICES; j++) {
        zb_device_t d;
        if (zigbee_host_get_device_at(j, &d) && d.homekit_expose) {
            add_bridged_device(&d);
        }
    }

    /* Virtual groups — independent of member HomeKit exposure. */
    for (uint16_t ti = 0; ti < GROUP_MAX; ti++) {
        if (!s_groups[ti].used) {
            continue;
        }
        group_t t;
        if (!group_get_by_id(s_groups[ti].group_id, &t)) {
            if (s_groups[ti].missing_ticks < 255) {
                s_groups[ti].missing_ticks++;
            }
            if (s_groups[ti].missing_ticks >= 6) {
                remove_group_acc(&s_groups[ti], "group missing");
            }
            continue;
        }
        s_groups[ti].missing_ticks = 0;
        if (!t.homekit_expose) {
            remove_group_acc(&s_groups[ti], "homekit_expose off");
            continue;
        }
        if (t.type != s_groups[ti].group_type) {
            remove_group_acc(&s_groups[ti], "group type changed");
            continue;
        }
        if (t.type == GROUP_TYPE_THERMOSTAT) {
            if (t.has_current_temp) {
                update_char_float(s_groups[ti].curr_temp_char, t.current_temp_c, 0.0f, 100.0f);
            }
            update_char_float(s_groups[ti].targ_temp_char, t.target_c, 10.0f, 38.0f);
            update_char_uint8(s_groups[ti].curr_state_char, t.heating ? 1 : 0, 0, 2);
            update_char_uint8(s_groups[ti].targ_state_char,
                              t.mode == THERMO_MODE_HEAT ? 1 : 0, 0, 1);
        } else {
            update_char_bool(s_groups[ti].on_char, t.on);
            if (s_groups[ti].brightness_char) {
                update_char_int(s_groups[ti].brightness_char, (int)t.brightness_pct, 0, 100);
            }
        }
    }
    for (uint16_t tj = 0; tj < GROUP_MAX; tj++) {
        group_t t;
        if (group_get_at(tj, &t) && t.homekit_expose) {
            add_bridged_group(&t);
        }
    }

    snprintf(s_st.status, sizeof(s_st.status), "%u accessory(ies)", (unsigned)s_st.accessory_count);
}

/** Stable hash of what we intend to expose — used to bump HomeKit c# only when the
 * inventory set actually changes (so Home refreshes without wiping rooms on every boot). */
static uint32_t compute_inventory_sig(void)
{
    uint32_t h = 2166136261u;
    for (uint16_t j = 0; j < ZB_HOST_MAX_DEVICES; j++) {
        zb_device_t d;
        if (!zigbee_host_get_device_at(j, &d) || !d.homekit_expose) {
            continue;
        }
        zb_device_kind_t k = effective_kind(&d);
        if (k == ZB_DEVICE_KIND_UNKNOWN) {
            continue;
        }
        h ^= (uint32_t)k + 0x9e3779b9u;
        h *= 16777619u;
        for (int b = 0; b < 8; b++) {
            h ^= d.eui64[b];
            h *= 16777619u;
        }
        if (k == ZB_DEVICE_KIND_REMOTE) {
            uint8_t nbtn = zigbee_host_remote_button_count(&d);
            h ^= ((uint32_t)REMOTE_LAYOUT_VER << 16) | nbtn;
            h *= 16777619u;
            for (uint8_t bi = 0; bi < nbtn && bi < ZB_REMOTE_MAX_BUTTONS; bi++) {
                h ^= (uint32_t)d.btn_mode[bi] << (bi * 2);
                h *= 16777619u;
            }
        }
    }
    for (uint16_t tj = 0; tj < GROUP_MAX; tj++) {
        group_t t;
        if (!group_get_at(tj, &t) || !t.homekit_expose) {
            continue;
        }
        h ^= 0x67720000u | ((uint32_t)t.type << 8) | t.id;
        h *= 16777619u;
    }
    h ^= (uint32_t)s_st.accessory_count * 0x01000193u;
    return h;
}

static void announce_inventory_if_changed(void)
{
    uint32_t sig = compute_inventory_sig();
    if (s_inv_sig_cached_valid && s_inv_sig_cached == sig) {
        return; /* Fast path — no NVS / no mDNS churn */
    }

    nvs_handle_t nh;
    uint32_t prev = 0;
    bool have_prev = false;
    if (nvs_open("hkbr", NVS_READWRITE, &nh) != ESP_OK) {
        /* Do NOT bump c# on transient NVS failure — that flaps Home accessories. */
        ESP_LOGW(TAG, "NVS open failed — skip config# bump this cycle");
        return;
    }
    if (nvs_get_u32(nh, "inv3", &prev) == ESP_OK) {
        have_prev = true;
    }
    s_inv_sig_cached = sig;
    s_inv_sig_cached_valid = true;
    if (!have_prev || prev != sig) {
        hap_update_config_number();
        nvs_set_u32(nh, "inv3", sig);
        nvs_commit(nh);
        ESP_LOGI(TAG, "HomeKit inventory changed (sig %08lx -> %08lx) — config# bumped",
                 (unsigned long)prev, (unsigned long)sig);
        nvs_close(nh);
        for (uint16_t i = 0; i < HK_MAX_BRIDGED; i++) {
            if (!s_bridged[i].used) {
                continue;
            }
            zb_device_t d;
            if (zigbee_host_get_device(s_bridged[i].eui64, &d)) {
                ESP_LOGI(TAG, "  bridged device: '%s' kind=%s", d.name[0] ? d.name : d.model,
                         s_bridged[i].kind == ZB_DEVICE_KIND_LIGHT
                             ? "light"
                             : (s_bridged[i].kind == ZB_DEVICE_KIND_SWITCH
                                    ? "switch"
                                    : (s_bridged[i].kind == ZB_DEVICE_KIND_REMOTE ? "remote"
                                                                                   : "sensor")));
            }
        }
        for (uint16_t i = 0; i < GROUP_MAX; i++) {
            if (!s_groups[i].used) {
                continue;
            }
            group_t t;
            if (group_get_by_id(s_groups[i].group_id, &t)) {
                ESP_LOGI(TAG, "  bridged group: '%s' id=%u type=%u", t.name, (unsigned)t.id,
                         (unsigned)t.type);
            }
        }
    } else {
        nvs_close(nh);
        ESP_LOGD(TAG, "HomeKit inventory unchanged (sig %08lx, %u acc)", (unsigned long)sig,
                 (unsigned)s_st.accessory_count);
    }
}

static void attach_exposed_before_advertise(void)
{
    /* Wait until Zigbee host has restored NVS devices (or NCP is up). */
    for (int i = 0; i < 50; i++) {
        zigbee_host_status_t *snap = calloc(1, sizeof(*snap));
        if (snap) {
            zigbee_host_get_status(snap);
            bool ready = (snap->icc_status == ICC_STATUS_CONNECTED) || (snap->device_count > 0);
            free(snap);
            if (ready) {
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* Attach every currently exposed device BEFORE hap_start()/mDNS. If Home sees an
     * incomplete accessory list even once, it forgets room and custom names. */
    for (int attempt = 0; attempt < 25; attempt++) {
        sync_from_zigbee();
        uint16_t want =
            zigbee_host_count_homekit_exposed() + group_count_homekit_exposed();
        if (s_st.accessory_count >= want) {
            ESP_LOGI(TAG, "Pre-advertise attach complete: %u / %u exposed",
                     (unsigned)s_st.accessory_count, (unsigned)want);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGW(TAG, "Pre-advertise attach incomplete: %u accessories (wanted %u)",
             (unsigned)s_st.accessory_count,
             (unsigned)(zigbee_host_count_homekit_exposed() + group_count_homekit_exposed()));
}

static void sync_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1500));
    for (;;) {
        if (s_started) {
            if (s_sync_mu) {
                xSemaphoreTake(s_sync_mu, portMAX_DELAY);
            }
            sync_from_zigbee();
            if (s_allow_config_bump) {
                announce_inventory_if_changed();
            }
            if (s_sync_mu) {
                xSemaphoreGive(s_sync_mu);
            }
        }
        /* Wake early on new controller connect (see hk_hap_event). */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(4000));
    }
}

static void hk_hap_event(hap_event_t event, void *data)
{
    (void)data;
    /* Only re-sync when a controller connects. SET_CHAR_COMPLETED fires on every
     * Home write and was starving HAP httpd sockets with sync storms. */
    if (event == HAP_EVENT_CTRL_CONNECTED) {
        if (s_sync_task) {
            xTaskNotifyGive(s_sync_task);
        }
    }
}

esp_err_t homekit_bridge_sync_devices(void)
{
    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Never run a full sync on the httpd stack — it blocks name-save HTTP and
     * races the periodic sync task. Wake the sync task instead. */
    if (s_sync_task) {
        xTaskNotifyGive(s_sync_task);
        return ESP_OK;
    }
    if (s_sync_mu) {
        xSemaphoreTake(s_sync_mu, portMAX_DELAY);
    }
    sync_from_zigbee();
    if (s_allow_config_bump) {
        announce_inventory_if_changed();
    }
    if (s_sync_mu) {
        xSemaphoreGive(s_sync_mu);
    }
    return ESP_OK;
}

static void homekit_start_task(void *arg)
{
    (void)arg;
    /* Wait for STA IP. Defer HAP until STA has settled — starting HAP/mDNS
     * immediately after GOT_IP can wedge C3 TX. */
    for (int i = 0; i < 40; i++) {
        if (wifi_manager_is_connected() && !wifi_manager_is_ap_active()) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    /* Short settle after link-up (app_main already waited ~12s). */
    vTaskDelay(pdMS_TO_TICKS(2000));

    snprintf(s_st.setup_code, sizeof(s_st.setup_code), "%s", CONFIG_HK_SETUP_CODE);
    snprintf(s_st.setup_id, sizeof(s_st.setup_id), "%s", CONFIG_HK_SETUP_ID);
    snprintf(s_st.status, sizeof(s_st.status), "starting");

    hap_cfg_t hap_cfg;
    hap_get_config(&hap_cfg);
    hap_cfg.task_priority = 3; /* Below Wi‑Fi/lwIP; was 5 and starved STA on C3 */
    hap_cfg.task_stack_size = 12288;
    hap_cfg.max_event_notif_chars = 48; /* Sensors + 5 remote buttons must not drop EV */
    /* Re-attaching bridged sensors on boot must not bump c# — that makes Home
     * forget room assignment and user-chosen names. */
    hap_cfg.disable_config_num_update = true;
    hap_set_config(&hap_cfg);

    hap_register_event_handler(hk_hap_event);

    if (hap_init(HAP_TRANSPORT_WIFI) != HAP_SUCCESS) {
        snprintf(s_st.status, sizeof(s_st.status), "hap_init failed");
        vTaskDelete(NULL);
        return;
    }

    hap_acc_cfg_t cfg = {
        .name = "ICC Gateway",
        .manufacturer = "ICC",
        .model = "ICC1-Zigbee-Bridge",
        .serial_num = "ICC1-001",
        .fw_rev = "1.0.2",
        .hw_rev = "1.0",
        .pv = "1.1.0",
        .cid = HAP_CID_BRIDGE,
        .identify_routine = bridge_identify,
    };

    s_bridge_acc = hap_acc_create(&cfg);
    if (!s_bridge_acc) {
        snprintf(s_st.status, sizeof(s_st.status), "acc create failed");
        vTaskDelete(NULL);
        return;
    }

    uint8_t product_data[] = {'I', 'C', 'C', '1', 'Z', 'B', 'G', 'W'};
    hap_acc_add_product_data(s_bridge_acc, product_data, sizeof(product_data));
    hap_add_accessory(s_bridge_acc);

    hap_set_setup_code(CONFIG_HK_SETUP_CODE);
    hap_set_setup_id(CONFIG_HK_SETUP_ID);

    /* Must attach bridged accessories before advertising — incomplete lists wipe Home rooms/names. */
    attach_exposed_before_advertise();

    int hap_rc = HAP_FAIL;
    for (int attempt = 1; attempt <= 5; attempt++) {
        hap_rc = hap_start();
        if (hap_rc == HAP_SUCCESS) {
            break;
        }
        ESP_LOGW(TAG, "hap_start failed (try %d/5) — retrying", attempt);
        snprintf(s_st.status, sizeof(s_st.status), "hap_start retry %d", attempt);
        vTaskDelay(pdMS_TO_TICKS(1500 * attempt));
    }
    if (hap_rc != HAP_SUCCESS) {
        snprintf(s_st.status, sizeof(s_st.status), "hap_start failed");
        ESP_LOGE(TAG, "HomeKit hap_start failed permanently — portal stays up");
        vTaskDelete(NULL);
        return;
    }

    s_started = true;
    s_st.started = true;
    /* Catch anything that appeared during hap_start, still without c# bump. */
    sync_from_zigbee();
    s_allow_config_bump = true;
    /* If the exposed set changed since last announce (e.g. Kitchen 2 added while Home
     * was offline / across reboot), bump c# once so Home re-fetches accessories. */
    announce_inventory_if_changed();
    snprintf(s_st.status, sizeof(s_st.status), "%u accessory(ies)", (unsigned)s_st.accessory_count);
    ESP_LOGI(TAG, "HomeKit bridge started — setup code %s  setup id %s  accessories=%u",
             CONFIG_HK_SETUP_CODE, CONFIG_HK_SETUP_ID, (unsigned)s_st.accessory_count);

    if (!s_sync_task) {
        xTaskCreate(sync_task, "hk_sync", 8192, NULL, 3, &s_sync_task);
    }
    vTaskDelete(NULL);
}

esp_err_t homekit_bridge_start(void)
{
    if (s_started || s_bridge_acc) {
        return ESP_OK;
    }
    if (!s_sync_mu) {
        s_sync_mu = xSemaphoreCreateMutex();
    }
    zigbee_host_set_remote_button_cb(on_remote_button);
    zigbee_host_set_sensor_update_cb(on_sensor_update);
    snprintf(s_st.setup_code, sizeof(s_st.setup_code), "%s", CONFIG_HK_SETUP_CODE);
    snprintf(s_st.setup_id, sizeof(s_st.setup_id), "%s", CONFIG_HK_SETUP_ID);
    snprintf(s_st.status, sizeof(s_st.status), "starting");
    if (xTaskCreate(homekit_start_task, "hk_start", 12288, NULL, 4, NULL) != pdPASS) {
        snprintf(s_st.status, sizeof(s_st.status), "no mem");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void homekit_bridge_get_status(homekit_bridge_status_t *out)
{
    if (!out) {
        return;
    }
    *out = s_st;
    out->paired = hap_get_paired_controller_count() > 0;
}
