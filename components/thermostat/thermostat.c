/**
 * @file thermostat.c
 * @brief Grouped virtual accessories: general multi-device groups + thermostats.
 */

#include "thermostat.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "zigbee_host.h"

static const char *TAG = "group";
#define NVS_NS "thermo"
#define NVS_KEY "grp1"
#define NVS_KEY_LEGACY "list1"

typedef struct {
    bool used;
    uint8_t id;
    char name[32];
    uint8_t type;
    bool homekit_expose;
    uint8_t sensor_eui[8];
    uint8_t switch_eui[8];
    float target_c;
    uint8_t mode;
    bool heating;
    uint8_t member_count;
    uint8_t members[GROUP_MAX_MEMBERS][8];
} group_persist_t;

/* Legacy thermostat-only persist (migration). */
typedef struct {
    bool used;
    uint8_t id;
    char name[32];
    uint8_t sensor_eui[8];
    uint8_t switch_eui[8];
    float target_c;
    uint8_t mode;
    bool heating;
    bool homekit_expose;
} thermo_legacy_t;

static group_t s_list[GROUP_MAX];
static SemaphoreHandle_t s_mutex;
static TaskHandle_t s_task;
static uint8_t s_next_id = 1;

static void lock(void)
{
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
}

static void unlock(void)
{
    if (s_mutex) {
        xSemaphoreGive(s_mutex);
    }
}

static void nvs_save_locked(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    group_persist_t packed[GROUP_MAX];
    memset(packed, 0, sizeof(packed));
    uint16_t n = 0;
    for (uint16_t i = 0; i < GROUP_MAX; i++) {
        if (!s_list[i].used) {
            continue;
        }
        group_persist_t *p = &packed[n++];
        p->used = true;
        p->id = s_list[i].id;
        snprintf(p->name, sizeof(p->name), "%s", s_list[i].name);
        p->type = s_list[i].type;
        p->homekit_expose = s_list[i].homekit_expose;
        memcpy(p->sensor_eui, s_list[i].sensor_eui, 8);
        memcpy(p->switch_eui, s_list[i].switch_eui, 8);
        p->target_c = s_list[i].target_c;
        p->mode = s_list[i].mode;
        p->heating = s_list[i].heating;
        p->member_count = s_list[i].member_count;
        memcpy(p->members, s_list[i].members, sizeof(p->members));
    }
    if (nvs_set_blob(h, NVS_KEY, packed, sizeof(packed)) == ESP_OK) {
        nvs_set_u8(h, "next_id", s_next_id);
        nvs_erase_key(h, NVS_KEY_LEGACY);
        nvs_commit(h);
        ESP_LOGI(TAG, "Saved %u group(s)", (unsigned)n);
    }
    nvs_close(h);
}

static void nvs_load_locked(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    memset(s_list, 0, sizeof(s_list));
    group_persist_t packed[GROUP_MAX];
    size_t sz = sizeof(packed);
    bool loaded = false;
    if (nvs_get_blob(h, NVS_KEY, packed, &sz) == ESP_OK && sz == sizeof(packed)) {
        for (uint16_t i = 0; i < GROUP_MAX; i++) {
            if (!packed[i].used) {
                continue;
            }
            group_t *g = &s_list[i];
            g->used = true;
            g->id = packed[i].id;
            snprintf(g->name, sizeof(g->name), "%s", packed[i].name);
            g->type = packed[i].type;
            g->homekit_expose = packed[i].homekit_expose;
            memcpy(g->sensor_eui, packed[i].sensor_eui, 8);
            memcpy(g->switch_eui, packed[i].switch_eui, 8);
            g->target_c = packed[i].target_c;
            g->mode = packed[i].mode;
            g->heating = packed[i].heating;
            g->member_count = packed[i].member_count;
            if (g->member_count > GROUP_MAX_MEMBERS) {
                g->member_count = GROUP_MAX_MEMBERS;
            }
            memcpy(g->members, packed[i].members, sizeof(g->members));
        }
        loaded = true;
        ESP_LOGI(TAG, "Loaded groups from NVS");
    }
    if (!loaded) {
        thermo_legacy_t legacy[GROUP_MAX];
        sz = sizeof(legacy);
        if (nvs_get_blob(h, NVS_KEY_LEGACY, legacy, &sz) == ESP_OK && sz == sizeof(legacy)) {
            for (uint16_t i = 0; i < GROUP_MAX; i++) {
                if (!legacy[i].used) {
                    continue;
                }
                group_t *g = &s_list[i];
                memset(g, 0, sizeof(*g));
                g->used = true;
                g->id = legacy[i].id;
                snprintf(g->name, sizeof(g->name), "%s", legacy[i].name);
                g->type = GROUP_TYPE_THERMOSTAT;
                g->homekit_expose = legacy[i].homekit_expose;
                memcpy(g->sensor_eui, legacy[i].sensor_eui, 8);
                memcpy(g->switch_eui, legacy[i].switch_eui, 8);
                g->target_c = legacy[i].target_c;
                g->mode = legacy[i].mode;
                g->heating = legacy[i].heating;
            }
            ESP_LOGI(TAG, "Migrated legacy thermostats to groups");
        }
    }
    uint8_t next = 1;
    if (nvs_get_u8(h, "next_id", &next) == ESP_OK && next >= 1) {
        s_next_id = next;
    }
    nvs_close(h);
}

static group_t *find_by_id_locked(uint8_t id)
{
    for (uint16_t i = 0; i < GROUP_MAX; i++) {
        if (s_list[i].used && s_list[i].id == id) {
            return &s_list[i];
        }
    }
    return NULL;
}

static float clamp_target(float c)
{
    if (c < 10.0f) {
        return 10.0f;
    }
    if (c > 38.0f) {
        return 38.0f;
    }
    return c;
}

static uint8_t alloc_id_locked(void)
{
    uint8_t id = s_next_id;
    for (int tries = 0; tries < 250; tries++) {
        if (id == 0) {
            id = 1;
        }
        if (!find_by_id_locked(id)) {
            s_next_id = (uint8_t)(id + 1);
            if (s_next_id == 0) {
                s_next_id = 1;
            }
            return id;
        }
        id++;
    }
    return 0;
}

static int find_free_slot_locked(void)
{
    for (uint16_t i = 0; i < GROUP_MAX; i++) {
        if (!s_list[i].used) {
            return (int)i;
        }
    }
    return -1;
}

static void refresh_runtime_locked(group_t *g)
{
    if (!g) {
        return;
    }
    if (g->type == GROUP_TYPE_THERMOSTAT) {
        zb_device_t sensor;
        if (zigbee_host_get_device(g->sensor_eui, &sensor) && sensor.has_temp) {
            g->has_current_temp = true;
            g->current_temp_c = sensor.temperature_c;
        } else {
            g->has_current_temp = false;
        }
        return;
    }
    /* General group: aggregate on/brightness from members, but never invent 100%
     * when members have no Level report — that made HomeKit snap brightness back. */
    bool saw_member = false;
    bool any_on = false;
    bool any_level = false;
    bool lightish = false;
    uint8_t max_bright = 0;
    for (uint8_t i = 0; i < g->member_count; i++) {
        zb_device_t d;
        if (!zigbee_host_get_device(g->members[i], &d)) {
            continue;
        }
        saw_member = true;
        if (zigbee_host_device_kind(&d) == ZB_DEVICE_KIND_REMOTE) {
            continue;
        }
        if (zigbee_host_device_kind(&d) == ZB_DEVICE_KIND_LIGHT || d.has_level) {
            lightish = true;
        }
        if (d.has_onoff && d.onoff_on) {
            any_on = true;
        }
        if (d.has_level) {
            any_level = true;
            uint8_t b = zigbee_host_level_to_brightness(d.level);
            if (b > max_bright) {
                max_bright = b;
            }
            if (d.level > 0) {
                any_on = true;
            }
        }
    }
    g->is_light_group = lightish || g->is_light_group;
    if (saw_member) {
        g->on = any_on;
        if (any_level) {
            g->brightness_pct = max_bright;
        } else if (!any_on) {
            /* Keep last commanded brightness while off (HomeKit UX). */
            if (g->brightness_pct == 0) {
                g->brightness_pct = 100;
            }
        }
        /* If on but no level attr yet, keep commanded brightness_pct as-is. */
    }
}

static void apply_thermo_control(group_t *g)
{
    if (!g || g->type != GROUP_TYPE_THERMOSTAT) {
        return;
    }
    refresh_runtime_locked(g);
    bool want_heat = g->heating;
    if (g->mode == THERMO_MODE_OFF) {
        want_heat = false;
    } else if (g->has_current_temp) {
        float cur = g->current_temp_c;
        float tgt = g->target_c;
        if (cur <= tgt - THERMOSTAT_HYSTERESIS_C) {
            want_heat = true;
        } else if (cur >= tgt + THERMOSTAT_HYSTERESIS_C) {
            want_heat = false;
        }
    } else {
        return;
    }
    if (want_heat == g->heating) {
        return;
    }
    if (zigbee_host_set_onoff(g->switch_eui, want_heat) != ESP_OK) {
        ESP_LOGW(TAG, "Group %u heater cmd failed", (unsigned)g->id);
        return;
    }
    g->heating = want_heat;
    ESP_LOGI(TAG, "Thermostat %u '%s': heat %s (cur=%.1f tgt=%.1f)", (unsigned)g->id, g->name,
             want_heat ? "ON" : "OFF", g->has_current_temp ? (double)g->current_temp_c : 0.0,
             (double)g->target_c);
    nvs_save_locked();
}

void group_on_sensor_updated(const uint8_t sensor_eui[8])
{
    if (!sensor_eui || !s_mutex) {
        return;
    }
    lock();
    for (uint16_t i = 0; i < GROUP_MAX; i++) {
        group_t *g = &s_list[i];
        if (!g->used || g->type != GROUP_TYPE_THERMOSTAT) {
            continue;
        }
        if (memcmp(g->sensor_eui, sensor_eui, 8) != 0) {
            continue;
        }
        apply_thermo_control(g);
    }
    unlock();
}

static void control_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(4000));
    for (;;) {
        lock();
        for (uint16_t i = 0; i < GROUP_MAX; i++) {
            if (s_list[i].used && s_list[i].type == GROUP_TYPE_THERMOSTAT) {
                apply_thermo_control(&s_list[i]);
            }
        }
        unlock();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

esp_err_t group_start(void)
{
    if (!s_mutex) {
        s_mutex = xSemaphoreCreateMutex();
    }
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }
    lock();
    nvs_load_locked();
    unlock();
    if (!s_task) {
        if (xTaskCreate(control_task, "groups", 4096, NULL, 4, &s_task) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }
    ESP_LOGI(TAG, "Grouped devices started (%u configured)", (unsigned)group_count());
    return ESP_OK;
}

esp_err_t thermostat_start(void)
{
    return group_start();
}

uint16_t group_count(void)
{
    uint16_t n = 0;
    lock();
    for (uint16_t i = 0; i < GROUP_MAX; i++) {
        if (s_list[i].used) {
            n++;
        }
    }
    unlock();
    return n;
}

uint16_t thermostat_count(void)
{
    return group_count();
}

uint16_t group_count_homekit_exposed(void)
{
    uint16_t n = 0;
    lock();
    for (uint16_t i = 0; i < GROUP_MAX; i++) {
        if (s_list[i].used && s_list[i].homekit_expose) {
            n++;
        }
    }
    unlock();
    return n;
}

uint16_t thermostat_count_homekit_exposed(void)
{
    return group_count_homekit_exposed();
}

bool group_get_at(uint16_t index, group_t *out)
{
    if (!out || index >= GROUP_MAX) {
        return false;
    }
    lock();
    if (!s_list[index].used) {
        unlock();
        return false;
    }
    *out = s_list[index];
    refresh_runtime_locked(out);
    unlock();
    return true;
}

bool thermostat_get_at(uint16_t index, thermostat_t *out)
{
    return group_get_at(index, out);
}

bool group_get_by_id(uint8_t id, group_t *out)
{
    if (!out) {
        return false;
    }
    lock();
    group_t *g = find_by_id_locked(id);
    if (!g) {
        unlock();
        return false;
    }
    *out = *g;
    refresh_runtime_locked(out);
    unlock();
    return true;
}

bool thermostat_get_by_id(uint8_t id, thermostat_t *out)
{
    return group_get_by_id(id, out);
}

esp_err_t group_create_thermostat(const char *name, const uint8_t sensor_eui[8],
                                  const uint8_t switch_eui[8], float target_c, uint8_t mode,
                                  bool homekit_expose, uint8_t *id_out)
{
    if (!name || !name[0] || !sensor_eui || !switch_eui) {
        return ESP_ERR_INVALID_ARG;
    }
    zb_device_t sens, sw;
    if (!zigbee_host_get_device(sensor_eui, &sens) || !zigbee_host_get_device(switch_eui, &sw)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (mode != THERMO_MODE_OFF && mode != THERMO_MODE_HEAT) {
        mode = THERMO_MODE_HEAT;
    }
    lock();
    int slot = find_free_slot_locked();
    uint8_t id = alloc_id_locked();
    if (slot < 0 || id == 0) {
        unlock();
        return ESP_ERR_NO_MEM;
    }
    group_t *g = &s_list[slot];
    memset(g, 0, sizeof(*g));
    g->used = true;
    g->id = id;
    g->type = GROUP_TYPE_THERMOSTAT;
    snprintf(g->name, sizeof(g->name), "%s", name);
    memcpy(g->sensor_eui, sensor_eui, 8);
    memcpy(g->switch_eui, switch_eui, 8);
    g->target_c = clamp_target(target_c > 0 ? target_c : 21.0f);
    g->mode = mode;
    g->homekit_expose = homekit_expose;
    nvs_save_locked();
    if (id_out) {
        *id_out = id;
    }
    unlock();
    return ESP_OK;
}

esp_err_t group_create_general(const char *name, const uint8_t members[][8], uint8_t member_count,
                               bool homekit_expose, uint8_t *id_out)
{
    if (!name || !name[0] || !members || member_count == 0 || member_count > GROUP_MAX_MEMBERS) {
        return ESP_ERR_INVALID_ARG;
    }
    for (uint8_t i = 0; i < member_count; i++) {
        zb_device_t d;
        if (!zigbee_host_get_device(members[i], &d)) {
            return ESP_ERR_NOT_FOUND;
        }
    }
    lock();
    int slot = find_free_slot_locked();
    uint8_t id = alloc_id_locked();
    if (slot < 0 || id == 0) {
        unlock();
        return ESP_ERR_NO_MEM;
    }
    group_t *g = &s_list[slot];
    memset(g, 0, sizeof(*g));
    g->used = true;
    g->id = id;
    g->type = GROUP_TYPE_GENERAL;
    snprintf(g->name, sizeof(g->name), "%s", name);
    g->member_count = member_count;
    memcpy(g->members, members, (size_t)member_count * 8);
    g->homekit_expose = homekit_expose;
    refresh_runtime_locked(g);
    nvs_save_locked();
    if (id_out) {
        *id_out = id;
    }
    ESP_LOGI(TAG, "Created general group %u '%s' members=%u light=%d", (unsigned)id, g->name,
             (unsigned)g->member_count, (int)g->is_light_group);
    for (uint8_t i = 0; i < g->member_count; i++) {
        const uint8_t *e = g->members[i];
        ESP_LOGI(TAG, "  member[%u]=%02X%02X%02X%02X%02X%02X%02X%02X", (unsigned)i, e[7], e[6],
                 e[5], e[4], e[3], e[2], e[1], e[0]);
    }
    unlock();
    return ESP_OK;
}

esp_err_t thermostat_create(const char *name, const uint8_t sensor_eui[8],
                            const uint8_t switch_eui[8], float target_c, uint8_t mode,
                            bool homekit_expose, uint8_t *id_out)
{
    return group_create_thermostat(name, sensor_eui, switch_eui, target_c, mode, homekit_expose,
                                   id_out);
}

esp_err_t group_update(uint8_t id, const group_update_t *upd)
{
    if (!upd) {
        return ESP_ERR_INVALID_ARG;
    }
    lock();
    group_t *g = find_by_id_locked(id);
    if (!g) {
        unlock();
        return ESP_ERR_NOT_FOUND;
    }
    if (upd->set_name) {
        snprintf(g->name, sizeof(g->name), "%s", upd->name);
    }
    if (upd->set_homekit) {
        g->homekit_expose = upd->homekit_expose;
    }
    if (g->type == GROUP_TYPE_THERMOSTAT) {
        if (upd->set_sensor) {
            memcpy(g->sensor_eui, upd->sensor_eui, 8);
        }
        if (upd->set_switch) {
            memcpy(g->switch_eui, upd->switch_eui, 8);
        }
        if (upd->set_target) {
            g->target_c = clamp_target(upd->target_c);
        }
        if (upd->set_mode) {
            g->mode = (upd->mode == THERMO_MODE_HEAT) ? THERMO_MODE_HEAT : THERMO_MODE_OFF;
        }
    } else if (upd->set_members && upd->member_count > 0 &&
               upd->member_count <= GROUP_MAX_MEMBERS) {
        g->member_count = upd->member_count;
        memcpy(g->members, upd->members, (size_t)upd->member_count * 8);
        g->is_light_group = false;
        refresh_runtime_locked(g);
    }
    nvs_save_locked();
    if (g->type == GROUP_TYPE_THERMOSTAT) {
        apply_thermo_control(g);
    }
    unlock();
    return ESP_OK;
}

esp_err_t group_remove(uint8_t id)
{
    lock();
    group_t *g = find_by_id_locked(id);
    if (!g) {
        unlock();
        return ESP_ERR_NOT_FOUND;
    }
    if (g->type == GROUP_TYPE_THERMOSTAT && g->heating) {
        zigbee_host_set_onoff(g->switch_eui, false);
    }
    memset(g, 0, sizeof(*g));
    nvs_save_locked();
    unlock();
    return ESP_OK;
}

esp_err_t thermostat_remove(uint8_t id)
{
    return group_remove(id);
}

esp_err_t thermostat_update(uint8_t id, const thermostat_update_t *upd)
{
    return group_update(id, upd);
}

esp_err_t thermostat_set_target(uint8_t id, float target_c)
{
    group_update_t upd = {0};
    upd.set_target = true;
    upd.target_c = target_c;
    return group_update(id, &upd);
}

esp_err_t thermostat_set_mode(uint8_t id, uint8_t mode)
{
    group_update_t upd = {0};
    upd.set_mode = true;
    upd.mode = mode;
    return group_update(id, &upd);
}

esp_err_t group_set_onoff(uint8_t id, bool on)
{
    lock();
    group_t *g = find_by_id_locked(id);
    if (!g || g->type != GROUP_TYPE_GENERAL) {
        unlock();
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t count = g->member_count;
    uint8_t members[GROUP_MAX_MEMBERS][8];
    memcpy(members, g->members, sizeof(members));
    g->on = on;
    uint8_t bright = g->brightness_pct ? g->brightness_pct : 100;
    if (!on) {
        /* leave brightness_pct as last value for HomeKit */
    } else if (g->brightness_pct == 0) {
        g->brightness_pct = 100;
        bright = 100;
    }
    bool prefer_level = g->is_light_group;
    unlock();

    ESP_LOGI(TAG, "Group %u On/Off -> %s (%u members)", (unsigned)id, on ? "ON" : "OFF",
             (unsigned)count);
    for (uint8_t i = 0; i < count; i++) {
        zb_device_t d;
        bool is_light = prefer_level;
        if (zigbee_host_get_device(members[i], &d)) {
            is_light = (zigbee_host_device_kind(&d) == ZB_DEVICE_KIND_LIGHT || d.has_level);
        }
        if (is_light) {
            /* MoveToLevel-with-OnOff drives both power and dim on TRADFRI drivers. */
            zigbee_host_set_brightness(members[i], on ? bright : 0);
        } else {
            zigbee_host_set_onoff(members[i], on);
        }
    }
    return ESP_OK;
}

esp_err_t group_set_brightness(uint8_t id, uint8_t brightness_pct)
{
    lock();
    group_t *g = find_by_id_locked(id);
    if (!g || g->type != GROUP_TYPE_GENERAL) {
        unlock();
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t count = g->member_count;
    uint8_t members[GROUP_MAX_MEMBERS][8];
    memcpy(members, g->members, sizeof(members));
    g->brightness_pct = brightness_pct > 100 ? 100 : brightness_pct;
    g->on = (g->brightness_pct > 0);
    g->is_light_group = true;
    unlock();

    ESP_LOGI(TAG, "Group %u brightness -> %u%% (%u members)", (unsigned)id,
             (unsigned)brightness_pct, (unsigned)count);
    for (uint8_t i = 0; i < count; i++) {
        zb_device_t d;
        if (!zigbee_host_get_device(members[i], &d)) {
            ESP_LOGW(TAG, "Group %u member %u not in inventory", (unsigned)id, (unsigned)i);
            continue;
        }
        /* Always try Level Control first — TRADFRI drivers need it for dimming.
         * Fall back to On/Off if the device is unknown to inventory as a light. */
        esp_err_t err = zigbee_host_set_brightness(members[i], brightness_pct);
        if (err != ESP_OK) {
            zigbee_host_set_onoff(members[i], brightness_pct > 0);
        }
    }
    return ESP_OK;
}
