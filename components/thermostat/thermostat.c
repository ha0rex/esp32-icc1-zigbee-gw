/**
 * @file thermostat.c
 * @brief Grouped virtual accessories: general multi-device groups + thermostats.
 */

#include "thermostat.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "zigbee_host.h"

static const char *TAG = "group";
#define NVS_NS "thermo"
#define NVS_KEY "grp4"       /* cooler + gap/hysteresis + humidity force */
#define NVS_KEY_V3 "grp3"    /* humidity thermostat fields */
#define NVS_KEY_V2 "grp2"    /* power-fail + last on/brightness */
#define NVS_KEY_V1 "grp1"    /* members only — migrate */
#define NVS_KEY_LEGACY "list1"

typedef struct {
    bool used;
    uint8_t id;
    char name[32];
    uint8_t type;
    bool homekit_expose;
    uint8_t sensor_eui[8];
    uint8_t humidity_sensor_eui[8];
    uint8_t switch_eui[8];
    uint8_t cooler_eui[8];
    float target_c;
    uint8_t mode;
    bool heating;
    bool cooling;
    uint8_t member_count;
    uint8_t members[GROUP_MAX_MEMBERS][8];
    uint8_t power_fail_mode;
    bool last_on;
    uint8_t last_brightness_pct;
    uint8_t thermo_kind;
    float target_humidity_pct;
    float gap_c;
    float hysteresis_c;
    bool humidity_force_heat;
} group_persist_t;

/* grp3 blob (pre–cooler / gap). */
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
    uint8_t power_fail_mode;
    bool last_on;
    uint8_t last_brightness_pct;
    uint8_t thermo_kind;
    float target_humidity_pct;
} group_persist_v3_t;

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
    uint8_t power_fail_mode;
    bool last_on;
    uint8_t last_brightness_pct;
} group_persist_v2_t;

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
} group_persist_v1_t;

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
static TaskHandle_t s_power_task;
static uint8_t s_next_id = 1;
static bool s_power_applied;

static bool eui_is_zero(const uint8_t eui[8])
{
    if (!eui) {
        return true;
    }
    for (int i = 0; i < 8; i++) {
        if (eui[i] != 0) {
            return false;
        }
    }
    return true;
}

bool group_thermo_has_heater(const group_t *g)
{
    return g && !eui_is_zero(g->switch_eui);
}

bool group_thermo_has_cooler(const group_t *g)
{
    return g && !eui_is_zero(g->cooler_eui);
}

static uint8_t clamp_power_fail(uint8_t mode)
{
    if (mode == GROUP_POWER_FAIL_ON || mode == GROUP_POWER_FAIL_OFF) {
        return mode;
    }
    return GROUP_POWER_FAIL_PREVIOUS;
}

static uint8_t clamp_brightness(uint8_t pct)
{
    if (pct == 0) {
        return 100;
    }
    return pct > 100 ? 100 : pct;
}

static uint8_t clamp_thermo_kind(uint8_t kind)
{
    return (kind == GROUP_THERMO_KIND_HUMIDITY) ? GROUP_THERMO_KIND_HUMIDITY : GROUP_THERMO_KIND_TEMP;
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

static float clamp_target_humidity(float pct)
{
    if (pct < 20.0f) {
        return 20.0f;
    }
    if (pct > 80.0f) {
        return 80.0f;
    }
    return pct;
}

static float clamp_gap(float c)
{
    if (c < 0.1f) {
        return 0.1f;
    }
    if (c > 5.0f) {
        return 5.0f;
    }
    return c;
}

static float clamp_hysteresis(float c)
{
    if (c < 0.1f) {
        return 0.1f;
    }
    if (c > 3.0f) {
        return 3.0f;
    }
    return c;
}

static uint8_t normalize_thermo_mode(uint8_t mode, uint8_t thermo_kind, bool has_heater,
                                     bool has_cooler)
{
    if (thermo_kind == GROUP_THERMO_KIND_HUMIDITY) {
        if (mode == THERMO_MODE_HUMIDIFY || mode == THERMO_MODE_DEHUMIDIFY) {
            return mode;
        }
        return THERMO_MODE_OFF;
    }
    if (mode == THERMO_MODE_OFF) {
        return THERMO_MODE_OFF;
    }
    if (mode == THERMO_MODE_HEAT && has_heater) {
        return THERMO_MODE_HEAT;
    }
    if (mode == THERMO_MODE_COOL && has_cooler) {
        return THERMO_MODE_COOL;
    }
    if (mode == THERMO_MODE_AUTO) {
        if (has_heater && has_cooler) {
            return THERMO_MODE_AUTO;
        }
        if (has_heater) {
            return THERMO_MODE_HEAT;
        }
        if (has_cooler) {
            return THERMO_MODE_COOL;
        }
    }
    if (has_heater) {
        return THERMO_MODE_HEAT;
    }
    if (has_cooler) {
        return THERMO_MODE_COOL;
    }
    return THERMO_MODE_OFF;
}

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
        memcpy(p->humidity_sensor_eui, s_list[i].humidity_sensor_eui, 8);
        memcpy(p->switch_eui, s_list[i].switch_eui, 8);
        memcpy(p->cooler_eui, s_list[i].cooler_eui, 8);
        p->target_c = s_list[i].target_c;
        p->mode = s_list[i].mode;
        p->heating = s_list[i].heating;
        p->cooling = s_list[i].cooling;
        p->member_count = s_list[i].member_count;
        memcpy(p->members, s_list[i].members, sizeof(p->members));
        p->power_fail_mode = clamp_power_fail(s_list[i].power_fail_mode);
        p->last_on = s_list[i].last_on;
        p->last_brightness_pct = s_list[i].last_brightness_pct;
        p->thermo_kind = clamp_thermo_kind(s_list[i].thermo_kind);
        p->target_humidity_pct = s_list[i].target_humidity_pct;
        p->gap_c = s_list[i].gap_c;
        p->hysteresis_c = s_list[i].hysteresis_c;
        p->humidity_force_heat = s_list[i].humidity_force_heat;
    }
    if (nvs_set_blob(h, NVS_KEY, packed, sizeof(packed)) == ESP_OK) {
        nvs_set_u8(h, "next_id", s_next_id);
        nvs_erase_key(h, NVS_KEY_V3);
        nvs_erase_key(h, NVS_KEY_V2);
        nvs_erase_key(h, NVS_KEY_V1);
        nvs_erase_key(h, NVS_KEY_LEGACY);
        nvs_commit(h);
        ESP_LOGI(TAG, "Saved %u group(s)", (unsigned)n);
    }
    nvs_close(h);
}

static void apply_persist_row(group_t *g, const group_persist_t *p)
{
    memset(g, 0, sizeof(*g));
    g->used = true;
    g->id = p->id;
    snprintf(g->name, sizeof(g->name), "%s", p->name);
    g->type = p->type;
    g->homekit_expose = p->homekit_expose;
    memcpy(g->sensor_eui, p->sensor_eui, 8);
    memcpy(g->humidity_sensor_eui, p->humidity_sensor_eui, 8);
    memcpy(g->switch_eui, p->switch_eui, 8);
    memcpy(g->cooler_eui, p->cooler_eui, 8);
    g->target_c = p->target_c;
    g->mode = p->mode;
    g->heating = p->heating;
    g->cooling = p->cooling;
    g->member_count = p->member_count;
    if (g->member_count > GROUP_MAX_MEMBERS) {
        g->member_count = GROUP_MAX_MEMBERS;
    }
    memcpy(g->members, p->members, sizeof(g->members));
    g->power_fail_mode = clamp_power_fail(p->power_fail_mode);
    g->last_on = p->last_on;
    g->last_brightness_pct = p->last_brightness_pct;
    g->thermo_kind = clamp_thermo_kind(p->thermo_kind);
    g->target_humidity_pct = clamp_target_humidity(p->target_humidity_pct > 0 ? p->target_humidity_pct
                                                                               : 45.0f);
    g->gap_c = clamp_gap(p->gap_c > 0 ? p->gap_c : THERMOSTAT_DEFAULT_GAP_C);
    g->hysteresis_c =
        clamp_hysteresis(p->hysteresis_c > 0 ? p->hysteresis_c : THERMOSTAT_DEFAULT_HYSTERESIS_C);
    g->humidity_force_heat = p->humidity_force_heat;
    if (g->type == GROUP_TYPE_GENERAL) {
        g->on = g->last_on;
        g->brightness_pct = clamp_brightness(g->last_brightness_pct);
    }
}

static void fill_defaults_from_v3(group_persist_t *p, const group_persist_v3_t *v)
{
    memset(p, 0, sizeof(*p));
    p->used = true;
    p->id = v->id;
    snprintf(p->name, sizeof(p->name), "%s", v->name);
    p->type = v->type;
    p->homekit_expose = v->homekit_expose;
    memcpy(p->sensor_eui, v->sensor_eui, 8);
    memcpy(p->switch_eui, v->switch_eui, 8);
    p->target_c = v->target_c;
    p->mode = v->mode;
    p->heating = v->heating;
    p->member_count = v->member_count;
    memcpy(p->members, v->members, sizeof(p->members));
    p->power_fail_mode = v->power_fail_mode;
    p->last_on = v->last_on;
    p->last_brightness_pct = v->last_brightness_pct;
    p->thermo_kind = v->thermo_kind;
    p->target_humidity_pct = v->target_humidity_pct;
    p->gap_c = THERMOSTAT_DEFAULT_GAP_C;
    p->hysteresis_c = THERMOSTAT_DEFAULT_HYSTERESIS_C;
    p->humidity_force_heat = false;
    p->cooling = false;
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
            apply_persist_row(&s_list[i], &packed[i]);
        }
        loaded = true;
        ESP_LOGI(TAG, "Loaded groups from NVS (grp4)");
    }
    if (!loaded) {
        group_persist_v3_t v3[GROUP_MAX];
        sz = sizeof(v3);
        if (nvs_get_blob(h, NVS_KEY_V3, v3, &sz) == ESP_OK && sz == sizeof(v3)) {
            for (uint16_t i = 0; i < GROUP_MAX; i++) {
                if (!v3[i].used) {
                    continue;
                }
                group_persist_t p;
                fill_defaults_from_v3(&p, &v3[i]);
                apply_persist_row(&s_list[i], &p);
            }
            loaded = true;
            ESP_LOGI(TAG, "Migrated groups from grp3");
        }
    }
    if (!loaded) {
        group_persist_v2_t v2[GROUP_MAX];
        sz = sizeof(v2);
        if (nvs_get_blob(h, NVS_KEY_V2, v2, &sz) == ESP_OK && sz == sizeof(v2)) {
            for (uint16_t i = 0; i < GROUP_MAX; i++) {
                if (!v2[i].used) {
                    continue;
                }
                group_persist_v3_t v3row;
                memset(&v3row, 0, sizeof(v3row));
                v3row.used = true;
                v3row.id = v2[i].id;
                snprintf(v3row.name, sizeof(v3row.name), "%s", v2[i].name);
                v3row.type = v2[i].type;
                v3row.homekit_expose = v2[i].homekit_expose;
                memcpy(v3row.sensor_eui, v2[i].sensor_eui, 8);
                memcpy(v3row.switch_eui, v2[i].switch_eui, 8);
                v3row.target_c = v2[i].target_c;
                v3row.mode = v2[i].mode;
                v3row.heating = v2[i].heating;
                v3row.member_count = v2[i].member_count;
                memcpy(v3row.members, v2[i].members, sizeof(v3row.members));
                v3row.power_fail_mode = v2[i].power_fail_mode;
                v3row.last_on = v2[i].last_on;
                v3row.last_brightness_pct = v2[i].last_brightness_pct;
                v3row.thermo_kind = GROUP_THERMO_KIND_TEMP;
                v3row.target_humidity_pct = 45.0f;
                group_persist_t p;
                fill_defaults_from_v3(&p, &v3row);
                apply_persist_row(&s_list[i], &p);
            }
            loaded = true;
            ESP_LOGI(TAG, "Migrated groups from grp2");
        }
    }
    if (!loaded) {
        group_persist_v1_t v1[GROUP_MAX];
        sz = sizeof(v1);
        if (nvs_get_blob(h, NVS_KEY_V1, v1, &sz) == ESP_OK && sz == sizeof(v1)) {
            for (uint16_t i = 0; i < GROUP_MAX; i++) {
                if (!v1[i].used) {
                    continue;
                }
                group_persist_v3_t v3row;
                memset(&v3row, 0, sizeof(v3row));
                v3row.used = true;
                v3row.id = v1[i].id;
                snprintf(v3row.name, sizeof(v3row.name), "%s", v1[i].name);
                v3row.type = v1[i].type;
                v3row.homekit_expose = v1[i].homekit_expose;
                memcpy(v3row.sensor_eui, v1[i].sensor_eui, 8);
                memcpy(v3row.switch_eui, v1[i].switch_eui, 8);
                v3row.target_c = v1[i].target_c;
                v3row.mode = v1[i].mode;
                v3row.heating = v1[i].heating;
                v3row.member_count = v1[i].member_count;
                memcpy(v3row.members, v1[i].members, sizeof(v3row.members));
                v3row.power_fail_mode = GROUP_POWER_FAIL_PREVIOUS;
                v3row.last_on = false;
                v3row.last_brightness_pct = 100;
                v3row.thermo_kind = GROUP_THERMO_KIND_TEMP;
                v3row.target_humidity_pct = 45.0f;
                group_persist_t p;
                fill_defaults_from_v3(&p, &v3row);
                apply_persist_row(&s_list[i], &p);
            }
            loaded = true;
            ESP_LOGI(TAG, "Migrated groups from grp1");
        }
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
                g->gap_c = THERMOSTAT_DEFAULT_GAP_C;
                g->hysteresis_c = THERMOSTAT_DEFAULT_HYSTERESIS_C;
                g->target_humidity_pct = 45.0f;
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

static void refresh_humidity_locked(group_t *g)
{
    const uint8_t *hum_eui =
        eui_is_zero(g->humidity_sensor_eui) ? g->sensor_eui : g->humidity_sensor_eui;
    zb_device_t hum;
    if (zigbee_host_get_device(hum_eui, &hum) && hum.has_humidity) {
        g->has_current_humidity = true;
        g->current_humidity_pct = hum.humidity_pct;
    } else {
        g->has_current_humidity = false;
    }
}

static void refresh_runtime_locked(group_t *g)
{
    if (!g) {
        return;
    }
    if (g->type == GROUP_TYPE_THERMOSTAT) {
        if (g->thermo_kind == GROUP_THERMO_KIND_HUMIDITY) {
            zb_device_t sensor;
            if (!zigbee_host_get_device(g->sensor_eui, &sensor)) {
                g->has_current_temp = false;
                g->has_current_humidity = false;
                return;
            }
            g->has_current_humidity = sensor.has_humidity;
            if (sensor.has_humidity) {
                g->current_humidity_pct = sensor.humidity_pct;
            }
            g->has_current_temp = false;
            return;
        }
        zb_device_t sensor;
        if (!zigbee_host_get_device(g->sensor_eui, &sensor)) {
            g->has_current_temp = false;
        } else {
            g->has_current_temp = sensor.has_temp;
            if (sensor.has_temp) {
                g->current_temp_c = sensor.temperature_c;
            }
        }
        refresh_humidity_locked(g);
        return;
    }
    bool lightish = false;
    for (uint8_t i = 0; i < g->member_count; i++) {
        zb_device_t d;
        if (!zigbee_host_get_device(g->members[i], &d)) {
            continue;
        }
        if (zigbee_host_device_kind(&d) == ZB_DEVICE_KIND_REMOTE) {
            continue;
        }
        if (zigbee_host_device_kind(&d) == ZB_DEVICE_KIND_LIGHT || d.has_level) {
            lightish = true;
        }
    }
    g->is_light_group = lightish || g->is_light_group;
    g->on = g->last_on;
    g->brightness_pct = clamp_brightness(g->last_brightness_pct);
}

static void remember_state_locked(group_t *g, bool on, uint8_t brightness_pct)
{
    g->on = on;
    if (on) {
        g->brightness_pct = clamp_brightness(brightness_pct);
        g->last_brightness_pct = g->brightness_pct;
    } else if (brightness_pct > 0) {
        g->brightness_pct = brightness_pct > 100 ? 100 : brightness_pct;
        g->last_brightness_pct = g->brightness_pct;
    } else if (g->brightness_pct == 0) {
        g->brightness_pct = clamp_brightness(g->last_brightness_pct);
    }
    g->last_on = on;
    if (g->last_brightness_pct == 0) {
        g->last_brightness_pct = 100;
    }
}

static void fanout_general_locked(group_t *g, bool on, uint8_t bright)
{
    uint8_t count = g->member_count;
    uint8_t members[GROUP_MAX_MEMBERS][8];
    memcpy(members, g->members, sizeof(members));
    bool prefer_level = g->is_light_group;
    unlock();

    for (uint8_t i = 0; i < count; i++) {
        zb_device_t d;
        bool is_light = prefer_level;
        if (zigbee_host_get_device(members[i], &d)) {
            is_light = (zigbee_host_device_kind(&d) == ZB_DEVICE_KIND_LIGHT || d.has_level);
        }
        if (is_light) {
            zigbee_host_set_brightness(members[i], on ? bright : 0);
        } else {
            zigbee_host_set_onoff(members[i], on);
        }
    }
    lock();
}

static void resolve_power_fail(const group_t *g, bool *want_on, uint8_t *bright)
{
    uint8_t b = clamp_brightness(g->last_brightness_pct);
    switch (clamp_power_fail(g->power_fail_mode)) {
    case GROUP_POWER_FAIL_ON:
        *want_on = true;
        *bright = b;
        break;
    case GROUP_POWER_FAIL_OFF:
        *want_on = false;
        *bright = b;
        break;
    case GROUP_POWER_FAIL_PREVIOUS:
    default:
        *want_on = g->last_on;
        *bright = b;
        break;
    }
}

static void apply_power_fail_all(void)
{
    lock();
    if (s_power_applied) {
        unlock();
        return;
    }
    s_power_applied = true;
    for (uint16_t i = 0; i < GROUP_MAX; i++) {
        group_t *g = &s_list[i];
        if (!g->used || g->type != GROUP_TYPE_GENERAL || g->member_count == 0) {
            continue;
        }
        bool want_on = false;
        uint8_t bright = 100;
        resolve_power_fail(g, &want_on, &bright);
        remember_state_locked(g, want_on, bright);
        ESP_LOGI(TAG, "Power-fail apply group %u '%s' mode=%u -> %s @ %u%%", (unsigned)g->id,
                 g->name, (unsigned)g->power_fail_mode, want_on ? "ON" : "OFF", (unsigned)bright);
        fanout_general_locked(g, want_on, bright);
    }
    nvs_save_locked();
    unlock();
}

/** True power loss only — not USB flash / software / watchdog reboots. */
static bool reset_was_mains_power_event(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
    case ESP_RST_BROWNOUT:
        return true;
    default:
        return false;
    }
}

static void power_fail_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(6000));
    if (!reset_was_mains_power_event()) {
        ESP_LOGI(TAG,
                 "Skip power-fail restore (reset=%d) — lights keep their own state after "
                 "flash/reboot",
                 (int)esp_reset_reason());
        s_power_applied = true; /* do not retry until next boot */
        s_power_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    apply_power_fail_all();
    s_power_task = NULL;
    vTaskDelete(NULL);
}

static esp_err_t set_actuator(const uint8_t eui[8], bool on)
{
    if (eui_is_zero(eui)) {
        return ESP_OK;
    }
    return zigbee_host_set_onoff(eui, on);
}

/**
 * Temperature thermostat:
 *  - gap: heat ON at target-gap, cool ON at target+gap; deadband in between
 *  - hysteresis: stay on until temp moves hysteresis toward the deadband
 *  - humidity_force_heat: RH above target forces heat (cools never), ignoring temp
 *  - heater and cooler never both on
 */
static void apply_temp_control(group_t *g)
{
    bool has_heater = group_thermo_has_heater(g);
    bool has_cooler = group_thermo_has_cooler(g);
    bool want_heat = false;
    bool want_cool = false;

    if (g->mode == THERMO_MODE_OFF) {
        want_heat = false;
        want_cool = false;
    } else {
        bool humidity_forcing = false;
        if (g->humidity_force_heat && has_heater && g->has_current_humidity) {
            float rh = g->current_humidity_pct;
            float tgt_rh = g->target_humidity_pct;
            if (g->heating && rh > tgt_rh - THERMOSTAT_HYSTERESIS_RH) {
                humidity_forcing = true;
            } else if (rh >= tgt_rh + THERMOSTAT_HYSTERESIS_RH) {
                humidity_forcing = true;
            }
        }

        if (humidity_forcing) {
            want_heat = true;
            want_cool = false;
        } else if (g->has_current_temp) {
            float cur = g->current_temp_c;
            float tgt = g->target_c;
            float gap = clamp_gap(g->gap_c);
            float hyst = clamp_hysteresis(g->hysteresis_c);
            float heat_on = tgt - gap;
            float cool_on = tgt + gap;
            float heat_off = heat_on + hyst;
            float cool_off = cool_on - hyst;
            bool allow_heat = has_heater && (g->mode == THERMO_MODE_HEAT || g->mode == THERMO_MODE_AUTO);
            bool allow_cool = has_cooler && (g->mode == THERMO_MODE_COOL || g->mode == THERMO_MODE_AUTO);

            if (allow_heat) {
                if (g->heating) {
                    want_heat = (cur < heat_off);
                } else {
                    want_heat = (cur <= heat_on);
                }
            }
            if (allow_cool) {
                if (g->cooling) {
                    want_cool = (cur > cool_off);
                } else {
                    want_cool = (cur >= cool_on);
                }
            }
            /* Never both — prefer heat if somehow overlapping. */
            if (want_heat && want_cool) {
                want_cool = false;
            }
        } else {
            return;
        }
    }

    if (!has_heater) {
        want_heat = false;
    }
    if (!has_cooler) {
        want_cool = false;
    }

    if (want_heat == g->heating && want_cool == g->cooling) {
        return;
    }

    /* Mutual exclusion on the wire: turn the other off before turning one on. */
    if (want_heat && g->cooling) {
        if (set_actuator(g->cooler_eui, false) != ESP_OK) {
            ESP_LOGW(TAG, "Group %u cooler OFF failed", (unsigned)g->id);
            return;
        }
        g->cooling = false;
    }
    if (want_cool && g->heating) {
        if (set_actuator(g->switch_eui, false) != ESP_OK) {
            ESP_LOGW(TAG, "Group %u heater OFF failed", (unsigned)g->id);
            return;
        }
        g->heating = false;
    }

    if (want_heat != g->heating) {
        if (set_actuator(g->switch_eui, want_heat) != ESP_OK) {
            ESP_LOGW(TAG, "Group %u heater cmd failed", (unsigned)g->id);
            return;
        }
        g->heating = want_heat;
    }
    if (want_cool != g->cooling) {
        if (set_actuator(g->cooler_eui, want_cool) != ESP_OK) {
            ESP_LOGW(TAG, "Group %u cooler cmd failed", (unsigned)g->id);
            return;
        }
        g->cooling = want_cool;
    }

    ESP_LOGI(TAG,
             "Thermostat %u '%s': heat=%s cool=%s (T=%.1f tgt=%.1f gap=%.1f RH=%.0f mode=%u)",
             (unsigned)g->id, g->name, g->heating ? "ON" : "OFF", g->cooling ? "ON" : "OFF",
             g->has_current_temp ? (double)g->current_temp_c : 0.0, (double)g->target_c,
             (double)g->gap_c, g->has_current_humidity ? (double)g->current_humidity_pct : -1.0,
             (unsigned)g->mode);
    nvs_save_locked();
}

static void apply_humidity_regulator(group_t *g)
{
    bool want_on = g->heating;
    if (g->mode == THERMO_MODE_OFF) {
        want_on = false;
    } else if (!g->has_current_humidity) {
        return;
    } else {
        float cur = g->current_humidity_pct;
        float tgt = g->target_humidity_pct;
        if (g->mode == THERMO_MODE_HUMIDIFY) {
            if (cur <= tgt - THERMOSTAT_HYSTERESIS_RH) {
                want_on = true;
            } else if (cur >= tgt + THERMOSTAT_HYSTERESIS_RH) {
                want_on = false;
            }
        } else if (g->mode == THERMO_MODE_DEHUMIDIFY) {
            if (cur >= tgt + THERMOSTAT_HYSTERESIS_RH) {
                want_on = true;
            } else if (cur <= tgt - THERMOSTAT_HYSTERESIS_RH) {
                want_on = false;
            }
        } else {
            want_on = false;
        }
    }
    if (want_on == g->heating) {
        return;
    }
    if (set_actuator(g->switch_eui, want_on) != ESP_OK) {
        ESP_LOGW(TAG, "Group %u humidity actuator failed", (unsigned)g->id);
        return;
    }
    g->heating = want_on;
    g->cooling = false;
    ESP_LOGI(TAG, "Humidity %u '%s': relay %s (cur=%.0f%% tgt=%.0f%% mode=%u)", (unsigned)g->id,
             g->name, want_on ? "ON" : "OFF", (double)g->current_humidity_pct,
             (double)g->target_humidity_pct, (unsigned)g->mode);
    nvs_save_locked();
}

static void apply_thermo_control(group_t *g)
{
    if (!g || g->type != GROUP_TYPE_THERMOSTAT) {
        return;
    }
    refresh_runtime_locked(g);
    if (g->thermo_kind == GROUP_THERMO_KIND_HUMIDITY) {
        apply_humidity_regulator(g);
    } else {
        apply_temp_control(g);
    }
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
        bool match = (memcmp(g->sensor_eui, sensor_eui, 8) == 0);
        if (!match && !eui_is_zero(g->humidity_sensor_eui) &&
            memcmp(g->humidity_sensor_eui, sensor_eui, 8) == 0) {
            match = true;
        }
        if (!match) {
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
    nvs_save_locked();
    unlock();
    if (!s_task) {
        if (xTaskCreate(control_task, "groups", 4096, NULL, 4, &s_task) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (!s_power_task && !s_power_applied) {
        if (!reset_was_mains_power_event()) {
            ESP_LOGI(TAG, "Power-fail restore armed only for mains power-on/brownout (reset=%d)",
                     (int)esp_reset_reason());
            s_power_applied = true;
        } else if (xTaskCreate(power_fail_task, "grp_pwr", 4096, NULL, 4, &s_power_task) !=
                   pdPASS) {
            ESP_LOGW(TAG, "Power-fail apply task failed — applying immediately");
            apply_power_fail_all();
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

esp_err_t group_create_thermostat(const char *name, uint8_t thermo_kind, const uint8_t sensor_eui[8],
                                  const uint8_t humidity_sensor_eui[8], const uint8_t heater_eui[8],
                                  const uint8_t cooler_eui[8], float target_c,
                                  float target_humidity_pct, float gap_c, float hysteresis_c,
                                  bool humidity_force_heat, uint8_t mode, bool homekit_expose,
                                  uint8_t *id_out)
{
    if (!name || !name[0] || !sensor_eui) {
        return ESP_ERR_INVALID_ARG;
    }
    thermo_kind = clamp_thermo_kind(thermo_kind);
    zb_device_t sens;
    if (!zigbee_host_get_device(sensor_eui, &sens)) {
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t heater[8] = {0};
    uint8_t cooler[8] = {0};
    uint8_t hum_sens[8] = {0};
    if (heater_eui && !eui_is_zero(heater_eui)) {
        memcpy(heater, heater_eui, 8);
    }
    if (cooler_eui && !eui_is_zero(cooler_eui)) {
        memcpy(cooler, cooler_eui, 8);
    }
    if (humidity_sensor_eui && !eui_is_zero(humidity_sensor_eui)) {
        memcpy(hum_sens, humidity_sensor_eui, 8);
    }

    if (thermo_kind == GROUP_THERMO_KIND_HUMIDITY) {
        if (eui_is_zero(heater) || !sens.has_humidity) {
            return ESP_ERR_INVALID_ARG;
        }
        zb_device_t sw;
        if (!zigbee_host_get_device(heater, &sw)) {
            return ESP_ERR_NOT_FOUND;
        }
    } else {
        if (eui_is_zero(heater) && eui_is_zero(cooler)) {
            return ESP_ERR_INVALID_ARG;
        }
        if (!eui_is_zero(heater)) {
            zb_device_t sw;
            if (!zigbee_host_get_device(heater, &sw)) {
                return ESP_ERR_NOT_FOUND;
            }
        }
        if (!eui_is_zero(cooler)) {
            zb_device_t sw;
            if (!zigbee_host_get_device(cooler, &sw)) {
                return ESP_ERR_NOT_FOUND;
            }
        }
        if (!eui_is_zero(hum_sens)) {
            zb_device_t hs;
            /* Allow restore before ZCL interview has set has_humidity. */
            if (!zigbee_host_get_device(hum_sens, &hs)) {
                return ESP_ERR_NOT_FOUND;
            }
        }
    }

    bool has_h = !eui_is_zero(heater);
    bool has_c = !eui_is_zero(cooler);
    mode = normalize_thermo_mode(mode, thermo_kind, has_h, has_c);

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
    g->thermo_kind = thermo_kind;
    snprintf(g->name, sizeof(g->name), "%s", name);
    memcpy(g->sensor_eui, sensor_eui, 8);
    memcpy(g->humidity_sensor_eui, hum_sens, 8);
    memcpy(g->switch_eui, heater, 8);
    memcpy(g->cooler_eui, cooler, 8);
    g->target_c = clamp_target(target_c > 0 ? target_c : 21.0f);
    g->target_humidity_pct =
        clamp_target_humidity(target_humidity_pct > 0 ? target_humidity_pct : 45.0f);
    g->gap_c = clamp_gap(gap_c > 0 ? gap_c : THERMOSTAT_DEFAULT_GAP_C);
    g->hysteresis_c =
        clamp_hysteresis(hysteresis_c > 0 ? hysteresis_c : THERMOSTAT_DEFAULT_HYSTERESIS_C);
    g->humidity_force_heat = humidity_force_heat && has_h;
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
                               bool homekit_expose, uint8_t power_fail_mode, uint8_t *id_out)
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
    g->power_fail_mode = clamp_power_fail(power_fail_mode);
    g->last_on = false;
    g->last_brightness_pct = 100;
    g->on = false;
    g->brightness_pct = 100;
    refresh_runtime_locked(g);
    nvs_save_locked();
    if (id_out) {
        *id_out = id;
    }
    ESP_LOGI(TAG, "Created general group %u '%s' members=%u light=%d power_fail=%u", (unsigned)id,
             g->name, (unsigned)g->member_count, (int)g->is_light_group,
             (unsigned)g->power_fail_mode);
    unlock();
    return ESP_OK;
}

esp_err_t thermostat_create(const char *name, const uint8_t sensor_eui[8],
                            const uint8_t switch_eui[8], float target_c, uint8_t mode,
                            bool homekit_expose, uint8_t *id_out)
{
    return group_create_thermostat(name, GROUP_THERMO_KIND_TEMP, sensor_eui, NULL, switch_eui, NULL,
                                   target_c, 45.0f, THERMOSTAT_DEFAULT_GAP_C,
                                   THERMOSTAT_DEFAULT_HYSTERESIS_C, false, mode, homekit_expose,
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
        if (upd->set_thermo_kind) {
            g->thermo_kind = clamp_thermo_kind(upd->thermo_kind);
        }
        if (upd->set_sensor) {
            memcpy(g->sensor_eui, upd->sensor_eui, 8);
        }
        if (upd->set_humidity_sensor) {
            memcpy(g->humidity_sensor_eui, upd->humidity_sensor_eui, 8);
        }
        if (upd->set_switch) {
            memcpy(g->switch_eui, upd->switch_eui, 8);
        }
        if (upd->set_cooler) {
            memcpy(g->cooler_eui, upd->cooler_eui, 8);
        }
        if (upd->set_target) {
            g->target_c = clamp_target(upd->target_c);
        }
        if (upd->set_target_humidity) {
            g->target_humidity_pct = clamp_target_humidity(upd->target_humidity_pct);
        }
        if (upd->set_gap) {
            g->gap_c = clamp_gap(upd->gap_c);
        }
        if (upd->set_hysteresis) {
            g->hysteresis_c = clamp_hysteresis(upd->hysteresis_c);
        }
        if (upd->set_humidity_force_heat) {
            g->humidity_force_heat = upd->humidity_force_heat && group_thermo_has_heater(g);
        }
        if (g->thermo_kind == GROUP_THERMO_KIND_TEMP && !group_thermo_has_heater(g) &&
            !group_thermo_has_cooler(g)) {
            unlock();
            return ESP_ERR_INVALID_ARG;
        }
        g->mode = normalize_thermo_mode(upd->set_mode ? upd->mode : g->mode, g->thermo_kind,
                                        group_thermo_has_heater(g), group_thermo_has_cooler(g));
        if (!group_thermo_has_heater(g) && g->heating) {
            g->heating = false;
        }
        if (!group_thermo_has_cooler(g) && g->cooling) {
            g->cooling = false;
        }
    } else if (upd->set_members && upd->member_count > 0 &&
               upd->member_count <= GROUP_MAX_MEMBERS) {
        g->member_count = upd->member_count;
        memcpy(g->members, upd->members, (size_t)upd->member_count * 8);
        g->is_light_group = false;
        refresh_runtime_locked(g);
    }
    if (g->type == GROUP_TYPE_GENERAL && upd->set_power_fail) {
        g->power_fail_mode = clamp_power_fail(upd->power_fail_mode);
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
    if (g->type == GROUP_TYPE_THERMOSTAT) {
        if (g->heating) {
            set_actuator(g->switch_eui, false);
        }
        if (g->cooling) {
            set_actuator(g->cooler_eui, false);
        }
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

bool thermostat_on_sensor_button(const uint8_t sensor_eui[8], float *target_out)
{
    if (!sensor_eui) {
        return false;
    }
    uint8_t best_id = 0;
    float best_target = 0.0f;
    bool found = false;
    lock();
    for (uint16_t i = 0; i < GROUP_MAX; i++) {
        group_t *g = &s_list[i];
        if (!g->used || g->type != GROUP_TYPE_THERMOSTAT) {
            continue;
        }
        if (g->thermo_kind != GROUP_THERMO_KIND_TEMP) {
            continue;
        }
        if (memcmp(g->sensor_eui, sensor_eui, 8) != 0) {
            continue;
        }
        if (!found || g->id < best_id) {
            found = true;
            best_id = g->id;
            best_target = g->target_c;
        }
    }
    unlock();
    if (!found) {
        return false;
    }
    float next = best_target + 0.5f;
    if (next > 38.0f + 0.01f) {
        next = 10.0f;
    } else {
        next = clamp_target(next);
    }
    if (thermostat_set_target(best_id, next) != ESP_OK) {
        return false;
    }
    if (target_out) {
        *target_out = next;
    }
    ESP_LOGI(TAG, "Sensor button: thermostat id=%u target %.1f → %.1f °C", (unsigned)best_id,
             (double)best_target, (double)next);
    return true;
}

esp_err_t thermostat_set_mode(uint8_t id, uint8_t mode)
{
    group_update_t upd = {0};
    upd.set_mode = true;
    upd.mode = mode;
    return group_update(id, &upd);
}

esp_err_t thermostat_set_target_humidity(uint8_t id, float target_humidity_pct)
{
    group_update_t upd = {0};
    upd.set_target_humidity = true;
    upd.target_humidity_pct = target_humidity_pct;
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
    uint8_t bright = clamp_brightness(g->last_brightness_pct ? g->last_brightness_pct
                                                              : g->brightness_pct);
    remember_state_locked(g, on, bright);
    nvs_save_locked();
    ESP_LOGI(TAG, "Group %u On/Off -> %s (%u members)", (unsigned)id, on ? "ON" : "OFF",
             (unsigned)g->member_count);
    fanout_general_locked(g, on, bright);
    unlock();
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
    uint8_t pct = brightness_pct > 100 ? 100 : brightness_pct;
    bool on = pct > 0;
    remember_state_locked(g, on, on ? pct : g->last_brightness_pct);
    g->is_light_group = true;
    nvs_save_locked();
    ESP_LOGI(TAG, "Group %u brightness -> %u%% (%u members)", (unsigned)id, (unsigned)pct,
             (unsigned)g->member_count);
    uint8_t count = g->member_count;
    uint8_t members[GROUP_MAX_MEMBERS][8];
    memcpy(members, g->members, sizeof(members));
    unlock();

    for (uint8_t i = 0; i < count; i++) {
        zb_device_t d;
        if (!zigbee_host_get_device(members[i], &d)) {
            ESP_LOGW(TAG, "Group %u member %u not in inventory", (unsigned)id, (unsigned)i);
            continue;
        }
        esp_err_t err = zigbee_host_set_brightness(members[i], pct);
        if (err != ESP_OK) {
            zigbee_host_set_onoff(members[i], pct > 0);
        }
    }
    return ESP_OK;
}
