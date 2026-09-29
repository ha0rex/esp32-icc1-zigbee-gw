#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GROUP_MAX 8
#define GROUP_MAX_MEMBERS 8
#define THERMOSTAT_HYSTERESIS_C 0.5f

/** Keep old name as alias for callers during transition. */
#define THERMOSTAT_MAX GROUP_MAX

typedef enum {
    GROUP_TYPE_GENERAL = 0,
    GROUP_TYPE_THERMOSTAT = 1,
} group_type_t;

typedef enum {
    THERMO_MODE_OFF = 0,
    THERMO_MODE_HEAT = 1,
} thermo_mode_t;

typedef struct {
    bool used;
    uint8_t id;
    char name[32];
    uint8_t type; /**< group_type_t */
    bool homekit_expose;

    /* Thermostat */
    uint8_t sensor_eui[8];
    uint8_t switch_eui[8];
    float target_c;
    uint8_t mode;
    bool heating;

    /* General group members */
    uint8_t member_count;
    uint8_t members[GROUP_MAX_MEMBERS][8];

    /* Runtime */
    bool has_current_temp;
    float current_temp_c;
    bool on;
    uint8_t brightness_pct; /**< 0–100 for light groups */
    bool is_light_group;    /**< true if any member is a light */
} group_t;

typedef group_t thermostat_t; /* backward-compatible alias for HomeKit thermo path */

typedef struct {
    char name[32];
    uint8_t type;
    bool homekit_expose;
    uint8_t sensor_eui[8];
    uint8_t switch_eui[8];
    float target_c;
    uint8_t mode;
    uint8_t member_count;
    uint8_t members[GROUP_MAX_MEMBERS][8];
    bool set_name;
    bool set_homekit;
    bool set_sensor;
    bool set_switch;
    bool set_target;
    bool set_mode;
    bool set_members;
} group_update_t;

esp_err_t group_start(void);
/** @deprecated use group_start */
esp_err_t thermostat_start(void);

uint16_t group_count(void);
uint16_t group_count_homekit_exposed(void);
uint16_t thermostat_count(void);
uint16_t thermostat_count_homekit_exposed(void);

bool group_get_at(uint16_t index, group_t *out);
bool group_get_by_id(uint8_t id, group_t *out);
bool thermostat_get_at(uint16_t index, thermostat_t *out);
bool thermostat_get_by_id(uint8_t id, thermostat_t *out);

esp_err_t group_create_thermostat(const char *name, const uint8_t sensor_eui[8],
                                  const uint8_t switch_eui[8], float target_c, uint8_t mode,
                                  bool homekit_expose, uint8_t *id_out);

esp_err_t group_create_general(const char *name, const uint8_t members[][8], uint8_t member_count,
                               bool homekit_expose, uint8_t *id_out);

esp_err_t group_update(uint8_t id, const group_update_t *upd);
esp_err_t group_remove(uint8_t id);

esp_err_t thermostat_create(const char *name, const uint8_t sensor_eui[8],
                            const uint8_t switch_eui[8], float target_c, uint8_t mode,
                            bool homekit_expose, uint8_t *id_out);
esp_err_t thermostat_remove(uint8_t id);
esp_err_t thermostat_set_target(uint8_t id, float target_c);
esp_err_t thermostat_set_mode(uint8_t id, uint8_t mode);
typedef group_update_t thermostat_update_t;
esp_err_t thermostat_update(uint8_t id, const thermostat_update_t *upd);

/** HomeKit control for general groups. */
esp_err_t group_set_onoff(uint8_t id, bool on);
esp_err_t group_set_brightness(uint8_t id, uint8_t brightness_pct);

/** Re-read sensor into any thermostat using this EUI and re-evaluate heat. */
void group_on_sensor_updated(const uint8_t sensor_eui[8]);

#ifdef __cplusplus
}
#endif
