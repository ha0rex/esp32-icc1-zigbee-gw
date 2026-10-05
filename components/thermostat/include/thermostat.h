#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GROUP_MAX 8
#define GROUP_MAX_MEMBERS 8
/** Defaults when creating a temperature thermostat. */
#define THERMOSTAT_DEFAULT_GAP_C 1.0f
#define THERMOSTAT_DEFAULT_HYSTERESIS_C 0.5f
#define THERMOSTAT_HYSTERESIS_RH 3.0f
/** @deprecated old heat-only hysteresis; prefer gap_c / hysteresis_c */
#define THERMOSTAT_HYSTERESIS_C THERMOSTAT_DEFAULT_HYSTERESIS_C

/** Keep old name as alias for callers during transition. */
#define THERMOSTAT_MAX GROUP_MAX

typedef enum {
    GROUP_TYPE_GENERAL = 0,
    GROUP_TYPE_THERMOSTAT = 1,
} group_type_t;

/** What a general group does after gateway reboot / power restore. */
typedef enum {
    GROUP_POWER_FAIL_PREVIOUS = 0, /**< Restore last commanded on/off + brightness */
    GROUP_POWER_FAIL_ON = 1,
    GROUP_POWER_FAIL_OFF = 2,
} group_power_fail_t;

typedef enum {
    GROUP_THERMO_KIND_TEMP = 0,
    GROUP_THERMO_KIND_HUMIDITY = 1, /**< Dedicated humidifier/dehumidifier regulator */
} group_thermo_kind_t;

typedef enum {
    THERMO_MODE_OFF = 0,
    THERMO_MODE_HEAT = 1,
    THERMO_MODE_HUMIDIFY = 2,
    THERMO_MODE_DEHUMIDIFY = 3,
    THERMO_MODE_COOL = 4,
    THERMO_MODE_AUTO = 5, /**< Heat + cool with deadband; never both at once */
} thermo_mode_t;

typedef struct {
    bool used;
    uint8_t id;
    char name[32];
    uint8_t type; /**< group_type_t */
    bool homekit_expose;

    /* Thermostat / humidity regulator */
    uint8_t thermo_kind; /**< group_thermo_kind_t */
    uint8_t sensor_eui[8];          /**< Temperature sensor (temp kind) or RH sensor (humidity kind) */
    uint8_t humidity_sensor_eui[8]; /**< Optional RH sensor; all-zero → use sensor_eui */
    uint8_t switch_eui[8];          /**< Heater / humidity actuator; all-zero → none (temp kind) */
    uint8_t cooler_eui[8];          /**< Cooler switch; all-zero → none */
    float target_c;
    float target_humidity_pct;
    float gap_c;         /**< °C from target before heat/cool turns on (deadband half-width) */
    float hysteresis_c;  /**< °C past on-threshold before turning off */
    bool humidity_force_heat; /**< When RH above target, force heat regardless of temp */
    uint8_t mode;
    bool heating; /**< Heater / humidity actuator on */
    bool cooling; /**< Cooler on (temp kind only) */

    /**
     * "remote:<id>" when the device lives on the linked gateway.
     * Empty means the matching EUI is a local Zigbee device.
     */
    char sensor_ref[40];
    char switch_ref[40];
    char cooler_ref[40];
    char window_ref[40];
    bool window_open;
    bool window_hold;    /**< Heat is held off because the window is open */
    bool saved_heating;  /**< Heater state to restore when the window closes */

    /* General group members */
    uint8_t member_count;
    uint8_t members[GROUP_MAX_MEMBERS][8];

    /* General: power-fail behaviour (persisted) */
    uint8_t power_fail_mode; /**< group_power_fail_t */
    bool last_on;
    uint8_t last_brightness_pct; /**< 1–100 when known; 0 → treat as 100 */

    /* Runtime */
    bool has_current_temp;
    float current_temp_c;
    bool has_current_humidity;
    float current_humidity_pct;
    bool on;
    uint8_t brightness_pct; /**< 0–100 for light groups */
    bool is_light_group;    /**< true if any member is a light */
} group_t;

typedef group_t thermostat_t; /* backward-compatible alias for HomeKit thermo path */

typedef struct {
    char name[32];
    uint8_t type;
    bool homekit_expose;
    uint8_t thermo_kind;
    uint8_t sensor_eui[8];
    uint8_t humidity_sensor_eui[8];
    uint8_t switch_eui[8];
    uint8_t cooler_eui[8];
    float target_c;
    float target_humidity_pct;
    float gap_c;
    float hysteresis_c;
    bool humidity_force_heat;
    uint8_t mode;
    uint8_t member_count;
    uint8_t members[GROUP_MAX_MEMBERS][8];
    uint8_t power_fail_mode;
    bool set_name;
    bool set_homekit;
    bool set_thermo_kind;
    bool set_sensor;
    bool set_humidity_sensor;
    bool set_switch;
    bool set_cooler;
    char sensor_ref[40];
    char switch_ref[40];
    char cooler_ref[40];
    char window_ref[40];
    bool set_sensor_ref;
    bool set_switch_ref;
    bool set_cooler_ref;
    bool set_window_ref;
    bool set_target;
    bool set_target_humidity;
    bool set_gap;
    bool set_hysteresis;
    bool set_humidity_force_heat;
    bool set_mode;
    bool set_members;
    bool set_power_fail;
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

bool group_thermo_has_heater(const group_t *g);
bool group_thermo_has_cooler(const group_t *g);

esp_err_t group_create_thermostat(const char *name, uint8_t thermo_kind, const uint8_t sensor_eui[8],
                                  const uint8_t humidity_sensor_eui[8], const uint8_t heater_eui[8],
                                  const uint8_t cooler_eui[8], float target_c,
                                  float target_humidity_pct, float gap_c, float hysteresis_c,
                                  bool humidity_force_heat, uint8_t mode, bool homekit_expose,
                                  const char *sensor_ref, const char *switch_ref,
                                  const char *cooler_ref, const char *window_ref, uint8_t *id_out);

esp_err_t group_create_general(const char *name, const uint8_t members[][8], uint8_t member_count,
                               bool homekit_expose, uint8_t power_fail_mode, uint8_t *id_out);

esp_err_t group_update(uint8_t id, const group_update_t *upd);
esp_err_t group_remove(uint8_t id);

esp_err_t thermostat_create(const char *name, const uint8_t sensor_eui[8],
                            const uint8_t switch_eui[8], float target_c, uint8_t mode,
                            bool homekit_expose, uint8_t *id_out);
esp_err_t thermostat_remove(uint8_t id);
esp_err_t thermostat_set_target(uint8_t id, float target_c);
esp_err_t thermostat_set_target_humidity(uint8_t id, float target_humidity_pct);
esp_err_t thermostat_set_mode(uint8_t id, uint8_t mode);
typedef group_update_t thermostat_update_t;
esp_err_t thermostat_update(uint8_t id, const thermostat_update_t *upd);

/** HomeKit control for general groups. */
esp_err_t group_set_onoff(uint8_t id, bool on);
esp_err_t group_set_brightness(uint8_t id, uint8_t brightness_pct);

/** Re-read sensor into any thermostat using this EUI and re-evaluate heat/cool. */
void group_on_sensor_updated(const uint8_t sensor_eui[8]);

/**
 * Wake-button on a climate sensor: bump target of the first temperature thermostat
 * that uses this EUI as sensor_eui by +0.5°C, wrapping 38→10.
 * Returns true and writes *target_out if a group was updated.
 */
bool thermostat_on_sensor_button(const uint8_t sensor_eui[8], float *target_out);

#ifdef __cplusplus
}
#endif
