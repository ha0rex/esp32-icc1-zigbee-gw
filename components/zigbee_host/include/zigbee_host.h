#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ezsp.h"
#include "ash.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ZB_HOST_MAX_DEVICES 32

typedef enum {
    ICC_STATUS_UNKNOWN = 0,
    ICC_STATUS_NOT_CONNECTED,
    ICC_STATUS_CONNECTED,
} icc_link_status_t;

#define ZB_BTN_MODE_STATELESS 0
#define ZB_BTN_MODE_STATEFUL  1

#define ZB_REMOTE_TYPE_HOMEKIT  0 /**< Per-button HomeKit SPS / Switch */
#define ZB_REMOTE_TYPE_CONTROL  1 /**< Control selected lights/switches (GW-synced absolute) */
#define ZB_REMOTE_MAX_TARGETS   8

typedef enum {
    ZB_DEVICE_KIND_UNKNOWN = 0,
    ZB_DEVICE_KIND_SENSOR,     /**< Climate: temperature / humidity */
    ZB_DEVICE_KIND_SWITCH,
    ZB_DEVICE_KIND_LIGHT,
    ZB_DEVICE_KIND_REMOTE,
    ZB_DEVICE_KIND_OUTLET,
    ZB_DEVICE_KIND_IRRIGATION, /**< Water valve / sprinkler (e.g. Sonoff SWV) */
    ZB_DEVICE_KIND_CONTACT,
    ZB_DEVICE_KIND_MOTION,
    ZB_DEVICE_KIND_LEAK,
    ZB_DEVICE_KIND_SMOKE,
} zb_device_kind_t;

/** Real NCP-reported device with optional ZCL interview / sensor values. */
typedef struct {
    bool used;
    uint8_t eui64[8];
    uint16_t node_id;
    uint8_t node_type;
    char name[32];          /**< User-editable friendly name */
    char manufacturer[32];  /**< ZCL Basic manufacturerName */
    char model[32];         /**< ZCL Basic modelIdentifier */
    char firmware[32];      /**< ZCL Basic swBuildId or applicationVersion */
    char label[40];         /**< Short UI summary */
    bool has_temp;
    float temperature_c;
    bool has_humidity;
    float humidity_pct;
    bool has_battery;
    uint8_t battery_pct;
    int8_t last_rssi;
    uint8_t last_lqi;
    bool homekit_expose;
    int64_t last_seen_ms;
    int64_t last_interview_ms;
    /* Appended fields — older NVS blobs memcpy cleanly up to last_interview_ms. */
    bool has_onoff;
    bool onoff_on;
    uint8_t onoff_ep;
    bool has_level;
    uint8_t level; /**< ZCL CurrentLevel 0–254 */
    uint8_t level_ep;
    /* Remote / programmable switch (appended after level for NVS compatibility). */
    bool remote_bound; /**< Button ZCL seen (or binds installed) */
    uint8_t remote_ep; /**< Endpoint that emits button commands */
    bool remote_no_zdo_bind; /**< BindRsp 0x84 — remote rejects coordinator ZDO Bind; F&B only */
    /* Sensor reporting (appended — older NVS blobs zero-fill). */
    bool sensor_reporting; /**< Confirmed via CfgReportRsp or Attribute Report */
    uint8_t sensor_ep;     /**< Endpoint that reports temp/humidity */
    /**
     * Per-button HomeKit mode (appended). 0 = stateless programmable switch (default),
     * 1 = stateful On/Off switch. Length = ZB_REMOTE_MAX_BUTTONS.
     */
    uint8_t btn_mode[ZB_REMOTE_MAX_BUTTONS];
    /** On/Off latch for stateful buttons (ignored for stateless). */
    bool btn_on[ZB_REMOTE_MAX_BUTTONS];
    /**
     * Remote operating mode (appended). 0 = HomeKit button mapping (default),
     * 1 = control a selected light/switch with native on/off/dim.
     */
    uint8_t remote_type;
    uint8_t target_eui64[8]; /**< Legacy single target — mirrored from targets[0] */
    /** Touchlink / bind group this remote uses (e.g. 51500). 0 = unknown. */
    uint16_t remote_group_id;
    /** CONTROL-mode targets (appended). Flat device list after group expansion. */
    uint8_t target_count;
    uint8_t targets[ZB_REMOTE_MAX_TARGETS][8];
    /** Portal group ids selected in the UI (for round-trip; members also in targets). */
    uint8_t target_group_count;
    uint8_t target_groups[ZB_REMOTE_MAX_TARGETS];
    /**
     * Sleepy sensor setup progress (appended). One APS frame per step — flooding
     * bind+cfgReport+read while the SNZB is asleep always ends in DELIVERY_FAILED.
     */
    uint8_t sensor_cfg_step;
    /**
     * Per-button HomeKit Name (appended). Empty → default Power/Dimmer/… labels.
     * Portal + Home app renames persist here.
     */
    char btn_name[ZB_REMOTE_MAX_BUTTONS][24];
    /* Binary / IAS / occupancy (appended — older NVS zero-fills). */
    bool has_ias_zone;
    uint16_t ias_zone_type;   /**< ZCL ZoneType */
    uint16_t ias_zone_status; /**< ZCL ZoneStatus bitmap */
    uint8_t ias_zone_ep;
    bool has_occupancy;
    bool occupancy; /**< true = occupied / motion */
    uint8_t occupancy_ep;
    bool binary_on; /**< Open/detected/active derived state for portal + HK sync */
} zb_device_t;

typedef struct {
    icc_link_status_t icc_status;
    ash_state_t ash_state;
    uint8_t ash_reset_code;
    ezsp_ncp_info_t ncp;
    bool version_ok;
    ash_stats_t ash_stats;
    ezsp_stats_t ezsp_stats;
    uint32_t connect_attempts;
    uint32_t connect_successes;
    uint8_t permit_join_remaining; /**< 0 = closed; approx seconds left */
    uint16_t device_count;
    zb_device_t devices[ZB_HOST_MAX_DEVICES];
    char last_error[96];
} zigbee_host_status_t;

typedef struct {
    uint8_t channel;     /**< 11–26; 0 = default 15 */
    int8_t tx_power;     /**< dBm; typically 8 */
    uint16_t pan_id;     /**< 0 = random */
} zigbee_form_options_t;

typedef struct {
    char name[32];
    bool homekit_expose;
    bool set_name;
    bool set_homekit;
    bool set_btn_modes;
    uint8_t btn_modes[ZB_REMOTE_MAX_BUTTONS];
    uint8_t btn_mode_count; /**< how many entries in btn_modes are valid */
    bool set_btn_names;
    char btn_names[ZB_REMOTE_MAX_BUTTONS][24];
    uint8_t btn_name_count;
    bool set_remote_type;
    uint8_t remote_type;
    bool set_targets;
    uint8_t target_count;
    uint8_t targets[ZB_REMOTE_MAX_TARGETS][8];
    uint8_t target_group_count;
    uint8_t target_groups[ZB_REMOTE_MAX_TARGETS];
} zb_device_update_t;

esp_err_t zigbee_host_start(void);
void zigbee_host_get_status(zigbee_host_status_t *out);
/** True once ICC is up or NVS devices are present (no large status copy). */
bool zigbee_host_is_ready(void);

esp_err_t zigbee_host_form_network(const zigbee_form_options_t *opt);
esp_err_t zigbee_host_leave_network(void);
esp_err_t zigbee_host_permit_join(uint8_t duration_sec);
esp_err_t zigbee_host_refresh_devices(void);
esp_err_t zigbee_host_refresh_network(void);

/** Trigger ZCL attribute reads for a device (by EUI64). */
esp_err_t zigbee_host_interview_device(const uint8_t eui64[8]);
/** Classify device kind from model + clusters (light/switch/outlet/irrigation/…). */
zb_device_kind_t zigbee_host_device_kind(const zb_device_t *d);
/** True when device is an On/Off actuator HomeKit/groups can control (excl. remotes). */
bool zigbee_host_is_onoff_actuator(const zb_device_t *d);
/** True when kind is a binary HomeKit sensor (contact/motion/leak/smoke). */
bool zigbee_host_is_binary_sensor(const zb_device_t *d);
/** Button count for remotes (1–5). Returns 0 if not a remote. */
uint8_t zigbee_host_remote_button_count(const zb_device_t *d);
/** Default label for button index (Power/Left/…). */
const char *zigbee_host_remote_button_default_name(uint8_t nbtn, uint8_t button_index);
/** Effective display name (custom if set, else default). */
const char *zigbee_host_remote_button_name(const zb_device_t *d, uint8_t button_index);
/** Persist a custom button name (empty clears to default). */
esp_err_t zigbee_host_btn_set_name(const uint8_t eui64[8], uint8_t button_index, const char *name);
/** True if button_index (0-based) is configured as a stateful HomeKit switch. */
bool zigbee_host_btn_is_stateful(const zb_device_t *d, uint8_t button_index);
/** Read/toggle/set latch for a stateful remote button; persists to NVS. */
bool zigbee_host_btn_get_on(const uint8_t eui64[8], uint8_t button_index);
esp_err_t zigbee_host_btn_set_on(const uint8_t eui64[8], uint8_t button_index, bool on);
/** Toggle and return new state (stateful only). */
bool zigbee_host_btn_toggle(const uint8_t eui64[8], uint8_t button_index);
/**
 * Optional callback fired when a remote button command is received.
 * event is HK_BTN_EVENT_SINGLE / DOUBLE / LONG; button_index is 0-based.
 */
typedef void (*zb_remote_button_cb_t)(const uint8_t eui64[8], uint8_t button_index, uint8_t event);
void zigbee_host_set_remote_button_cb(zb_remote_button_cb_t cb);

/**
 * Optional callback when a sensor's temp/humidity/battery cache changes
 * (attribute report or read response). Used to push live HomeKit EVENTs.
 */
typedef void (*zb_sensor_update_cb_t)(const uint8_t eui64[8]);
void zigbee_host_set_sensor_update_cb(zb_sensor_update_cb_t cb);

#define ZB_REMOTE_PRESS_LOG 12

/** Recent remote button press (RAM only — for portal live test). */
typedef struct {
    bool used;
    uint8_t eui64[8];
    uint8_t button; /**< 1-based button number (HomeKit Button N) */
    uint8_t event;  /**< HK_BTN_EVENT_* */
    int64_t ms;     /**< now_ms() when received */
} zb_remote_press_t;

/** Copy up to max recent presses (newest first). Returns count copied. */
uint8_t zigbee_host_copy_remote_presses(zb_remote_press_t *out, uint8_t max);

/** Send On/Off command to a switch; returns ESP_ERR_NOT_FOUND if unknown. */
esp_err_t zigbee_host_set_onoff(const uint8_t eui64[8], bool on);
/** Set brightness 0–100% (Level Control). Queued like On/Off. */
esp_err_t zigbee_host_set_brightness(const uint8_t eui64[8], uint8_t brightness_pct);
/** Convert stored ZCL level 0–254 to HomeKit brightness 0–100. */
uint8_t zigbee_host_level_to_brightness(uint8_t level);
/** Convert HomeKit brightness 0–100 to ZCL level 0–254. */
uint8_t zigbee_host_brightness_to_level(uint8_t brightness_pct);
/** Update editable fields; returns ESP_ERR_NOT_FOUND if unknown. */
esp_err_t zigbee_host_update_device(const uint8_t eui64[8], const zb_device_update_t *upd);
/** Remove device from local inventory (does not leave the Zigbee network). */
esp_err_t zigbee_host_remove_device(const uint8_t eui64[8]);
/** Copy one device by EUI64; returns false if not found. */
bool zigbee_host_get_device(const uint8_t eui64[8], zb_device_t *out);
/** Copy device at slot index (0 .. ZB_HOST_MAX_DEVICES-1). Returns false if empty. */
bool zigbee_host_get_device_at(uint16_t index, zb_device_t *out);
/** Force-flush device inventory (names, modes, expose) to NVS now. */
esp_err_t zigbee_host_save_devices_now(void);
/** How many devices currently have homekit_expose enabled. */
uint16_t zigbee_host_count_homekit_exposed(void);
/** Register / refresh a device in the local inventory (e.g. recover after NVS loss). */
esp_err_t zigbee_host_register_device(const uint8_t eui64[8], uint16_t node_id, uint8_t node_type);

const char *zigbee_host_icc_status_str(icc_link_status_t s);
const char *zigbee_host_ash_state_str(ash_state_t s);
const char *zigbee_host_network_state_str(ezsp_network_status_t s);

#ifdef __cplusplus
}
#endif
