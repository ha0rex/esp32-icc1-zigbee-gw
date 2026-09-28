#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EZSP_PROTOCOL_VERSION_V8 8

#define EZSP_FRAME_VERSION                 0x0000
#define EZSP_FRAME_GET_EUI64               0x0026
#define EZSP_FRAME_GET_NODE_ID             0x0027
#define EZSP_FRAME_NETWORK_STATE           0x0018
#define EZSP_FRAME_NOP                     0x0005
#define EZSP_FRAME_NETWORK_INIT            0x0017
#define EZSP_FRAME_FORM_NETWORK            0x001E
#define EZSP_FRAME_LEAVE_NETWORK           0x0020
#define EZSP_FRAME_PERMIT_JOINING          0x0022
#define EZSP_FRAME_GET_NETWORK_PARAMETERS  0x0028
#define EZSP_FRAME_GET_CHILD_DATA          0x0029
#define EZSP_FRAME_SET_INITIAL_SECURITY    0x0068
#define EZSP_FRAME_SET_CONFIG_VALUE        0x0053
#define EZSP_FRAME_SET_POLICY              0x0055
#define EZSP_FRAME_ADD_ENDPOINT            0x0002
#define EZSP_FRAME_SEND_UNICAST            0x0034
#define EZSP_FRAME_CALLBACK                0x0006
#define EZSP_FRAME_NO_CALLBACKS            0x0007
#define EZSP_FRAME_INVALID_CMD             0x0058
#define EZSP_FRAME_STACK_STATUS_HANDLER    0x0019
#define EZSP_FRAME_CHILD_JOIN_HANDLER      0x0023
#define EZSP_FRAME_TRUST_CENTER_JOIN_HANDLER 0x0024
#define EZSP_FRAME_INCOMING_MESSAGE_HANDLER 0x0045
#define EZSP_FRAME_MESSAGE_SENT_HANDLER    0x003F
#define EZSP_FRAME_POLL_HANDLER            0x0080

#define ZDO_PROFILE_ID                     0x0000
#define ZDO_CLUSTER_BIND_REQ               0x0021
#define ZDO_CLUSTER_UNBIND_REQ             0x0022
#define ZDO_CLUSTER_BIND_RSP               0x8021
#define ZDO_CLUSTER_UNBIND_RSP             0x8022
#define ZDO_CLUSTER_MGMT_BIND_REQ          0x0033
#define ZDO_CLUSTER_MGMT_BIND_RSP          0x8033
#define ZDO_CLUSTER_ACTIVE_EP_REQ          0x0005
#define ZDO_CLUSTER_ACTIVE_EP_RSP          0x8005
#define ZDO_CLUSTER_SIMPLE_DESC_REQ        0x0004
#define ZDO_CLUSTER_SIMPLE_DESC_RSP        0x8004

/** IKEA "default bind group" used by many TRADFRI remotes (zigbee2mqtt). */
#define ZB_IKEA_DEFAULT_BIND_GROUP         901
/** Internal fake dimmable-light endpoint for Tradfri remote Finding & Binding.
 *  Not shown as a device. Use 11 (not 10) so addEndpoint succeeds on NCPs that
 *  still have a stale ep10 token from older firmware. */
#define ZB_FAKE_BULB_ENDPOINT              11

#define EZSP_FRAME_SET_MULTICAST_TABLE_ENTRY 0x0063
#define EZSP_FRAME_GET_MULTICAST_TABLE_ENTRY 0x0064
#define EZSP_FRAME_SEND_MULTICAST            0x0038
#define EZSP_FRAME_SET_BINDING               0x002B
#define EZSP_FRAME_SEND_BROADCAST            0x0036
#define EZSP_FRAME_SET_VALUE                 0x00AB
#define EZSP_FRAME_MAC_FILTER_MATCH_MESSAGE_HANDLER 0x0046
#define EZSP_FRAME_MAC_PASSTHROUGH_MESSAGE_HANDLER  0x0097

#define EZSP_VALUE_MAC_PASSTHROUGH_FLAGS     0x0D
#define EMBER_MAC_PASSTHROUGH_APPLICATION    0x08
#define EMBER_MAC_PASSTHROUGH_EMBERNET       0x02

#define EZSP_CONFIG_PACKET_BUFFER_COUNT           0x01
#define EZSP_CONFIG_APS_UNICAST_MESSAGE_COUNT     0x32
#define EZSP_CONFIG_BINDING_TABLE_SIZE            0x23

#define EMBER_MULTICAST_BINDING              0x03
#define EMBER_OUTGOING_MULTICAST             0x03
/** ZLL / Touchlink (only if NCP firmware includes ZLL plugin). */
#define EZSP_FRAME_ZLL_SET_RX_ON_WHEN_IDLE   0x00B5
#define EZSP_FRAME_ZLL_TOUCH_LINK_TARGET_HANDLER 0x00BB
#define EZSP_FRAME_IS_ZLL_NETWORK            0x00BE
#define EZSP_FRAME_SET_ZLL_ADDITIONAL_STATE  0x00D6
#define EZSP_FRAME_ZLL_OPERATION_IN_PROGRESS 0x00D7
#define EZSP_FRAME_GET_ZLL_PRIMARY_CHANNEL_MASK 0x00D9
#define EZSP_FRAME_SET_ZLL_PRIMARY_CHANNEL_MASK 0x00DB
#define EZSP_FRAME_SET_ZLL_SECONDARY_CHANNEL_MASK 0x00DC
/** Profile interoperability bit — required for ZB3 / non-IKEA Touchlink peers. */
#define EMBER_ZLL_STATE_PROFILE_INTEROP      0x0080
/** Standard ZLL primary channels 11,15,20,25. */
#define ZLL_PRIMARY_CHANNEL_MASK \
    ((1u << 11) | (1u << 15) | (1u << 20) | (1u << 25))
#define ZLL_SECONDARY_CHANNEL_MASK                                                                               \
    ((1u << 12) | (1u << 13) | (1u << 14) | (1u << 16) | (1u << 17) | (1u << 18) | (1u << 19) | (1u << 21) | \
     (1u << 22) | (1u << 23) | (1u << 24) | (1u << 26))

#define EMBER_OUTGOING_BROADCAST           0x02
#define EMBER_BROADCAST_ADDRESS            0xFFFF

#define ZDO_STATUS_SUCCESS                 0x00
#define ZDO_STATUS_NOT_SUPPORTED           0x84
#define ZDO_STATUS_TABLE_FULL              0x8C
#define ZDO_STATUS_NOT_PERMITTED           0x8B

#define EMBER_SUCCESS                      0x00
#define EMBER_DELIVERY_FAILED              0x66
#define EMBER_MAX_MESSAGE_LIMIT_REACHED    0x72
#define EMBER_NETWORK_UP                   0x90
#define EMBER_NETWORK_DOWN                 0x91

#define EMBER_TRUST_CENTER_GLOBAL_LINK_KEY 0x0004
#define EMBER_HAVE_PRECONFIGURED_KEY       0x0100
#define EMBER_HAVE_NETWORK_KEY             0x0200
#define EMBER_REQUIRE_ENCRYPTED_KEY        0x0800
#define EMBER_NO_FRAME_COUNTER_RESET       0x1000

#define EMBER_USE_MAC_ASSOCIATION          0x00

#define EMBER_COORDINATOR                  1
#define EMBER_ROUTER                       2
#define EMBER_END_DEVICE                   3
#define EMBER_SLEEPY_END_DEVICE            4

/* EzspConfigId (subset) */
#define EZSP_CONFIG_STACK_PROFILE                 0x0C
#define EZSP_CONFIG_SECURITY_LEVEL                0x0D
#define EZSP_CONFIG_MAX_END_DEVICE_CHILDREN       0x11
#define EZSP_CONFIG_INDIRECT_TRANSMIT_TIMEOUT     0x12 /**< ms — sleepy child pickup window */
#define EZSP_CONFIG_END_DEVICE_POLL_TIMEOUT       0x13 /**< 0=10s; N=2^N minutes child aging */
#define EZSP_CONFIG_MULTICAST_TABLE_SIZE          0x06
#define EZSP_CONFIG_TRUST_CENTER_ADDRESS_CACHE_SIZE 0x19

/* EzspPolicyId */
#define EZSP_TRUST_CENTER_POLICY           0x00
#define EZSP_TC_KEY_REQUEST_POLICY         0x05
#define EZSP_APP_KEY_REQUEST_POLICY        0x06
#define EZSP_MESSAGE_CONTENTS_IN_CALLBACK_POLICY 0x04
#define EZSP_ZLL_POLICY                    0x08
/** EmberZllPolicy bitmask (setPolicy decision for EZSP_ZLL_POLICY). */
#define EMBER_ZLL_POLICY_DISABLED          0x00
#define EMBER_ZLL_POLICY_ENABLED           0x01
#define EMBER_ZLL_POLICY_TARGET            0x02
#define EMBER_ZLL_POLICY_STEALING_ENABLED  0x04
#define EMBER_ZLL_POLICY_REMOTE_RESET_ENABLED 0x08

/* EzspDecisionBitmask for TRUST_CENTER_POLICY (EZSP v8) */
#define EZSP_DECISION_ALLOW_JOINS                 0x0001
#define EZSP_DECISION_ALLOW_UNSECURED_REJOINS     0x0002
#define EZSP_DECISION_ALLOW_PRECONFIGURED_KEY_JOINS 0x0001

/* EzspDecisionId for key-request / message-content policies */
#define EZSP_DECISION_ALLOW_TC_KEY_REQUESTS_AND_SEND_CURRENT_KEY 0x01
#define EZSP_DECISION_DENY_APP_KEY_REQUESTS                      0x00
#define EZSP_MESSAGE_TAG_AND_CONTENTS                            0x01

/* EmberOutgoingMessageType */
#define EMBER_OUTGOING_DIRECT                  0x00

/* EmberApsOption */
#define EMBER_APS_OPTION_RETRY                 0x0040
#define EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY 0x0100

#define ZCL_PROFILE_HA                         0x0104
#define ZCL_PROFILE_ZLL                        0xC05E /**< Zigbee Light Link / Touchlink profile */
#define ZCL_CLUSTER_BASIC                      0x0000
#define ZCL_CLUSTER_POWER_CONFIG               0x0001
#define ZCL_CLUSTER_IDENTIFY                   0x0003
#define ZCL_CLUSTER_GROUPS                     0x0004
#define ZCL_CLUSTER_SCENES                     0x0005
#define ZCL_CLUSTER_ON_OFF                     0x0006
#define ZCL_CLUSTER_LEVEL_CONTROL              0x0008
#define ZCL_CLUSTER_MULTISTATE_INPUT           0x0012
#define ZCL_CLUSTER_TEMP_MEASUREMENT           0x0402
#define ZCL_CLUSTER_REL_HUMIDITY               0x0405
#define ZCL_CLUSTER_IKEA_BUTTON                0xFC7F /**< IKEA manufacturer-specific */

#define ZCL_ATTR_MANUFACTURER_NAME             0x0004
#define ZCL_ATTR_MODEL_IDENTIFIER              0x0005
#define ZCL_ATTR_APPLICATION_VERSION           0x0001
#define ZCL_ATTR_SW_BUILD_ID                   0x4000
#define ZCL_ATTR_IDENTIFY_TIME                 0x0000
#define ZCL_ATTR_MEASURED_VALUE                0x0000
#define ZCL_ATTR_BATTERY_PCT_REMAINING         0x0021
#define ZCL_ATTR_ON_OFF                        0x0000
#define ZCL_ATTR_CURRENT_LEVEL                 0x0000
#define ZCL_ATTR_PRESENT_VALUE                 0x0055 /**< Multistate Input */

#define ZCL_CMD_OFF                            0x00
#define ZCL_CMD_ON                             0x01
#define ZCL_CMD_TOGGLE                         0x02
/** Identify cluster (0x0003) commands */
#define ZCL_CMD_IDENTIFY_CMD                   0x00
#define ZCL_CMD_IDENTIFY_QUERY                 0x01
#define ZCL_CMD_IDENTIFY_QUERY_RSP             0x00 /**< server→client */
#define ZCL_CMD_MOVE_TO_LEVEL                  0x00
#define ZCL_CMD_MOVE                           0x01
#define ZCL_CMD_STEP                           0x02
#define ZCL_CMD_STOP                           0x03
#define ZCL_CMD_MOVE_TO_LEVEL_WITH_ON_OFF      0x04
#define ZCL_CMD_MOVE_WITH_ON_OFF               0x05
#define ZCL_CMD_STEP_WITH_ON_OFF               0x06
#define ZCL_CMD_STOP_WITH_ON_OFF               0x07
#define ZCL_CMD_RECALL_SCENE                   0x05
#define ZCL_LEVEL_DIR_UP                       0x00
#define ZCL_LEVEL_DIR_DOWN                     0x01

#define ZCL_IKEA_MFG_CODE                      0x117C
#define ZCL_IKEA_CMD_ARROW_CLICK               0x07
#define ZCL_IKEA_CMD_ARROW_HOLD                0x08
#define ZCL_IKEA_CMD_ARROW_RELEASE             0x09
#define ZCL_IKEA_ARROW_RIGHT                   0x00
#define ZCL_IKEA_ARROW_LEFT                    0x01

#define EZSP_APS_FRAME_SIZE                    11
#define EZSP_ZCL_MSG_MAX                       64

/** HomeKit Programmable Switch Event values */
#define HK_BTN_EVENT_SINGLE                    0
#define HK_BTN_EVENT_DOUBLE                    1
#define HK_BTN_EVENT_LONG                      2

#define ZB_REMOTE_MAX_BUTTONS                  5

typedef enum {
    EZSP_NO_NETWORK = 0,
    EZSP_JOINING_NETWORK = 1,
    EZSP_JOINED_NETWORK = 2,
    EZSP_JOINED_NETWORK_NO_PARENT = 3,
} ezsp_network_status_t;

/** EmberNetworkParameters — 20 bytes on the wire (EZSP LE). */
typedef struct __attribute__((packed)) {
    uint8_t extended_pan_id[8];
    uint16_t pan_id;
    int8_t radio_tx_power;
    uint8_t radio_channel;
    uint8_t join_method;
    uint16_t nwk_manager_id;
    uint8_t nwk_update_id;
    uint32_t channels;
} ezsp_network_params_t;

typedef struct {
    uint8_t protocol_version;
    uint8_t stack_type;
    uint16_t stack_version;
    uint8_t eui64[8];
    bool eui64_valid;
    ezsp_network_status_t network_state;
    bool network_state_valid;
    uint8_t node_type;
    ezsp_network_params_t net;
    bool net_params_valid;
} ezsp_ncp_info_t;

typedef struct {
    uint32_t commands_sent;
    uint32_t responses_received;
    uint32_t timeouts;
    uint32_t unexpected;
} ezsp_stats_t;

typedef struct {
    bool valid;
    uint8_t eui64[8];
    uint16_t node_id;
    uint8_t node_type;
} ezsp_child_t;

/** Join/leave event captured from NCP callbacks (real devices only). */
typedef struct {
    bool joining; /**< false = left */
    uint8_t eui64[8];
    uint16_t node_id;
    uint8_t node_type;
    uint8_t tc_status;
} ezsp_join_event_t;

/** EmberApsFrame — 11 bytes on the wire (EZSP LE). */
typedef struct __attribute__((packed)) {
    uint16_t profile_id;
    uint16_t cluster_id;
    uint8_t source_endpoint;
    uint8_t destination_endpoint;
    uint16_t options;
    uint16_t group_id;
    uint8_t sequence;
} ezsp_aps_frame_t;

/** Queued ZCL / APS payload from incomingMessageHandler. */
typedef struct {
    uint16_t sender;
    uint16_t profile_id;
    uint16_t cluster_id;
    uint16_t group_id;
    uint8_t msg_type; /**< EmberIncomingMessageType */
    uint8_t source_endpoint;
    uint8_t destination_endpoint;
    int8_t last_hop_rssi;
    uint8_t last_hop_lqi;
    uint8_t len;
    uint8_t data[EZSP_ZCL_MSG_MAX];
} ezsp_zcl_message_t;

/** Portal sniffer ring-buffer entry (raw Zigbee RX/TX/events). */
#define EZSP_SNIFF_LOG  64
#define EZSP_SNIFF_DATA 48
typedef enum {
    EZSP_SNIFF_RX = 0,
    EZSP_SNIFF_TX = 1,
    EZSP_SNIFF_EVT = 2,
} ezsp_sniff_dir_t;

typedef struct {
    int64_t ms; /**< esp_timer microseconds / 1000 at capture */
    uint8_t dir; /**< ezsp_sniff_dir_t */
    uint8_t msg_type; /**< EmberIncomingMessageType / outgoing type; 0xFF for EVT */
    uint16_t node;
    uint16_t profile;
    uint16_t cluster;
    uint16_t group;
    uint8_t src_ep;
    uint8_t dst_ep;
    int8_t rssi;
    uint8_t status; /**< Ember status for TX/messageSent; 0 otherwise */
    uint8_t len;
    uint8_t data[EZSP_SNIFF_DATA];
} ezsp_sniff_entry_t;

/** Copy newest sniff entries (oldest-first in out). Returns count. */
uint8_t ezsp_copy_sniff_log(ezsp_sniff_entry_t *out, uint8_t max);
void ezsp_clear_sniff_log(void);
uint32_t ezsp_sniff_seq(void); /**< Monotonic counter of pushes (for UI dirty check). */

esp_err_t ezsp_init(void);
esp_err_t ezsp_negotiate_version(ezsp_ncp_info_t *info);
esp_err_t ezsp_get_eui64(uint8_t eui64_out[8]);
esp_err_t ezsp_get_network_state(ezsp_network_status_t *out);
esp_err_t ezsp_get_node_id(uint16_t *out);
esp_err_t ezsp_nop(void);

esp_err_t ezsp_network_init(uint8_t *ember_status_out);
esp_err_t ezsp_set_configuration_value(uint8_t config_id, uint16_t value, uint8_t *status_out);
esp_err_t ezsp_set_policy(uint8_t policy_id, uint16_t decision, uint8_t *status_out);
esp_err_t ezsp_configure_for_coordinator(void);
/**
 * Arm ZLL Touchlink light-target behaviour (optional, beside normal coordinator).
 * Sets ZLL channel masks + profile-interop bit, and leaves radio RX-on for duration_ms
 * so a Tradfri remote's 10s hold (Inter-PAN scan) can see our fake bulb.
 */
esp_err_t ezsp_zll_arm_touchlink_target(uint32_t duration_ms);
/** True after a successful ZLL probe at configure time. */
bool ezsp_zll_supported(void);
esp_err_t ezsp_set_initial_security_state(const uint8_t network_key[16], uint8_t *ember_status_out);
esp_err_t ezsp_form_network(const ezsp_network_params_t *params, uint8_t *ember_status_out);
esp_err_t ezsp_leave_network(uint8_t *ember_status_out);
esp_err_t ezsp_permit_joining(uint8_t duration_sec, uint8_t *ember_status_out);
esp_err_t ezsp_set_join_policy(bool allow_joins);
esp_err_t ezsp_get_network_parameters(uint8_t *node_type_out, ezsp_network_params_t *params_out);
esp_err_t ezsp_get_child_data(uint8_t index, ezsp_child_t *out);

/** True if a join-related callback arrived since last clear. */
bool ezsp_consume_join_event(void);
/** Pop one queued join/leave event; returns false if queue empty. */
bool ezsp_pop_join_event(ezsp_join_event_t *out);
/** Pop one queued ZCL/APS message; returns false if queue empty. */
bool ezsp_pop_zcl_message(ezsp_zcl_message_t *out);

/** Register HA endpoint 1 so the NCP delivers ZCL responses/reports to us. */
esp_err_t ezsp_add_ha_endpoint(uint8_t endpoint);
/** Register a dimmable-light endpoint (for IKEA Finding & Binding targets). */
esp_err_t ezsp_add_light_endpoint(uint8_t endpoint);
esp_err_t ezsp_send_unicast(uint16_t node_id, const ezsp_aps_frame_t *aps, const uint8_t *msg,
                            size_t msg_len, uint8_t *ember_status_out);
/** Build and send a ZCL Read Attributes command. */
esp_err_t ezsp_zcl_read_attributes(uint16_t node_id, uint8_t dest_ep, uint16_t cluster_id,
                                   const uint16_t *attr_ids, size_t attr_count);
/**
 * ZCL Configure Reporting (cmd 0x06) for one attribute.
 * change_le: little-endian reportable-change bytes (length depends on data_type).
 */
esp_err_t ezsp_zcl_configure_reporting(uint16_t node_id, uint8_t dest_ep, uint16_t cluster_id,
                                       uint16_t attr_id, uint8_t data_type, uint16_t min_interval_s,
                                       uint16_t max_interval_s, const uint8_t *change_le,
                                       size_t change_len);
/** Send an On/Off cluster command (ZCL_CMD_OFF / ON / TOGGLE). */
esp_err_t ezsp_zcl_on_off_command(uint16_t node_id, uint8_t dest_ep, uint8_t cmd);
/** ZCL Identify command (client→server) — lights the remote LED for time_s seconds. */
esp_err_t ezsp_zcl_identify(uint16_t node_id, uint8_t dest_ep, uint16_t time_s);
/**
 * Reply to Identify Query (Finding & Binding target).
 * zcl_seq must echo the query's Transaction Sequence Number.
 */
esp_err_t ezsp_zcl_identify_query_response(uint16_t node_id, uint8_t dest_ep, uint8_t src_ep,
                                           uint8_t zcl_seq, uint16_t timeout_ds);
/** Send any Identify Query Responses queued from the RX callback (call from host task). */
void ezsp_flush_identify_query_responses(void);
/** After IdentifyQueryRsp, pop a node that should get light-target F&B prep (multicast). */
bool ezsp_take_identify_bind_node(uint16_t *node_out);
/** Host registers a wake hook so Identify Query can break the poll sleep. */
void ezsp_set_host_wake(void (*fn)(void));
/** Start/refresh local Identify Time on our light endpoint (F&B target). */
void ezsp_light_start_identify(uint16_t seconds);
/** Remaining Identify Time in seconds (0 = not identifying). */
uint16_t ezsp_light_identify_time_s(void);
bool ezsp_light_is_identifying(void);
/** ZCL Groups: Add Group Response + join multicast so groupcast button cmds reach us. */
esp_err_t ezsp_zcl_groups_add_group_response(uint16_t node_id, uint8_t dest_ep, uint8_t src_ep,
                                             uint8_t zcl_seq, uint8_t status, uint16_t group_id);
/** ZCL Groups: Remove Group — detach a light/plug from a Touchlink bind group. */
esp_err_t ezsp_zcl_groups_remove_group(uint16_t node_id, uint8_t dest_ep, uint16_t group_id);
/** ZCL Groups: Add Group — put a light/plug into a remote's bind group (native control). */
esp_err_t ezsp_zcl_groups_add_group(uint16_t node_id, uint8_t dest_ep, uint16_t group_id);
/** ZCL Read Attributes Response helpers for our light-target server clusters. */
esp_err_t ezsp_zcl_read_attr_response_u16(uint16_t node_id, uint8_t dest_ep, uint8_t src_ep,
                                          uint8_t zcl_seq, uint16_t cluster_id, uint16_t attr_id,
                                          uint8_t data_type, uint16_t value);
esp_err_t ezsp_zcl_read_attr_response_u8(uint16_t node_id, uint8_t dest_ep, uint8_t src_ep,
                                         uint8_t zcl_seq, uint16_t cluster_id, uint16_t attr_id,
                                         uint8_t data_type, uint8_t value);
/** Move to Level (0–254). Uses Move to Level with On/Off when with_on_off is true. */
esp_err_t ezsp_zcl_move_to_level(uint16_t node_id, uint8_t dest_ep, uint8_t level, bool with_on_off);
/**
 * ZDO Bind_req: bind src_eui/src_ep/cluster to dst_eui/dst_ep (IEEE mode).
 * Sent to the remote's node_id endpoint 0 so button presses are forwarded to us.
 */
esp_err_t ezsp_zdo_bind(uint16_t node_id, const uint8_t src_eui[8], uint8_t src_ep,
                        uint16_t cluster_id, const uint8_t dst_eui[8], uint8_t dst_ep);
/** ZDO Bind/Unbind to a Zigbee group (DstAddrMode=group). Many IKEA remotes only support this. */
esp_err_t ezsp_zdo_bind_group(uint16_t node_id, const uint8_t src_eui[8], uint8_t src_ep,
                              uint16_t cluster_id, uint16_t group_id, bool unbind);
/** ZDO Mgmt_Bind_req — read remote's binding table (start_index usually 0). */
esp_err_t ezsp_zdo_mgmt_bind_req(uint16_t node_id, uint8_t start_index);
/** ZDO Active_EP_req / Simple_Desc_req for endpoint discovery. */
esp_err_t ezsp_zdo_active_ep_req(uint16_t node_id);
esp_err_t ezsp_zdo_simple_desc_req(uint16_t node_id, uint8_t endpoint);
/** Subscribe endpoint to a Zigbee multicast/group so groupcast cmds are delivered. */
esp_err_t ezsp_set_multicast_group(uint8_t table_index, uint16_t group_id, uint8_t endpoint);
/** Join groups IKEA remotes groupcast to. Call after NETWORK_UP (table is cleared on init). */
esp_err_t ezsp_join_ikea_multicast_groups(void);
/**
 * Ensure we receive groupcasts for group_id (Touchlink often uses a non-901 group).
 * Idempotent; uses multicast table slots 3+.
 */
esp_err_t ezsp_ensure_multicast_group(uint16_t group_id);
/** Remember which remote short-addr owns a Touchlink bind group (from MgmtBind). */
void ezsp_note_group_owner(uint16_t group_id, uint16_t node_id);
/** Hint which remote is generating button groupcasts (for messageSent synth). */
void ezsp_hint_button_remote(uint16_t node_id);
/** Pop a group id overheard on multicast TX (messageSent) — join via ensure_multicast_group. */
bool ezsp_take_learned_group(uint16_t *group_out);
/** True after stackStatus NETWORK_UP — host should re-join multicast groups. */
bool ezsp_take_network_up_event(void);
/** True when a sleepy child polled; optional node_id_out gets the child short address. */
bool ezsp_take_poll_event(uint16_t *node_id_out);
/**
 * Pop a unicast messageSent result (for sleepy sensor one-frame-at-a-time setup).
 * Returns false if the queue is empty.
 */
bool ezsp_take_sent_event(uint16_t *node_id_out, uint16_t *cluster_out, uint8_t *status_out);

esp_err_t ezsp_command(uint16_t frame_id, const uint8_t *params, size_t params_len,
                       uint8_t *resp_params, size_t resp_max, size_t *resp_len,
                       uint32_t timeout_ms);

void ezsp_get_stats(ezsp_stats_t *out);
void ezsp_format_stack_version(uint16_t stack_version, char *buf, size_t buflen);
void ezsp_format_eui64(const uint8_t eui64[8], char *buf, size_t buflen);
void ezsp_format_epid(const uint8_t epid[8], char *buf, size_t buflen);
const char *ezsp_ember_status_str(uint8_t status);
const char *ezsp_node_type_str(uint8_t t);

#ifdef __cplusplus
}
#endif
