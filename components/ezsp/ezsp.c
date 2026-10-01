/**
 * @file ezsp.c
 * @brief Modular EZSP host layer above ASH.
 *
 * Currently implements the commands needed for ICC bring-up:
 *   - version (0x0000)
 *   - getEui64 (0x0026)
 *   - networkState (0x0018)
 *   - nop (0x0005)
 *
 * Framing is EZSP v8 (2-byte frame control + 2-byte frame ID). Additional
 * commands for network formation / ZDO / ZCL can be added without changing ASH.
 */

#include "ezsp.h"
#include "ezsp_v8.h"
#include "ash.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "ezsp";

#define EZSP_MAX_FRAME      128
#define EZSP_EVT_RESPONSE   BIT0

typedef struct {
    uint8_t seq;
    uint16_t frame_id;
    uint8_t data[EZSP_MAX_FRAME];
    size_t len;
    bool waiting;
} ezsp_pending_t;

static uint8_t s_seq;
static ezsp_stats_t s_stats;
static ezsp_pending_t s_pending;
static SemaphoreHandle_t s_cmd_mutex;
static EventGroupHandle_t s_events;

#if CONFIG_EZSP_LOG_HEX_FRAMES
static void log_hex(const char *prefix, const uint8_t *data, size_t len)
{
    char line[3 * 48 + 8];
    size_t n = len > 48 ? 48 : len;
    size_t pos = 0;
    for (size_t i = 0; i < n && pos + 4 < sizeof(line); i++) {
        pos += (size_t)snprintf(line + pos, sizeof(line) - pos, "%02X ", data[i]);
    }
    if (len > 48 && pos + 4 < sizeof(line)) {
        snprintf(line + pos, sizeof(line) - pos, "...");
    }
    ESP_LOGI(TAG, "%s%s", prefix, line);
}
#else
static void log_hex(const char *prefix, const uint8_t *data, size_t len)
{
    (void)prefix;
    (void)data;
    (void)len;
}
#endif

static volatile bool s_join_event;
static volatile bool s_network_up_event;
static void (*s_host_wake)(void);
#define EZSP_POLL_Q_LEN 8
static uint16_t s_poll_q[EZSP_POLL_Q_LEN];
static uint8_t s_poll_q_head;
static uint8_t s_poll_q_tail;
static portMUX_TYPE s_poll_mux = portMUX_INITIALIZER_UNLOCKED;
typedef struct {
    uint16_t node;
    uint16_t cluster;
    uint8_t status;
} ezsp_sent_event_t;
#define EZSP_SENT_Q_LEN 8
static ezsp_sent_event_t s_sent_q[EZSP_SENT_Q_LEN];
static uint8_t s_sent_q_head;
static uint8_t s_sent_q_tail;
static portMUX_TYPE s_sent_mux = portMUX_INITIALIZER_UNLOCKED;
#define EZSP_JOIN_Q_LEN 8
static ezsp_join_event_t s_join_q[EZSP_JOIN_Q_LEN];
static uint8_t s_join_q_head;
static uint8_t s_join_q_tail;
static portMUX_TYPE s_join_mux = portMUX_INITIALIZER_UNLOCKED;

static void poll_q_push(uint16_t node)
{
    portENTER_CRITICAL(&s_poll_mux);
    uint8_t next = (uint8_t)((s_poll_q_head + 1) % EZSP_POLL_Q_LEN);
    if (next != s_poll_q_tail) {
        s_poll_q[s_poll_q_head] = node;
        s_poll_q_head = next;
    }
    portEXIT_CRITICAL(&s_poll_mux);
    if (s_host_wake) {
        s_host_wake();
    }
}

static void sent_q_push(uint16_t node, uint16_t cluster, uint8_t status)
{
    portENTER_CRITICAL(&s_sent_mux);
    uint8_t next = (uint8_t)((s_sent_q_head + 1) % EZSP_SENT_Q_LEN);
    if (next != s_sent_q_tail) {
        s_sent_q[s_sent_q_head].node = node;
        s_sent_q[s_sent_q_head].cluster = cluster;
        s_sent_q[s_sent_q_head].status = status;
        s_sent_q_head = next;
    }
    portEXIT_CRITICAL(&s_sent_mux);
    if (s_host_wake) {
        s_host_wake();
    }
}

#define EZSP_ZCL_Q_LEN 12
static ezsp_zcl_message_t s_zcl_q[EZSP_ZCL_Q_LEN];
static uint8_t s_zcl_q_head;
static uint8_t s_zcl_q_tail;
static portMUX_TYPE s_zcl_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_zcl_seq;

/** Deferred Identify Query Response — never sendUnicast from the EZSP RX callback. */
typedef struct {
    uint16_t node_id;
    uint8_t dest_ep;
    uint8_t src_ep;
    uint8_t zcl_seq;
    uint16_t timeout_ds;
} ezsp_id_query_rsp_t;
#define EZSP_IDQ_Q_LEN 4
static ezsp_id_query_rsp_t s_idq_q[EZSP_IDQ_Q_LEN];
static uint8_t s_idq_head;
static uint8_t s_idq_tail;
static portMUX_TYPE s_idq_mux = portMUX_INITIALIZER_UNLOCKED;

/** Nodes that just got IdentifyQueryRsp — host prepares light-target F&B (no ZDO Bind). */
#define EZSP_ID_BIND_Q_LEN 4
static uint16_t s_id_bind_q[EZSP_ID_BIND_Q_LEN];
static uint8_t s_id_bind_head;
static uint8_t s_id_bind_tail;

/** Group ids overheard while relaying remote→bulb groupcasts (join so app gets RX). */
#define EZSP_LEARN_GROUP_Q_LEN 4
static uint16_t s_learn_group_q[EZSP_LEARN_GROUP_Q_LEN];
static uint8_t s_learn_group_head;
static uint8_t s_learn_group_tail;
static portMUX_TYPE s_learn_group_mux = portMUX_INITIALIZER_UNLOCKED;
#define EZSP_LEARNED_GROUP_MAX 2
static uint16_t s_learned_groups[EZSP_LEARNED_GROUP_MAX];
static uint8_t s_learned_group_count;
/** Remote short addr that MgmtBind showed owning a Touchlink group. */
static uint16_t s_group_owner_gid[EZSP_LEARNED_GROUP_MAX];
static uint16_t s_group_owner_node[EZSP_LEARNED_GROUP_MAX];
static uint8_t s_group_owner_count;
/** Last remote we MgmtBind-probed — fallback owner for synth button RX. */
static uint16_t s_last_btn_remote_node;
/** Prefer real multicast RX over empty messageSent synth (avoids double Toggle). */
static int64_t s_last_real_btn_rx_ms;
static uint16_t s_last_real_btn_gid;
static uint16_t s_last_real_btn_cluster;
/** Any multicast RX on a button cluster/group (incl. light reports) — kill empty invent. */
static int64_t s_last_mcast_rx_ms;
static uint16_t s_last_mcast_rx_gid;
static uint16_t s_last_mcast_rx_cluster;
/** Last known Level/arrow direction from a real payload (empty synth fallback). */
static uint8_t s_last_level_dir = ZCL_LEVEL_DIR_UP;
static uint8_t s_last_arrow = ZCL_IKEA_ARROW_LEFT;

/** Local Identify Time for our light endpoint (F&B target). Seconds remaining. */
static volatile uint16_t s_identify_time_s;
static int64_t s_identify_deadline_ms;

static int64_t ezsp_now_ms(void)
{
    return (int64_t)(esp_timer_get_time() / 1000LL);
}

void ezsp_light_start_identify(uint16_t seconds)
{
    if (seconds == 0) {
        seconds = 180;
    }
    s_identify_time_s = seconds;
    s_identify_deadline_ms = ezsp_now_ms() + (int64_t)seconds * 1000;
    ESP_LOGI(TAG, "Light target identifying for %u s (F&B)", (unsigned)seconds);
}

uint16_t ezsp_light_identify_time_s(void)
{
    if (s_identify_time_s == 0) {
        return 0;
    }
    int64_t now = ezsp_now_ms();
    if (now >= s_identify_deadline_ms) {
        s_identify_time_s = 0;
        return 0;
    }
    uint16_t left = (uint16_t)((s_identify_deadline_ms - now + 999) / 1000);
    s_identify_time_s = left;
    return left;
}

bool ezsp_light_is_identifying(void)
{
    return ezsp_light_identify_time_s() > 0;
}

static void idq_push(uint16_t node_id, uint8_t dest_ep, uint8_t src_ep, uint8_t zcl_seq,
                     uint16_t timeout_ds)
{
    ezsp_id_query_rsp_t e = {.node_id = node_id,
                             .dest_ep = dest_ep,
                             .src_ep = src_ep,
                             .zcl_seq = zcl_seq,
                             .timeout_ds = timeout_ds};
    portENTER_CRITICAL(&s_idq_mux);
    uint8_t next = (uint8_t)((s_idq_head + 1) % EZSP_IDQ_Q_LEN);
    if (next != s_idq_tail) {
        s_idq_q[s_idq_head] = e;
        s_idq_head = next;
    }
    portEXIT_CRITICAL(&s_idq_mux);
    if (s_host_wake) {
        s_host_wake();
    }
}

static bool idq_pop(ezsp_id_query_rsp_t *out)
{
    bool ok = false;
    portENTER_CRITICAL(&s_idq_mux);
    if (s_idq_tail != s_idq_head) {
        *out = s_idq_q[s_idq_tail];
        s_idq_tail = (uint8_t)((s_idq_tail + 1) % EZSP_IDQ_Q_LEN);
        ok = true;
    }
    portEXIT_CRITICAL(&s_idq_mux);
    return ok;
}

static void id_bind_push(uint16_t node_id)
{
    portENTER_CRITICAL(&s_idq_mux);
    uint8_t next = (uint8_t)((s_id_bind_head + 1) % EZSP_ID_BIND_Q_LEN);
    if (next != s_id_bind_tail) {
        s_id_bind_q[s_id_bind_head] = node_id;
        s_id_bind_head = next;
    }
    portEXIT_CRITICAL(&s_idq_mux);
}

#define EZSP_SNIFF_Q_LEN EZSP_SNIFF_LOG
static ezsp_sniff_entry_t s_sniff[EZSP_SNIFF_Q_LEN];
static uint8_t s_sniff_head; /* next write */
static uint8_t s_sniff_count;
static uint32_t s_sniff_seq;
static portMUX_TYPE s_sniff_mux = portMUX_INITIALIZER_UNLOCKED;

static void sniff_push(ezsp_sniff_dir_t dir, uint8_t msg_type, uint16_t node, uint16_t profile,
                       uint16_t cluster, uint16_t group, uint8_t src_ep, uint8_t dst_ep, int8_t rssi,
                       uint8_t status, const uint8_t *data, uint8_t len)
{
    ezsp_sniff_entry_t e;
    memset(&e, 0, sizeof(e));
    e.ms = esp_timer_get_time() / 1000;
    e.dir = (uint8_t)dir;
    e.msg_type = msg_type;
    e.node = node;
    e.profile = profile;
    e.cluster = cluster;
    e.group = group;
    e.src_ep = src_ep;
    e.dst_ep = dst_ep;
    e.rssi = rssi;
    e.status = status;
    e.len = len > EZSP_SNIFF_DATA ? EZSP_SNIFF_DATA : len;
    if (data && e.len) {
        memcpy(e.data, data, e.len);
    }
    portENTER_CRITICAL(&s_sniff_mux);
    s_sniff[s_sniff_head] = e;
    s_sniff_head = (uint8_t)((s_sniff_head + 1) % EZSP_SNIFF_Q_LEN);
    if (s_sniff_count < EZSP_SNIFF_Q_LEN) {
        s_sniff_count++;
    }
    s_sniff_seq++;
    portEXIT_CRITICAL(&s_sniff_mux);
}

uint8_t ezsp_copy_sniff_log(ezsp_sniff_entry_t *out, uint8_t max)
{
    if (!out || !max) {
        return 0;
    }
    uint8_t n = 0;
    portENTER_CRITICAL(&s_sniff_mux);
    uint8_t count = s_sniff_count;
    if (count > max) {
        count = max;
    }
    uint8_t start = (uint8_t)((s_sniff_head + EZSP_SNIFF_Q_LEN - s_sniff_count) % EZSP_SNIFF_Q_LEN);
    /* If truncated, show the newest `count` entries. */
    if (s_sniff_count > count) {
        start = (uint8_t)((s_sniff_head + EZSP_SNIFF_Q_LEN - count) % EZSP_SNIFF_Q_LEN);
    }
    for (uint8_t i = 0; i < count; i++) {
        out[n++] = s_sniff[(start + i) % EZSP_SNIFF_Q_LEN];
    }
    portEXIT_CRITICAL(&s_sniff_mux);
    return n;
}

void ezsp_clear_sniff_log(void)
{
    portENTER_CRITICAL(&s_sniff_mux);
    s_sniff_head = 0;
    s_sniff_count = 0;
    s_sniff_seq++;
    portEXIT_CRITICAL(&s_sniff_mux);
}

uint32_t ezsp_sniff_seq(void)
{
    return s_sniff_seq;
}

static void join_q_push(const ezsp_join_event_t *ev)
{
    if (!ev) {
        return;
    }
    portENTER_CRITICAL(&s_join_mux);
    uint8_t next = (uint8_t)((s_join_q_head + 1) % EZSP_JOIN_Q_LEN);
    if (next != s_join_q_tail) {
        s_join_q[s_join_q_head] = *ev;
        s_join_q_head = next;
    }
    s_join_event = true;
    portEXIT_CRITICAL(&s_join_mux);
}

bool ezsp_pop_join_event(ezsp_join_event_t *out)
{
    if (!out) {
        return false;
    }
    bool ok = false;
    portENTER_CRITICAL(&s_join_mux);
    if (s_join_q_tail != s_join_q_head) {
        *out = s_join_q[s_join_q_tail];
        s_join_q_tail = (uint8_t)((s_join_q_tail + 1) % EZSP_JOIN_Q_LEN);
        ok = true;
    }
    portEXIT_CRITICAL(&s_join_mux);
    return ok;
}

static void zcl_q_push(const ezsp_zcl_message_t *msg)
{
    if (!msg || msg->len == 0) {
        return;
    }
    portENTER_CRITICAL(&s_zcl_mux);
    uint8_t next = (uint8_t)((s_zcl_q_head + 1) % EZSP_ZCL_Q_LEN);
    if (next != s_zcl_q_tail) {
        s_zcl_q[s_zcl_q_head] = *msg;
        s_zcl_q_head = next;
    }
    portEXIT_CRITICAL(&s_zcl_mux);
}

bool ezsp_pop_zcl_message(ezsp_zcl_message_t *out)
{
    if (!out) {
        return false;
    }
    bool ok = false;
    portENTER_CRITICAL(&s_zcl_mux);
    if (s_zcl_q_tail != s_zcl_q_head) {
        *out = s_zcl_q[s_zcl_q_tail];
        s_zcl_q_tail = (uint8_t)((s_zcl_q_tail + 1) % EZSP_ZCL_Q_LEN);
        ok = true;
    }
    portEXIT_CRITICAL(&s_zcl_mux);
    return ok;
}

static void handle_incoming_message(const uint8_t *params, size_t params_len)
{
    /* type(1) + EmberApsFrame(11) + lqi(1) + rssi(1) + sender(2) + binding(1) + address(1) + LVBytes */
    if (params_len < 1 + EZSP_APS_FRAME_SIZE + 6) {
        ESP_LOGI(TAG, "incomingMessageHandler len=%u (short)", (unsigned)params_len);
        return;
    }
    uint8_t msg_type = params[0];
    const uint8_t *p = params + 1;
    ezsp_zcl_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.profile_id = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    msg.cluster_id = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
    msg.source_endpoint = p[4];
    msg.destination_endpoint = p[5];
    uint16_t group_id = (uint16_t)p[8] | ((uint16_t)p[9] << 8); /* after options @6 */
    msg.group_id = group_id;
    msg.msg_type = msg_type;
    p += EZSP_APS_FRAME_SIZE;
    msg.last_hop_lqi = p[0];
    msg.last_hop_rssi = (int8_t)p[1];
    msg.sender = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
    p += 6; /* lqi, rssi, sender, binding, address */
    if ((size_t)(p - params) >= params_len) {
        return;
    }
    uint8_t mlen = *p++;
    size_t remain = params_len - (size_t)(p - params);
    if (mlen > remain) {
        mlen = (uint8_t)remain;
    }
    if (mlen > EZSP_ZCL_MSG_MAX) {
        mlen = EZSP_ZCL_MSG_MAX;
    }
    msg.len = mlen;
    memcpy(msg.data, p, mlen);
    const char *tt = msg_type == 0   ? "UNICAST"
                     : msg_type == 1 ? "UNICAST_REPLY"
                     : msg_type == 2 ? "MULTICAST"
                     : msg_type == 3 ? "MULTICAST_LOOP"
                     : msg_type == 4 ? "BROADCAST"
                     : msg_type == 5 ? "BROADCAST_LOOP"
                                     : "OTHER";
    ESP_LOGI(TAG, "incomingMessage %s node=0x%04X cluster=0x%04X ep=%u->%u group=%u len=%u rssi=%d",
             tt, msg.sender, msg.cluster_id, (unsigned)msg.source_endpoint,
             (unsigned)msg.destination_endpoint, (unsigned)group_id, (unsigned)msg.len,
             (int)msg.last_hop_rssi);
    if (msg.cluster_id == ZCL_CLUSTER_ON_OFF || msg.cluster_id == ZCL_CLUSTER_LEVEL_CONTROL ||
        msg.cluster_id == ZCL_CLUSTER_SCENES || msg.cluster_id == ZCL_CLUSTER_IKEA_BUTTON ||
        msg.cluster_id == ZCL_CLUSTER_IDENTIFY || msg.cluster_id == ZCL_CLUSTER_GROUPS ||
        msg.cluster_id == ZDO_CLUSTER_BIND_RSP || msg.profile_id == ZDO_PROFILE_ID) {
        char hex[96];
        size_t h = 0;
        for (uint8_t i = 0; i < msg.len && h + 3 < sizeof(hex); i++) {
            h += (size_t)snprintf(hex + h, sizeof(hex) - h, "%02X ", msg.data[i]);
        }
        ESP_LOGI(TAG, "  payload: %s", hex);
    }
    sniff_push(EZSP_SNIFF_RX, msg_type, msg.sender, msg.profile_id, msg.cluster_id, group_id,
               msg.source_endpoint, msg.destination_endpoint, msg.last_hop_rssi, 0, msg.data,
               msg.len);
    /* Any groupcast RX on button clusters — suppress empty messageSent synth (lights report
     * OnOff/Level on the same group; inventing Toggle from those relay confirms floods the
     * host and starves HTTP/HomeKit). */
    if (group_id != 0 &&
        (msg.cluster_id == ZCL_CLUSTER_ON_OFF || msg.cluster_id == ZCL_CLUSTER_LEVEL_CONTROL ||
         msg.cluster_id == ZCL_CLUSTER_SCENES || msg.cluster_id == ZCL_CLUSTER_IKEA_BUTTON)) {
        s_last_mcast_rx_ms = ezsp_now_ms();
        s_last_mcast_rx_gid = group_id;
        s_last_mcast_rx_cluster = msg.cluster_id;
    }
    /* Real button multicast — remember direction from client→server cmds. */
    if (msg.len >= 3 &&
        (msg.cluster_id == ZCL_CLUSTER_ON_OFF || msg.cluster_id == ZCL_CLUSTER_LEVEL_CONTROL ||
         msg.cluster_id == ZCL_CLUSTER_SCENES || msg.cluster_id == ZCL_CLUSTER_IKEA_BUTTON)) {
        uint8_t fc = msg.data[0];
        bool cluster_specific = (fc & 0x01) != 0;
        bool server_to_client = (fc & 0x08) != 0;
        if (cluster_specific && !server_to_client) {
            s_last_real_btn_rx_ms = ezsp_now_ms();
            s_last_real_btn_gid = group_id;
            s_last_real_btn_cluster = msg.cluster_id;
            size_t hdr = 1 + ((fc & 0x04) ? 2 : 0) + 1; /* fc [mfg] seq */
            if (msg.cluster_id == ZCL_CLUSTER_LEVEL_CONTROL && msg.len > hdr) {
                s_last_level_dir = msg.data[hdr];
            } else if ((msg.cluster_id == ZCL_CLUSTER_SCENES ||
                        msg.cluster_id == ZCL_CLUSTER_IKEA_BUTTON) &&
                       msg.len > hdr) {
                s_last_arrow =
                    msg.data[msg.len - 1] ? ZCL_IKEA_ARROW_LEFT : ZCL_IKEA_ARROW_RIGHT;
            }
        }
    }
    /* IKEA F&B: queue Identify Query Response from fake-bulb endpoint only. */
    if (msg.cluster_id == ZCL_CLUSTER_IDENTIFY && msg.len >= 3) {
        uint8_t fc = msg.data[0];
        bool manuf = (fc & 0x04) != 0;
        bool cluster_specific = (fc & 0x01) != 0;
        bool direction_server = (fc & 0x08) != 0;
        size_t seq_off = 1 + (manuf ? 2 : 0);
        size_t cmd_off = seq_off + 1;
        if (cluster_specific && !direction_server && cmd_off < msg.len &&
            msg.data[cmd_off] == ZCL_CMD_IDENTIFY_QUERY) {
            uint8_t zcl_seq = msg.data[seq_off];
            uint8_t dest_ep = msg.source_endpoint ? msg.source_endpoint : 1;
            if (!ezsp_light_is_identifying()) {
                s_identify_time_s = 180;
                s_identify_deadline_ms = ezsp_now_ms() + 180000;
            }
            idq_push(msg.sender, dest_ep, ZB_FAKE_BULB_ENDPOINT, zcl_seq,
                     ezsp_light_identify_time_s());
            ESP_LOGI(TAG, "Identify Query from 0x%04X seq=%u — defer Rsp ep%u", msg.sender,
                     (unsigned)zcl_seq, (unsigned)ZB_FAKE_BULB_ENDPOINT);
        }
    }
    zcl_q_push(&msg);
}

static void handle_unsolicited(uint16_t fid, const uint8_t *params, size_t params_len)
{
    if (fid == EZSP_FRAME_STACK_STATUS_HANDLER && params_len >= 1) {
        ESP_LOGI(TAG, "stackStatusHandler: 0x%02X (%s)", params[0],
                 params[0] == EMBER_NETWORK_UP     ? "NETWORK_UP"
                 : params[0] == EMBER_NETWORK_DOWN ? "NETWORK_DOWN"
                                                   : "other");
        if (params[0] == EMBER_NETWORK_UP) {
            s_join_event = true;
            s_network_up_event = true;
        }
        sniff_push(EZSP_SNIFF_EVT, 0xFF, 0, 0, EZSP_FRAME_STACK_STATUS_HANDLER, 0, 0, 0, 0,
                   params[0], params, 1);
    } else if (fid == EZSP_FRAME_CHILD_JOIN_HANDLER) {
        /* index(1) joining(1) childId(2) eui64(8) childType(1) */
        ezsp_join_event_t ev = {0};
        if (params_len >= 13) {
            ev.joining = params[1] != 0;
            ev.node_id = (uint16_t)params[2] | ((uint16_t)params[3] << 8);
            memcpy(ev.eui64, params + 4, 8);
            ev.node_type = params[12];
            char eui[32];
            ezsp_format_eui64(ev.eui64, eui, sizeof(eui));
            ESP_LOGI(TAG, "childJoinHandler: %s %s nodeId=0x%04X type=%u", eui,
                     ev.joining ? "joined" : "left", ev.node_id, ev.node_type);
            join_q_push(&ev);
        } else {
            ESP_LOGI(TAG, "childJoinHandler (len=%u)", (unsigned)params_len);
            s_join_event = true;
        }
    } else if (fid == EZSP_FRAME_TRUST_CENTER_JOIN_HANDLER) {
        /* newNodeId(2) eui64(8) status(1) decision(1) parent(2) */
        /* EmberDeviceUpdate: 0x01=UNSECURED_JOIN, 0x02=DEVICE_LEFT, … */
        ezsp_join_event_t ev = {0};
        if (params_len >= 12) {
            ev.node_id = (uint16_t)params[0] | ((uint16_t)params[1] << 8);
            memcpy(ev.eui64, params + 2, 8);
            ev.tc_status = params[10];
            ev.joining = (ev.tc_status != 0x02); /* DEVICE_LEFT */
            ev.node_type = EMBER_SLEEPY_END_DEVICE;
            char eui[32];
            ezsp_format_eui64(ev.eui64, eui, sizeof(eui));
            ESP_LOGI(TAG, "trustCenterJoinHandler: %s nodeId=0x%04X status=0x%02X decision=0x%02X",
                     eui, ev.node_id, params[10], params[11]);
            join_q_push(&ev);
        } else {
            ESP_LOGI(TAG, "trustCenterJoinHandler (len=%u)", (unsigned)params_len);
            s_join_event = true;
        }
    } else if (fid == EZSP_FRAME_INCOMING_MESSAGE_HANDLER) {
        handle_incoming_message(params, params_len);
    } else if (fid == EZSP_FRAME_POLL_HANDLER) {
        /* childId(2) — sleepy ED checked in; queue so host can send ONE pending frame. */
        if (params_len >= 2) {
            uint16_t child = (uint16_t)params[0] | ((uint16_t)params[1] << 8);
            poll_q_push(child);
            ESP_LOGI(TAG, "pollHandler child=0x%04X (awake)", child);
            /* Do not sniff polls — they flood the ring and bury fake-bulb pairing frames. */
        }
    } else if (fid == EZSP_FRAME_MESSAGE_SENT_HANDLER) {
        /* type(1) indexOrDest(2) apsFrame(11) tag(1) status(1) message... */
        if (params_len >= 1 + 2 + EZSP_APS_FRAME_SIZE + 2) {
            uint8_t st = params[1 + 2 + EZSP_APS_FRAME_SIZE + 1];
            uint16_t dest = (uint16_t)params[1] | ((uint16_t)params[2] << 8);
            const uint8_t *aps = params + 1 + 2;
            uint16_t profile = (uint16_t)aps[0] | ((uint16_t)aps[1] << 8);
            uint16_t cluster = (uint16_t)aps[2] | ((uint16_t)aps[3] << 8);
            uint16_t group = (uint16_t)aps[8] | ((uint16_t)aps[9] << 8);
            if (st != EMBER_SUCCESS || cluster == ZDO_CLUSTER_BIND_REQ) {
                ESP_LOGI(TAG, "messageSent dest=0x%04X cluster=0x%04X -> %s", dest, cluster,
                         ezsp_ember_status_str(st));
            }
            size_t off = 1 + 2 + EZSP_APS_FRAME_SIZE + 2;
            uint8_t plen = 0;
            const uint8_t *pdata = NULL;
            if (params_len > off) {
                plen = params[off];
                if ((size_t)off + 1 + plen > params_len) {
                    plen = (uint8_t)(params_len - off - 1);
                }
                pdata = params + off + 1;
            }
            /* Sensor / bind delivery results drive the one-frame sleepy setup state machine. */
            bool sensorish = cluster == ZCL_CLUSTER_TEMP_MEASUREMENT ||
                             cluster == ZCL_CLUSTER_REL_HUMIDITY ||
                             cluster == ZCL_CLUSTER_POWER_CONFIG || cluster == ZCL_CLUSTER_BASIC ||
                             cluster == ZDO_CLUSTER_BIND_REQ || cluster == ZDO_CLUSTER_BIND_RSP;
            if (sensorish) {
                sent_q_push(dest, cluster, st);
            }
            /* Only sniff delivery confirms for fake-bulb / button clusters (and failures).
             * Sensor Basic/Temp messageSent would otherwise bury Identify/OnOff. */
            bool interesting = cluster == ZCL_CLUSTER_IDENTIFY || cluster == ZCL_CLUSTER_GROUPS ||
                               cluster == ZCL_CLUSTER_ON_OFF || cluster == ZCL_CLUSTER_LEVEL_CONTROL ||
                               cluster == ZCL_CLUSTER_SCENES || cluster == ZCL_CLUSTER_IKEA_BUTTON ||
                               cluster == ZDO_CLUSTER_BIND_REQ || cluster == ZDO_CLUSTER_BIND_RSP ||
                               st != EMBER_SUCCESS;
            if (interesting) {
                sniff_push(EZSP_SNIFF_EVT, params[0], dest, profile, cluster, group, aps[4], aps[5],
                           0, st, pdata, plen);
            }
            /* Relayed remote→bulb groupcasts show up here (dest 0xFFFD) even when we are
             * not in the multicast table — learn the group so the next press is delivered. */
            uint16_t learn_gid = group;
            if (learn_gid == 0 && dest != 0 && dest != 0xFFFF && dest != 0xFFFD && dest >= 0x0001) {
                /* Some stacks put the group id in indexOrDest for OUTGOING_MULTICAST. */
                if (params[0] == 2 /* EMBER_OUTGOING_MULTICAST */ ||
                    params[0] == 3 /* EMBER_OUTGOING_MULTICAST_WITH_ALIAS */) {
                    learn_gid = dest;
                }
            }
            if (learn_gid != 0 &&
                (cluster == ZCL_CLUSTER_ON_OFF || cluster == ZCL_CLUSTER_LEVEL_CONTROL ||
                 cluster == ZCL_CLUSTER_SCENES || cluster == ZCL_CLUSTER_IKEA_BUTTON)) {
                /* Skip learn-queue spam once the group is already in the multicast table. */
                bool already_learned = false;
                for (uint8_t li = 0; li < s_learned_group_count; li++) {
                    if (s_learned_groups[li] == learn_gid) {
                        already_learned = true;
                        break;
                    }
                }
                if (!already_learned) {
                    portENTER_CRITICAL(&s_learn_group_mux);
                    uint8_t next = (uint8_t)((s_learn_group_head + 1) % EZSP_LEARN_GROUP_Q_LEN);
                    if (next != s_learn_group_tail) {
                        s_learn_group_q[s_learn_group_head] = learn_gid;
                        s_learn_group_head = next;
                    }
                    portEXIT_CRITICAL(&s_learn_group_mux);
                }

                /* If the NCP retransmits groupcasts without delivering them to the app
                 * (not in multicast table yet, or no loopback), synthesize an RX from the
                 * payload (or a Toggle when payload is empty for OnOff). */
                uint16_t owner = 0;
                for (uint8_t gi = 0; gi < s_group_owner_count; gi++) {
                    if (s_group_owner_gid[gi] == learn_gid) {
                        owner = s_group_owner_node[gi];
                        break;
                    }
                }
                if (owner == 0) {
                    owner = s_last_btn_remote_node;
                }
                if (owner == 0 && s_group_owner_count > 0) {
                    /* Fall back to any known remote that owns a Touchlink group. */
                    owner = s_group_owner_node[s_group_owner_count - 1];
                }
                if (owner != 0) {
                    ezsp_zcl_message_t syn = {0};
                    syn.msg_type = 2; /* MULTICAST */
                    syn.profile_id = profile ? profile : ZCL_PROFILE_HA;
                    syn.cluster_id = cluster;
                    syn.group_id = learn_gid;
                    syn.source_endpoint = aps[4] ? aps[4] : 1;
                    syn.destination_endpoint = 0xFF;
                    syn.sender = owner;
                    if (pdata && plen > 0 && plen != 0xFF) {
                        syn.len = plen > EZSP_ZCL_MSG_MAX ? EZSP_ZCL_MSG_MAX : plen;
                        memcpy(syn.data, pdata, syn.len);
                        if (cluster == ZCL_CLUSTER_LEVEL_CONTROL && syn.len >= 4) {
                            s_last_level_dir = syn.data[3];
                        } else if ((cluster == ZCL_CLUSTER_SCENES ||
                                    cluster == ZCL_CLUSTER_IKEA_BUTTON) &&
                                   syn.len >= 4) {
                            s_last_arrow =
                                syn.data[syn.len - 1] ? ZCL_IKEA_ARROW_LEFT : ZCL_IKEA_ARROW_RIGHT;
                        }
                    } else if (cluster == ZCL_CLUSTER_ON_OFF) {
                        /* NCP omits APS payload on multicast relay confirms (len=0xFF).
                         * OnOff has no direction — Toggle is unambiguous. Level/Scenes need a
                         * real direction byte; inventing/flipping it maps bright↔dim and
                         * left↔right at random, so those wait for incomingMessage. */
                        syn.data[0] = 0x01; /* cluster-specific, client→server */
                        syn.data[1] = 0;    /* seq */
                        syn.data[2] = ZCL_CMD_TOGGLE;
                        syn.len = 3;
                    } else {
                        /* Level / Scenes / IKEA: no payload ⇒ cannot know direction. */
                        syn.len = 0;
                    }
                    static int64_t s_last_synth_ms;
                    static uint16_t s_last_synth_cluster;
                    static uint16_t s_last_synth_gid;
                    int64_t now = ezsp_now_ms();
                    /* OnOff: wider window — one press → messageSent + delayed real RX. */
                    int64_t dup_ms = (cluster == ZCL_CLUSTER_ON_OFF) ? 2500 : 400;
                    bool dup = (cluster == s_last_synth_cluster) && (learn_gid == s_last_synth_gid) &&
                               (now - s_last_synth_ms) < dup_ms;
                    /* Real button cmd OR any group multicast (light report) — skip invent.
                     * OnOff power presses often deliver real RX 0.5–2s after messageSent. */
                    int64_t skip_ms = (cluster == ZCL_CLUSTER_ON_OFF) ? 2500 : 600;
                    bool real_recent = (learn_gid == s_last_real_btn_gid) &&
                                       (cluster == s_last_real_btn_cluster) &&
                                       (now - s_last_real_btn_rx_ms) < skip_ms;
                    bool mcast_recent = (learn_gid == s_last_mcast_rx_gid) &&
                                        (cluster == s_last_mcast_rx_cluster) &&
                                        (now - s_last_mcast_rx_ms) < skip_ms;
                    if (syn.len > 0 && !dup && !real_recent && !mcast_recent) {
                        s_last_synth_ms = now;
                        s_last_synth_cluster = cluster;
                        s_last_synth_gid = learn_gid;
                        ESP_LOGI(TAG,
                                 "Synth button RX from relay group=%u cluster=0x%04X node=0x%04X "
                                 "len=%u",
                                 (unsigned)learn_gid, cluster, owner, (unsigned)syn.len);
                        zcl_q_push(&syn);
                        sniff_push(EZSP_SNIFF_RX, 2, owner, syn.profile_id, cluster, learn_gid,
                                   syn.source_endpoint, syn.destination_endpoint, 0, 0, syn.data,
                                   syn.len);
                    }
                } else {
                    ESP_LOGW(TAG,
                             "Button groupcast grp=%u cluster=0x%04X — no remote owner yet "
                             "(open remote details or wait for MgmtBind)",
                             (unsigned)learn_gid, cluster);
                }
                if (s_host_wake) {
                    s_host_wake();
                }
            }
        }
    } else if (fid == EZSP_FRAME_MAC_FILTER_MATCH_MESSAGE_HANDLER ||
               fid == EZSP_FRAME_MAC_PASSTHROUGH_MESSAGE_HANDLER) {
        /* Raw MAC / passthrough — try to recover ZCL direction for groupcast buttons. */
        ESP_LOGI(TAG, "MAC passthrough/filter fid=0x%04X len=%u", fid, (unsigned)params_len);
        char hx[120];
        size_t h = 0;
        size_t n = params_len < 48 ? params_len : 48;
        for (size_t i = 0; i < n && h + 3 < sizeof(hx); i++) {
            h += (size_t)snprintf(hx + h, sizeof(hx) - h, "%02X", params[i]);
        }
        ESP_LOGI(TAG, "  mac: %s", hx);
        /* Best-effort: search for HA profile + Level/Scenes cluster + ZCL payload. */
        for (size_t i = 0; i + 8 < params_len; i++) {
            uint16_t prof = (uint16_t)params[i] | ((uint16_t)params[i + 1] << 8);
            uint16_t clus = (uint16_t)params[i + 2] | ((uint16_t)params[i + 3] << 8);
            if (prof != ZCL_PROFILE_HA) {
                continue;
            }
            if (clus != ZCL_CLUSTER_LEVEL_CONTROL && clus != ZCL_CLUSTER_SCENES &&
                clus != ZCL_CLUSTER_ON_OFF && clus != ZCL_CLUSTER_IKEA_BUTTON) {
                continue;
            }
            /* APS-like: profile(2) cluster(2) sep(1) dep(1) opt(2) gid(2) — then ZCL */
            size_t zoff = i + 10;
            if (zoff >= params_len) {
                continue;
            }
            ezsp_zcl_message_t syn = {0};
            syn.msg_type = 2;
            syn.profile_id = prof;
            syn.cluster_id = clus;
            if (i + 9 < params_len) {
                syn.group_id = (uint16_t)params[i + 8] | ((uint16_t)params[i + 9] << 8);
            }
            syn.source_endpoint = (i + 4 < params_len) ? params[i + 4] : 1;
            syn.destination_endpoint = 0xFF;
            syn.sender = s_last_btn_remote_node;
            syn.len = (uint8_t)((params_len - zoff) > EZSP_ZCL_MSG_MAX ? EZSP_ZCL_MSG_MAX
                                                                       : (params_len - zoff));
            memcpy(syn.data, params + zoff, syn.len);
            ESP_LOGI(TAG, "MAC→ZCL cluster=0x%04X grp=%u len=%u", clus, (unsigned)syn.group_id,
                     (unsigned)syn.len);
            if (syn.sender) {
                zcl_q_push(&syn);
                if (s_host_wake) {
                    s_host_wake();
                }
            }
            break;
        }
    } else if (fid == EZSP_FRAME_ZLL_TOUCH_LINK_TARGET_HANDLER) {
        /* Do not send EZSP commands from this callback (ASH/cmd mutex). Host task arms RX. */
        ESP_LOGI(TAG, "ZLL Touchlink TARGET hit (len=%u) — remote pairing like to a bulb",
                 (unsigned)params_len);
        sniff_push(EZSP_SNIFF_EVT, 0xFE, 0, 0xC05E, 0x1000, 0, 0, ZB_FAKE_BULB_ENDPOINT, 0, 0,
                   params, params_len > 48 ? 48 : params_len);
        if (s_host_wake) {
            s_host_wake(); /* host loop arms fake bulb / drains ZCL */
        }
    } else {
        s_stats.unexpected++;
        ESP_LOGI(TAG, "Unsolicited EZSP frame id=0x%04X len=%u", fid, (unsigned)params_len);
        if (params_len > 0 && params_len < 48) {
            char hx[100];
            size_t h = 0;
            for (size_t i = 0; i < params_len && h + 3 < sizeof(hx); i++) {
                h += (size_t)snprintf(hx + h, sizeof(hx) - h, "%02X", params[i]);
            }
            ESP_LOGI(TAG, "  raw: %s", hx);
        }
    }
}

static void ezsp_on_ash_data(const uint8_t *ezsp, size_t len, void *user)
{
    (void)user;
    uint8_t seq = 0;
    uint16_t fid = 0;
    const uint8_t *params = NULL;
    size_t params_len = 0;

    if (ezsp_v8_parse(ezsp, len, &seq, &fid, &params, &params_len) != ESP_OK) {
        ESP_LOGW(TAG, "Drop malformed EZSP frame len=%u", (unsigned)len);
        return;
    }

    log_hex("RX EZSP: ", ezsp, len);
    s_stats.responses_received++;

    if (s_pending.waiting && seq == s_pending.seq &&
        (fid == s_pending.frame_id || fid == EZSP_FRAME_INVALID_CMD)) {
        size_t copy = len < sizeof(s_pending.data) ? len : sizeof(s_pending.data);
        memcpy(s_pending.data, ezsp, copy);
        s_pending.len = copy;
        s_pending.waiting = false;
        xEventGroupSetBits(s_events, EZSP_EVT_RESPONSE);
        return;
    }

    handle_unsolicited(fid, params, params_len);
}

bool ezsp_consume_join_event(void)
{
    bool v = s_join_event;
    s_join_event = false;
    return v;
}

esp_err_t ezsp_init(void)
{
    if (!s_cmd_mutex) {
        s_cmd_mutex = xSemaphoreCreateMutex();
    }
    if (!s_events) {
        s_events = xEventGroupCreate();
    }
    if (!s_cmd_mutex || !s_events) {
        return ESP_ERR_NO_MEM;
    }

    memset(&s_stats, 0, sizeof(s_stats));
    s_seq = 0;
    return ash_init(ezsp_on_ash_data, NULL);
}

esp_err_t ezsp_command(uint16_t frame_id, const uint8_t *params, size_t params_len,
                       uint8_t *resp_params, size_t resp_max, size_t *resp_len,
                       uint32_t timeout_ms)
{
    if (!s_cmd_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t frame[EZSP_MAX_FRAME];
    size_t frame_len = 0;

    xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);

    uint8_t seq = s_seq++;
    esp_err_t err =
        ezsp_v8_build(seq, frame_id, params, params_len, frame, sizeof(frame), &frame_len);
    if (err != ESP_OK) {
        xSemaphoreGive(s_cmd_mutex);
        return err;
    }

    s_pending.seq = seq;
    s_pending.frame_id = frame_id;
    s_pending.len = 0;
    s_pending.waiting = true;
    xEventGroupClearBits(s_events, EZSP_EVT_RESPONSE);

    log_hex("TX EZSP: ", frame, frame_len);
    s_stats.commands_sent++;

    err = ash_send_data(frame, frame_len,
                        timeout_ms ? timeout_ms : CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        s_pending.waiting = false;
        xSemaphoreGive(s_cmd_mutex);
        ESP_LOGE(TAG, "ASH send failed for frame 0x%04X: %s", frame_id, esp_err_to_name(err));
        return err;
    }

    EventBits_t bits = xEventGroupWaitBits(s_events, EZSP_EVT_RESPONSE, pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms ? timeout_ms
                                                                    : CONFIG_EZSP_COMMAND_TIMEOUT_MS));
    if (!(bits & EZSP_EVT_RESPONSE)) {
        s_pending.waiting = false;
        s_stats.timeouts++;
        xSemaphoreGive(s_cmd_mutex);
        ESP_LOGW(TAG, "EZSP timeout waiting for 0x%04X", frame_id);
        return ESP_ERR_TIMEOUT;
    }

    uint8_t rseq = 0;
    uint16_t rfid = 0;
    const uint8_t *rparams = NULL;
    size_t rparams_len = 0;
    err = ezsp_v8_parse(s_pending.data, s_pending.len, &rseq, &rfid, &rparams, &rparams_len);
    if (err != ESP_OK) {
        xSemaphoreGive(s_cmd_mutex);
        return err;
    }

    if (rfid == EZSP_FRAME_INVALID_CMD) {
        xSemaphoreGive(s_cmd_mutex);
        ESP_LOGE(TAG, "NCP rejected command 0x%04X (invalidCommand)", frame_id);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (resp_len) {
        *resp_len = rparams_len;
    }
    if (resp_params && rparams && rparams_len) {
        size_t copy = rparams_len < resp_max ? rparams_len : resp_max;
        memcpy(resp_params, rparams, copy);
        if (resp_len) {
            *resp_len = copy;
        }
    }

    xSemaphoreGive(s_cmd_mutex);
    return ESP_OK;
}

esp_err_t ezsp_negotiate_version(ezsp_ncp_info_t *info)
{
    uint8_t desired = EZSP_PROTOCOL_VERSION_V8;
    uint8_t resp[8];
    size_t resp_len = 0;

    ESP_LOGI(TAG, "EZSP: negotiating protocol version %u", (unsigned)desired);
    esp_err_t err = ezsp_command(EZSP_FRAME_VERSION, &desired, 1, resp, sizeof(resp), &resp_len,
                                 CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 4) {
        ESP_LOGE(TAG, "version response too short (%u)", (unsigned)resp_len);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t proto = resp[0];
    uint8_t stack_type = resp[1];
    uint16_t stack_ver = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);

    if (info) {
        info->protocol_version = proto;
        info->stack_type = stack_type;
        info->stack_version = stack_ver;
    }

    char verbuf[32];
    ezsp_format_stack_version(stack_ver, verbuf, sizeof(verbuf));
    ESP_LOGI(TAG, "EZSP: protocol version %u", (unsigned)proto);
    ESP_LOGI(TAG, "EmberZNet: %s (stackType=%u raw=0x%04X)", verbuf, (unsigned)stack_type,
             stack_ver);

    if (proto != EZSP_PROTOCOL_VERSION_V8) {
        ESP_LOGW(TAG, "NCP reported EZSP %u (host expects %u) — continuing carefully",
                 (unsigned)proto, EZSP_PROTOCOL_VERSION_V8);
    }
    return ESP_OK;
}

esp_err_t ezsp_get_eui64(uint8_t eui64_out[8])
{
    uint8_t resp[8];
    size_t resp_len = 0;
    esp_err_t err =
        ezsp_command(EZSP_FRAME_GET_EUI64, NULL, 0, resp, sizeof(resp), &resp_len,
                     CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 8) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* Wire format is little-endian 64-bit; store as received for formatting. */
    memcpy(eui64_out, resp, 8);
    return ESP_OK;
}

esp_err_t ezsp_get_network_state(ezsp_network_status_t *out)
{
    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err =
        ezsp_command(EZSP_FRAME_NETWORK_STATE, NULL, 0, resp, sizeof(resp), &resp_len,
                     CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1 || !out) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    *out = (ezsp_network_status_t)resp[0];
    return ESP_OK;
}

esp_err_t ezsp_nop(void)
{
    return ezsp_command(EZSP_FRAME_NOP, NULL, 0, NULL, 0, NULL, CONFIG_EZSP_COMMAND_TIMEOUT_MS);
}

void ezsp_get_stats(ezsp_stats_t *out)
{
    if (out) {
        *out = s_stats;
    }
}

void ezsp_format_stack_version(uint16_t stack_version, char *buf, size_t buflen)
{
    if (!buf || buflen == 0) {
        return;
    }
    unsigned major = (stack_version >> 12) & 0x0F;
    unsigned minor = (stack_version >> 8) & 0x0F;
    unsigned patch = (stack_version >> 4) & 0x0F;
    unsigned special = stack_version & 0x0F;
    snprintf(buf, buflen, "%u.%u.%u.%u", major, minor, patch, special);
}

void ezsp_format_eui64(const uint8_t eui64[8], char *buf, size_t buflen)
{
    if (!buf || buflen == 0 || !eui64) {
        return;
    }
    snprintf(buf, buflen, "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", eui64[7], eui64[6], eui64[5],
             eui64[4], eui64[3], eui64[2], eui64[1], eui64[0]);
}

void ezsp_format_epid(const uint8_t epid[8], char *buf, size_t buflen)
{
    ezsp_format_eui64(epid, buf, buflen);
}

const char *ezsp_ember_status_str(uint8_t status)
{
    switch (status) {
    case EMBER_SUCCESS:
        return "SUCCESS";
    case 0x18:
        return "NOT_JOINED";
    case 0x66:
        return "DELIVERY_FAILED";
    case 0x70:
        return "NETWORK_BUSY";
    case 0x72:
        return "MAX_MESSAGE_LIMIT";
    case 0xC5:
        return "INVALID_CALL";
    default:
        return "ERROR";
    }
}

const char *ezsp_node_type_str(uint8_t t)
{
    switch (t) {
    case EMBER_COORDINATOR:
        return "coordinator";
    case EMBER_ROUTER:
        return "router";
    case EMBER_END_DEVICE:
        return "end_device";
    case EMBER_SLEEPY_END_DEVICE:
        return "sleepy_end_device";
    default:
        return "unknown";
    }
}

esp_err_t ezsp_get_node_id(uint16_t *out)
{
    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_GET_NODE_ID, NULL, 0, resp, sizeof(resp), &resp_len,
                                 CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK || resp_len < 2 || !out) {
        return err != ESP_OK ? err : ESP_ERR_INVALID_RESPONSE;
    }
    *out = (uint16_t)resp[0] | ((uint16_t)resp[1] << 8);
    return ESP_OK;
}

esp_err_t ezsp_network_init(uint8_t *ember_status_out)
{
    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_NETWORK_INIT, NULL, 0, resp, sizeof(resp), &resp_len,
                                 CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (ember_status_out) {
        *ember_status_out = resp[0];
    }
    ESP_LOGI(TAG, "networkInit -> %s (0x%02X)", ezsp_ember_status_str(resp[0]), resp[0]);
    return (resp[0] == EMBER_SUCCESS) ? ESP_OK : ESP_FAIL;
}

esp_err_t ezsp_set_configuration_value(uint8_t config_id, uint16_t value, uint8_t *status_out)
{
    uint8_t params[3] = {config_id, (uint8_t)(value & 0xFF), (uint8_t)(value >> 8)};
    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_SET_CONFIG_VALUE, params, sizeof(params), resp,
                                 sizeof(resp), &resp_len, CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (status_out) {
        *status_out = resp[0];
    }
    if (resp[0] != EMBER_SUCCESS) {
        ESP_LOGW(TAG, "setConfigurationValue(0x%02X=%u) -> 0x%02X", config_id, (unsigned)value,
                 resp[0]);
    }
    return (resp[0] == EMBER_SUCCESS) ? ESP_OK : ESP_FAIL;
}

esp_err_t ezsp_set_policy(uint8_t policy_id, uint16_t decision, uint8_t *status_out)
{
    uint8_t params[3] = {policy_id, (uint8_t)(decision & 0xFF), (uint8_t)(decision >> 8)};
    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_SET_POLICY, params, sizeof(params), resp, sizeof(resp),
                                 &resp_len, CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (status_out) {
        *status_out = resp[0];
    }
    ESP_LOGI(TAG, "setPolicy(0x%02X, 0x%04X) -> 0x%02X", policy_id, decision, resp[0]);
    return (resp[0] == EMBER_SUCCESS) ? ESP_OK : ESP_FAIL;
}

static bool s_zll_supported;

bool ezsp_zll_supported(void)
{
    return s_zll_supported;
}

esp_err_t ezsp_zll_arm_touchlink_target(uint32_t duration_ms)
{
    if (!s_zll_supported) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    /* Critical: default ZLL policy is DISABLED — without TARGET|ENABLED the NCP
     * ignores Inter-PAN Scan Requests (IKEA 10s hold never reaches us). */
    uint8_t pol_st = 0xFF;
    uint16_t zll_pol = (uint16_t)(EMBER_ZLL_POLICY_ENABLED | EMBER_ZLL_POLICY_TARGET |
                                  EMBER_ZLL_POLICY_STEALING_ENABLED);
    (void)ezsp_set_policy(EZSP_ZLL_POLICY, zll_pol, &pol_st);
    ESP_LOGI(TAG, "ZLL policy ENABLED|TARGET|STEALING -> 0x%02X", pol_st);

    uint8_t mask[4];
    uint32_t primary = ZLL_PRIMARY_CHANNEL_MASK;
    uint32_t secondary = ZLL_SECONDARY_CHANNEL_MASK;
    mask[0] = (uint8_t)(primary & 0xFF);
    mask[1] = (uint8_t)((primary >> 8) & 0xFF);
    mask[2] = (uint8_t)((primary >> 16) & 0xFF);
    mask[3] = (uint8_t)((primary >> 24) & 0xFF);
    (void)ezsp_command(EZSP_FRAME_SET_ZLL_PRIMARY_CHANNEL_MASK, mask, 4, NULL, 0, NULL,
                       CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    mask[0] = (uint8_t)(secondary & 0xFF);
    mask[1] = (uint8_t)((secondary >> 8) & 0xFF);
    mask[2] = (uint8_t)((secondary >> 16) & 0xFF);
    mask[3] = (uint8_t)((secondary >> 24) & 0xFF);
    (void)ezsp_command(EZSP_FRAME_SET_ZLL_SECONDARY_CHANNEL_MASK, mask, 4, NULL, 0, NULL,
                       CONFIG_EZSP_COMMAND_TIMEOUT_MS);

    uint8_t state[2] = {(uint8_t)(EMBER_ZLL_STATE_PROFILE_INTEROP & 0xFF),
                        (uint8_t)(EMBER_ZLL_STATE_PROFILE_INTEROP >> 8)};
    (void)ezsp_command(EZSP_FRAME_SET_ZLL_ADDITIONAL_STATE, state, 2, NULL, 0, NULL,
                       CONFIG_EZSP_COMMAND_TIMEOUT_MS);

    if (duration_ms == 0) {
        duration_ms = 180000;
    }
    uint8_t dur[4] = {(uint8_t)(duration_ms & 0xFF), (uint8_t)((duration_ms >> 8) & 0xFF),
                      (uint8_t)((duration_ms >> 16) & 0xFF), (uint8_t)((duration_ms >> 24) & 0xFF)};
    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_ZLL_SET_RX_ON_WHEN_IDLE, dur, 4, resp, sizeof(resp),
                                 &resp_len, CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    ESP_LOGI(TAG, "ZLL Touchlink TARGET armed (%lu ms RX-on, mask=0x%08lX) -> %s",
             (unsigned long)duration_ms, (unsigned long)primary,
             err == ESP_OK ? "ok" : esp_err_to_name(err));
    return err;
}

esp_err_t ezsp_configure_for_coordinator(void)
{
    uint8_t st = 0;
    s_zll_supported = false;
    /* Best-effort — some NCPs reject individual configs; continue anyway. */
    ezsp_set_configuration_value(EZSP_CONFIG_STACK_PROFILE, 2, &st);
    ezsp_set_configuration_value(EZSP_CONFIG_SECURITY_LEVEL, 5, &st);
    ezsp_set_configuration_value(EZSP_CONFIG_MAX_END_DEVICE_CHILDREN, 32, &st);
    ezsp_set_configuration_value(EZSP_CONFIG_TRUST_CENTER_ADDRESS_CACHE_SIZE, 2, &st);
    /* Need room for groups 0/1/901 × ep1+ep10 (+ dynamic Add Group). */
    ezsp_set_configuration_value(EZSP_CONFIG_MULTICAST_TABLE_SIZE, 16, &st);
    /* More packet buffers so multicast RX + messageSent contents aren't dropped. */
    ezsp_set_configuration_value(EZSP_CONFIG_PACKET_BUFFER_COUNT, 40, &st);
    ezsp_set_configuration_value(EZSP_CONFIG_APS_UNICAST_MESSAGE_COUNT, 20, &st);
    ezsp_set_configuration_value(EZSP_CONFIG_BINDING_TABLE_SIZE, 32, &st);
    /* Sleepy sensors (SNZB-02D) may poll infrequently — keep ONE frame queued longer. */
    ezsp_set_configuration_value(EZSP_CONFIG_INDIRECT_TRANSMIT_TIMEOUT, 30000, &st);
    /* Keep sleepy children in the table for days (N=14 → 2^14 minutes). */
    ezsp_set_configuration_value(EZSP_CONFIG_END_DEVICE_POLL_TIMEOUT, 14, &st);

    ezsp_set_policy(EZSP_TC_KEY_REQUEST_POLICY,
                    EZSP_DECISION_ALLOW_TC_KEY_REQUESTS_AND_SEND_CURRENT_KEY, &st);
    ezsp_set_policy(EZSP_APP_KEY_REQUEST_POLICY, EZSP_DECISION_DENY_APP_KEY_REQUESTS, &st);
    /* Idle TC policy: allow preconfigured-key (ZB3) joins when permit is later opened. */
    ezsp_set_policy(EZSP_TRUST_CENTER_POLICY, EZSP_DECISION_ALLOW_PRECONFIGURED_KEY_JOINS, &st);
    /* Deliver full APS payloads in incomingMessageHandler callbacks. */
    ezsp_set_policy(EZSP_MESSAGE_CONTENTS_IN_CALLBACK_POLICY, EZSP_MESSAGE_TAG_AND_CONTENTS, &st);
    /* Local HA endpoints — ep1 general; ep11 fresh fake bulb (ep10 may be a stale NCP token). */
    ezsp_add_ha_endpoint(1);
    ezsp_add_light_endpoint(ZB_FAKE_BULB_ENDPOINT);
    /* Best-effort: also refresh classic ep10 if NCP still has it. */
    ezsp_add_light_endpoint(10);

    /* Probe ZLL/Touchlink — only present if NCP firmware was built with ZLL plugins. */
    {
        uint8_t resp[8];
        size_t resp_len = 0;
        esp_err_t zerr =
            ezsp_command(EZSP_FRAME_GET_ZLL_PRIMARY_CHANNEL_MASK, NULL, 0, resp, sizeof(resp),
                         &resp_len, CONFIG_EZSP_COMMAND_TIMEOUT_MS);
        if (zerr == ESP_OK && resp_len >= 4) {
            uint32_t mask = (uint32_t)resp[0] | ((uint32_t)resp[1] << 8) |
                            ((uint32_t)resp[2] << 16) | ((uint32_t)resp[3] << 24);
            s_zll_supported = true;
            ESP_LOGI(TAG, "ZLL/Touchlink SUPPORTED by NCP (primaryChannelMask=0x%08lX)",
                     (unsigned long)mask);
            resp_len = 0;
            if (ezsp_command(EZSP_FRAME_IS_ZLL_NETWORK, NULL, 0, resp, sizeof(resp), &resp_len,
                             CONFIG_EZSP_COMMAND_TIMEOUT_MS) == ESP_OK &&
                resp_len >= 1) {
                ESP_LOGI(TAG, "ZLL isZllNetwork=%u", (unsigned)resp[0]);
            }
            /* Enable channel masks even when mask was 0 — otherwise remotes never see us. */
            (void)ezsp_zll_arm_touchlink_target(60000);
        } else if (zerr == ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW(TAG,
                     "ZLL/Touchlink NOT in this NCP firmware — cannot emulate Touchlink light "
                     "target on ICC-1 (use bind-on-wake instead)");
        } else {
            ESP_LOGW(TAG, "ZLL probe failed: %s (len=%u) — Touchlink target unavailable",
                     esp_err_to_name(zerr), (unsigned)resp_len);
        }
    }
    return ESP_OK;
}

esp_err_t ezsp_set_join_policy(bool allow_joins)
{
    uint16_t decision = allow_joins ? (uint16_t)(EZSP_DECISION_ALLOW_JOINS |
                                                 EZSP_DECISION_ALLOW_UNSECURED_REJOINS)
                                    : (uint16_t)EZSP_DECISION_ALLOW_PRECONFIGURED_KEY_JOINS;
    uint8_t st = 0;
    return ezsp_set_policy(EZSP_TRUST_CENTER_POLICY, decision, &st);
}

esp_err_t ezsp_set_initial_security_state(const uint8_t network_key[16], uint8_t *ember_status_out)
{
    /* EmberInitialSecurityState packing (LE):
     * bitmask u16 | preconfiguredKey[16] | networkKey[16] | seq u8 | tcEui64[8] */
    uint8_t params[2 + 16 + 16 + 1 + 8];
    memset(params, 0, sizeof(params));

    uint16_t bitmask = (uint16_t)(EMBER_HAVE_NETWORK_KEY | EMBER_HAVE_PRECONFIGURED_KEY |
                                  EMBER_TRUST_CENTER_GLOBAL_LINK_KEY | EMBER_REQUIRE_ENCRYPTED_KEY |
                                  EMBER_NO_FRAME_COUNTER_RESET);
    params[0] = (uint8_t)(bitmask & 0xFF);
    params[1] = (uint8_t)(bitmask >> 8);

    /* Well-known ZigBeeAlliance09 global link / preconfigured key (ZB3 install code path) */
    static const uint8_t zigbee_alliance09[16] = {'Z', 'i', 'g', 'B', 'e', 'e', 'A', 'l',
                                                  'l', 'i', 'a', 'n', 'c', 'e', '0', '9'};
    memcpy(params + 2, zigbee_alliance09, 16);
    memcpy(params + 18, network_key, 16);
    params[34] = 0; /* networkKeySequenceNumber */
    /* tc eui64 left zero = use local EUI */

    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_SET_INITIAL_SECURITY, params, sizeof(params), resp,
                                 sizeof(resp), &resp_len, CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (ember_status_out) {
        *ember_status_out = resp[0];
    }
    ESP_LOGI(TAG, "setInitialSecurityState -> %s (0x%02X)", ezsp_ember_status_str(resp[0]), resp[0]);
    return (resp[0] == EMBER_SUCCESS) ? ESP_OK : ESP_FAIL;
}

esp_err_t ezsp_form_network(const ezsp_network_params_t *params, uint8_t *ember_status_out)
{
    if (!params) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err =
        ezsp_command(EZSP_FRAME_FORM_NETWORK, (const uint8_t *)params, sizeof(*params), resp,
                     sizeof(resp), &resp_len, CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (ember_status_out) {
        *ember_status_out = resp[0];
    }
    ESP_LOGI(TAG, "formNetwork ch=%u pan=0x%04X -> %s (0x%02X)", params->radio_channel,
             params->pan_id, ezsp_ember_status_str(resp[0]), resp[0]);
    return (resp[0] == EMBER_SUCCESS) ? ESP_OK : ESP_FAIL;
}

esp_err_t ezsp_leave_network(uint8_t *ember_status_out)
{
    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_LEAVE_NETWORK, NULL, 0, resp, sizeof(resp), &resp_len,
                                 CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (ember_status_out) {
        *ember_status_out = resp[0];
    }
    ESP_LOGI(TAG, "leaveNetwork -> %s (0x%02X)", ezsp_ember_status_str(resp[0]), resp[0]);
    return (resp[0] == EMBER_SUCCESS || resp[0] == 0x18 /* NOT_JOINED */) ? ESP_OK : ESP_FAIL;
}

esp_err_t ezsp_permit_joining(uint8_t duration_sec, uint8_t *ember_status_out)
{
    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_PERMIT_JOINING, &duration_sec, 1, resp, sizeof(resp),
                                 &resp_len, CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (ember_status_out) {
        *ember_status_out = resp[0];
    }
    ESP_LOGI(TAG, "permitJoining(%u) -> %s", (unsigned)duration_sec, ezsp_ember_status_str(resp[0]));
    return (resp[0] == EMBER_SUCCESS) ? ESP_OK : ESP_FAIL;
}

esp_err_t ezsp_get_network_parameters(uint8_t *node_type_out, ezsp_network_params_t *params_out)
{
    /* EmberZNet 6.7 / EZSP v8 returns: EmberStatus (1) + EmberNodeType (1) + params (20).
     * Older docs omit the status byte; accept both lengths. */
    uint8_t resp[2 + sizeof(ezsp_network_params_t)];
    size_t resp_len = 0;
    _Static_assert(sizeof(ezsp_network_params_t) == 20, "EmberNetworkParameters must be 20 bytes");
    esp_err_t err =
        ezsp_command(EZSP_FRAME_GET_NETWORK_PARAMETERS, NULL, 0, resp, sizeof(resp), &resp_len,
                     CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    size_t off = 0;
    if (resp_len >= 2 + sizeof(ezsp_network_params_t)) {
        off = 1; /* skip EmberStatus */
    } else if (resp_len < 1 + sizeof(ezsp_network_params_t)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (node_type_out) {
        *node_type_out = resp[off];
    }
    if (params_out) {
        memcpy(params_out, resp + off + 1, sizeof(*params_out));
    }
    return ESP_OK;
}

esp_err_t ezsp_get_child_data(uint8_t index, ezsp_child_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    uint8_t resp[32];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_GET_CHILD_DATA, &index, 1, resp, sizeof(resp),
                                 &resp_len, CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* status(1) + eui64(8) + type(1) + id(2) [+ phy/power/timeout] */
    if (resp[0] != EMBER_SUCCESS || resp_len < 12) {
        out->valid = false;
        return ESP_OK;
    }
    memcpy(out->eui64, resp + 1, 8);
    out->node_type = resp[9];
    out->node_id = (uint16_t)resp[10] | ((uint16_t)resp[11] << 8);

    bool eui_zero = true;
    for (int i = 0; i < 8; i++) {
        if (out->eui64[i] != 0) {
            eui_zero = false;
            break;
        }
    }
    if (eui_zero || out->node_id == 0x0000 || out->node_id == 0xFFFF) {
        out->valid = false;
        return ESP_OK;
    }
    out->valid = true;
    return ESP_OK;
}

esp_err_t ezsp_add_ha_endpoint(uint8_t endpoint)
{
    /* endpoint, profileId, deviceId, appFlags, inCount, inClusters[], outCount, outClusters[]
     * In-clusters include On/Off, Level, Scenes (+ IKEA) so remotes can bind button cmds to us. */
    const uint16_t in_clusters[] = {ZCL_CLUSTER_BASIC,        ZCL_CLUSTER_POWER_CONFIG,
                                    ZCL_CLUSTER_IDENTIFY,     ZCL_CLUSTER_SCENES,
                                    ZCL_CLUSTER_ON_OFF,       ZCL_CLUSTER_LEVEL_CONTROL,
                                    ZCL_CLUSTER_TEMP_MEASUREMENT, ZCL_CLUSTER_REL_HUMIDITY,
                                    ZCL_CLUSTER_IKEA_BUTTON};
    const uint16_t out_clusters[] = {ZCL_CLUSTER_BASIC,        ZCL_CLUSTER_POWER_CONFIG,
                                     ZCL_CLUSTER_IDENTIFY,     ZCL_CLUSTER_SCENES,
                                     ZCL_CLUSTER_ON_OFF,       ZCL_CLUSTER_LEVEL_CONTROL,
                                     ZCL_CLUSTER_TEMP_MEASUREMENT, ZCL_CLUSTER_REL_HUMIDITY};
    uint8_t params[8 + sizeof(in_clusters) + sizeof(out_clusters)];
    size_t pos = 0;
    params[pos++] = endpoint;
    params[pos++] = (uint8_t)(ZCL_PROFILE_HA & 0xFF);
    params[pos++] = (uint8_t)(ZCL_PROFILE_HA >> 8);
    /* Combined Interface — NOT a light. Only ep10 is Dimmable Light so IKEA F&B
     * binds OnOff/Level/Scenes/Groups to the real light target, not this endpoint. */
    params[pos++] = 0x07; /* deviceId lo: 0x0007 Combined Interface */
    params[pos++] = 0x00; /* deviceId hi */
    params[pos++] = 0x00; /* appFlags */
    params[pos++] = (uint8_t)(sizeof(in_clusters) / sizeof(in_clusters[0]));
    for (size_t i = 0; i < sizeof(in_clusters) / sizeof(in_clusters[0]); i++) {
        params[pos++] = (uint8_t)(in_clusters[i] & 0xFF);
        params[pos++] = (uint8_t)(in_clusters[i] >> 8);
    }
    params[pos++] = (uint8_t)(sizeof(out_clusters) / sizeof(out_clusters[0]));
    for (size_t i = 0; i < sizeof(out_clusters) / sizeof(out_clusters[0]); i++) {
        params[pos++] = (uint8_t)(out_clusters[i] & 0xFF);
        params[pos++] = (uint8_t)(out_clusters[i] >> 8);
    }

    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_ADD_ENDPOINT, params, pos, resp, sizeof(resp), &resp_len,
                                 CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "addEndpoint(%u) -> 0x%02X", endpoint, resp[0]);
    /* 0x00 success; 0x36 EZSP_ERROR_INVALID_CALL often means endpoint already present. */
    return (resp[0] == EMBER_SUCCESS || resp[0] == 0x36) ? ESP_OK : ESP_FAIL;
}

esp_err_t ezsp_add_light_endpoint(uint8_t endpoint)
{
    /* Minimal light target: Identify + OnOff + Level + Scenes (IKEA F&B looks for these). */
    const uint16_t in_clusters[] = {ZCL_CLUSTER_BASIC, ZCL_CLUSTER_IDENTIFY, ZCL_CLUSTER_GROUPS,
                                    ZCL_CLUSTER_SCENES, ZCL_CLUSTER_ON_OFF,
                                    ZCL_CLUSTER_LEVEL_CONTROL};
    const uint16_t out_clusters[] = {ZCL_CLUSTER_IDENTIFY};
    uint8_t params[8 + sizeof(in_clusters) + sizeof(out_clusters)];
    size_t pos = 0;
    params[pos++] = endpoint ? endpoint : ZB_FAKE_BULB_ENDPOINT;
    params[pos++] = (uint8_t)(ZCL_PROFILE_HA & 0xFF);
    params[pos++] = (uint8_t)(ZCL_PROFILE_HA >> 8);
    params[pos++] = 0x01; /* deviceId 0x0101 Dimmable Light */
    params[pos++] = 0x01;
    params[pos++] = 0x00; /* appFlags */
    params[pos++] = (uint8_t)(sizeof(in_clusters) / sizeof(in_clusters[0]));
    for (size_t i = 0; i < sizeof(in_clusters) / sizeof(in_clusters[0]); i++) {
        params[pos++] = (uint8_t)(in_clusters[i] & 0xFF);
        params[pos++] = (uint8_t)(in_clusters[i] >> 8);
    }
    params[pos++] = (uint8_t)(sizeof(out_clusters) / sizeof(out_clusters[0]));
    for (size_t i = 0; i < sizeof(out_clusters) / sizeof(out_clusters[0]); i++) {
        params[pos++] = (uint8_t)(out_clusters[i] & 0xFF);
        params[pos++] = (uint8_t)(out_clusters[i] >> 8);
    }

    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_ADD_ENDPOINT, params, pos, resp, sizeof(resp), &resp_len,
                                 CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "addLightEndpoint(%u) -> 0x%02X", endpoint ? endpoint : ZB_FAKE_BULB_ENDPOINT,
             resp[0]);
    return (resp[0] == EMBER_SUCCESS || resp[0] == 0x36) ? ESP_OK : ESP_FAIL;
}

esp_err_t ezsp_send_unicast(uint16_t node_id, const ezsp_aps_frame_t *aps, const uint8_t *msg,
                            size_t msg_len, uint8_t *ember_status_out)
{
    if (!aps || (!msg && msg_len) || msg_len > 80) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t params[1 + 2 + EZSP_APS_FRAME_SIZE + 1 + 1 + 80];
    size_t pos = 0;
    params[pos++] = EMBER_OUTGOING_DIRECT;
    params[pos++] = (uint8_t)(node_id & 0xFF);
    params[pos++] = (uint8_t)(node_id >> 8);
    memcpy(params + pos, aps, EZSP_APS_FRAME_SIZE);
    pos += EZSP_APS_FRAME_SIZE;
    params[pos++] = ++s_zcl_seq; /* messageTag */
    params[pos++] = (uint8_t)msg_len;
    if (msg_len) {
        memcpy(params + pos, msg, msg_len);
        pos += msg_len;
    }

    uint8_t resp[8];
    size_t resp_len = 0;
    esp_err_t err = ezsp_command(EZSP_FRAME_SEND_UNICAST, params, pos, resp, sizeof(resp), &resp_len,
                                 CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (resp_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (ember_status_out) {
        *ember_status_out = resp[0];
    }
    sniff_push(EZSP_SNIFF_TX, EMBER_OUTGOING_DIRECT, node_id, aps->profile_id, aps->cluster_id,
               aps->group_id, aps->source_endpoint, aps->destination_endpoint, 0, resp[0], msg,
               (uint8_t)(msg_len > 255 ? 255 : msg_len));
    return (resp[0] == EMBER_SUCCESS) ? ESP_OK : ESP_FAIL;
}

esp_err_t ezsp_zcl_read_attributes(uint16_t node_id, uint8_t dest_ep, uint16_t cluster_id,
                                   const uint16_t *attr_ids, size_t attr_count)
{
    if (!attr_ids || attr_count == 0 || attr_count > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t zcl[3 + 16];
    size_t zlen = 0;
    zcl[zlen++] = 0x00; /* frame control: client→server, general */
    zcl[zlen++] = ++s_zcl_seq;
    zcl[zlen++] = 0x00; /* Read Attributes */
    for (size_t i = 0; i < attr_count; i++) {
        zcl[zlen++] = (uint8_t)(attr_ids[i] & 0xFF);
        zcl[zlen++] = (uint8_t)(attr_ids[i] >> 8);
    }

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = cluster_id;
    aps.source_endpoint = 1;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = s_zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, zcl, zlen, &st);
    ESP_LOGI(TAG, "ZCL read cluster=0x%04X node=0x%04X -> %s", cluster_id, node_id,
             ezsp_ember_status_str(st));
    return err;
}

esp_err_t ezsp_zcl_configure_reporting(uint16_t node_id, uint8_t dest_ep, uint16_t cluster_id,
                                       uint16_t attr_id, uint8_t data_type, uint16_t min_interval_s,
                                       uint16_t max_interval_s, const uint8_t *change_le,
                                       size_t change_len)
{
    if (change_len > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    /* FC + seq + cmd + direction + attr + type + min + max + change */
    uint8_t zcl[3 + 1 + 2 + 1 + 2 + 2 + 8];
    size_t zlen = 0;
    zcl[zlen++] = 0x00; /* general, client→server */
    zcl[zlen++] = ++s_zcl_seq;
    zcl[zlen++] = 0x06; /* Configure Reporting */
    zcl[zlen++] = 0x00; /* direction: reported */
    zcl[zlen++] = (uint8_t)(attr_id & 0xFF);
    zcl[zlen++] = (uint8_t)(attr_id >> 8);
    zcl[zlen++] = data_type;
    zcl[zlen++] = (uint8_t)(min_interval_s & 0xFF);
    zcl[zlen++] = (uint8_t)(min_interval_s >> 8);
    zcl[zlen++] = (uint8_t)(max_interval_s & 0xFF);
    zcl[zlen++] = (uint8_t)(max_interval_s >> 8);
    if (change_le && change_len) {
        memcpy(zcl + zlen, change_le, change_len);
        zlen += change_len;
    }

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = cluster_id;
    aps.source_endpoint = 1;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = s_zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, zcl, zlen, &st);
    ESP_LOGI(TAG, "ZCL cfgReport cluster=0x%04X attr=0x%04X node=0x%04X ep%u min=%u max=%u -> %s",
             cluster_id, attr_id, node_id, (unsigned)aps.destination_endpoint,
             (unsigned)min_interval_s, (unsigned)max_interval_s, ezsp_ember_status_str(st));
    return err;
}

esp_err_t ezsp_zcl_on_off_command(uint16_t node_id, uint8_t dest_ep, uint8_t cmd)
{
    uint8_t zcl[3];
    zcl[0] = 0x01; /* frame control: client→server, cluster-specific */
    zcl[1] = ++s_zcl_seq;
    zcl[2] = cmd;

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = ZCL_CLUSTER_ON_OFF;
    aps.source_endpoint = 1;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = s_zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, zcl, sizeof(zcl), &st);
    ESP_LOGI(TAG, "ZCL OnOff cmd=0x%02X node=0x%04X ep=%u -> %s", cmd, node_id,
             (unsigned)aps.destination_endpoint, ezsp_ember_status_str(st));
    return err;
}

esp_err_t ezsp_zcl_ias_zone_enroll_response(uint16_t node_id, uint8_t dest_ep, uint8_t src_ep,
                                            uint8_t zcl_seq, uint8_t enroll_code, uint8_t zone_id)
{
    uint8_t zcl[5];
    zcl[0] = 0x01; /* client→server, cluster-specific */
    zcl[1] = zcl_seq;
    zcl[2] = ZCL_CMD_IAS_ZONE_ENROLL_RSP;
    zcl[3] = enroll_code; /* 0 = success */
    zcl[4] = zone_id;

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = ZCL_CLUSTER_IAS_ZONE;
    aps.source_endpoint = src_ep ? src_ep : 1;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, zcl, sizeof(zcl), &st);
    ESP_LOGI(TAG, "IAS ZoneEnrollRsp node=0x%04X ep=%u code=%u zone=%u -> %s", node_id,
             (unsigned)aps.destination_endpoint, (unsigned)enroll_code, (unsigned)zone_id,
             ezsp_ember_status_str(st));
    return err;
}

esp_err_t ezsp_zcl_identify(uint16_t node_id, uint8_t dest_ep, uint16_t time_s)
{
    uint8_t zcl[5];
    zcl[0] = 0x01; /* client→server, cluster-specific */
    zcl[1] = ++s_zcl_seq;
    zcl[2] = ZCL_CMD_IDENTIFY_CMD;
    zcl[3] = (uint8_t)(time_s & 0xFF);
    zcl[4] = (uint8_t)(time_s >> 8);

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = ZCL_CLUSTER_IDENTIFY;
    aps.source_endpoint = 1;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = s_zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, zcl, sizeof(zcl), &st);
    ESP_LOGI(TAG, "ZCL Identify node=0x%04X ep=%u time=%u -> %s", node_id,
             (unsigned)aps.destination_endpoint, (unsigned)time_s, ezsp_ember_status_str(st));
    return err;
}

esp_err_t ezsp_zcl_identify_query_response(uint16_t node_id, uint8_t dest_ep, uint8_t src_ep,
                                           uint8_t zcl_seq, uint16_t timeout_ds)
{
    /* FC 0x19: cluster-specific + server→client + disable default response.
     * Transaction seq MUST echo the Identify Query (ZCL). */
    uint8_t zcl[5];
    zcl[0] = 0x19;
    zcl[1] = zcl_seq;
    zcl[2] = ZCL_CMD_IDENTIFY_QUERY_RSP;
    zcl[3] = (uint8_t)(timeout_ds & 0xFF);
    zcl[4] = (uint8_t)(timeout_ds >> 8);

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = ZCL_CLUSTER_IDENTIFY;
    aps.source_endpoint = src_ep ? src_ep : ZB_FAKE_BULB_ENDPOINT;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, zcl, sizeof(zcl), &st);
    ESP_LOGI(TAG, "ZCL IdentifyQueryRsp seq=%u from ep%u → 0x%04X ep%u timeout=%u -> %s",
             (unsigned)zcl_seq, (unsigned)aps.source_endpoint, node_id,
             (unsigned)aps.destination_endpoint, (unsigned)timeout_ds, ezsp_ember_status_str(st));
    return err;
}

void ezsp_flush_identify_query_responses(void)
{
    ezsp_id_query_rsp_t e;
    while (idq_pop(&e)) {
        /* Reply ONLY from fake-bulb endpoint (Groups+OnOff+Level). */
        uint16_t t = ezsp_light_identify_time_s();
        if (t == 0) {
            ezsp_light_start_identify(180);
            t = 180;
        }
        (void)ezsp_zcl_identify_query_response(e.node_id, e.dest_ep, ZB_FAKE_BULB_ENDPOINT,
                                               e.zcl_seq, t);
        id_bind_push(e.node_id);
        (void)ezsp_join_ikea_multicast_groups();
    }
}

bool ezsp_take_identify_bind_node(uint16_t *node_out)
{
    bool ok = false;
    portENTER_CRITICAL(&s_idq_mux);
    if (s_id_bind_tail != s_id_bind_head) {
        if (node_out) {
            *node_out = s_id_bind_q[s_id_bind_tail];
        }
        s_id_bind_tail = (uint8_t)((s_id_bind_tail + 1) % EZSP_ID_BIND_Q_LEN);
        ok = true;
    }
    portEXIT_CRITICAL(&s_idq_mux);
    return ok;
}

void ezsp_set_host_wake(void (*fn)(void))
{
    s_host_wake = fn;
}

esp_err_t ezsp_zcl_groups_add_group_response(uint16_t node_id, uint8_t dest_ep, uint8_t src_ep,
                                             uint8_t zcl_seq, uint8_t status, uint16_t group_id)
{
    uint8_t zcl[6];
    zcl[0] = 0x19; /* cluster-specific, server→client, disable default rsp */
    zcl[1] = zcl_seq;
    zcl[2] = 0x00; /* Add Group Response */
    zcl[3] = status;
    zcl[4] = (uint8_t)(group_id & 0xFF);
    zcl[5] = (uint8_t)(group_id >> 8);

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = ZCL_CLUSTER_GROUPS;
    aps.source_endpoint = src_ep ? src_ep : ZB_FAKE_BULB_ENDPOINT;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, zcl, sizeof(zcl), &st);
    ESP_LOGI(TAG, "ZCL AddGroupRsp group=%u status=0x%02X → 0x%04X -> %s", (unsigned)group_id,
             status, node_id, ezsp_ember_status_str(st));
    return err;
}

esp_err_t ezsp_zcl_groups_remove_group(uint16_t node_id, uint8_t dest_ep, uint16_t group_id)
{
    /* Client→server Remove Group (0x03): GroupId */
    uint8_t zcl[5];
    zcl[0] = 0x01;
    zcl[1] = ++s_zcl_seq;
    zcl[2] = 0x03; /* Remove Group */
    zcl[3] = (uint8_t)(group_id & 0xFF);
    zcl[4] = (uint8_t)(group_id >> 8);

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = ZCL_CLUSTER_GROUPS;
    aps.source_endpoint = ZB_FAKE_BULB_ENDPOINT;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = s_zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, zcl, sizeof(zcl), &st);
    ESP_LOGI(TAG, "ZCL RemoveGroup %u → 0x%04X ep%u -> %s", (unsigned)group_id, node_id,
             (unsigned)aps.destination_endpoint, ezsp_ember_status_str(st));
    return err;
}

esp_err_t ezsp_zcl_groups_add_group(uint16_t node_id, uint8_t dest_ep, uint16_t group_id)
{
    /* Client→server Add Group (0x00): GroupId + empty name */
    uint8_t zcl[6];
    zcl[0] = 0x01;
    zcl[1] = ++s_zcl_seq;
    zcl[2] = 0x00; /* Add Group */
    zcl[3] = (uint8_t)(group_id & 0xFF);
    zcl[4] = (uint8_t)(group_id >> 8);
    zcl[5] = 0; /* group name length */

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = ZCL_CLUSTER_GROUPS;
    aps.source_endpoint = ZB_FAKE_BULB_ENDPOINT;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = s_zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, zcl, sizeof(zcl), &st);
    ESP_LOGI(TAG, "ZCL AddGroup %u → 0x%04X ep%u -> %s", (unsigned)group_id, node_id,
             (unsigned)aps.destination_endpoint, ezsp_ember_status_str(st));
    return err;
}

void ezsp_hint_button_remote(uint16_t node_id)
{
    if (node_id != 0 && node_id != 0xFFFF) {
        s_last_btn_remote_node = node_id;
    }
}

esp_err_t ezsp_zcl_read_attr_response_u16(uint16_t node_id, uint8_t dest_ep, uint8_t src_ep,
                                          uint8_t zcl_seq, uint16_t cluster_id, uint16_t attr_id,
                                          uint8_t data_type, uint16_t value)
{
    uint8_t zcl[10];
    zcl[0] = 0x18; /* general, server→client, disable default rsp */
    zcl[1] = zcl_seq;
    zcl[2] = 0x01; /* Read Attributes Response */
    zcl[3] = (uint8_t)(attr_id & 0xFF);
    zcl[4] = (uint8_t)(attr_id >> 8);
    zcl[5] = 0x00; /* SUCCESS */
    zcl[6] = data_type;
    zcl[7] = (uint8_t)(value & 0xFF);
    zcl[8] = (uint8_t)(value >> 8);

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = cluster_id;
    aps.source_endpoint = src_ep ? src_ep : ZB_FAKE_BULB_ENDPOINT;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, zcl, 9, &st);
    ESP_LOGI(TAG, "ZCL ReadAttrRsp cluster=0x%04X attr=0x%04X val=%u → 0x%04X -> %s", cluster_id,
             attr_id, (unsigned)value, node_id, ezsp_ember_status_str(st));
    return err;
}

esp_err_t ezsp_zcl_read_attr_response_u8(uint16_t node_id, uint8_t dest_ep, uint8_t src_ep,
                                         uint8_t zcl_seq, uint16_t cluster_id, uint16_t attr_id,
                                         uint8_t data_type, uint8_t value)
{
    uint8_t zcl[9];
    zcl[0] = 0x18;
    zcl[1] = zcl_seq;
    zcl[2] = 0x01;
    zcl[3] = (uint8_t)(attr_id & 0xFF);
    zcl[4] = (uint8_t)(attr_id >> 8);
    zcl[5] = 0x00;
    zcl[6] = data_type;
    zcl[7] = value;

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = cluster_id;
    aps.source_endpoint = src_ep ? src_ep : ZB_FAKE_BULB_ENDPOINT;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = zcl_seq;

    uint8_t st = 0;
    return ezsp_send_unicast(node_id, &aps, zcl, 8, &st);
}

esp_err_t ezsp_zcl_move_to_level(uint16_t node_id, uint8_t dest_ep, uint8_t level, bool with_on_off)
{
    uint8_t zcl[6];
    zcl[0] = 0x01; /* client→server, cluster-specific */
    zcl[1] = ++s_zcl_seq;
    zcl[2] = with_on_off ? ZCL_CMD_MOVE_TO_LEVEL_WITH_ON_OFF : ZCL_CMD_MOVE_TO_LEVEL;
    zcl[3] = level;
    zcl[4] = 0; /* transition time LE — immediate */
    zcl[5] = 0;

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZCL_PROFILE_HA;
    aps.cluster_id = ZCL_CLUSTER_LEVEL_CONTROL;
    aps.source_endpoint = 1;
    aps.destination_endpoint = dest_ep ? dest_ep : 1;
    aps.options = (uint16_t)(EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = s_zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, zcl, sizeof(zcl), &st);
    ESP_LOGI(TAG, "ZCL MoveToLevel level=%u node=0x%04X ep=%u -> %s", (unsigned)level, node_id,
             (unsigned)aps.destination_endpoint, ezsp_ember_status_str(st));
    return err;
}

esp_err_t ezsp_zdo_bind(uint16_t node_id, const uint8_t src_eui[8], uint8_t src_ep,
                        uint16_t cluster_id, const uint8_t dst_eui[8], uint8_t dst_ep)
{
    if (!src_eui || !dst_eui || !src_ep) {
        return ESP_ERR_INVALID_ARG;
    }
    /* ZDO Bind_req: TransactionSeq(1) + SrcAddress(8) SrcEndpoint(1) ClusterId(2)
     * DstAddrMode(1) DstAddress(8) DstEndpoint(1). The leading TSN is required —
     * without it sleepy IKEA remotes ignore the bind. */
    uint8_t payload[22];
    size_t pos = 0;
    payload[pos++] = (uint8_t)((++s_zcl_seq) & 0x7F); /* ZDO TSN: app range 0–127 */
    if (payload[0] == 0) {
        payload[0] = (uint8_t)((++s_zcl_seq) & 0x7F);
        if (payload[0] == 0) {
            payload[0] = 1;
        }
    }
    memcpy(payload + pos, src_eui, 8);
    pos += 8;
    payload[pos++] = src_ep;
    payload[pos++] = (uint8_t)(cluster_id & 0xFF);
    payload[pos++] = (uint8_t)(cluster_id >> 8);
    payload[pos++] = 0x03; /* IEEE / 64-bit extended */
    memcpy(payload + pos, dst_eui, 8);
    pos += 8;
    payload[pos++] = dst_ep ? dst_ep : 1;

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZDO_PROFILE_ID;
    aps.cluster_id = ZDO_CLUSTER_BIND_REQ;
    aps.source_endpoint = 0;
    aps.destination_endpoint = 0;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = s_zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, payload, pos, &st);
    ESP_LOGI(TAG, "ZDO Bind cluster=0x%04X node=0x%04X srcEp=%u dstEp=%u -> %s", cluster_id,
             node_id, (unsigned)src_ep, (unsigned)(dst_ep ? dst_ep : 1), ezsp_ember_status_str(st));
    return err;
}

esp_err_t ezsp_zdo_bind_group(uint16_t node_id, const uint8_t src_eui[8], uint8_t src_ep,
                              uint16_t cluster_id, uint16_t group_id, bool unbind)
{
    if (!src_eui || !src_ep) {
        return ESP_ERR_INVALID_ARG;
    }
    /* TSN + SrcAddress(8) SrcEp ClusterId DstAddrMode=0x01 DstGroup(2) — no DstEp */
    uint8_t payload[15];
    size_t pos = 0;
    payload[pos++] = (uint8_t)((++s_zcl_seq) & 0x7F);
    if (payload[0] == 0) {
        payload[0] = 1;
    }
    memcpy(payload + pos, src_eui, 8);
    pos += 8;
    payload[pos++] = src_ep;
    payload[pos++] = (uint8_t)(cluster_id & 0xFF);
    payload[pos++] = (uint8_t)(cluster_id >> 8);
    payload[pos++] = 0x01; /* group */
    payload[pos++] = (uint8_t)(group_id & 0xFF);
    payload[pos++] = (uint8_t)(group_id >> 8);

    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZDO_PROFILE_ID;
    aps.cluster_id = unbind ? ZDO_CLUSTER_UNBIND_REQ : ZDO_CLUSTER_BIND_REQ;
    aps.source_endpoint = 0;
    aps.destination_endpoint = 0;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = s_zcl_seq;

    uint8_t st = 0;
    esp_err_t err = ezsp_send_unicast(node_id, &aps, payload, pos, &st);
    ESP_LOGI(TAG, "ZDO %sGroup cluster=0x%04X node=0x%04X group=%u -> %s", unbind ? "Unbind" : "Bind",
             cluster_id, node_id, (unsigned)group_id, ezsp_ember_status_str(st));
    return err;
}

static esp_err_t zdo_send(uint16_t node_id, uint16_t cluster, const uint8_t *payload, size_t len)
{
    ezsp_aps_frame_t aps;
    memset(&aps, 0, sizeof(aps));
    aps.profile_id = ZDO_PROFILE_ID;
    aps.cluster_id = cluster;
    aps.source_endpoint = 0;
    aps.destination_endpoint = 0;
    aps.options = (uint16_t)(EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY);
    aps.group_id = 0;
    aps.sequence = s_zcl_seq;
    uint8_t st = 0;
    return ezsp_send_unicast(node_id, &aps, payload, len, &st);
}

esp_err_t ezsp_zdo_mgmt_bind_req(uint16_t node_id, uint8_t start_index)
{
    uint8_t payload[2] = {++s_zcl_seq, start_index};
    s_last_btn_remote_node = node_id;
    esp_err_t err = zdo_send(node_id, ZDO_CLUSTER_MGMT_BIND_REQ, payload, sizeof(payload));
    ESP_LOGI(TAG, "ZDO MgmtBindReq node=0x%04X start=%u", node_id, (unsigned)start_index);
    return err;
}

esp_err_t ezsp_zdo_active_ep_req(uint16_t node_id)
{
    uint8_t payload[3] = {++s_zcl_seq, (uint8_t)(node_id & 0xFF), (uint8_t)(node_id >> 8)};
    esp_err_t err = zdo_send(node_id, ZDO_CLUSTER_ACTIVE_EP_REQ, payload, sizeof(payload));
    ESP_LOGI(TAG, "ZDO ActiveEpReq node=0x%04X", node_id);
    return err;
}

esp_err_t ezsp_zdo_simple_desc_req(uint16_t node_id, uint8_t endpoint)
{
    uint8_t payload[4] = {++s_zcl_seq, (uint8_t)(node_id & 0xFF), (uint8_t)(node_id >> 8),
                          endpoint};
    esp_err_t err = zdo_send(node_id, ZDO_CLUSTER_SIMPLE_DESC_REQ, payload, sizeof(payload));
    ESP_LOGI(TAG, "ZDO SimpleDescReq node=0x%04X ep=%u", node_id, (unsigned)endpoint);
    return err;
}

/** Join Zigbee groups so IKEA remotes that groupcast button cmds are delivered to us. */
esp_err_t ezsp_set_multicast_group(uint8_t table_index, uint16_t group_id, uint8_t endpoint)
{
    /* EmberMulticastTableEntry: multicastId(2) endpoint(1) networkIndex(1) */
    uint8_t params[1 + 4];
    params[0] = table_index;
    params[1] = (uint8_t)(group_id & 0xFF);
    params[2] = (uint8_t)(group_id >> 8);
    params[3] = endpoint ? endpoint : ZB_FAKE_BULB_ENDPOINT;
    params[4] = 0; /* networkIndex */

    uint8_t resp[4];
    size_t resp_len = 0;
    esp_err_t err =
        ezsp_command(EZSP_FRAME_SET_MULTICAST_TABLE_ENTRY, params, sizeof(params), resp,
                     sizeof(resp), &resp_len, CONFIG_EZSP_COMMAND_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t st = (resp_len >= 1) ? resp[0] : 0xFF;
    ESP_LOGI(TAG, "setMulticastTableEntry[%u] group=%u (0x%04X) ep=%u -> 0x%02X", params[0],
             (unsigned)group_id, group_id, (unsigned)(endpoint ? endpoint : 1), st);
    if (st != EMBER_SUCCESS) {
        return ESP_FAIL;
    }
    /* MULTICAST bindings only for Touchlink remote groups — binding slots 8–15 are
     * scarce (2 eps × group). Binding 0/1/901 first starved Outdoors (33513) so
     * Level/Scenes never arrived as incomingMessage with direction payload. */
    static uint16_t s_bound_gids[4];
    static uint8_t s_bound_n;
    static uint8_t s_bind_idx = 8;
    bool touchlink = (group_id != 0 && group_id != 1 && group_id != ZB_IKEA_DEFAULT_BIND_GROUP);
    bool already = false;
    for (uint8_t i = 0; i < s_bound_n; i++) {
        if (s_bound_gids[i] == group_id) {
            already = true;
            break;
        }
    }
    if (touchlink && !already && s_bound_n < 4) {
        s_bound_gids[s_bound_n++] = group_id;
        uint8_t ep = endpoint ? endpoint : ZB_FAKE_BULB_ENDPOINT;
        for (int pass = 0; pass < 2; pass++) {
            uint8_t use_ep = (pass == 0) ? ep : 1;
            if (pass == 1 && use_ep == ep) {
                continue;
            }
            uint8_t bp[1 + 14];
            size_t pos = 0;
            bp[pos++] = s_bind_idx;
            s_bind_idx++;
            if (s_bind_idx < 8 || s_bind_idx > 15) {
                s_bind_idx = 8;
            }
            bp[pos++] = EMBER_MULTICAST_BINDING;
            bp[pos++] = use_ep;
            bp[pos++] = 0xFF;
            bp[pos++] = 0xFF;
            bp[pos++] = 0;
            bp[pos++] = (uint8_t)(group_id & 0xFF);
            bp[pos++] = (uint8_t)(group_id >> 8);
            for (int i = 0; i < 6; i++) {
                bp[pos++] = 0;
            }
            bp[pos++] = 0;
            uint8_t br[4];
            size_t bl = 0;
            if (ezsp_command(EZSP_FRAME_SET_BINDING, bp, pos, br, sizeof(br), &bl,
                             CONFIG_EZSP_COMMAND_TIMEOUT_MS) == ESP_OK) {
                ESP_LOGI(TAG, "setBinding multicast gid=%u ep=%u idx=%u -> 0x%02X",
                         (unsigned)group_id, (unsigned)use_ep, (unsigned)bp[0], bl ? br[0] : 0xFF);
            }
        }
    }
    return ESP_OK;
}

esp_err_t ezsp_join_ikea_multicast_groups(void)
{
    /* Compact base membership into slots 0–2 so 3–7 stay free for Touchlink groups.
     * (Many NCPs keep MULTICAST_TABLE_SIZE=8 even if we request 16.) */
    (void)ezsp_set_multicast_group(0, 0x0000, ZB_FAKE_BULB_ENDPOINT);
    (void)ezsp_set_multicast_group(1, 0x0001, ZB_FAKE_BULB_ENDPOINT);
    (void)ezsp_set_multicast_group(2, ZB_IKEA_DEFAULT_BIND_GROUP, ZB_FAKE_BULB_ENDPOINT);
    /* Re-apply Touchlink-learned groups into slots 3+. */
    for (uint8_t i = 0; i < s_learned_group_count && i < EZSP_LEARNED_GROUP_MAX; i++) {
        uint16_t gid = s_learned_groups[i];
        if (gid == 0) {
            continue;
        }
        /* Two slots per group: fake-bulb ep + ep1 (APS often dest-ep 1 or 255). */
        uint8_t base = (uint8_t)(3 + i * 2);
        if (base < 8) {
            (void)ezsp_set_multicast_group(base, gid, ZB_FAKE_BULB_ENDPOINT);
        }
        if (base + 1 < 8) {
            (void)ezsp_set_multicast_group((uint8_t)(base + 1), gid, 1);
        }
    }
    ESP_LOGI(TAG, "IKEA multicast: 0/1/901 on ep%u + %u Touchlink group(s) in slots 3–7",
             (unsigned)ZB_FAKE_BULB_ENDPOINT, (unsigned)s_learned_group_count);
    return ESP_OK;
}

esp_err_t ezsp_ensure_multicast_group(uint16_t group_id)
{
    if (group_id == 0) {
        return ESP_OK;
    }
    if (group_id == 0x0001 || group_id == ZB_IKEA_DEFAULT_BIND_GROUP) {
        return ezsp_join_ikea_multicast_groups();
    }
    for (uint8_t i = 0; i < s_learned_group_count; i++) {
        if (s_learned_groups[i] == group_id) {
            /* Already installed — avoid EZSP setMulticast spam on every button overhear. */
            return ESP_OK;
        }
    }
    /* Max 2 learned groups with ep11+ep1 (slots 3-6); 3rd uses slot 7 alone. */
    if (s_learned_group_count >= 2) {
        for (uint8_t i = 1; i < 2; i++) {
            s_learned_groups[i - 1] = s_learned_groups[i];
        }
        s_learned_group_count = 1;
    }
    uint8_t idx = s_learned_group_count++;
    s_learned_groups[idx] = group_id;
    uint8_t base = (uint8_t)(3 + idx * 2);
    ESP_LOGI(TAG, "Learning Touchlink group %u (0x%04X) on ep%u+ep1 (slots %u/%u)",
             (unsigned)group_id, group_id, (unsigned)ZB_FAKE_BULB_ENDPOINT, (unsigned)base,
             (unsigned)(base + 1));
    esp_err_t e1 = ezsp_set_multicast_group(base, group_id, ZB_FAKE_BULB_ENDPOINT);
    esp_err_t e2 = ESP_OK;
    if (base + 1 < 8) {
        e2 = ezsp_set_multicast_group((uint8_t)(base + 1), group_id, 1);
    }
    if (e1 != ESP_OK && e2 != ESP_OK) {
        ESP_LOGW(TAG, "setMulticastTableEntry failed for group %u — button RX will not work",
                 (unsigned)group_id);
        return ESP_FAIL;
    }
    return ESP_OK;
}

void ezsp_note_group_owner(uint16_t group_id, uint16_t node_id)
{
    if (group_id == 0 || node_id == 0 || node_id == 0xFFFF) {
        return;
    }
    for (uint8_t i = 0; i < s_group_owner_count; i++) {
        if (s_group_owner_gid[i] == group_id) {
            s_group_owner_node[i] = node_id;
            return;
        }
    }
    if (s_group_owner_count >= EZSP_LEARNED_GROUP_MAX) {
        s_group_owner_gid[0] = s_group_owner_gid[1];
        s_group_owner_node[0] = s_group_owner_node[1];
        s_group_owner_count = 1;
    }
    s_group_owner_gid[s_group_owner_count] = group_id;
    s_group_owner_node[s_group_owner_count] = node_id;
    s_group_owner_count++;
    ESP_LOGI(TAG, "Group %u (0x%04X) owned by remote 0x%04X", (unsigned)group_id, group_id,
             node_id);
}

bool ezsp_take_learned_group(uint16_t *group_out)
{
    bool got = false;
    uint16_t gid = 0;
    portENTER_CRITICAL(&s_learn_group_mux);
    if (s_learn_group_tail != s_learn_group_head) {
        gid = s_learn_group_q[s_learn_group_tail];
        s_learn_group_tail = (uint8_t)((s_learn_group_tail + 1) % EZSP_LEARN_GROUP_Q_LEN);
        got = true;
    }
    portEXIT_CRITICAL(&s_learn_group_mux);
    if (got && group_out) {
        *group_out = gid;
    }
    return got;
}

bool ezsp_take_network_up_event(void)
{
    bool v = s_network_up_event;
    s_network_up_event = false;
    return v;
}

bool ezsp_take_poll_event(uint16_t *node_id_out)
{
    bool got = false;
    portENTER_CRITICAL(&s_poll_mux);
    if (s_poll_q_tail != s_poll_q_head) {
        if (node_id_out) {
            *node_id_out = s_poll_q[s_poll_q_tail];
        }
        s_poll_q_tail = (uint8_t)((s_poll_q_tail + 1) % EZSP_POLL_Q_LEN);
        got = true;
    }
    portEXIT_CRITICAL(&s_poll_mux);
    return got;
}

bool ezsp_take_sent_event(uint16_t *node_id_out, uint16_t *cluster_out, uint8_t *status_out)
{
    bool got = false;
    portENTER_CRITICAL(&s_sent_mux);
    if (s_sent_q_tail != s_sent_q_head) {
        if (node_id_out) {
            *node_id_out = s_sent_q[s_sent_q_tail].node;
        }
        if (cluster_out) {
            *cluster_out = s_sent_q[s_sent_q_tail].cluster;
        }
        if (status_out) {
            *status_out = s_sent_q[s_sent_q_tail].status;
        }
        s_sent_q_tail = (uint8_t)((s_sent_q_tail + 1) % EZSP_SENT_Q_LEN);
        got = true;
    }
    portEXIT_CRITICAL(&s_sent_mux);
    return got;
}
