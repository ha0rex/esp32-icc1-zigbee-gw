#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PEER_LINK_PORT 41240
#define PEER_DEV_MAX 16

typedef struct {
    const char *role; /**< "thread" or "zigbee" */
    const char *name;
    /** Write a JSON array of local devices the other gateway may use. */
    int (*write_devices)(char *buf, size_t len);
    /** Turn a local on/off device on or off. id is the device id string. */
    esp_err_t (*set_on)(const char *id, bool on);
} peer_link_cfg_t;

typedef struct {
    char id[24];
    char name[40];
    char kind[16];
    bool has_temp;
    float temp_c;
    bool has_onoff;
    bool on;
    bool has_contact;
    bool contact_open;
    bool ok;
} peer_dev_t;

void peer_link_start(const peer_link_cfg_t *cfg);

/** JSON object for the System tab. Includes link state and remote devices. */
int peer_link_status_json(char *buf, size_t len);

/** Broadcast a hello and keep replies that arrive over the next moment. */
esp_err_t peer_link_discover(void);

/** Ask the gateway at this IPv4 address to show an Accept prompt. */
esp_err_t peer_link_pair(const char *host);

esp_err_t peer_link_accept(void);
esp_err_t peer_link_decline(void);
esp_err_t peer_link_unpair(void);

/** Called when the other gateway posts an invite / confirm / goodbye. */
esp_err_t peer_link_on_invite(const char *id, const char *name, const char *role,
                              const char *host, const char *token);
esp_err_t peer_link_on_confirm(const char *id, const char *name, const char *role,
                               const char *host, const char *token);
esp_err_t peer_link_on_bye(const char *token);

bool peer_link_token_ok(const char *token);

int peer_link_write_local_devices(char *buf, size_t len);
esp_err_t peer_link_set_local(const char *id, bool on);

/** Refresh the remote device cache. Safe to call when unpaired. */
void peer_link_refresh(void);

bool peer_link_device(const char *id, peer_dev_t *out);
int peer_link_copy_devices(peer_dev_t *out, int max);

/** Command a device on the linked gateway. */
esp_err_t peer_link_set_remote(const char *id, bool on);

#ifdef __cplusplus
}
#endif
