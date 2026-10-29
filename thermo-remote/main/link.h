#pragma once

#include <stdbool.h>
#include <stdint.h>

#define LINK_MAX 8

typedef struct {
    uint8_t id;
    char name[24];
    bool has_current;
    float current_c;
    float target_c;
} link_thermo_t;

typedef struct {
    link_thermo_t items[LINK_MAX];
    int count;
} link_list_t;

void link_init(void);
bool link_have_creds(void);
const char *link_ssid(void);
bool link_sta_up(void);
bool link_have_gateway(void);

/** Blocks until the setup form is saved. */
void link_portal_until_saved(void);
bool link_connect(int timeout_ms);
bool link_discover(void);
bool link_fetch(link_list_t *out);
bool link_set_target(uint8_t id, float target_c);
bool link_post_climate(float temp_c, float humidity);
void link_forget(void);
