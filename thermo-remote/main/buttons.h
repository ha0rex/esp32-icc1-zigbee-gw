#pragma once

typedef enum {
    BTN_NONE = 0,
    BTN_LEFT,
    BTN_RIGHT,
    BTN_CENTER,
    BTN_FORGET,
} btn_ev_t;

void buttons_init(void);
btn_ev_t buttons_poll(void);
