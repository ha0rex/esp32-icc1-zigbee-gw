#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Start lightweight HTTP status server (embedded HTML, no CDN). */
esp_err_t web_server_start(void);

#ifdef __cplusplus
}
#endif
