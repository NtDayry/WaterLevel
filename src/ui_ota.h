#pragma once

#include <Arduino.h>
#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern lv_obj_t *ui_ScreenOTA;

void ui_ota_init(void);
void ui_ota_show(const char *updateType);
void ui_ota_update_progress(uint32_t progress, uint32_t total, uint32_t pct);
void ui_ota_show_complete(void);
void ui_ota_show_error(const char *errorMsg);

#ifdef __cplusplus
}
#endif
