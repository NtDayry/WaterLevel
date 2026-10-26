#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <LovyanGFX.hpp>
#include <lvgl.h>

#if defined(LGFX_LCD2_3)
#include "../gfx/LGFX_ESP32S3_RGB_MakerfabsParallelTFTwithTouch2_3.h"

#elif defined(LGFX_LCD4_3)
#include "../gfx/LGFX_ESP32S3_RGB_MakerfabsParallelTFTwithTouch4_3.h"

#elif defined(LGFX_LCD4_3_B)
#include "../gfx/LGFX_ESP32S3_RGB_MakerfabsParallelTFTwithTouch4_3_B.h"

#elif defined(LGFX_LCD7_0)
#include "../gfx/LGFX_ESP32S3_RGB_MakerfabsParallelTFTwithTouch7_0.h"

#endif

LGFX gfx;

#define LVGL_PORT_TASK_MAX_DELAY_MS (500)
#define LVGL_PORT_TASK_MIN_DELAY_MS (1)
// 48 lines = 800 * 48 * 2 = 76,800 bytes per buffer.
// Fits completely within ESP32-S3 internal DMA SRAM for true Double-Buffering!
#define LVGL_BUF_LINES (48)
#define LVGL_BUF_SIZE (800 * LVGL_BUF_LINES)

static lv_display_t *disp_handle = NULL;
static lv_indev_t *indev_handle = NULL;
static uint8_t *buf1 = NULL;
static uint8_t *buf2 = NULL;
static SemaphoreHandle_t lvgl_mux = NULL;

// ------------------ TICK ------------------
static uint32_t my_tick_get(void) {
    return (uint32_t)millis();
}

// ------------------ LOCK ------------------
inline bool lvgl_port_lock(int timeout_ms) {
    assert(lvgl_mux && "lvgl_port_init must be called first");
    TickType_t t = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, t) == pdTRUE;
}

inline bool lvgl_port_unlock() {
    assert(lvgl_mux && "lvgl_port_init must be called first");
    return xSemaphoreGiveRecursive(lvgl_mux) == pdTRUE;
}

// ------------------ FLUSH ------------------
static void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;

    gfx.pushImageDMA(area->x1, area->y1, w, h, (const lgfx::rgb565_t *)px_map);
    lv_display_flush_ready(disp);
}

// ------------------ TOUCH ------------------
static void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
    uint16_t x, y;

    if (gfx.getTouch(&x, &y)) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ------------------ TASK ------------------
static void lvgl_task(void *arg) {
    uint32_t delay_ms = 1;

    while (1) {
        if (xSemaphoreTakeRecursive(lvgl_mux, portMAX_DELAY)) {
            delay_ms = lv_timer_handler();
            xSemaphoreGiveRecursive(lvgl_mux);
        }

        // Fast clamp for responsive 60 FPS UI
        if (delay_ms < 1)
            delay_ms = 1;
        if (delay_ms > 10)
            delay_ms = 10;

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// ------------------ INIT ------------------
inline void lvgl_port_init() {
    gfx.begin();
    gfx.setBrightness(128);

    lv_init();
    lv_tick_set_cb(my_tick_get);

    // Allocate in DMA-capable internal SRAM for maximum speed
    const uint32_t buf_byte_size = LVGL_BUF_SIZE * sizeof(uint16_t);
    buf1 = (uint8_t *)heap_caps_malloc(buf_byte_size, MALLOC_CAP_DMA);
    buf2 = (uint8_t *)heap_caps_malloc(buf_byte_size, MALLOC_CAP_DMA);

    assert(buf1);

    disp_handle = lv_display_create(800, 480);
    lv_display_set_color_format(disp_handle, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp_handle, my_disp_flush);

    if (buf2) {
        lv_display_set_buffers(disp_handle, buf1, buf2, buf_byte_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
        Serial.println("🔥 LVGL 9.5: TRUE DOUBLE BUFFER ENABLED (DMA)");
    } else {
        lv_display_set_buffers(disp_handle, buf1, NULL, buf_byte_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
        Serial.println("⚠️ LVGL 9.5: SINGLE BUFFER FALLBACK");
    }

    // Touch driver
    indev_handle = lv_indev_create();
    lv_indev_set_type(indev_handle, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_handle, my_touchpad_read);

    // Recursive Mutex
    lvgl_mux = xSemaphoreCreateRecursiveMutex();

    // Run LVGL task on Core 1 with high priority
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 10240, NULL, 3, NULL, 1);
}
