#include "ui_ota.h"
#include "ui/ui.h"

lv_obj_t *ui_ScreenOTA = nullptr;
static lv_obj_t *otaCard = nullptr;
static lv_obj_t *otaTitleLabel = nullptr;
static lv_obj_t *otaBadgeLabel = nullptr;
static lv_obj_t *otaPercentLabel = nullptr;
static lv_obj_t *otaProgressBar = nullptr;
static lv_obj_t *otaDetailLabel = nullptr;
static lv_obj_t *otaWarningLabel = nullptr;

void ui_ota_init(void) {
    if (ui_ScreenOTA != nullptr) return;

    // Create Screen with Dark Navy Background
    ui_ScreenOTA = lv_obj_create(NULL);
    lv_obj_remove_flag(ui_ScreenOTA, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(ui_ScreenOTA, lv_color_hex(0x090D16), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ScreenOTA, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Center Dialog Card
    otaCard = lv_obj_create(ui_ScreenOTA);
    lv_obj_set_size(otaCard, 680, 380);
    lv_obj_center(otaCard);
    lv_obj_remove_flag(otaCard, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(otaCard, lv_color_hex(0x131C2E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(otaCard, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(otaCard, lv_color_hex(0x0284C7), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(otaCard, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(otaCard, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(otaCard, 24, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(otaCard, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(otaCard, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Top Header Row
    lv_obj_t *headerRow = lv_obj_create(otaCard);
    lv_obj_remove_flag(headerRow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(headerRow, LV_PCT(100), 45);
    lv_obj_set_style_bg_opa(headerRow, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(headerRow, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(headerRow, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(headerRow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(headerRow, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    otaTitleLabel = lv_label_create(headerRow);
    lv_label_set_text(otaTitleLabel, "FIRMWARE OVER-THE-AIR UPDATE");
    lv_obj_set_style_text_color(otaTitleLabel, lv_color_hex(0xF8FAFC), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(otaTitleLabel, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *badge = lv_obj_create(headerRow);
    lv_obj_remove_flag(badge, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(badge, LV_SIZE_CONTENT, 32);
    lv_obj_set_style_bg_color(badge, lv_color_hex(0x0284C7), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(badge, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(badge, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(badge, 16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_hor(badge, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_ver(badge, 4, LV_PART_MAIN | LV_STATE_DEFAULT);

    otaBadgeLabel = lv_label_create(badge);
    lv_label_set_text(otaBadgeLabel, "UPDATING...");
    lv_obj_set_style_text_color(otaBadgeLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(otaBadgeLabel, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Percentage Label
    otaPercentLabel = lv_label_create(otaCard);
    lv_label_set_text(otaPercentLabel, "0 %");
    lv_obj_set_style_text_color(otaPercentLabel, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(otaPercentLabel, &ui_font_Chakra50, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Progress Bar
    otaProgressBar = lv_bar_create(otaCard);
    lv_obj_set_size(otaProgressBar, 610, 26);
    lv_bar_set_range(otaProgressBar, 0, 100);
    lv_bar_set_value(otaProgressBar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(otaProgressBar, lv_color_hex(0x0B1120), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(otaProgressBar, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(otaProgressBar, 13, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(otaProgressBar, lv_color_hex(0x334155), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(otaProgressBar, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(otaProgressBar, lv_color_hex(0x0284C7), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(otaProgressBar, lv_color_hex(0x38BDF8), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(otaProgressBar, LV_GRAD_DIR_HOR, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(otaProgressBar, 13, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    // Detail Label (Bytes Transferred)
    otaDetailLabel = lv_label_create(otaCard);
    lv_label_set_text(otaDetailLabel, "Connecting & preparing flash partition...");
    lv_obj_set_style_text_color(otaDetailLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(otaDetailLabel, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Warning Banner
    otaWarningLabel = lv_label_create(otaCard);
    lv_label_set_text(otaWarningLabel, "! DO NOT POWER OFF OR DISCONNECT WI-FI !");
    lv_obj_set_style_text_color(otaWarningLabel, lv_color_hex(0xF59E0B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(otaWarningLabel, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
}

void ui_ota_show(const char *updateType) {
    ui_ota_init();
    if (!ui_ScreenOTA) return;

    if (otaTitleLabel) {
        char title[64];
        snprintf(title, sizeof(title), "OTA UPDATE (%s)", updateType ? updateType : "Firmware");
        lv_label_set_text(otaTitleLabel, title);
    }
    if (otaBadgeLabel) {
        lv_label_set_text(otaBadgeLabel, "UPDATING");
        lv_obj_set_style_bg_color(lv_obj_get_parent(otaBadgeLabel), lv_color_hex(0x0284C7), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (otaPercentLabel) {
        lv_label_set_text(otaPercentLabel, "UPDATING...");
        lv_obj_set_style_text_color(otaPercentLabel, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(otaPercentLabel, &ui_font_Chakra50, LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (otaProgressBar) {
        lv_bar_set_value(otaProgressBar, 100, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(otaProgressBar, lv_color_hex(0x0284C7), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(otaProgressBar, lv_color_hex(0x38BDF8), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    }
    if (otaDetailLabel) {
        lv_label_set_text(otaDetailLabel, "Receiving and writing firmware... Please wait");
        lv_obj_set_style_text_color(otaDetailLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (otaWarningLabel) {
        lv_label_set_text(otaWarningLabel, "! DO NOT POWER OFF - ESP32 WILL REBOOT AUTOMATICALLY !");
        lv_obj_set_style_text_color(otaWarningLabel, lv_color_hex(0xF59E0B), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    lv_screen_load(ui_ScreenOTA);
}

void ui_ota_update_progress(uint32_t progress, uint32_t total, uint32_t pct) {
    if (!ui_ScreenOTA) return;

    if (otaPercentLabel) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%u %%", pct);
        lv_label_set_text(otaPercentLabel, buf);
    }
    if (otaProgressBar) {
        lv_bar_set_value(otaProgressBar, pct, LV_ANIM_OFF);
    }
    if (otaDetailLabel) {
        char buf[64];
        if (total > 0) {
            snprintf(buf, sizeof(buf), "Transferred: %u KB / %u KB (%u%%)", progress / 1024, total / 1024, pct);
        } else {
            snprintf(buf, sizeof(buf), "Transferred: %u KB", progress / 1024);
        }
        lv_label_set_text(otaDetailLabel, buf);
    }
}

void ui_ota_show_complete(void) {
    if (!ui_ScreenOTA) return;

    if (otaBadgeLabel) {
        lv_label_set_text(otaBadgeLabel, "SUCCESS");
    }
    if (otaPercentLabel) {
        lv_label_set_text(otaPercentLabel, "100 %");
        lv_obj_set_style_text_color(otaPercentLabel, lv_color_hex(0x22C55E), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (otaProgressBar) {
        lv_bar_set_value(otaProgressBar, 100, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(otaProgressBar, lv_color_hex(0x16A34A), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(otaProgressBar, lv_color_hex(0x22C55E), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    }
    if (otaDetailLabel) {
        lv_label_set_text(otaDetailLabel, "Update Complete! Rebooting system...");
        lv_obj_set_style_text_color(otaDetailLabel, lv_color_hex(0x22C55E), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (otaWarningLabel) {
        lv_label_set_text(otaWarningLabel, "System is restarting, please wait a moment.");
        lv_obj_set_style_text_color(otaWarningLabel, lv_color_hex(0x86EFAC), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

void ui_ota_show_error(const char *errorMsg) {
    if (!ui_ScreenOTA) return;

    if (otaBadgeLabel) {
        lv_label_set_text(otaBadgeLabel, "FAILED");
    }
    if (otaPercentLabel) {
        lv_obj_set_style_text_color(otaPercentLabel, lv_color_hex(0xEF4444), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (otaProgressBar) {
        lv_obj_set_style_bg_color(otaProgressBar, lv_color_hex(0xDC2626), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(otaProgressBar, lv_color_hex(0xEF4444), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    }
    if (otaDetailLabel) {
        char buf[80];
        snprintf(buf, sizeof(buf), "Error: %s", errorMsg ? errorMsg : "Unknown OTA failure");
        lv_label_set_text(otaDetailLabel, buf);
        lv_obj_set_style_text_color(otaDetailLabel, lv_color_hex(0xEF4444), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (otaWarningLabel) {
        lv_label_set_text(otaWarningLabel, "Update aborted. System returning to normal operation.");
        lv_obj_set_style_text_color(otaWarningLabel, lv_color_hex(0xFCA5A5), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}
