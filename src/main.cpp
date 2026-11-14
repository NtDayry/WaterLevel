/**
 * Waveshare ESP32-S3-Touch-LCD-4.3B - Multi-Core High-Performance Industrial Firmware
 *
 * FreeRTOS Dual-Core Architecture:
 *   - CORE 1: Dedicated to Display Rendering (LovyanGFX ST7701 800x480), Touch (GT911) & LVGL 8.3 UI (60 FPS)
 *   - CORE 0: Dedicated Background Task for RS485 (Modbus RTU), CAN Bus (TWAI) & IO Expander polling
 *
 * Optimizations Applied:
 *   1. Hardware I2C touch frequency raised to 400kHz (8x faster touch response)
 *   2. Bus-shared flag enabled for safe I2C arbitration between GT911 and CH422G
 *   3. DMA SRAM buffer tuned to 48 lines enabling TRUE DOUBLE BUFFERING
 *   4. IO Expander polling throttled to 100ms (frees 90% of I2C bus bandwidth for touch)
 *   5. UI TextArea log trimming prevents memory bloat and font re-layout lag
 */

#define LGFX_LCD4_3_B

#include "lvgl_port/lvgl_port.h"
#include "ui/ui.h"
#include "ui_ota.h"
#include "waveshare_ch422g.h"
#include "waveshare_sdcard.h"
#include <Arduino.h>
#include <ModbusMaster.h>
#include "water_settings.h"
#include <lv_conf.h>
#include <lvgl.h>
#include <math.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "modbus_config.h"

// Include Waveshare 4.3B Library (RS485, CAN, I2C, IO Expander)
#include <WaveShare4_3B.h>
#include "pcf85063.h"

ModbusMaster modbus;
ModbusMaster rs485Io;
#if WATER_METER_SENSOR_PRESENT
ModbusMaster waterMeter;
#endif

// FreeRTOS Task and Queue Handles
TaskHandle_t commTaskHandle = NULL;
QueueHandle_t logQueue = NULL;

// Asynchronous Request Flags (from UI to Core 0)
volatile bool requestScanModbus = false;
volatile int8_t pendingCoilIndex = -1;
volatile bool pendingCoilState = false;
volatile uint8_t pendingIoMask = 0;

// Shared State between Core 0 and Core 1
volatile bool shared_di0 = false;
volatile bool shared_di1 = false;
volatile bool di0_changed = false;
volatile bool di1_changed = false;
volatile uint16_t shared_reg0 = 1500;
volatile uint16_t shared_reg1 = 0;
volatile bool reg_data_updated = false;

// Water meter sensor data (Slave ID 1, Reg 0-4)
volatile uint16_t shared_meter_regs[5] = {0};
volatile bool meter_data_updated = false;
volatile bool water_meter_ok = false;
volatile float shared_flow_rate = 0.0f;
volatile float shared_volume_l = 0.0f;

volatile uint8_t shared_rs485_inputs = 0;
volatile uint8_t shared_rs485_outputs = 0;
volatile bool shared_pressure_switch = false;
volatile bool shared_pump1_overload = false;
volatile bool shared_pump2_overload = false;
volatile bool rs485_io_ok = false;
// The UI shadow is the source of truth for output commands. After a
// reconnect, outputs are reset to a safe all-off state.
volatile bool rs485_output_synced = false;
volatile bool sensor_rs485_ok = false;

// Pressure Transmitter (Slave ID 6, Reg 0, 0-6.00 bar)
volatile float shared_pressure_bar = 0.0f;
volatile bool pressure_sensor_ok = false;

// Inverter 1 (OMRON SYSDRIVE 3G3JZ, Slave ID 4 - Pump 1)
volatile float shared_pump1_freq = 0.0f;      // Hz
volatile float shared_pump1_current = 0.0f;   // A
volatile uint16_t shared_pump1_status = 0;
volatile bool shared_pump1_running = false;
volatile bool shared_pump1_alarm = false;
volatile bool pump1_modbus_ok = false;

// Inverter 2 (OMRON SYSDRIVE 3G3JZ, Slave ID 5 - Pump 2)
volatile float shared_pump2_freq = 0.0f;      // Hz
volatile float shared_pump2_current = 0.0f;   // A
volatile uint16_t shared_pump2_status = 0;
volatile bool shared_pump2_running = false;
volatile bool shared_pump2_alarm = false;
volatile bool pump2_modbus_ok = false;

// Inverter PID control target frequency calculated on ESP32
volatile float shared_pid_target_freq = 0.0f;
volatile bool shared_is_sleeping = false;

// RS485 input & output coil assignments. Keep the hardware mapping in one place so
// the code does not depend on hard-coded bit positions throughout the UI and
// control logic.

// RS485 input coil assignments (mapped to CH422G IO pins)
static constexpr uint8_t rs485_di1_io_pin = 0;
static constexpr uint8_t rs485_di2_io_pin = 1;
static constexpr uint8_t rs485_di3_io_pin = 2;
static constexpr uint8_t rs485_di4_io_pin = 3;
static constexpr uint8_t rs485_di5_io_pin = 4;
static constexpr uint8_t rs485_di6_io_pin = 5;
static constexpr uint8_t rs485_di7_io_pin = 6;
static constexpr uint8_t rs485_di8_io_pin = 7;
// RS485 output coil assignments (mapped to CH422G IO pins)
static constexpr uint8_t rs485_ch1_io_pin = 0;
static constexpr uint8_t rs485_ch2_io_pin = 1;
static constexpr uint8_t rs485_ch3_io_pin = 2;
static constexpr uint8_t rs485_ch4_io_pin = 3;
static constexpr uint8_t rs485_ch5_io_pin = 4;
static constexpr uint8_t rs485_ch6_io_pin = 5;
static constexpr uint8_t rs485_ch7_io_pin = 6;
static constexpr uint8_t rs485_ch8_io_pin = 7;

// RS485 input/output mapping for the water pump control system
static constexpr uint8_t pressure_switch_io_pin = rs485_di1_io_pin;
static constexpr uint8_t pump1_overload_io_pin = rs485_di2_io_pin;
static constexpr uint8_t pump2_overload_io_pin = rs485_di3_io_pin;

// RS485 output coil mapping (Relay / DO module)
static constexpr uint8_t pump1_run_io_pin   = rs485_ch1_io_pin; // CH1: ไฟสถานะรัน pump1
static constexpr uint8_t pump2_run_io_pin   = rs485_ch2_io_pin; // CH2: ไฟสถานะรัน pump2
static constexpr uint8_t pump1_alarm_io_pin = rs485_ch3_io_pin; // CH3: Alarm ของ pump1
static constexpr uint8_t pump2_alarm_io_pin = rs485_ch4_io_pin; // CH4: Alarm ของ pump2
static constexpr uint8_t alarm_io_pin       = rs485_ch5_io_pin; // CH5: Alarm ของระบบ alarm_io_pin
static constexpr uint8_t valve_io_pin       = rs485_ch6_io_pin; // CH6: valve_io_pin

// Compatibility aliases
static constexpr uint8_t pump1_io_pin = pump1_run_io_pin;
static constexpr uint8_t pump2_io_pin = pump2_run_io_pin;

// Water-level gauge created inside the SquareLine placeholder object.
static lv_chart_series_t *waterLevelChartSeries = NULL;
static lv_chart_series_t *flowRateChartSeries = NULL;
static uint32_t lastSimulationUpdateMs = 0;
static float simulatedLevelPct = 60.0f;

static bool manualValveOverride = false;
static uint32_t alertLowSince = 0;
static uint32_t alertHighSince = 0;
static bool alertOutputOn = false;
static bool lastRs485Ok = false;

static float configuredLevelMaxCm();
static void set_output_state(uint8_t output, bool on);
static void set_rs485_output(uint8_t coil, bool on);
static void update_pump_status_outputs();
static void update_auto_pumps_from_pressure(float levelCm = -1.0f);

static Pcf85063 rtc;
static lv_obj_t *rtcClockLabel = nullptr;
static lv_obj_t *rtcStatusLabel = nullptr;
static lv_obj_t *rtcFields[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
static uint32_t lastRtcUpdateMs = 0;
static lv_obj_t *wifiSsidInput = nullptr;
static lv_obj_t *wifiPasswordInput = nullptr;
static lv_obj_t *wifiIpInput = nullptr;
static lv_obj_t *wifiMaskInput = nullptr;
static lv_obj_t *wifiGatewayInput = nullptr;
static lv_obj_t *wifiDnsInput = nullptr;
static lv_obj_t *wifiFixIpSwitch = nullptr;
static lv_obj_t *wifiStaticIpContainer = nullptr;
static lv_obj_t *wifiStatusLabel = nullptr;
static lv_obj_t *wifiScanList = nullptr;
static char scannedSsids[20][33] = {};
static bool wifiScanBusy = false;
static bool wifiInitialized = false;
static Preferences wifiPreferences;

// MQTT Client & Configuration
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static Preferences mqttPreferences;
static lv_obj_t *mqttServerInput = nullptr;
static lv_obj_t *mqttPortInput = nullptr;
static lv_obj_t *mqttTopicInput = nullptr;
static lv_obj_t *mqttClientIdInput = nullptr;
static lv_obj_t *mqttUserInput = nullptr;
static lv_obj_t *mqttPassInput = nullptr;
static lv_obj_t *mqttEnableSwitch = nullptr;
static lv_obj_t *mqttStatusLabel = nullptr;
static bool mqttEnabled = true;
static uint32_t lastMqttPublishMs = 0;
static uint32_t lastMqttReconnectMs = 0;
static uint32_t mqttPublishIntervalMs = 1000; // 1 second

// Power & System Settings (Auto Backlight & Auto Restart)
static Preferences sysPreferences;
volatile bool autoBacklightEnabled = false;
volatile uint32_t autoBacklightMin = 5;
volatile bool autoRestartEnabled = false;
volatile uint32_t autoRestartMin = 1440;
volatile bool isBacklightOn = true;
volatile uint32_t lastUserActivityMs = 0;
uint32_t bootTimeMs = 0;
volatile uint32_t autoRestartStartMs = 0;

static lv_obj_t *autoBacklightSwitch = nullptr;
static lv_obj_t *autoBacklightInput = nullptr;
static lv_obj_t *autoRestartSwitch = nullptr;
static lv_obj_t *autoRestartInput = nullptr;
static lv_obj_t *sysStatusLabel = nullptr;

// Wi-Fi Enable control
static lv_obj_t *wifiEnableSwitch = nullptr;

void on_touch_wakeup() {
    lastUserActivityMs = millis();
    if (!isBacklightOn) {
        BL_ON;
        isBacklightOn = true;
        Serial.println("[SYSTEM] Touch detected -> Backlight ON");
    }
}

static bool is_sys_input(lv_obj_t *obj) {
    return obj == autoBacklightInput || obj == autoRestartInput;
}

// Forward declarations for Wi-Fi helpers defined later in this file
static bool init_wifi_once();
static void connect_wifi_from_saved();

static void wifi_enable_toggle(lv_event_t *) {
    const bool enabled = wifiEnableSwitch && lv_obj_has_state(wifiEnableSwitch, LV_STATE_CHECKED);

    // Persist enable state to Preferences
    wifiPreferences.begin("wifi", false);
    wifiPreferences.putBool("enabled", enabled);
    wifiPreferences.end();

    if (enabled) {
        // Reset flag so init_wifi_once() can reinitialise the radio after WIFI_OFF
        wifiInitialized = false;
        init_wifi_once();
        connect_wifi_from_saved();
        Serial.println("[WiFi] Enabled by user");
    } else {
        mqttClient.disconnect();
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        wifiInitialized = false;  // allow re-init next time the user enables Wi-Fi
        Serial.println("[WiFi] Disabled by user");
        if (wifiStatusLabel) {
            lv_label_set_text(wifiStatusLabel, "Wi-Fi: Disabled");
            lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

static void brightness_slider_event(lv_event_t *) { /* slider removed */ }

static void save_system_settings_from_ui(lv_event_t *e = nullptr);
static void auto_backlight_toggle(lv_event_t *e = nullptr);
static void auto_restart_toggle(lv_event_t *e = nullptr);
static void load_system_settings();
static void update_system_status_label();
static void update_settings_ui();
static void update_mode_ui();
static void update_manual_leds(bool force = false);
static void publish_mqtt_data();
static void publish_mqtt_settings();
static void publish_mqtt_configs();

static void set_status_led(lv_obj_t *led, bool enabled, uint32_t on_color);
static void set_pump_led(lv_obj_t *led, bool running, bool alarm);
static uint32_t boundedUint(const char *text, uint32_t fallback, uint32_t minValue, uint32_t maxValue);
static void open_numeric_keyboard(lv_event_t *e);
static void select_wifi_clock(lv_event_t *e);
static bool is_wifi_input(lv_obj_t *obj);
static float configuredLevelMaxCm();

static bool is_rtc_input(lv_obj_t *obj) {
    for (lv_obj_t *field : rtcFields)
        if (obj == field) return true;
    return false;
}

static void set_rtc_field(uint8_t index, uint16_t value) {
    if (!rtcFields[index]) return;
    char text[8];
    snprintf(text, sizeof(text), index == 0 ? "%04u" : "%02u", value);
    lv_textarea_set_text(rtcFields[index], text);
}

static void update_rtc_ui() {
    if (!rtc.available()) {
        if (rtcClockLabel) lv_label_set_text(rtcClockLabel, "RTC --:--:--");
        if (rtcStatusLabel) lv_label_set_text(rtcStatusLabel, "PCF85063 not detected at I2C 0x51");
        return;
    }
    RtcDateTime now;
    if (!rtc.read(now)) {
        if (rtcClockLabel) lv_label_set_text(rtcClockLabel, "RTC --:--:--");
        if (rtcStatusLabel) lv_label_set_text(rtcStatusLabel, "RTC read error");
        return;
    }
    char clockText[24];
    snprintf(clockText, sizeof(clockText), "%02u/%02u/%04u %02u:%02u:%02u", now.day, now.month, now.year,
             now.hour, now.minute, now.second);
    if (rtcClockLabel) lv_label_set_text(rtcClockLabel, clockText);
    if (rtcStatusLabel) lv_label_set_text(rtcStatusLabel, "RTC online  |  PCF85063 0x51");
}

static void read_rtc_to_fields(lv_event_t * = nullptr) {
    if (!rtc.available()) return;
    RtcDateTime now;
    if (rtc.read(now)) {
        set_rtc_field(0, now.year);
        set_rtc_field(1, now.month);
        set_rtc_field(2, now.day);
        set_rtc_field(3, now.hour);
        set_rtc_field(4, now.minute);
        set_rtc_field(5, now.second);
        if (rtcStatusLabel) {
            lv_label_set_text(rtcStatusLabel, "RTC time loaded into fields");
            lv_obj_set_style_text_color(rtcStatusLabel, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

static void save_rtc_from_ui(lv_event_t *) {
    RtcDateTime value;
    value.year = (uint16_t)boundedUint(lv_textarea_get_text(rtcFields[0]), 2026, 2000, 2099);
    value.month = (uint8_t)boundedUint(lv_textarea_get_text(rtcFields[1]), 1, 1, 12);
    value.day = (uint8_t)boundedUint(lv_textarea_get_text(rtcFields[2]), 1, 1, 31);
    value.hour = (uint8_t)boundedUint(lv_textarea_get_text(rtcFields[3]), 0, 0, 23);
    value.minute = (uint8_t)boundedUint(lv_textarea_get_text(rtcFields[4]), 0, 0, 59);
    value.second = (uint8_t)boundedUint(lv_textarea_get_text(rtcFields[5]), 0, 0, 59);
    if (rtc.write(value)) {
        Serial.println("[RTC] Date/time updated");
        update_rtc_ui();
        if (rtcStatusLabel) {
            lv_label_set_text(rtcStatusLabel, "Date/time updated successfully");
            lv_obj_set_style_text_color(rtcStatusLabel, lv_color_hex(0x22C55E), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    } else if (rtcStatusLabel) {
        lv_label_set_text(rtcStatusLabel, "Unable to write RTC; check wiring");
        lv_obj_set_style_text_color(rtcStatusLabel, lv_color_hex(0xEF4444), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

// =============================================================================
// System Settings (Auto Backlight / Auto Restart) - Persistence & Logic
// =============================================================================
static void update_system_status_label() {
    if (!sysStatusLabel) return;
    char buf[80];
    if (autoBacklightEnabled)
        snprintf(buf, sizeof(buf), "Backlight off after %lu min | ", autoBacklightMin);
    else
        snprintf(buf, sizeof(buf), "Backlight: Always ON | ");
    if (autoRestartEnabled) {
        char tmp[40];
        snprintf(tmp, sizeof(tmp), "Restart after %lu min", autoRestartMin);
        strncat(buf, tmp, sizeof(buf) - strlen(buf) - 1);
    } else {
        strncat(buf, "Restart: Disabled", sizeof(buf) - strlen(buf) - 1);
    }
    lv_label_set_text(sysStatusLabel, buf);
    lv_obj_set_style_text_color(sysStatusLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void load_system_settings() {
    sysPreferences.begin("syscfg", false);
    autoBacklightEnabled = sysPreferences.getBool("autoBL", false);
    autoBacklightMin     = sysPreferences.getUInt("blMin", 5);
    autoRestartEnabled   = sysPreferences.getBool("autoRS", false);
    autoRestartMin       = sysPreferences.getUInt("rsMin", 1440);
    sysPreferences.end();

    // Apply to UI
    if (autoBacklightSwitch)
        lv_obj_set_state(autoBacklightSwitch, LV_STATE_CHECKED, autoBacklightEnabled);
    if (autoBacklightInput) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%lu", autoBacklightMin);
        lv_textarea_set_text(autoBacklightInput, buf);
    }
    if (autoRestartSwitch)
        lv_obj_set_state(autoRestartSwitch, LV_STATE_CHECKED, autoRestartEnabled);
    if (autoRestartInput) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%lu", autoRestartMin);
        lv_textarea_set_text(autoRestartInput, buf);
    }

    lastUserActivityMs = millis();
    autoRestartStartMs = millis();
    bootTimeMs = millis();
    update_system_status_label();
}

static void auto_backlight_toggle(lv_event_t *) {
    const bool enabled = autoBacklightSwitch && lv_obj_has_state(autoBacklightSwitch, LV_STATE_CHECKED);
    autoBacklightEnabled = enabled;
    if (enabled) {
        lastUserActivityMs = millis();
        if (!isBacklightOn) {
            BL_ON;
            isBacklightOn = true;
        }
        Serial.printf("[SYS] Auto Backlight turned ON -> timer started (%lu min)\n", autoBacklightMin);
    } else {
        Serial.println("[SYS] Auto Backlight turned OFF");
    }

    sysPreferences.begin("syscfg", false);
    sysPreferences.putBool("autoBL", autoBacklightEnabled);
    sysPreferences.end();

    update_system_status_label();
}

static void auto_restart_toggle(lv_event_t *) {
    const bool enabled = autoRestartSwitch && lv_obj_has_state(autoRestartSwitch, LV_STATE_CHECKED);
    autoRestartEnabled = enabled;
    if (enabled) {
        autoRestartStartMs = millis();
        Serial.printf("[SYS] Auto Restart turned ON -> timer started (%lu min)\n", autoRestartMin);
    } else {
        Serial.println("[SYS] Auto Restart turned OFF");
    }

    sysPreferences.begin("syscfg", false);
    sysPreferences.putBool("autoRS", autoRestartEnabled);
    sysPreferences.end();

    update_system_status_label();
}

static void save_system_settings_from_ui(lv_event_t *e) {
    const bool prevAutoBL = autoBacklightEnabled;
    const bool prevAutoRS = autoRestartEnabled;
    const uint32_t prevBLMin = autoBacklightMin;
    const uint32_t prevRSMin = autoRestartMin;

    autoBacklightEnabled = autoBacklightSwitch && lv_obj_has_state(autoBacklightSwitch, LV_STATE_CHECKED);
    autoRestartEnabled = autoRestartSwitch && lv_obj_has_state(autoRestartSwitch, LV_STATE_CHECKED);
    if (autoBacklightInput)
        autoBacklightMin = boundedUint(lv_textarea_get_text(autoBacklightInput), 5, 1, 1440);
    if (autoRestartInput)
        autoRestartMin = boundedUint(lv_textarea_get_text(autoRestartInput), 1440, 1, 43200);

    // Refresh UI display with validated values
    if (autoBacklightInput) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%lu", autoBacklightMin);
        lv_textarea_set_text(autoBacklightInput, buf);
    }
    if (autoRestartInput) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%lu", autoRestartMin);
        lv_textarea_set_text(autoRestartInput, buf);
    }

    sysPreferences.begin("syscfg", false);
    sysPreferences.putBool("autoBL", autoBacklightEnabled);
    sysPreferences.putUInt("blMin", autoBacklightMin);
    sysPreferences.putBool("autoRS", autoRestartEnabled);
    sysPreferences.putUInt("rsMin", autoRestartMin);
    sysPreferences.end();

    // Start / reset counting when time is edited or when turned ON, or on explicit save button click
    if ((autoBacklightEnabled && !prevAutoBL) || (autoBacklightMin != prevBLMin) || e != nullptr) {
        lastUserActivityMs = millis();
        if (autoBacklightEnabled && !isBacklightOn) {
            BL_ON;
            isBacklightOn = true;
        }
        Serial.printf("[SYS] Auto Backlight timer reset (%lu min)\n", autoBacklightMin);
    }

    if ((autoRestartEnabled && !prevAutoRS) || (autoRestartMin != prevRSMin) || e != nullptr) {
        autoRestartStartMs = millis();
        Serial.printf("[SYS] Auto Restart timer reset (%lu min)\n", autoRestartMin);
    }

    if (sysStatusLabel) {
        if (e != nullptr) {
            lv_label_set_text(sysStatusLabel, "Settings saved!");
            lv_obj_set_style_text_color(sysStatusLabel, lv_color_hex(0x22C55E), LV_PART_MAIN | LV_STATE_DEFAULT);
        } else {
            update_system_status_label();
        }
    }
    Serial.printf("[SYS] AutoBL=%s(%lum) AutoRS=%s(%lum)\n",
        autoBacklightEnabled ? "ON" : "OFF", autoBacklightMin,
        autoRestartEnabled ? "ON" : "OFF", autoRestartMin);
}

static lv_obj_t *create_rtc_textarea(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t width) {
    lv_obj_t *field = lv_textarea_create(parent);
    lv_obj_set_size(field, width, 46);
    lv_obj_set_pos(field, x, y);
    lv_textarea_set_one_line(field, true);
    lv_obj_set_style_radius(field, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(field, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(field, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(field, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(field, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(field, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(field, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(field, lv_color_hex(0xEEEEEE), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(field, lv_color_hex(0x101C31), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(field, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(field, lv_color_hex(0x2B446E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(field, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(field, open_numeric_keyboard, LV_EVENT_CLICKED, nullptr);
    return field;
}

static void on_clock_header_clicked(lv_event_t *e) {
    select_wifi_clock(e);
}

static void setup_rtc_ui() {
    rtcClockLabel = ui_DateTime;
    lv_obj_add_flag(rtcClockLabel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(rtcClockLabel, on_clock_header_clicked, LV_EVENT_CLICKED, nullptr);
    update_rtc_ui();
}

static lv_obj_t *create_wifi_input(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t width,
                                   bool password = false) {
    lv_obj_t *input = lv_textarea_create(parent);
    lv_obj_set_size(input, width, 46);
    lv_obj_set_pos(input, x, y);
    lv_textarea_set_one_line(input, true);
    lv_textarea_set_password_mode(input, password);
    lv_obj_set_style_radius(input, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(input, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(input, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(input, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(input, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(input, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(input, lv_color_hex(0xEEEEEE), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(input, lv_color_hex(0x101C31), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(input, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(input, lv_color_hex(0x2B446E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(input, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(input, open_numeric_keyboard, LV_EVENT_CLICKED, nullptr);
    return input;
}

static void create_wifi_field_label(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, const char *text) {
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_color(label, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
}

static bool is_wifi_input(lv_obj_t *obj) {
    return obj == wifiSsidInput || obj == wifiPasswordInput || obj == wifiIpInput ||
           obj == wifiMaskInput || obj == wifiGatewayInput || obj == wifiDnsInput ||
           obj == mqttServerInput || obj == mqttPortInput || obj == mqttTopicInput ||
           obj == mqttClientIdInput || obj == mqttUserInput || obj == mqttPassInput;
}

static void update_wifi_status() {
    if (!wifiStatusLabel) return;
    if (WiFi.status() == WL_CONNECTED) {
        char text[128];
        snprintf(text, sizeof(text), "Connected: %s  |  IP: %s  |  RSSI: %d dBm", WiFi.SSID().c_str(),
                 WiFi.localIP().toString().c_str(), WiFi.RSSI());
        lv_label_set_text(wifiStatusLabel, text);
        lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0x22C55E), LV_PART_MAIN | LV_STATE_DEFAULT);
    } else {
        lv_label_set_text(wifiStatusLabel, "Wi-Fi: Disconnected");
        lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

static bool init_wifi_once() {
    if (wifiInitialized)
        return true;
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    wifiInitialized = true;
    Serial.printf("[WiFi] initialized, free heap: %u bytes\n", ESP.getFreeHeap());
    return true;
}

static void load_wifi_settings() {
    wifiPreferences.begin("wifi", false);
    lv_textarea_set_text(wifiSsidInput, wifiPreferences.getString("ssid", "").c_str());
    lv_textarea_set_text(wifiPasswordInput, wifiPreferences.getString("pass", "").c_str());
    lv_textarea_set_text(wifiIpInput, wifiPreferences.getString("ip", "192.168.1.50").c_str());
    lv_textarea_set_text(wifiMaskInput, wifiPreferences.getString("mask", "255.255.255.0").c_str());
    lv_textarea_set_text(wifiGatewayInput, wifiPreferences.getString("gw", "192.168.1.1").c_str());
    lv_textarea_set_text(wifiDnsInput, wifiPreferences.getString("dns", "8.8.8.8").c_str());
    const bool fixIp = wifiPreferences.getBool("fix_ip", !wifiPreferences.getBool("dhcp", true));
    const bool wifiEnabled = wifiPreferences.getBool("enabled", true);
    wifiPreferences.end();

    if (wifiFixIpSwitch) {
        if (fixIp) lv_obj_add_state(wifiFixIpSwitch, LV_STATE_CHECKED);
        else lv_obj_clear_state(wifiFixIpSwitch, LV_STATE_CHECKED);
    }
    if (wifiStaticIpContainer) {
        if (fixIp) lv_obj_clear_flag(wifiStaticIpContainer, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(wifiStaticIpContainer, LV_OBJ_FLAG_HIDDEN);
    }
    if (wifiEnableSwitch) {
        if (wifiEnabled) lv_obj_add_state(wifiEnableSwitch, LV_STATE_CHECKED);
        else lv_obj_clear_state(wifiEnableSwitch, LV_STATE_CHECKED);
    }
}

static bool parse_ip_field(lv_obj_t *input, IPAddress &address) {
    return input && address.fromString(lv_textarea_get_text(input));
}

static void save_wifi_settings(lv_event_t *) {
    const bool fixIp = wifiFixIpSwitch && lv_obj_has_state(wifiFixIpSwitch, LV_STATE_CHECKED);
    IPAddress address;
    if (fixIp && (!parse_ip_field(wifiIpInput, address) || !parse_ip_field(wifiMaskInput, address) ||
                  !parse_ip_field(wifiGatewayInput, address) || !parse_ip_field(wifiDnsInput, address))) {
        lv_label_set_text(wifiStatusLabel, "Invalid static IP / Subnet / Gateway / DNS");
        lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0xEF4444), LV_PART_MAIN | LV_STATE_DEFAULT);
        return;
    }

    const char *ssid = wifiSsidInput ? lv_textarea_get_text(wifiSsidInput) : "";
    const char *pass = wifiPasswordInput ? lv_textarea_get_text(wifiPasswordInput) : "";
    const char *ipStr = wifiIpInput ? lv_textarea_get_text(wifiIpInput) : "192.168.1.50";
    const char *maskStr = wifiMaskInput ? lv_textarea_get_text(wifiMaskInput) : "255.255.255.0";
    const char *gwStr = wifiGatewayInput ? lv_textarea_get_text(wifiGatewayInput) : "192.168.1.1";
    const char *dnsStr = wifiDnsInput ? lv_textarea_get_text(wifiDnsInput) : "8.8.8.8";

    if (strlen(ssid) == 0) {
        if (wifiStatusLabel) {
            lv_label_set_text(wifiStatusLabel, "SSID cannot be empty");
            lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0xEF4444), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        return;
    }

    wifiPreferences.begin("wifi", false);
    wifiPreferences.putString("ssid", ssid);
    wifiPreferences.putString("pass", pass);
    wifiPreferences.putString("ip", ipStr);
    wifiPreferences.putString("mask", maskStr);
    wifiPreferences.putString("gw", gwStr);
    wifiPreferences.putString("dns", dnsStr);
    wifiPreferences.putBool("fix_ip", fixIp);
    wifiPreferences.putBool("dhcp", !fixIp);
    wifiPreferences.putBool("enabled", true);
    wifiPreferences.end();

    if (wifiEnableSwitch) {
        lv_obj_add_state(wifiEnableSwitch, LV_STATE_CHECKED);
    }

    init_wifi_once();
    if (fixIp) {
        IPAddress ip, mask, gateway, dns;
        parse_ip_field(wifiIpInput, ip);
        parse_ip_field(wifiMaskInput, mask);
        parse_ip_field(wifiGatewayInput, gateway);
        parse_ip_field(wifiDnsInput, dns);
        WiFi.config(ip, gateway, mask, dns);
    } else {
        WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
    }
    WiFi.begin(ssid, pass);
    lv_label_set_text(wifiStatusLabel, "Connecting to Wi-Fi...");
    lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    Serial.printf("[WiFi] Settings saved & connecting to '%s'...\n", ssid);
}

static void on_wifi_ssid_selected(lv_event_t *e) {
    const char *ssid = (const char *)lv_event_get_user_data(e);
    if (ssid && wifiSsidInput) {
        lv_textarea_set_text(wifiSsidInput, ssid);
        if (wifiStatusLabel) {
            char msg[80];
            snprintf(msg, sizeof(msg), "Selected SSID: %s", ssid);
            lv_label_set_text(wifiStatusLabel, msg);
            lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

static void scan_wifi_networks(lv_event_t *) {
    if (wifiScanBusy) return;
    wifiScanBusy = true;
    if (wifiScanList) {
        lv_obj_clean(wifiScanList);
        lv_list_add_text(wifiScanList, "Scanning nearby Wi-Fi networks...");
    }
    if (wifiStatusLabel) {
        lv_label_set_text(wifiStatusLabel, "Scanning Wi-Fi...");
        lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    lv_refr_now(NULL);

    init_wifi_once();

    // If Wi-Fi is not connected, it might be in the middle of connecting/associating
    // (e.g. auto-connect to saved credentials), which causes esp_wifi_scan_start()
    // to fail with ESP_ERR_WIFI_STATE (-2). Calling disconnect() resets it to idle.
    const bool wasConnected = (WiFi.status() == WL_CONNECTED);
    if (!wasConnected) {
        WiFi.disconnect();
        delay(150);
    }

    WiFi.scanDelete();
    int16_t count = WiFi.scanNetworks(false, true);

    // If scan failed (e.g. negative error code), reset Wi-Fi mode and retry once
    if (count < 0) {
        Serial.printf("[WiFi] Initial scan failed (%d). Resetting STA radio and retrying...\n", count);
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        delay(100);
        WiFi.mode(WIFI_STA);
        delay(200);
        count = WiFi.scanNetworks(false, true);
        Serial.printf("[WiFi] Retry scan count: %d\n", count);
    }

    if (wifiScanList) lv_obj_clean(wifiScanList);

    if (count <= 0) {
        if (wifiScanList) lv_list_add_text(wifiScanList, "No Wi-Fi networks found");
        if (wifiStatusLabel) {
            lv_label_set_text(wifiStatusLabel, "No Wi-Fi networks found");
            lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    } else {
        const int shown = min((int)count, 20);
        int validCount = 0;
        for (int i = 0; i < shown; ++i) {
            String rawSsid = WiFi.SSID(i);
            if (rawSsid.length() == 0) continue;
            strncpy(scannedSsids[validCount], rawSsid.c_str(), sizeof(scannedSsids[validCount]) - 1);
            scannedSsids[validCount][sizeof(scannedSsids[validCount]) - 1] = '\0';

            char itemText[80];
            snprintf(itemText, sizeof(itemText), "%s %s  (%d dBm)  %s",
                     LV_SYMBOL_WIFI, scannedSsids[validCount], WiFi.RSSI(i),
                     WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "[Open]" : "[Lock]");

            lv_obj_t *btn = lv_list_add_button(wifiScanList, nullptr, itemText);
            lv_obj_set_style_text_font(btn, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(btn, lv_color_hex(0xEEEEEE), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x131E33), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(btn, 220, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_top(btn, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_bottom(btn, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_left(btn, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_right(btn, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_radius(btn, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_add_event_cb(btn, on_wifi_ssid_selected, LV_EVENT_CLICKED, scannedSsids[validCount]);
            validCount++;
        }
        if (wifiStatusLabel) {
            char buf[64];
            snprintf(buf, sizeof(buf), "Found %d networks. Tap network to select SSID.", validCount);
            lv_label_set_text(wifiStatusLabel, buf);
            lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0x22C55E), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
    WiFi.scanDelete();
    wifiScanBusy = false;
}

static void wifi_fix_ip_changed(lv_event_t *) {
    const bool fixIp = wifiFixIpSwitch && lv_obj_has_state(wifiFixIpSwitch, LV_STATE_CHECKED);
    if (wifiStaticIpContainer) {
        if (fixIp) lv_obj_clear_flag(wifiStaticIpContainer, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(wifiStaticIpContainer, LV_OBJ_FLAG_HIDDEN);
    }
}

static void connect_wifi_from_saved() {
    wifiPreferences.begin("wifi", false);
    const bool wifiEnabled = wifiPreferences.getBool("enabled", true);
    String ssid = wifiPreferences.getString("ssid", "");
    String pass = wifiPreferences.getString("pass", "");
    const bool fixIp = wifiPreferences.getBool("fix_ip", !wifiPreferences.getBool("dhcp", true));

    if (!wifiEnabled) {
        wifiPreferences.end();
        Serial.println("[WiFi] Disabled by saved setting — skipping auto-connect");
        WiFi.mode(WIFI_OFF);
        if (wifiStatusLabel) {
            lv_label_set_text(wifiStatusLabel, "Wi-Fi: Disabled");
            lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        return;
    }

    if (ssid.length() == 0) {
        wifiPreferences.end();
        Serial.println("[WiFi] No saved SSID to auto-connect on boot");
        return;
    }

    init_wifi_once();
    if (fixIp) {
        IPAddress ip, mask, gateway, dns;
        ip.fromString(wifiPreferences.getString("ip", "192.168.1.50"));
        mask.fromString(wifiPreferences.getString("mask", "255.255.255.0"));
        gateway.fromString(wifiPreferences.getString("gw", "192.168.1.1"));
        dns.fromString(wifiPreferences.getString("dns", "8.8.8.8"));
        wifiPreferences.end();
        WiFi.config(ip, gateway, mask, dns);
    } else {
        wifiPreferences.end();
        WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
    }

    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.printf("[WiFi] Auto-connecting on boot to '%s'...\n", ssid.c_str());
    if (wifiStatusLabel) {
        char buf[80];
        snprintf(buf, sizeof(buf), "Connecting to %s...", ssid.c_str());
        lv_label_set_text(wifiStatusLabel, buf);
        lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

static void load_mqtt_settings() {
    mqttPreferences.begin("mqtt", false);
    mqttEnabled = mqttPreferences.getBool("enabled", true);
    String server = mqttPreferences.getString("server", "broker.emqx.io");
    uint16_t port = mqttPreferences.getUShort("port", 1883);
    String topic = mqttPreferences.getString("topic", "water_level/data");
    String clientId = mqttPreferences.getString("client_id", "ESP32_WaterLevel");
    String user = mqttPreferences.getString("user", "");
    String pass = mqttPreferences.getString("pass", "");
    mqttPublishIntervalMs = mqttPreferences.getUInt("pub_int", 1000);
    if (mqttPublishIntervalMs < 500) mqttPublishIntervalMs = 1000;

    if (mqttEnableSwitch) {
        if (mqttEnabled) lv_obj_add_state(mqttEnableSwitch, LV_STATE_CHECKED);
        else lv_obj_clear_state(mqttEnableSwitch, LV_STATE_CHECKED);
    }
    if (mqttServerInput) lv_textarea_set_text(mqttServerInput, server.c_str());
    if (mqttPortInput) {
        char portStr[8];
        snprintf(portStr, sizeof(portStr), "%u", port);
        lv_textarea_set_text(mqttPortInput, portStr);
    }
    if (mqttTopicInput) lv_textarea_set_text(mqttTopicInput, topic.c_str());
    if (mqttClientIdInput) lv_textarea_set_text(mqttClientIdInput, clientId.c_str());
    if (mqttUserInput) lv_textarea_set_text(mqttUserInput, user.c_str());
    if (mqttPassInput) lv_textarea_set_text(mqttPassInput, pass.c_str());
    mqttPreferences.end();
}

static void save_mqtt_settings(lv_event_t *) {
    mqttPreferences.begin("mqtt", false);
    mqttEnabled = mqttEnableSwitch && lv_obj_has_state(mqttEnableSwitch, LV_STATE_CHECKED);
    mqttPreferences.putBool("enabled", mqttEnabled);

    String server = mqttServerInput ? lv_textarea_get_text(mqttServerInput) : "broker.emqx.io";
    uint16_t port = (uint16_t)boundedUint(mqttPortInput ? lv_textarea_get_text(mqttPortInput) : "1883", 1883, 1, 65535);
    String topic = mqttTopicInput ? lv_textarea_get_text(mqttTopicInput) : "water_level/data";
    String clientId = mqttClientIdInput ? lv_textarea_get_text(mqttClientIdInput) : "ESP32_WaterLevel";
    String user = mqttUserInput ? lv_textarea_get_text(mqttUserInput) : "";
    String pass = mqttPassInput ? lv_textarea_get_text(mqttPassInput) : "";

    mqttPreferences.putString("server", server);
    mqttPreferences.putUShort("port", port);
    mqttPreferences.putString("topic", topic);
    mqttPreferences.putString("client_id", clientId);
    mqttPreferences.putString("user", user);
    mqttPreferences.putString("pass", pass);
    mqttPreferences.end();

    if (mqttClient.connected()) {
        mqttClient.disconnect();
    }
    mqttClient.setServer(server.c_str(), port);

    if (mqttStatusLabel) {
        lv_label_set_text(mqttStatusLabel, mqttEnabled ? "MQTT: Settings saved. Connecting..." : "MQTT: Disabled");
        lv_obj_set_style_text_color(mqttStatusLabel, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    Serial.println("[MQTT] Settings saved");
}

static void mqtt_enable_changed(lv_event_t *) {
    mqttEnabled = mqttEnableSwitch && lv_obj_has_state(mqttEnableSwitch, LV_STATE_CHECKED);
    if (!mqttEnabled && mqttClient.connected()) {
        mqttClient.disconnect();
        if (mqttStatusLabel) {
            lv_label_set_text(mqttStatusLabel, "MQTT: Disabled");
            lv_obj_set_style_text_color(mqttStatusLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

static String get_mqtt_base_topic() {
    mqttPreferences.begin("mqtt", false);
    String topic = mqttPreferences.getString("topic", "water_level");
    mqttPreferences.end();
    topic.trim();
    if (topic.endsWith("/data")) {
        topic = topic.substring(0, topic.length() - 5);
    } else if (topic.endsWith("/settings")) {
        topic = topic.substring(0, topic.length() - 9);
    } else if (topic.endsWith("/configs")) {
        topic = topic.substring(0, topic.length() - 8);
    }
    while (topic.endsWith("/")) {
        topic = topic.substring(0, topic.length() - 1);
    }
    if (topic.length() == 0) topic = "water_level";
    return topic;
}

static uint32_t lastMqttSettingsPublishMs = 0;
static uint32_t lastMqttConfigsPublishMs = 0;

static void publish_mqtt_data() {
    if (!mqttClient.connected()) return;

    String topic = get_mqtt_base_topic() + "/data";

    // Sensor & level values
    float level_cm = 0.0f;
    float volumn = 0.0f;
    float flow_rate = 0.0f;
    float tank_capacity_cm = 0.0f;

    if (lastRs485Ok) {
#if WATER_LEVEL_SIMULATION
        level_cm = (simulatedLevelPct / 100.0f) * configuredLevelMaxCm();
#else
        level_cm = (float)shared_reg0 / 10.0f;
#endif
        tank_capacity_cm = (waterSettings.tankCapacity > 0.0f) ? waterSettings.tankCapacity : 0.0f;
    } else {
        // หากไม่มีการเชื่อมต่อ หรือไม่มีข้อมูล ให้เท่ากับ 0
        level_cm = 0.0f;
        tank_capacity_cm = 0.0f;
    }

    float level_pct = 0.0f;
    if (tank_capacity_cm > 0.0f) {
        level_pct = (level_cm / tank_capacity_cm) * 100.0f;
        level_pct = constrain(level_pct, 0.0f, 100.0f);
    }

#if WATER_METER_SENSOR_PRESENT
    if (water_meter_ok) {
        volumn = shared_volume_l;
        flow_rate = shared_flow_rate;
    } else {
        volumn = 0.0f;
        flow_rate = 0.0f;
    }
#else
    volumn = 0.0f;
    flow_rate = 0.0f;
#endif

    const bool valveOn = (shared_rs485_outputs & (1u << valve_io_pin)) != 0;
    const bool pump1On = shared_pump1_running || ((shared_rs485_outputs & (1u << pump1_io_pin)) != 0);
    const bool pump2On = shared_pump2_running || ((shared_rs485_outputs & (1u << pump2_io_pin)) != 0);
    const bool alarmOn = alertOutputOn || shared_pump1_alarm || shared_pump2_alarm || ((shared_rs485_outputs & (1u << alarm_io_pin)) != 0);

    JsonDocument doc;

    // 1. Water Level
    JsonObject levelObj = doc["water_level"].to<JsonObject>();
    levelObj["level_cm"] = serialized(String(level_cm, 1));
    levelObj["level_pct"] = serialized(String(level_pct, 1));
    levelObj["volumn"] = serialized(String(volumn, 3));
    levelObj["flow_rate"] = serialized(String(flow_rate, 3));
    levelObj["tank_capacity_cm"] = serialized(String(tank_capacity_cm, 1));

    // 2. Controls & Modes
    JsonObject ctrlObj = doc["controls"].to<JsonObject>();
    ctrlObj["valve_on"] = valveOn;
    ctrlObj["pump1_on"] = pump1On;
    ctrlObj["pump2_on"] = pump2On;
    ctrlObj["alarm_on"] = alarmOn;
    ctrlObj["pump1_alarm"] = shared_pump1_alarm;
    ctrlObj["pump2_alarm"] = shared_pump2_alarm;
    ctrlObj["auto_mode"] = waterSettings.autoMode;
    ctrlObj["auto_pump1"] = waterSettings.autoPump1;
    ctrlObj["auto_pump2"] = waterSettings.autoPump2;
    ctrlObj["manual_pump1"] = waterSettings.manualPump1;
    ctrlObj["manual_pump2"] = waterSettings.manualPump2;
    ctrlObj["manual_valve"] = waterSettings.manualValve;
    ctrlObj["manual_speed_pump1"] = serialized(String(waterSettings.manualSpeedPump1, 1));
    ctrlObj["manual_speed_pump2"] = serialized(String(waterSettings.manualSpeedPump2, 1));
    ctrlObj["pressure_bar"] = serialized(String(shared_pressure_bar, 2));
    ctrlObj["set_pressure"] = serialized(String(waterSettings.setPressure, 2));
    ctrlObj["pid_kp"] = serialized(String(waterSettings.pidKp, 2));
    ctrlObj["pid_ki"] = serialized(String(waterSettings.pidKi, 2));
    ctrlObj["acc_dec"] = serialized(String(waterSettings.accDec, 1));
    ctrlObj["sleep_freq"] = serialized(String(waterSettings.sleepFreq, 1));
    ctrlObj["sleep_delay_sec"] = waterSettings.sleepDelaySec;
    ctrlObj["is_sleeping"] = shared_is_sleeping;
    ctrlObj["lower_limit_cm"] = serialized(String(waterSettings.lowerLimitCm, 1));
    ctrlObj["sensor_type_switch"] = waterSettings.sensorTypeSwitch;
    ctrlObj["sensor_type"] = waterSettings.sensorTypeSwitch ? "pressure_switch" : "rs485";
    ctrlObj["pressure_switch"] = shared_pressure_switch;
    ctrlObj["pressure_sensor_ok"] = pressure_sensor_ok;
    ctrlObj["pid_target_freq"] = serialized(String(shared_pid_target_freq, 1));

    // Inverter Pump 1 and Pump 2 individual statuses
    JsonObject p1Obj = doc["pump1"].to<JsonObject>();
    p1Obj["running"] = shared_pump1_running;
    p1Obj["alarm"] = shared_pump1_alarm;
    p1Obj["freq"] = serialized(String(shared_pump1_freq, 1));
    p1Obj["current"] = serialized(String(shared_pump1_current, 1));
    p1Obj["status_word"] = shared_pump1_status;
    p1Obj["auto_enabled"] = waterSettings.autoPump1;
    p1Obj["manual_enabled"] = waterSettings.manualPump1;
    p1Obj["modbus_ok"] = pump1_modbus_ok;

    JsonObject p2Obj = doc["pump2"].to<JsonObject>();
    p2Obj["running"] = shared_pump2_running;
    p2Obj["alarm"] = shared_pump2_alarm;
    p2Obj["freq"] = serialized(String(shared_pump2_freq, 1));
    p2Obj["current"] = serialized(String(shared_pump2_current, 1));
    p2Obj["status_word"] = shared_pump2_status;
    p2Obj["auto_enabled"] = waterSettings.autoPump2;
    p2Obj["manual_enabled"] = waterSettings.manualPump2;
    p2Obj["modbus_ok"] = pump2_modbus_ok;

    // 3. Local IO
    JsonObject localObj = doc["local_io"].to<JsonObject>();
    localObj["di0"] = shared_di0;
    localObj["di1"] = shared_di1;
    localObj["do0"] = (DO_Read(DO0) == 0);
    localObj["do1"] = (DO_Read(DO1) == 0);

    // 4. RS485 IO (Separated into input di1-di8 & output ch1-ch8)
    JsonObject rs485Obj = doc["rs485_io"].to<JsonObject>();
    rs485Obj["status_ok"] = rs485_io_ok;

    JsonObject inObj = rs485Obj["input"].to<JsonObject>();
    inObj["di1"] = shared_pressure_switch;
    inObj["di2"] = shared_pump1_overload;
    inObj["di3"] = shared_pump2_overload;
    inObj["di4"] = ((shared_rs485_inputs >> rs485_di4_io_pin) & 1) != 0;
    inObj["di5"] = ((shared_rs485_inputs >> rs485_di5_io_pin) & 1) != 0;
    inObj["di6"] = ((shared_rs485_inputs >> rs485_di6_io_pin) & 1) != 0;
    inObj["di7"] = ((shared_rs485_inputs >> rs485_di7_io_pin) & 1) != 0;
    inObj["di8"] = ((shared_rs485_inputs >> rs485_di8_io_pin) & 1) != 0;

    JsonObject outObj = rs485Obj["output"].to<JsonObject>();
    outObj["ch1"] = ((shared_rs485_outputs >> 0) & 1) != 0;
    outObj["ch2"] = ((shared_rs485_outputs >> 1) & 1) != 0;
    outObj["ch3"] = ((shared_rs485_outputs >> 2) & 1) != 0;
    outObj["ch4"] = ((shared_rs485_outputs >> 3) & 1) != 0;
    outObj["ch5"] = ((shared_rs485_outputs >> 4) & 1) != 0;
    outObj["ch6"] = ((shared_rs485_outputs >> 5) & 1) != 0;
    outObj["ch7"] = ((shared_rs485_outputs >> 6) & 1) != 0;
    outObj["ch8"] = ((shared_rs485_outputs >> 7) & 1) != 0;

    // 5. System
    JsonObject sysObj = doc["system"].to<JsonObject>();
    sysObj["wifi_rssi"] = WiFi.RSSI();
    sysObj["ip"] = WiFi.localIP().toString();
    sysObj["sensor_comm_ok"] = lastRs485Ok;
    sysObj["rs485_io_ok"] = rs485_io_ok;
    sysObj["meter_comm_ok"] = water_meter_ok;
    sysObj["level_comm_ok"] = lastRs485Ok;
    sysObj["pump1_comm_ok"] = pump1_modbus_ok;
    sysObj["pump2_comm_ok"] = pump2_modbus_ok;
    sysObj["pressure_comm_ok"] = pressure_sensor_ok;
    sysObj["uptime_sec"] = millis() / 1000;

    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(topic.c_str(), payload.c_str());
    // Also publish to root base topic so subscribing directly without /# or /data also receives it
    String baseTopic = get_mqtt_base_topic();
    if (baseTopic != topic) {
        mqttClient.publish(baseTopic.c_str(), payload.c_str());
    }
}

static void publish_mqtt_settings() {
    if (!mqttClient.connected()) return;

    String topic = get_mqtt_base_topic() + "/settings";

    JsonDocument doc;
    doc["valve_on_cm"] = serialized(String(waterSettings.valveOnCm, 1));
    doc["tank_capacity"] = serialized(String(waterSettings.tankCapacity, 1));
    doc["valve_period_sec"] = serialized(String(waterSettings.valvePeriodSec, 1));
    doc["alert_min_cm"] = serialized(String(waterSettings.alertMinCm, 1));
    doc["alert_max_cm"] = serialized(String(waterSettings.alertMaxCm, 1));
    doc["alert_period_sec"] = waterSettings.alertPeriodSec;
    doc["lower_limit_cm"] = serialized(String(waterSettings.lowerLimitCm, 1));
    doc["auto_mode"] = waterSettings.autoMode;
    doc["auto_pump1"] = waterSettings.autoPump1;
    doc["auto_pump2"] = waterSettings.autoPump2;
    doc["manual_pump1"] = waterSettings.manualPump1;
    doc["manual_pump2"] = waterSettings.manualPump2;
    doc["manual_valve"] = waterSettings.manualValve;
    doc["manual_speed_pump1"] = serialized(String(waterSettings.manualSpeedPump1, 1));
    doc["manual_speed_pump2"] = serialized(String(waterSettings.manualSpeedPump2, 1));
    doc["pid_kp"] = serialized(String(waterSettings.pidKp, 2));
    doc["pid_ki"] = serialized(String(waterSettings.pidKi, 2));
    doc["acc_dec"] = serialized(String(waterSettings.accDec, 1));
    doc["set_pressure"] = serialized(String(waterSettings.setPressure, 2));
    doc["sleep_freq"] = serialized(String(waterSettings.sleepFreq, 1));
    doc["sleep_delay_sec"] = waterSettings.sleepDelaySec;
    doc["sensor_type_switch"] = waterSettings.sensorTypeSwitch;
    doc["sensor_type"] = waterSettings.sensorTypeSwitch ? "pressure_switch" : "rs485";

    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

static void publish_mqtt_configs() {
    if (!mqttClient.connected()) return;

    String topic = get_mqtt_base_topic() + "/configs";

    JsonDocument doc;

    JsonObject sysObj = doc["system"].to<JsonObject>();
    sysObj["device"] = "Waveshare ESP32-S3 Touch LCD 4.3B";
    sysObj["firmware_version"] = "1.1.0";
    sysObj["uptime_sec"] = millis() / 1000;
    sysObj["free_heap"] = ESP.getFreeHeap();

    JsonObject pwrObj = doc["power_system"].to<JsonObject>();
    pwrObj["auto_backlight_enabled"] = autoBacklightEnabled;
    pwrObj["auto_backlight_min"] = autoBacklightMin;
    pwrObj["auto_restart_enabled"] = autoRestartEnabled;
    pwrObj["auto_restart_min"] = autoRestartMin;

    JsonObject mbObj = doc["modbus"].to<JsonObject>();
    mbObj["baud_rate"] = MODBUS_BAUD_RATE;

    JsonObject meterCfg = mbObj["water_meter"].to<JsonObject>();
    meterCfg["slave_id"] = WATER_METER_SLAVE_ID;
    meterCfg["poll_ms"] = WATER_METER_POLL_MS;

    JsonObject rs485IoCfg = mbObj["rs485_io"].to<JsonObject>();
    rs485IoCfg["slave_id"] = RS485_IO_SLAVE_ID;
    rs485IoCfg["poll_ms"] = RS485_IO_POLL_MS;

    JsonObject levelSensor = mbObj["level_sensor"].to<JsonObject>();
    levelSensor["slave_id"] = MODBUS_SLAVE_ID;
    levelSensor["start_reg"] = MODBUS_START_REG;
    levelSensor["reg_count"] = MODBUS_REG_COUNT;
    levelSensor["poll_ms"] = MODBUS_POLL_MS;

    JsonObject inv1Cfg = mbObj["inverter_pump1"].to<JsonObject>();
    inv1Cfg["slave_id"] = INVERTER_PUMP1_SLAVE_ID;
    inv1Cfg["poll_ms"] = DEFAULT_INVERTER_POLL_MS;

    JsonObject inv2Cfg = mbObj["inverter_pump2"].to<JsonObject>();
    inv2Cfg["slave_id"] = INVERTER_PUMP2_SLAVE_ID;
    inv2Cfg["poll_ms"] = DEFAULT_INVERTER_POLL_MS;

    JsonObject pressCfg = mbObj["pressure_sensor"].to<JsonObject>();
    pressCfg["slave_id"] = PRESSURE_SLAVE_ID;
    pressCfg["poll_ms"] = DEFAULT_PRESSURE_POLL_MS;

    JsonObject netObj = doc["network"].to<JsonObject>();
    netObj["ip"] = WiFi.localIP().toString();
    netObj["mac"] = WiFi.macAddress();
    netObj["wifi_rssi"] = WiFi.RSSI();

    JsonObject mqttObj = doc["mqtt"].to<JsonObject>();
    mqttObj["base_topic"] = get_mqtt_base_topic();

    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

static void mqtt_callback(char *topic, byte *payload, unsigned int length) {
    if (!payload || length == 0) return;

    String topicStr = String(topic);
    String base = get_mqtt_base_topic();

    // Ignore telemetry and broadcast topics under all conditions to prevent feedback loops
    if (topicStr.endsWith("/data") || 
        topicStr.endsWith("/settings") || 
        topicStr.endsWith("/configs") ||
        topicStr.indexOf("/data/") >= 0 ||
        topicStr.indexOf("/settings/") >= 0 ||
        topicStr.indexOf("/configs/") >= 0) {
        return;
    }

    // Only process topics intended for commands or settings
    if (topicStr.indexOf("/set") < 0 && 
        topicStr.indexOf("/control") < 0 && 
        topicStr.indexOf("/command") < 0 && 
        topicStr.indexOf("/cmd") < 0) {
        return;
    }

    String payloadStr;
    payloadStr.reserve(length + 1);
    for (unsigned int i = 0; i < length; i++) {
        payloadStr += (char)payload[i];
    }
    payloadStr.trim();
    Serial.printf("[MQTT RX] Topic: %s | Payload: %s\n", topic, payloadStr.c_str());

    JsonDocument doc;
    bool isJson = false;
    if (payloadStr.startsWith("{") && payloadStr.endsWith("}")) {
        DeserializationError err = deserializeJson(doc, payloadStr);
        if (!err && doc.is<JsonObject>()) {
            isJson = true;
        }
    }

    if (!isJson) {
        String sub = topicStr;
        if (sub.startsWith(base)) {
            sub = sub.substring(base.length());
        }
        while (sub.startsWith("/")) sub = sub.substring(1);
        if (sub.startsWith("set/")) sub = sub.substring(4);
        else if (sub.startsWith("cmd/")) sub = sub.substring(4);
        else if (sub.startsWith("command/")) sub = sub.substring(8);
        else if (sub.startsWith("control/")) sub = sub.substring(8);
        else if (sub.startsWith("settings/set/")) sub = sub.substring(13);
        else if (sub.startsWith("configs/set/")) sub = sub.substring(12);

        sub.toLowerCase();

        if (sub.length() > 0 && sub != "set" && sub != "cmd" && sub != "command" && sub != "control") {
            String lowerVal = payloadStr;
            lowerVal.toLowerCase();
            if (lowerVal == "true" || lowerVal == "on" || lowerVal == "open" || lowerVal == "run" || lowerVal == "1") {
                doc[sub] = true;
            } else if (lowerVal == "false" || lowerVal == "off" || lowerVal == "close" || lowerVal == "stop" || lowerVal == "0") {
                doc[sub] = false;
            } else {
                char *endPtr = nullptr;
                float num = strtof(payloadStr.c_str(), &endPtr);
                if (endPtr != payloadStr.c_str() && *endPtr == '\0') {
                    doc[sub] = num;
                } else {
                    doc[sub] = payloadStr;
                }
            }
        } else {
            DeserializationError err = deserializeJson(doc, payloadStr);
            if (err) {
                Serial.printf("[MQTT RX ERR] Invalid payload for topic %s: %s\n", topic, payloadStr.c_str());
                return;
            }
        }
    }

    bool settingsChanged = false;
    bool modeChanged = false;
    bool sysChanged = false;
    const float maxLevelCm = configuredLevelMaxCm();

    // 0. MQTT Telemetry Publish Interval
    if (!doc["telemetry_interval"].isNull()) {
        uint32_t sec = doc["telemetry_interval"].as<uint32_t>();
        if (sec >= 1 && sec <= 3600) {
            mqttPublishIntervalMs = sec * 1000UL;
            mqttPreferences.begin("mqtt", false);
            mqttPreferences.putUInt("pub_int", mqttPublishIntervalMs);
            mqttPreferences.end();
            Serial.printf("[MQTT] Telemetry interval set to %u s\n", sec);
        }
    } else if (!doc["interval_sec"].isNull()) {
        uint32_t sec = doc["interval_sec"].as<uint32_t>();
        if (sec >= 1 && sec <= 3600) {
            mqttPublishIntervalMs = sec * 1000UL;
            mqttPreferences.begin("mqtt", false);
            mqttPreferences.putUInt("pub_int", mqttPublishIntervalMs);
            mqttPreferences.end();
            Serial.printf("[MQTT] Telemetry interval set to %u s\n", sec);
        }
    }

    // 1. Water Level Threshold Settings
    if (!doc["valve_on_cm"].isNull()) {
        waterSettings.valveOnCm = constrain(doc["valve_on_cm"].as<float>(), 0.0f, maxLevelCm);
        settingsChanged = true;
    }
    if (!doc["tank_capacity"].isNull()) {
        waterSettings.tankCapacity = constrain(doc["tank_capacity"].as<float>(), 0.1f, 1000000.0f);
        settingsChanged = true;
    }
    if (!doc["valve_period_sec"].isNull()) {
        waterSettings.valvePeriodSec = constrain(doc["valve_period_sec"].as<float>(), 0.1f, 86400.0f);
        settingsChanged = true;
    }
    if (!doc["pump_delay_sec"].isNull()) {
        waterSettings.sleepDelaySec = constrain(doc["pump_delay_sec"].as<uint32_t>(), (uint32_t)0, (uint32_t)3600);
        settingsChanged = true;
    }
    if (!doc["sleep_delay_sec"].isNull()) {
        waterSettings.sleepDelaySec = constrain(doc["sleep_delay_sec"].as<uint32_t>(), (uint32_t)0, (uint32_t)3600);
        settingsChanged = true;
    }
    if (!doc["sleep_delay"].isNull()) {
        waterSettings.sleepDelaySec = constrain(doc["sleep_delay"].as<uint32_t>(), (uint32_t)0, (uint32_t)3600);
        settingsChanged = true;
    }
    if (!doc["pid_kp"].isNull()) {
        waterSettings.pidKp = constrain(doc["pid_kp"].as<float>(), 0.0f, 100.0f);
        settingsChanged = true;
    } else if (!doc["kp"].isNull()) {
        waterSettings.pidKp = constrain(doc["kp"].as<float>(), 0.0f, 100.0f);
        settingsChanged = true;
    }
    if (!doc["pid_ki"].isNull()) {
        waterSettings.pidKi = constrain(doc["pid_ki"].as<float>(), 0.0f, 100.0f);
        settingsChanged = true;
    } else if (!doc["ki"].isNull()) {
        waterSettings.pidKi = constrain(doc["ki"].as<float>(), 0.0f, 100.0f);
        settingsChanged = true;
    }
    if (!doc["acc_dec"].isNull()) {
        waterSettings.accDec = constrain(doc["acc_dec"].as<float>(), 0.1f, 3600.0f);
        settingsChanged = true;
    } else if (!doc["acc"].isNull()) {
        waterSettings.accDec = constrain(doc["acc"].as<float>(), 0.1f, 3600.0f);
        settingsChanged = true;
    } else if (!doc["dec"].isNull()) {
        waterSettings.accDec = constrain(doc["dec"].as<float>(), 0.1f, 3600.0f);
        settingsChanged = true;
    } else if (!doc["accel"].isNull()) {
        waterSettings.accDec = constrain(doc["accel"].as<float>(), 0.1f, 3600.0f);
        settingsChanged = true;
    } else if (!doc["decel"].isNull()) {
        waterSettings.accDec = constrain(doc["decel"].as<float>(), 0.1f, 3600.0f);
        settingsChanged = true;
    }
    if (!doc["set_pressure"].isNull()) {
        waterSettings.setPressure = constrain(doc["set_pressure"].as<float>(), 0.0f, 6.0f);
        settingsChanged = true;
    } else if (!doc["pressure"].isNull() && (doc["pressure"].is<float>() || doc["pressure"].is<int>())) {
        waterSettings.setPressure = constrain(doc["pressure"].as<float>(), 0.0f, 6.0f);
        settingsChanged = true;
    } else if (!doc["sp"].isNull()) {
        waterSettings.setPressure = constrain(doc["sp"].as<float>(), 0.0f, 6.0f);
        settingsChanged = true;
    }
    if (!doc["sleep_freq"].isNull()) {
        waterSettings.sleepFreq = constrain(doc["sleep_freq"].as<float>(), 0.0f, 50.0f);
        settingsChanged = true;
    }
    if (!doc["alert_min_cm"].isNull()) {
        waterSettings.alertMinCm = constrain(doc["alert_min_cm"].as<float>(), 0.0f, maxLevelCm);
        settingsChanged = true;
    }
    if (!doc["alert_max_cm"].isNull()) {
        waterSettings.alertMaxCm = constrain(doc["alert_max_cm"].as<float>(), 0.0f, maxLevelCm);
        settingsChanged = true;
    }
    if (!doc["alert_period_sec"].isNull()) {
        waterSettings.alertPeriodSec = constrain(doc["alert_period_sec"].as<uint32_t>(), (uint32_t)1, (uint32_t)86400);
        settingsChanged = true;
    }
    if (!doc["lower_limit_cm"].isNull()) {
        waterSettings.lowerLimitCm = constrain(doc["lower_limit_cm"].as<float>(), 0.0f, maxLevelCm);
        settingsChanged = true;
    } else if (!doc["lower_limit"].isNull()) {
        waterSettings.lowerLimitCm = constrain(doc["lower_limit"].as<float>(), 0.0f, maxLevelCm);
        settingsChanged = true;
    }

    // Sensor Type (RS485 vs Pressure Switch)
    if (!doc["sensor_type"].isNull()) {
        if (doc["sensor_type"].is<bool>()) {
            waterSettings.sensorTypeSwitch = doc["sensor_type"].as<bool>();
            settingsChanged = true;
        } else {
            String st = doc["sensor_type"].as<String>();
            st.toLowerCase();
            if (st == "pressure_switch" || st == "switch" || st == "sw" || st == "1" || st == "true") {
                waterSettings.sensorTypeSwitch = true;
                settingsChanged = true;
            } else if (st == "rs485" || st == "sensor" || st == "modbus" || st == "0" || st == "false") {
                waterSettings.sensorTypeSwitch = false;
                settingsChanged = true;
            }
        }
    }
    if (!doc["sensor_type_switch"].isNull()) {
        waterSettings.sensorTypeSwitch = doc["sensor_type_switch"].as<bool>();
        settingsChanged = true;
    }
    if (!doc["sensor_switch"].isNull()) {
        waterSettings.sensorTypeSwitch = doc["sensor_switch"].as<bool>();
        settingsChanged = true;
    }

    if (waterSettings.alertMinCm >= waterSettings.alertMaxCm) {
        waterSettings.alertMaxCm = min(maxLevelCm, waterSettings.alertMinCm + 1.0f);
    }

    // 2. Mode and Control Commands
    if (!doc["mode"].isNull()) {
        if (doc["mode"].is<bool>()) {
            waterSettings.autoMode = doc["mode"].as<bool>();
        } else {
            String m = doc["mode"].as<String>();
            m.toLowerCase();
            waterSettings.autoMode = (m == "auto" || m == "1" || m == "true");
        }
        modeChanged = true;
    }
    if (!doc["auto_mode"].isNull()) {
        waterSettings.autoMode = doc["auto_mode"].as<bool>();
        modeChanged = true;
    }

    if (!doc["auto_pump1"].isNull()) {
        waterSettings.autoPump1 = doc["auto_pump1"].as<bool>();
        modeChanged = true;
    }
    if (!doc["auto_pump2"].isNull()) {
        waterSettings.autoPump2 = doc["auto_pump2"].as<bool>();
        modeChanged = true;
    }
    if (!doc["manual_pump1"].isNull()) {
        waterSettings.manualPump1 = doc["manual_pump1"].as<bool>();
        modeChanged = true;
    }
    if (!doc["manual_pump2"].isNull()) {
        waterSettings.manualPump2 = doc["manual_pump2"].as<bool>();
        modeChanged = true;
    }
    if (!doc["manual_valve"].isNull()) {
        waterSettings.manualValve = doc["manual_valve"].as<bool>();
        manualValveOverride = waterSettings.manualValve;
        set_rs485_output(valve_io_pin, waterSettings.manualValve);
        modeChanged = true;
    }
    if (!doc["manual_speed_pump1"].isNull()) {
        waterSettings.manualSpeedPump1 = constrain(doc["manual_speed_pump1"].as<float>(), waterSettings.sleepFreq, 50.0f);
        settingsChanged = true;
    } else if (!doc["speed_pump1"].isNull()) {
        waterSettings.manualSpeedPump1 = constrain(doc["speed_pump1"].as<float>(), waterSettings.sleepFreq, 50.0f);
        settingsChanged = true;
    } else if (!doc["speed1"].isNull()) {
        waterSettings.manualSpeedPump1 = constrain(doc["speed1"].as<float>(), waterSettings.sleepFreq, 50.0f);
        settingsChanged = true;
    }

    if (!doc["manual_speed_pump2"].isNull()) {
        waterSettings.manualSpeedPump2 = constrain(doc["manual_speed_pump2"].as<float>(), waterSettings.sleepFreq, 50.0f);
        settingsChanged = true;
    } else if (!doc["speed_pump2"].isNull()) {
        waterSettings.manualSpeedPump2 = constrain(doc["speed_pump2"].as<float>(), waterSettings.sleepFreq, 50.0f);
        settingsChanged = true;
    } else if (!doc["speed2"].isNull()) {
        waterSettings.manualSpeedPump2 = constrain(doc["speed2"].as<float>(), waterSettings.sleepFreq, 50.0f);
        settingsChanged = true;
    }

    if (!doc["speed"].isNull()) {
        float spd = constrain(doc["speed"].as<float>(), waterSettings.sleepFreq, 50.0f);
        waterSettings.manualSpeedPump1 = spd;
        waterSettings.manualSpeedPump2 = spd;
        settingsChanged = true;
    }

    // Pump 1 command
    if (!doc["pump1"].isNull() || !doc["pump1_on"].isNull()) {
        bool cmd = !doc["pump1"].isNull() ? doc["pump1"].as<bool>() : doc["pump1_on"].as<bool>();
        if (waterSettings.autoMode) waterSettings.autoPump1 = cmd;
        else waterSettings.manualPump1 = cmd;
        modeChanged = true;
    }

    // Pump 2 command
    if (!doc["pump2"].isNull() || !doc["pump2_on"].isNull()) {
        bool cmd = !doc["pump2"].isNull() ? doc["pump2"].as<bool>() : doc["pump2_on"].as<bool>();
        if (waterSettings.autoMode) waterSettings.autoPump2 = cmd;
        else waterSettings.manualPump2 = cmd;
        modeChanged = true;
    }

    // Valve command
    if (!doc["valve"].isNull() || !doc["valve_on"].isNull()) {
        bool cmd = !doc["valve"].isNull() ? doc["valve"].as<bool>() : doc["valve_on"].as<bool>();
        waterSettings.manualValve = cmd;
        manualValveOverride = cmd;
        set_rs485_output(valve_io_pin, cmd);
        modeChanged = true;
    }

    // Alarm command
    if (!doc["alarm"].isNull() || !doc["alarm_on"].isNull()) {
        bool cmd = doc["alarm"] | doc["alarm_on"];
        set_rs485_output(alarm_io_pin, cmd);
    }

    // Local IO Output (DO0, DO1)
    if (!doc["local_io"]["do0"].isNull()) {
        set_output_state(0, doc["local_io"]["do0"].as<bool>());
        modeChanged = true;
    } else if (!doc["do0"].isNull()) {
        set_output_state(0, doc["do0"].as<bool>());
        modeChanged = true;
    }
    if (!doc["local_io"]["do1"].isNull()) {
        set_output_state(1, doc["local_io"]["do1"].as<bool>());
        modeChanged = true;
    } else if (!doc["local_do1"].isNull()) {
        set_output_state(1, doc["local_do1"].as<bool>());
        modeChanged = true;
    } else if (!doc["do1"].isNull() && doc["rs485_io"].isNull()) {
        set_output_state(1, doc["do1"].as<bool>());
        modeChanged = true;
    }

    // RS485 direct output channels (do1..do8 / ch1..ch8)
    for (uint8_t ch = 1; ch <= 8; ch++) {
        char chKey[8];
        char doKey[8];
        snprintf(chKey, sizeof(chKey), "ch%u", ch);
        snprintf(doKey, sizeof(doKey), "do%u", ch);

        bool hasVal = false;
        bool state = false;

        if (!doc["rs485_io"].isNull()) {
            if (!doc["rs485_io"][doKey].isNull()) {
                state = doc["rs485_io"][doKey].as<bool>();
                hasVal = true;
            } else if (!doc["rs485_io"][chKey].isNull()) {
                state = doc["rs485_io"][chKey].as<bool>();
                hasVal = true;
            } else if (!doc["rs485_io"]["output"][chKey].isNull()) {
                state = doc["rs485_io"]["output"][chKey].as<bool>();
                hasVal = true;
            } else if (!doc["rs485_io"]["output"][doKey].isNull()) {
                state = doc["rs485_io"]["output"][doKey].as<bool>();
                hasVal = true;
            }
        }
        if (!hasVal) {
            if (!doc[chKey].isNull()) {
                state = doc[chKey].as<bool>();
                hasVal = true;
            } else {
                char rs485DoKey[16];
                snprintf(rs485DoKey, sizeof(rs485DoKey), "rs485_do%u", ch);
                if (!doc[rs485DoKey].isNull()) {
                    state = doc[rs485DoKey].as<bool>();
                    hasVal = true;
                }
            }
        }

        if (hasVal) {
            if (ch - 1 == pump1_io_pin) {
                if (waterSettings.autoMode) waterSettings.autoPump1 = state;
                else waterSettings.manualPump1 = state;
                modeChanged = true;
            } else if (ch - 1 == pump2_io_pin) {
                if (waterSettings.autoMode) waterSettings.autoPump2 = state;
                else waterSettings.manualPump2 = state;
                modeChanged = true;
            } else if (ch - 1 == valve_io_pin) {
                waterSettings.manualValve = state;
                manualValveOverride = state;
                set_rs485_output(valve_io_pin, state);
                modeChanged = true;
            } else {
                set_rs485_output(ch - 1, state);
                modeChanged = true;
            }
        }
    }

    // 3. System & Power Configs
    if (!doc["auto_backlight_enabled"].isNull()) {
        autoBacklightEnabled = doc["auto_backlight_enabled"].as<bool>();
        sysChanged = true;
    }
    if (!doc["auto_backlight_min"].isNull()) {
        autoBacklightMin = constrain(doc["auto_backlight_min"].as<uint32_t>(), (uint32_t)1, (uint32_t)1440);
        sysChanged = true;
    }
    if (!doc["auto_restart_enabled"].isNull()) {
        autoRestartEnabled = doc["auto_restart_enabled"].as<bool>();
        sysChanged = true;
    }
    if (!doc["auto_restart_min"].isNull()) {
        autoRestartMin = constrain(doc["auto_restart_min"].as<uint32_t>(), (uint32_t)1, (uint32_t)10080);
        sysChanged = true;
    }
    if (!doc["backlight"].isNull()) {
        bool bl = doc["backlight"].as<bool>();
        if (bl) { BL_ON; isBacklightOn = true; }
        else { BL_OFF; isBacklightOn = false; }
    }

    // 4. Modbus Configs
    bool modbusChanged = false;
    if (!doc["baud_rate"].isNull()) {
        uint32_t br = doc["baud_rate"].as<uint32_t>();
        if (br == 4800 || br == 9600 || br == 19200 || br == 38400 || br == 57600 || br == 115200) {
            MODBUS_BAUD_RATE = br;
            modbusChanged = true;
        }
    }
    if (!doc["level_slave_id"].isNull()) {
        MODBUS_SLAVE_ID = constrain(doc["level_slave_id"].as<uint8_t>(), (uint8_t)1, (uint8_t)247);
        modbusChanged = true;
    }
    if (!doc["level_poll_ms"].isNull()) {
        MODBUS_POLL_MS = constrain(doc["level_poll_ms"].as<uint32_t>(), (uint32_t)50, (uint32_t)60000);
        modbusChanged = true;
    }

    // 5. Apply changes, persist, and update UI
    if (modeChanged || settingsChanged) {
        if (waterSettings.autoMode) {
            waterSettings.manualPump1 = false;
            waterSettings.manualPump2 = false;
            waterSettings.manualValve = false;
            manualValveOverride = false;
            set_rs485_output(valve_io_pin, false);
        } else {
            set_rs485_output(valve_io_pin, waterSettings.manualValve);
            manualValveOverride = waterSettings.manualValve;
        }

        save_water_settings();

        if (lvgl_port_lock(50)) {
            if (settingsChanged) update_settings_ui();
            if (modeChanged) update_mode_ui();
            update_manual_leds();
            lvgl_port_unlock();
        }

        publish_mqtt_settings();
    }

    if (sysChanged) {
        sysPreferences.begin("sys", false);
        sysPreferences.putBool("auto_bl", autoBacklightEnabled);
        sysPreferences.putUInt("bl_min", autoBacklightMin);
        sysPreferences.putBool("auto_rs", autoRestartEnabled);
        sysPreferences.putUInt("rs_min", autoRestartMin);
        sysPreferences.end();

        if (lvgl_port_lock(50)) {
            update_system_status_label();
            lvgl_port_unlock();
        }
        publish_mqtt_configs();
    }

    if (modbusChanged) {
        publish_mqtt_configs();
    }

    // Remote Restart / Reboot Command
    if (doc["restart"] == true || doc["reboot"] == true) {
        Serial.println("[MQTT] Remote reboot command received! Restarting in 500ms...");
        delay(500);
        ESP.restart();
    }
}

static void handle_mqtt_loop() {
    if (WiFi.status() != WL_CONNECTED || !mqttEnabled) {
        if (mqttClient.connected()) mqttClient.disconnect();
        return;
    }

    const uint32_t now = millis();
    if (!mqttClient.connected()) {
        if (now - lastMqttReconnectMs >= 5000) {
            lastMqttReconnectMs = now;
            mqttPreferences.begin("mqtt", false);
            String server = mqttPreferences.getString("server", "broker.emqx.io");
            uint16_t port = mqttPreferences.getUShort("port", 1883);
            String clientId = mqttPreferences.getString("client_id", "ESP32_WaterLevel");
            String user = mqttPreferences.getString("user", "");
            String pass = mqttPreferences.getString("pass", "");
            mqttPreferences.end();

            if (server.length() > 0) {
                mqttClient.setServer(server.c_str(), port);
                mqttClient.setBufferSize(2048);
                mqttClient.setCallback(mqtt_callback);
                Serial.printf("[MQTT] Connecting to %s:%u as %s...\n", server.c_str(), port, clientId.c_str());
                bool ok = false;
                if (user.length() > 0) {
                    ok = mqttClient.connect(clientId.c_str(), user.c_str(), pass.c_str());
                } else {
                    ok = mqttClient.connect(clientId.c_str());
                }

                if (ok) {
                    Serial.println("[MQTT] Connected successfully!");
                    String base = get_mqtt_base_topic();
                    mqttClient.subscribe(String(base + "/set").c_str());
                    mqttClient.subscribe(String(base + "/set/#").c_str());
                    mqttClient.subscribe(String(base + "/control").c_str());
                    mqttClient.subscribe(String(base + "/control/#").c_str());
                    mqttClient.subscribe(String(base + "/command").c_str());
                    mqttClient.subscribe(String(base + "/command/#").c_str());
                    mqttClient.subscribe(String(base + "/cmd/#").c_str());
                    mqttClient.subscribe(String(base + "/settings/set").c_str());
                    mqttClient.subscribe(String(base + "/configs/set").c_str());
                    Serial.printf("[MQTT] Subscribed to %s/[set, control, command, cmd, settings/set, configs/set]\n", base.c_str());

                    if (mqttStatusLabel && lvgl_port_lock(5)) {
                        char msg[80];
                        snprintf(msg, sizeof(msg), "Connected: %s", server.c_str());
                        lv_label_set_text(mqttStatusLabel, msg);
                        lv_obj_set_style_text_color(mqttStatusLabel, lv_color_hex(0x22C55E), LV_PART_MAIN | LV_STATE_DEFAULT);
                        lvgl_port_unlock();
                    }

                    publish_mqtt_data();
                    publish_mqtt_settings();
                    publish_mqtt_configs();
                } else {
                    Serial.printf("[MQTT] Connect failed, rc=%d\n", mqttClient.state());
                    if (mqttStatusLabel && lvgl_port_lock(5)) {
                        char msg[80];
                        snprintf(msg, sizeof(msg), "MQTT: Connect failed (rc=%d)", mqttClient.state());
                        lv_label_set_text(mqttStatusLabel, msg);
                        lv_obj_set_style_text_color(mqttStatusLabel, lv_color_hex(0xEF4444), LV_PART_MAIN | LV_STATE_DEFAULT);
                        lvgl_port_unlock();
                    }
                }
            }
        }
    } else {
        mqttClient.loop();
        if (now - lastMqttPublishMs >= mqttPublishIntervalMs) {
            lastMqttPublishMs = now;
            publish_mqtt_data();
        }
        if (now - lastMqttSettingsPublishMs >= 1000) {
            lastMqttSettingsPublishMs = now;
            publish_mqtt_settings();
        }
        if (now - lastMqttConfigsPublishMs >= 30000) {
            lastMqttConfigsPublishMs = now;
            publish_mqtt_configs();
        }
    }
}

// ==========================================
// ArduinoOTA Support (Over-The-Air Update)
// ==========================================
static bool otaInitialized = false;
volatile bool isOtaUpdating = false;

static void setup_ota() {
    ArduinoOTA.setHostname("esp32-waterlevel");
    ArduinoOTA.setTimeout(10000); // 10s network timeout (default is only 1s)

    ArduinoOTA
        .onStart([]() {
            String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
            Serial.printf("\n[OTA] Start updating %s...\n", type.c_str());
            isOtaUpdating = true;

            // 1. Stop background communication tasks
            if (commTaskHandle != NULL) {
                vTaskSuspend(commTaskHandle);
            }
            if (mqttClient.connected()) {
                mqttClient.disconnect();
            }

            BL_ON;
            isBacklightOn = true;

            set_rs485_output(pump1_run_io_pin, false);
            set_rs485_output(pump2_run_io_pin, false);
            set_rs485_output(pump1_alarm_io_pin, false);
            set_rs485_output(pump2_alarm_io_pin, false);
            set_rs485_output(alarm_io_pin, false);
            set_rs485_output(valve_io_pin, false);

            // 2. Suspend LVGL task safely and wait for DMA to finish
            lvgl_port_suspend();
            gfx.waitDMA();

            // 3. Ultra-fast, clean direct draw (takes ~2ms, zero lag, no LVGL overhead)
            gfx.fillScreen(0x0F172A);
            gfx.fillRoundRect(80, 80, 640, 320, 16, 0x1E293B);
            gfx.drawRoundRect(80, 80, 640, 320, 16, 0x38BDF8);

            gfx.setTextDatum(textdatum_t::middle_center);

            gfx.setFont(&fonts::Font4);
            gfx.setTextColor(0xF8FAFC, 0x1E293B);
            gfx.drawString("OTA FIRMWARE UPDATE", 400, 160);

            gfx.setTextColor(0x38BDF8, 0x1E293B);
            gfx.drawString("UPDATING... PLEASE WAIT", 400, 230);

            gfx.setFont(&fonts::Font2);
            gfx.setTextColor(0xF59E0B, 0x1E293B);
            gfx.drawString("! DO NOT POWER OFF - WILL REBOOT AUTOMATICALLY !", 400, 310);
        })
        .onEnd([]() {
            Serial.println("\n[OTA] Update Complete! Rebooting ESP32...");
            gfx.setFont(&fonts::Font4);
            gfx.setTextColor(0x22C55E, 0x1E293B);
            gfx.drawString("UPDATE COMPLETE! REBOOTING...", 400, 230);
            delay(500);
            ESP.restart();
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            static unsigned int lastPct = 999;
            unsigned int pct = (total > 0) ? (progress / (total / 100)) : 0;
            if (pct != lastPct) {
                lastPct = pct;
                if (pct % 10 == 0) {
                    Serial.printf("[OTA] Progress: %u%%\n", pct);
                }
            }
        })
        .onError([](ota_error_t error) {
            Serial.printf("\n[OTA] Error occurred (%u)! Rebooting ESP32 immediately...\n", error);
            delay(500);
            ESP.restart();
        });

    ArduinoOTA.begin();
    otaInitialized = true;
    Serial.printf("[OTA] ArduinoOTA ready at %s (Hostname: esp32-waterlevel)\n", WiFi.localIP().toString().c_str());
}

static void handle_ota_loop() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!otaInitialized) {
            setup_ota();
        }
        ArduinoOTA.handle();
    } else {
        otaInitialized = false;
    }
}


static void setup_wifi_clock_ui() {
    lv_obj_set_style_pad_all(ui_PanelWiFiAndClock, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_PanelWiFiAndClock, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_PanelWiFiAndClock, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(ui_PanelWiFiAndClock, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_PanelWiFiAndClock, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(ui_PanelWiFiAndClock, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(ui_PanelWiFiAndClock, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(ui_PanelWiFiAndClock, LV_SCROLLBAR_MODE_AUTO);

    auto create_card = [](lv_obj_t *parent, lv_coord_t h) -> lv_obj_t * {
        lv_obj_t *card = lv_obj_create(parent);
        lv_obj_set_size(card, 465, h);
        lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_radius(card, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x0D1527), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(card, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(card, lv_color_hex(0x1E293B), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(card, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_pad_all(card, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        return card;
    };

    // ==========================================
    // 1. Wi-Fi Configuration Card
    // ==========================================
    lv_obj_t *wifiCard = create_card(ui_PanelWiFiAndClock, 324);

    lv_obj_t *wifiTitle = lv_label_create(wifiCard);
    lv_label_set_text(wifiTitle, "WI-FI SETTINGS");
    lv_obj_set_pos(wifiTitle, 15, 10);
    lv_obj_set_style_text_color(wifiTitle, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(wifiTitle, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    // --- Wi-Fi Enable/Disable toggle (top-right, same row as title) ---
    lv_obj_t *wifiEnableLbl = lv_label_create(wifiCard);
    lv_label_set_text(wifiEnableLbl, "WI-FI");
    lv_obj_set_pos(wifiEnableLbl, 365, 12);
    lv_obj_set_style_text_color(wifiEnableLbl, lv_color_hex(0xCBD5E1), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(wifiEnableLbl, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    wifiEnableSwitch = lv_switch_create(wifiCard);
    lv_obj_set_pos(wifiEnableSwitch, 405, 7);
    lv_obj_set_size(wifiEnableSwitch, 42, 22);
    lv_obj_add_state(wifiEnableSwitch, LV_STATE_CHECKED); // default ON
    lv_obj_add_event_cb(wifiEnableSwitch, wifi_enable_toggle, LV_EVENT_VALUE_CHANGED, nullptr);

    create_wifi_field_label(wifiCard, 15, 34, "SSID");
    wifiSsidInput = create_wifi_input(wifiCard, 15, 50, 210);

    create_wifi_field_label(wifiCard, 240, 34, "PASSWORD");
    wifiPasswordInput = create_wifi_input(wifiCard, 240, 50, 210, true);

    wifiFixIpSwitch = lv_switch_create(wifiCard);
    lv_obj_set_pos(wifiFixIpSwitch, 15, 108);
    lv_obj_set_size(wifiFixIpSwitch, 42, 22);
    lv_obj_add_event_cb(wifiFixIpSwitch, wifi_fix_ip_changed, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *fixIpLabel = lv_label_create(wifiCard);
    lv_label_set_text(fixIpLabel, "FIX IP");
    lv_obj_set_pos(fixIpLabel, 64, 111);
    lv_obj_set_style_text_color(fixIpLabel, lv_color_hex(0xCBD5E1), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(fixIpLabel, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *scanButton = lv_button_create(wifiCard);
    lv_obj_set_size(scanButton, 105, 32);
    lv_obj_set_pos(scanButton, 225, 104);
    lv_obj_set_style_radius(scanButton, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(scanButton, lv_color_hex(0x1E293B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(scanButton, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(scanButton, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_t *scanText = lv_label_create(scanButton);
    lv_label_set_text(scanText, "SCAN WIFI");
    lv_obj_set_style_text_color(scanText, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(scanText, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(scanText);
    lv_obj_add_event_cb(scanButton, scan_wifi_networks, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *saveButton = lv_button_create(wifiCard);
    lv_obj_set_size(saveButton, 110, 32);
    lv_obj_set_pos(saveButton, 340, 104);
    lv_obj_set_style_radius(saveButton, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(saveButton, lv_color_hex(0x0284C7), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(saveButton, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(saveButton, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_t *saveText = lv_label_create(saveButton);
    lv_label_set_text(saveText, "CONNECT");
    lv_obj_set_style_text_color(saveText, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(saveText, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(saveText);
    lv_obj_add_event_cb(saveButton, save_wifi_settings, LV_EVENT_CLICKED, nullptr);

    create_wifi_field_label(wifiCard, 15, 144, "NEARBY NETWORKS (TAP TO SELECT SSID)");

    wifiScanList = lv_list_create(wifiCard);
    lv_obj_set_size(wifiScanList, 435, 100);
    lv_obj_set_pos(wifiScanList, 15, 162);
    lv_obj_set_style_radius(wifiScanList, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(wifiScanList, lv_color_hex(0x0A101D), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(wifiScanList, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(wifiScanList, lv_color_hex(0x1E293B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(wifiScanList, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(wifiScanList, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_scrollbar_mode(wifiScanList, LV_SCROLLBAR_MODE_AUTO);
    lv_list_add_text(wifiScanList, "Press [SCAN WIFI] to discover and select networks");

    wifiStatusLabel = lv_label_create(wifiCard);
    lv_label_set_text(wifiStatusLabel, "Wi-Fi: Disconnected");
    lv_obj_set_pos(wifiStatusLabel, 15, 292);
    lv_obj_set_width(wifiStatusLabel, 435);
    lv_label_set_long_mode(wifiStatusLabel, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_font(wifiStatusLabel, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(wifiStatusLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);

    // ==========================================
    // 2. Static IP Settings (Shown only when FIX IP is toggled)
    // ==========================================
    wifiStaticIpContainer = create_card(ui_PanelWiFiAndClock, 176);
    lv_obj_add_flag(wifiStaticIpContainer, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *staticTitle = lv_label_create(wifiStaticIpContainer);
    lv_label_set_text(staticTitle, "FIX IP CONFIGURATION");
    lv_obj_set_pos(staticTitle, 15, 10);
    lv_obj_set_style_text_color(staticTitle, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(staticTitle, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    create_wifi_field_label(wifiStaticIpContainer, 15, 32, "IP ADDRESS");
    wifiIpInput = create_wifi_input(wifiStaticIpContainer, 15, 48, 210);

    create_wifi_field_label(wifiStaticIpContainer, 240, 32, "SUBNET MASK");
    wifiMaskInput = create_wifi_input(wifiStaticIpContainer, 240, 48, 210);

    create_wifi_field_label(wifiStaticIpContainer, 15, 104, "GATEWAY");
    wifiGatewayInput = create_wifi_input(wifiStaticIpContainer, 15, 120, 210);

    create_wifi_field_label(wifiStaticIpContainer, 240, 104, "DNS SERVER");
    wifiDnsInput = create_wifi_input(wifiStaticIpContainer, 240, 120, 210);

    // ==========================================
    // 3. MQTT Settings Card (Directly below Wi-Fi & Static IP)
    // ==========================================
    lv_obj_t *mqttCard = create_card(ui_PanelWiFiAndClock, 340);

    lv_obj_t *mqttTitle = lv_label_create(mqttCard);
    lv_label_set_text(mqttTitle, "MQTT SETTINGS (PUBLISH EVERY 1 SEC)");
    lv_obj_set_pos(mqttTitle, 15, 10);
    lv_obj_set_style_text_color(mqttTitle, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(mqttTitle, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    create_wifi_field_label(mqttCard, 15, 32, "BROKER / SERVER HOST");
    mqttServerInput = create_wifi_input(mqttCard, 15, 48, 280);

    create_wifi_field_label(mqttCard, 310, 32, "PORT");
    mqttPortInput = create_wifi_input(mqttCard, 310, 48, 140);

    create_wifi_field_label(mqttCard, 15, 100, "CLIENT ID");
    mqttClientIdInput = create_wifi_input(mqttCard, 15, 116, 210);

    create_wifi_field_label(mqttCard, 240, 100, "PUBLISH TOPIC");
    mqttTopicInput = create_wifi_input(mqttCard, 240, 116, 210);

    create_wifi_field_label(mqttCard, 15, 168, "USERNAME (OPTIONAL)");
    mqttUserInput = create_wifi_input(mqttCard, 15, 184, 210);

    create_wifi_field_label(mqttCard, 240, 168, "PASSWORD (OPTIONAL)");
    mqttPassInput = create_wifi_input(mqttCard, 240, 184, 210, true);

    mqttEnableSwitch = lv_switch_create(mqttCard);
    lv_obj_set_pos(mqttEnableSwitch, 15, 248);
    lv_obj_set_size(mqttEnableSwitch, 42, 22);
    lv_obj_add_event_cb(mqttEnableSwitch, mqtt_enable_changed, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *mqttEnableLbl = lv_label_create(mqttCard);
    lv_label_set_text(mqttEnableLbl, "ENABLE MQTT");
    lv_obj_set_pos(mqttEnableLbl, 64, 251);
    lv_obj_set_style_text_color(mqttEnableLbl, lv_color_hex(0xCBD5E1), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(mqttEnableLbl, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *saveMqttBtn = lv_button_create(mqttCard);
    lv_obj_set_size(saveMqttBtn, 140, 32);
    lv_obj_set_pos(saveMqttBtn, 310, 244);
    lv_obj_set_style_radius(saveMqttBtn, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(saveMqttBtn, lv_color_hex(0x0284C7), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(saveMqttBtn, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(saveMqttBtn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_t *saveMqttText = lv_label_create(saveMqttBtn);
    lv_label_set_text(saveMqttText, "SAVE MQTT");
    lv_obj_set_style_text_color(saveMqttText, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(saveMqttText, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(saveMqttText);
    lv_obj_add_event_cb(saveMqttBtn, save_mqtt_settings, LV_EVENT_CLICKED, nullptr);

    mqttStatusLabel = lv_label_create(mqttCard);
    lv_label_set_text(mqttStatusLabel, "MQTT: Disconnected");
    lv_obj_set_pos(mqttStatusLabel, 15, 292);
    lv_obj_set_width(mqttStatusLabel, 435);
    lv_label_set_long_mode(mqttStatusLabel, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_color(mqttStatusLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(mqttStatusLabel, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ==========================================
    // 4. RTC Clock Settings Card (Directly below MQTT)
    // ==========================================
    lv_obj_t *rtcCard = create_card(ui_PanelWiFiAndClock, 205);

    lv_obj_t *rtcCardTitle = lv_label_create(rtcCard);
    lv_label_set_text(rtcCardTitle, "RTC CLOCK SETTINGS");
    lv_obj_set_pos(rtcCardTitle, 15, 10);
    lv_obj_set_style_text_color(rtcCardTitle, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(rtcCardTitle, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    const char *rtcLabels[] = {"YEAR", "MONTH", "DAY", "HOUR", "MIN", "SEC"};
    const lv_coord_t rtcXs[] = {15, 108, 173, 247, 312, 377};
    const lv_coord_t rtcWidths[] = {82, 56, 56, 56, 56, 56};
    for (uint8_t i = 0; i < 6; ++i) {
        lv_obj_t *lbl = lv_label_create(rtcCard);
        lv_label_set_text(lbl, rtcLabels[i]);
        lv_obj_set_pos(lbl, rtcXs[i], 32);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
        rtcFields[i] = create_rtc_textarea(rtcCard, rtcXs[i], 48, rtcWidths[i]);
    }

    lv_obj_t *readRtcBtn = lv_button_create(rtcCard);
    lv_obj_set_size(readRtcBtn, 120, 34);
    lv_obj_set_pos(readRtcBtn, 15, 108);
    lv_obj_set_style_radius(readRtcBtn, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(readRtcBtn, lv_color_hex(0x1E293B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(readRtcBtn, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(readRtcBtn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_t *readText = lv_label_create(readRtcBtn);
    lv_label_set_text(readText, "READ RTC");
    lv_obj_set_style_text_color(readText, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(readText, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(readText);
    lv_obj_add_event_cb(readRtcBtn, read_rtc_to_fields, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *saveRtcBtn = lv_button_create(rtcCard);
    lv_obj_set_size(saveRtcBtn, 140, 34);
    lv_obj_set_pos(saveRtcBtn, 145, 108);
    lv_obj_set_style_radius(saveRtcBtn, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(saveRtcBtn, lv_color_hex(0x0284C7), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(saveRtcBtn, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(saveRtcBtn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_t *saveRtcText = lv_label_create(saveRtcBtn);
    lv_label_set_text(saveRtcText, "SAVE RTC TIME");
    lv_obj_set_style_text_color(saveRtcText, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(saveRtcText, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(saveRtcText);
    lv_obj_add_event_cb(saveRtcBtn, save_rtc_from_ui, LV_EVENT_CLICKED, nullptr);

    rtcStatusLabel = lv_label_create(rtcCard);
    lv_label_set_text(rtcStatusLabel, "");
    lv_obj_set_pos(rtcStatusLabel, 15, 160);
    lv_obj_set_width(rtcStatusLabel, 435);
    lv_label_set_long_mode(rtcStatusLabel, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_color(rtcStatusLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(rtcStatusLabel, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    read_rtc_to_fields(nullptr);
    load_wifi_settings();
    load_mqtt_settings();
    update_wifi_status();
    connect_wifi_from_saved();

    // ==========================================
    // 5. Auto Backlight & Auto Restart Card (Below RTC)
    // ==========================================
    lv_obj_t *sysCard = create_card(ui_PanelWiFiAndClock, 220);

    lv_obj_t *sysTitle = lv_label_create(sysCard);
    lv_label_set_text(sysTitle, "POWER & SYSTEM SETTINGS");
    lv_obj_set_pos(sysTitle, 15, 10);
    lv_obj_set_style_text_color(sysTitle, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(sysTitle, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    // --- Auto Backlight row ---
    autoBacklightSwitch = lv_switch_create(sysCard);
    lv_obj_set_pos(autoBacklightSwitch, 15, 42);
    lv_obj_set_size(autoBacklightSwitch, 42, 22);
    lv_obj_add_event_cb(autoBacklightSwitch, auto_backlight_toggle, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *autoBlLbl = lv_label_create(sysCard);
    lv_label_set_text(autoBlLbl, "AUTO BACKLIGHT OFF AFTER:");
    lv_obj_set_pos(autoBlLbl, 64, 46);
    lv_obj_set_style_text_color(autoBlLbl, lv_color_hex(0xCBD5E1), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(autoBlLbl, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    autoBacklightInput = lv_textarea_create(sysCard);
    lv_obj_set_size(autoBacklightInput, 70, 46);
    lv_obj_set_pos(autoBacklightInput, 300, 30);
    lv_textarea_set_one_line(autoBacklightInput, true);
    lv_obj_set_style_radius(autoBacklightInput, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(autoBacklightInput, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(autoBacklightInput, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(autoBacklightInput, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(autoBacklightInput, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(autoBacklightInput, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(autoBacklightInput, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(autoBacklightInput, lv_color_hex(0xEEEEEE), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(autoBacklightInput, lv_color_hex(0x101C31), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(autoBacklightInput, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(autoBacklightInput, lv_color_hex(0x2B446E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(autoBacklightInput, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(autoBacklightInput, open_numeric_keyboard, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *autoBlUnit = lv_label_create(sysCard);
    lv_label_set_text(autoBlUnit, "min");
    lv_obj_set_pos(autoBlUnit, 378, 46);
    lv_obj_set_style_text_color(autoBlUnit, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(autoBlUnit, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // --- Auto Restart row ---
    autoRestartSwitch = lv_switch_create(sysCard);
    lv_obj_set_pos(autoRestartSwitch, 15, 108);
    lv_obj_set_size(autoRestartSwitch, 42, 22);
    lv_obj_add_event_cb(autoRestartSwitch, auto_restart_toggle, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *autoRsLbl = lv_label_create(sysCard);
    lv_label_set_text(autoRsLbl, "AUTO RESTART AFTER:");
    lv_obj_set_pos(autoRsLbl, 64, 112);
    lv_obj_set_style_text_color(autoRsLbl, lv_color_hex(0xCBD5E1), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(autoRsLbl, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    autoRestartInput = lv_textarea_create(sysCard);
    lv_obj_set_size(autoRestartInput, 70, 46);
    lv_obj_set_pos(autoRestartInput, 300, 96);
    lv_textarea_set_one_line(autoRestartInput, true);
    lv_obj_set_style_radius(autoRestartInput, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(autoRestartInput, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(autoRestartInput, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(autoRestartInput, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(autoRestartInput, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(autoRestartInput, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(autoRestartInput, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(autoRestartInput, lv_color_hex(0xEEEEEE), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(autoRestartInput, lv_color_hex(0x101C31), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(autoRestartInput, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(autoRestartInput, lv_color_hex(0x2B446E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(autoRestartInput, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(autoRestartInput, open_numeric_keyboard, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *autoRsUnit = lv_label_create(sysCard);
    lv_label_set_text(autoRsUnit, "min");
    lv_obj_set_pos(autoRsUnit, 378, 112);
    lv_obj_set_style_text_color(autoRsUnit, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(autoRsUnit, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Save button
    lv_obj_t *saveSysBtn = lv_button_create(sysCard);
    lv_obj_set_size(saveSysBtn, 140, 34);
    lv_obj_set_pos(saveSysBtn, 15, 156);
    lv_obj_set_style_radius(saveSysBtn, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(saveSysBtn, lv_color_hex(0x0284C7), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(saveSysBtn, lv_color_hex(0x38BDF8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(saveSysBtn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_t *saveSysText = lv_label_create(saveSysBtn);
    lv_label_set_text(saveSysText, "SAVE SETTINGS");
    lv_obj_set_style_text_color(saveSysText, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(saveSysText, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(saveSysText);
    lv_obj_add_event_cb(saveSysBtn, [](lv_event_t *e) { save_system_settings_from_ui(e); }, LV_EVENT_CLICKED, nullptr);

    // Status label below save button
    sysStatusLabel = lv_label_create(sysCard);
    lv_label_set_text(sysStatusLabel, "");
    lv_obj_set_pos(sysStatusLabel, 15, 196);
    lv_obj_set_width(sysStatusLabel, 435);
    lv_label_set_long_mode(sysStatusLabel, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_color(sysStatusLabel, lv_color_hex(0x94A3B8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(sysStatusLabel, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    load_system_settings();
}

static uint8_t compile_month(const char *text) {
    static const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                   "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    for (uint8_t i = 0; i < 12; ++i)
        if (strncmp(text, months[i], 3) == 0) return (uint8_t)(i + 1);
    return 1;
}

static RtcDateTime build_compile_datetime() {
    RtcDateTime value;
    char month[4] = {};
    int day = 1;
    int year = 2026;
    int hour = 0;
    int minute = 0;
    int second = 0;
    sscanf(__DATE__, "%3s %d %d", month, &day, &year);
    sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);
    value.year = (uint16_t)year;
    value.month = compile_month(month);
    value.day = (uint8_t)day;
    value.hour = (uint8_t)hour;
    value.minute = (uint8_t)minute;
    value.second = (uint8_t)second;
    return value;
}

struct LogItem {
    char text[128];
};

// Thread-safe log function: safely enqueues text to be rendered by UI on Core 1
void logMsg(const char *msg) {
    Serial.print(msg);
    if (logQueue) {
        LogItem item;
        strncpy(item.text, msg, sizeof(item.text) - 1);
        item.text[sizeof(item.text) - 1] = '\0';
        xQueueSend(logQueue, &item, 0); // Non-blocking enqueue
    }
}

void logMsg(const String &str) { logMsg(str.c_str()); }

static float boundedFloat(const char *text, float fallback, float minValue, float maxValue) {
    if (!text || !*text)
        return fallback;
    char *end = nullptr;
    const float value = strtof(text, &end);
    if (end == text || !isfinite(value))
        return fallback;
    return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

static uint32_t boundedUint(const char *text, uint32_t fallback, uint32_t minValue, uint32_t maxValue) {
    if (!text || !*text)
        return fallback;
    char *end = nullptr;
    const long value = strtol(text, &end, 10);
    if (end == text)
        return fallback;
    return (uint32_t)(value < (long)minValue ? minValue : (value > (long)maxValue ? maxValue : value));
}

static void set_textarea_number(lv_obj_t *textarea, float value, uint8_t decimals = 0) {
    if (!textarea)
        return;
    char text[20];
    snprintf(text, sizeof(text), decimals ? "%.1f" : "%.0f", value);
    lv_textarea_set_text(textarea, text);
}

static void update_settings_ui() {
    if (ui_SetOnValve) set_textarea_number(ui_SetOnValve, waterSettings.valveOnCm, 1);
    if (ui_SetTankCapacity) set_textarea_number(ui_SetTankCapacity, waterSettings.tankCapacity, 1);
    if (ui_SetPeriodTimeContorl) set_textarea_number(ui_SetPeriodTimeContorl, waterSettings.valvePeriodSec, 1);
    if (ui_SetMinAlert) set_textarea_number(ui_SetMinAlert, waterSettings.alertMinCm, 1);
    if (ui_SetMaxAlert) set_textarea_number(ui_SetMaxAlert, waterSettings.alertMaxCm, 1);
    if (ui_SetPeriodTimeAlert) set_textarea_number(ui_SetPeriodTimeAlert, (float)waterSettings.alertPeriodSec);
    if (ui_SetLowerLimit) set_textarea_number(ui_SetLowerLimit, waterSettings.lowerLimitCm, 1);

    // Inverter PID textareas (ui_PanelAuto)
    if (ui_SetPidKp) set_textarea_number(ui_SetPidKp, waterSettings.pidKp, 2);
    if (ui_SetPidKi) set_textarea_number(ui_SetPidKi, waterSettings.pidKi, 2);
    if (ui_SetPressure) set_textarea_number(ui_SetPressure, waterSettings.setPressure, 2);
    if (ui_SetSleepFreq) set_textarea_number(ui_SetSleepFreq, waterSettings.sleepFreq, 1);
    if (ui_SetSleepDelay) set_textarea_number(ui_SetSleepDelay, (float)waterSettings.sleepDelaySec, 0);

    // Inverter Manual textareas (ui_PanelManual)
    if (ui_SetSpeedPump1) set_textarea_number(ui_SetSpeedPump1, waterSettings.manualSpeedPump1, 1);
    if (ui_SetSpeedPump2) set_textarea_number(ui_SetSpeedPump2, waterSettings.manualSpeedPump2, 1);
    if (ui_SetAccDec) set_textarea_number(ui_SetAccDec, waterSettings.accDec, 1);

    if (ui_SwitchSensorType)
        lv_obj_set_state(ui_SwitchSensorType, LV_STATE_CHECKED, waterSettings.sensorTypeSwitch);

    if (ui_LabelLimits) {
        char limits[64];
        snprintf(limits, sizeof(limits), "ALERT MIN: %.1f cm | MAX: %.1f cm", waterSettings.alertMinCm, waterSettings.alertMaxCm);
        lv_label_set_text(ui_LabelLimits, limits);
    }
}

static float configuredLevelMaxCm() {
    return max(1.0f, waterSettings.tankCapacity);
}

static void update_mode_ui() {
    if (ui_ManualAuto)
        lv_obj_set_state(ui_ManualAuto, LV_STATE_CHECKED, waterSettings.autoMode);
    if (ui_SwitchAutoPump1)
        lv_obj_set_state(ui_SwitchAutoPump1, LV_STATE_CHECKED, waterSettings.autoPump1);
    if (ui_SwitchAutoPump2)
        lv_obj_set_state(ui_SwitchAutoPump2, LV_STATE_CHECKED, waterSettings.autoPump2);
    if (ui_SwitchSensorType)
        lv_obj_set_state(ui_SwitchSensorType, LV_STATE_CHECKED, waterSettings.sensorTypeSwitch);

    if (waterSettings.autoMode) {
        waterSettings.manualPump1 = false;
        waterSettings.manualPump2 = false;
        waterSettings.manualValve = false;
        manualValveOverride = false;
    }

    auto apply_manual_switch_state = [](lv_obj_t *sw, bool checked, bool disabled) {
        if (!sw) return;
        lv_obj_set_state(sw, LV_STATE_CHECKED, checked);
        if (disabled) {
            lv_obj_add_state(sw, LV_STATE_DISABLED);
            lv_obj_remove_flag(sw, LV_OBJ_FLAG_CLICKABLE);
        } else {
            lv_obj_remove_state(sw, LV_STATE_DISABLED);
            lv_obj_add_flag(sw, LV_OBJ_FLAG_CLICKABLE);
        }
    };

    apply_manual_switch_state(ui_SwitchManualPump1, waterSettings.manualPump1, waterSettings.autoMode);
    apply_manual_switch_state(ui_SwitchManualPump2, waterSettings.manualPump2, waterSettings.autoMode);
    apply_manual_switch_state(ui_SwitchValve, waterSettings.manualValve, waterSettings.autoMode);

    if (ui_SetLowerLimit)
        set_textarea_number(ui_SetLowerLimit, waterSettings.lowerLimitCm, 1);
    if (ui_SetSpeedPump1)
        set_textarea_number(ui_SetSpeedPump1, waterSettings.manualSpeedPump1, 1);
    if (ui_SetSpeedPump2)
        set_textarea_number(ui_SetSpeedPump2, waterSettings.manualSpeedPump2, 1);
    if (ui_SetAccDec)
        set_textarea_number(ui_SetAccDec, waterSettings.accDec, 1);
}

static void save_settings_from_ui(lv_event_t *) {
    const float maxLevelCm = configuredLevelMaxCm();
    if (ui_SetTankCapacity)
        waterSettings.tankCapacity = boundedFloat(lv_textarea_get_text(ui_SetTankCapacity), waterSettings.tankCapacity, 0.1f, 1000000.0f);
    if (ui_SetOnValve)
        waterSettings.valveOnCm = boundedFloat(lv_textarea_get_text(ui_SetOnValve), waterSettings.valveOnCm, 0.0f, maxLevelCm);
    if (ui_SetPeriodTimeContorl)
        waterSettings.valvePeriodSec = boundedFloat(lv_textarea_get_text(ui_SetPeriodTimeContorl), waterSettings.valvePeriodSec, 0.1f, 86400.0f);
    if (ui_SetMinAlert)
        waterSettings.alertMinCm = boundedFloat(lv_textarea_get_text(ui_SetMinAlert), waterSettings.alertMinCm, 0.0f, maxLevelCm);
    if (ui_SetMaxAlert)
        waterSettings.alertMaxCm = boundedFloat(lv_textarea_get_text(ui_SetMaxAlert), waterSettings.alertMaxCm, 0.0f, maxLevelCm);
    if (ui_SetPeriodTimeAlert)
        waterSettings.alertPeriodSec = boundedUint(lv_textarea_get_text(ui_SetPeriodTimeAlert), waterSettings.alertPeriodSec, 1, 86400);
    if (ui_SetLowerLimit)
        waterSettings.lowerLimitCm = boundedFloat(lv_textarea_get_text(ui_SetLowerLimit), waterSettings.lowerLimitCm, 0.0f, maxLevelCm);

    // Inverter PID settings (ui_PanelAuto)
    if (ui_SetPidKp)
        waterSettings.pidKp = boundedFloat(lv_textarea_get_text(ui_SetPidKp), waterSettings.pidKp, 0.0f, 100.0f);
    if (ui_SetPidKi)
        waterSettings.pidKi = boundedFloat(lv_textarea_get_text(ui_SetPidKi), waterSettings.pidKi, 0.0f, 100.0f);
    if (ui_SetPressure)
        waterSettings.setPressure = boundedFloat(lv_textarea_get_text(ui_SetPressure), waterSettings.setPressure, 0.0f, 6.0f);
    if (ui_SetSleepFreq) {
        waterSettings.sleepFreq = boundedFloat(lv_textarea_get_text(ui_SetSleepFreq), waterSettings.sleepFreq, 0.0f, 50.0f);
        if (waterSettings.manualSpeedPump1 < waterSettings.sleepFreq) waterSettings.manualSpeedPump1 = waterSettings.sleepFreq;
        if (waterSettings.manualSpeedPump2 < waterSettings.sleepFreq) waterSettings.manualSpeedPump2 = waterSettings.sleepFreq;
    }
    if (ui_SetSleepDelay)
        waterSettings.sleepDelaySec = boundedUint(lv_textarea_get_text(ui_SetSleepDelay), waterSettings.sleepDelaySec, 0, 3600);

    // Inverter Manual settings (ui_PanelManual)
    if (ui_SetSpeedPump1)
        waterSettings.manualSpeedPump1 = boundedFloat(lv_textarea_get_text(ui_SetSpeedPump1), waterSettings.manualSpeedPump1, waterSettings.sleepFreq, 50.0f);
    if (ui_SetSpeedPump2)
        waterSettings.manualSpeedPump2 = boundedFloat(lv_textarea_get_text(ui_SetSpeedPump2), waterSettings.manualSpeedPump2, waterSettings.sleepFreq, 50.0f);
    if (ui_SetAccDec)
        waterSettings.accDec = boundedFloat(lv_textarea_get_text(ui_SetAccDec), waterSettings.accDec, 0.1f, 3600.0f);

    if (waterSettings.alertMinCm >= waterSettings.alertMaxCm)
        waterSettings.alertMaxCm = min(maxLevelCm, waterSettings.alertMinCm + 1.0f);

    update_settings_ui();
    save_water_settings();
}

static void sensor_type_event(lv_event_t *e) {
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    waterSettings.sensorTypeSwitch = lv_obj_has_state(obj, LV_STATE_CHECKED);
    save_water_settings();
    update_manual_leds(true);
}

static void save_mode_settings(lv_event_t *) {
    if (ui_ManualAuto)
        waterSettings.autoMode = lv_obj_has_state(ui_ManualAuto, LV_STATE_CHECKED);
    if (ui_SwitchAutoPump1)
        waterSettings.autoPump1 = lv_obj_has_state(ui_SwitchAutoPump1, LV_STATE_CHECKED);
    if (ui_SwitchAutoPump2)
        waterSettings.autoPump2 = lv_obj_has_state(ui_SwitchAutoPump2, LV_STATE_CHECKED);
    if (ui_SwitchSensorType)
        waterSettings.sensorTypeSwitch = lv_obj_has_state(ui_SwitchSensorType, LV_STATE_CHECKED);

    if (waterSettings.autoMode) {
        waterSettings.manualPump1 = false;
        waterSettings.manualPump2 = false;
        waterSettings.manualValve = false;
        manualValveOverride = false;
        set_rs485_output(valve_io_pin, false);
    } else {
        if (ui_SwitchManualPump1)
            waterSettings.manualPump1 = lv_obj_has_state(ui_SwitchManualPump1, LV_STATE_CHECKED);
        if (ui_SwitchManualPump2)
            waterSettings.manualPump2 = lv_obj_has_state(ui_SwitchManualPump2, LV_STATE_CHECKED);
        if (ui_SwitchValve)
            waterSettings.manualValve = lv_obj_has_state(ui_SwitchValve, LV_STATE_CHECKED);
        set_rs485_output(valve_io_pin, waterSettings.manualValve);
        manualValveOverride = waterSettings.manualValve;
    }

    if (ui_SetLowerLimit) {
        const float maxLevelCm = configuredLevelMaxCm();
        waterSettings.lowerLimitCm = boundedFloat(lv_textarea_get_text(ui_SetLowerLimit), waterSettings.lowerLimitCm, 0.0f, maxLevelCm);
    }

    save_water_settings();
    update_mode_ui();
    update_manual_leds(true);
}

// The SquareLine keyboard is a shared numeric keypad for all setting fields.
// The actual field being edited is kept separately so the keyboard display can
// be edited without changing the setting until OK/Enter is pressed.
static lv_obj_t *activeNumericInput = nullptr;

static void open_numeric_keyboard(lv_event_t *e) {
    activeNumericInput = (lv_obj_t *)lv_event_get_target(e);
    if (!activeNumericInput || !ui_ContainerKeyboard || !ui_KeyboardDisplay || !ui_Keyboard)
        return;

    lv_textarea_set_text(ui_KeyboardDisplay, lv_textarea_get_text(activeNumericInput));
    lv_keyboard_set_textarea(ui_Keyboard, ui_KeyboardDisplay);
    const bool isText = (activeNumericInput == wifiSsidInput || activeNumericInput == wifiPasswordInput ||
                         activeNumericInput == mqttServerInput || activeNumericInput == mqttTopicInput ||
                         activeNumericInput == mqttClientIdInput || activeNumericInput == mqttUserInput ||
                         activeNumericInput == mqttPassInput);
    lv_keyboard_set_mode(ui_Keyboard, isText ? LV_KEYBOARD_MODE_TEXT_LOWER : LV_KEYBOARD_MODE_NUMBER);
    lv_obj_clear_flag(ui_ContainerKeyboard, LV_OBJ_FLAG_HIDDEN);
    // The RTC dialog is created after the generated keyboard, so bring the
    // keyboard to the front when editing one of its fields.
    lv_obj_move_foreground(ui_ContainerKeyboard);
    lv_obj_add_state(ui_KeyboardDisplay, LV_STATE_FOCUSED);
    lv_textarea_set_cursor_pos(ui_KeyboardDisplay, LV_TEXTAREA_CURSOR_LAST);
}

static void close_numeric_keyboard(lv_event_t *e) {
    if (!activeNumericInput || !ui_KeyboardDisplay)
        return;

    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        lv_textarea_set_text(activeNumericInput, lv_textarea_get_text(ui_KeyboardDisplay));
        if (!is_rtc_input(activeNumericInput) && !is_wifi_input(activeNumericInput) && !is_sys_input(activeNumericInput))
            save_settings_from_ui(nullptr);
        if (is_sys_input(activeNumericInput))
            save_system_settings_from_ui(nullptr);
    }
    lv_obj_add_flag(ui_ContainerKeyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_state(ui_KeyboardDisplay, LV_STATE_FOCUSED);
    activeNumericInput = nullptr;
}

// Translate Modbus error code to readable string
const char *getModbusErrorStr(uint8_t errCode) {
    switch (errCode) {
    case ModbusMaster::ku8MBSuccess:
        return "Success";
    case ModbusMaster::ku8MBIllegalFunction:
        return "Illegal Function";
    case ModbusMaster::ku8MBIllegalDataAddress:
        return "Illegal Data Address";
    case ModbusMaster::ku8MBIllegalDataValue:
        return "Illegal Data Value";
    case ModbusMaster::ku8MBSlaveDeviceFailure:
        return "Slave Device Failure";
    case ModbusMaster::ku8MBInvalidSlaveID:
        return "Invalid Slave ID";
    case ModbusMaster::ku8MBInvalidFunction:
        return "Invalid Function";
    case ModbusMaster::ku8MBResponseTimedOut:
        return "Timeout (Check A/B wire, ID, Baud, Power)";
    case ModbusMaster::ku8MBInvalidCRC:
        return "CRC Error (Noise or wrong baud/parity)";
    default:
        return "Unknown Error";
    }
}

// -----------------------------------------------------------------------------
// Background Modbus Operations (Executes purely on Core 0)
// -----------------------------------------------------------------------------
static uint8_t lastLoggedErr = 0xFF;

void doModbusPoll() {
    modbus.begin(MODBUS_SLAVE_ID, Serial1);
    WS43B.rs485.clearRxBuffer();

    // 1. ลองอ่านแบบ Holding Registers (Function 0x03)
    uint8_t result = modbus.readHoldingRegisters(MODBUS_START_REG, MODBUS_REG_COUNT);

    // 2. ถ้าไม่สำเร็จ ลองอ่านแบบ Input Registers (Function 0x04)
    bool isInputReg = false;
    if (result == modbus.ku8MBIllegalFunction) {
        WS43B.rs485.clearRxBuffer();
        result = modbus.readInputRegisters(MODBUS_START_REG, MODBUS_REG_COUNT);
        isInputReg = true;
    }

    if (result == modbus.ku8MBSuccess) {
        sensor_rs485_ok = true;
        lastRs485Ok = true;
        lastLoggedErr = 0xFF;
        char buf[128];
        snprintf(buf, sizeof(buf), "[MB OK] ID:%d %s | R0=%u R1=%u\n", MODBUS_SLAVE_ID, isInputReg ? "InReg" : "HoldReg", modbus.getResponseBuffer(0),
                 (MODBUS_REG_COUNT > 1) ? modbus.getResponseBuffer(1) : 0);
        logMsg(buf);

        // ส่งต่อค่าออกบัส CAN (ID: 0x400)
        uint8_t canData[8] = {0};
        uint16_t v0 = modbus.getResponseBuffer(0);
        uint16_t v1 = (MODBUS_REG_COUNT > 1) ? modbus.getResponseBuffer(1) : 0;
        shared_reg0 = v0;
        shared_reg1 = v1;
        reg_data_updated = true;
        canData[0] = MODBUS_SLAVE_ID;
        canData[1] = (uint8_t)((v0 >> 8) & 0xFF);
        canData[2] = (uint8_t)(v0 & 0xFF);
        canData[3] = (uint8_t)((v1 >> 8) & 0xFF);
        canData[4] = (uint8_t)(v1 & 0xFF);
        WS43B.can.send(0x400, canData, 5);
    } else {
        sensor_rs485_ok = false;
        lastRs485Ok = false;
        // พิมพ์ออก Serial เสมอ แต่ขึ้นหน้าจอเฉพาะเมื่อ error มีการเปลี่ยนแปลงเพื่อลดภาระจอ
        char buf[128];
        snprintf(buf, sizeof(buf), "[MB ERR 0x%02X] %s\n", result, getModbusErrorStr(result));
        Serial.print(buf);

        if (result != lastLoggedErr) {
            lastLoggedErr = result;
            logMsg(buf);
        }
    }
}

static void pollRs485Io() {
    static bool wasConnected = false;
    rs485Io.begin(RS485_IO_SLAVE_ID, Serial1);
    WS43B.rs485.clearRxBuffer();
    const uint8_t inputResult = rs485Io.readDiscreteInputs(0, 8); // 1x: addresses 0..7
    if (inputResult != rs485Io.ku8MBSuccess) {
        rs485_io_ok = false;
        wasConnected = false;
        rs485_output_synced = false;
        return;
    }
    // Slave 3 is online as soon as the required 8-channel input read works.
    // This module supports input reads and function 0x0F writes, but its
    // read-coils response is not reliable. Do not use function 0x01 as a
    // reconnect test because it creates a false disconnect/reconnect loop.
    shared_rs485_inputs = (uint8_t)(rs485Io.getResponseBuffer(0) & 0xFF);
    shared_pressure_switch = (shared_rs485_inputs & (1u << pressure_switch_io_pin)) != 0;
    shared_pump1_overload = (shared_rs485_inputs & (1u << pump1_overload_io_pin)) != 0;
    shared_pump2_overload = (shared_rs485_inputs & (1u << pump2_overload_io_pin)) != 0;

    // Outputs are controlled only by the local shadow updated from the UI.
    // On the first connection or after reconnect, resend the latest UI state
    // once; never read or import an indeterminate coil state from the module.
    if (!wasConnected) {
        rs485_output_synced = true;
        pendingIoMask = 0xFF;
    }

    rs485_io_ok = true;
    wasConnected = true;
}

#if WATER_METER_SENSOR_PRESENT
static void pollWaterMeter() {
    waterMeter.begin(WATER_METER_SLAVE_ID, Serial1);
    WS43B.rs485.clearRxBuffer();

    // 1. Try Holding Registers (Function 0x03)
    uint8_t result = waterMeter.readHoldingRegisters(WATER_METER_START_REG, WATER_METER_REG_COUNT);

    // 2. Fallback to Input Registers (Function 0x04) if needed
    if (result == waterMeter.ku8MBIllegalFunction) {
        WS43B.rs485.clearRxBuffer();
        result = waterMeter.readInputRegisters(WATER_METER_START_REG, WATER_METER_REG_COUNT);
    }

    if (result == waterMeter.ku8MBSuccess) {
        water_meter_ok = true;
        for (uint8_t i = 0; i < WATER_METER_REG_COUNT && i < 5; ++i) {
            shared_meter_regs[i] = waterMeter.getResponseBuffer(i);
        }
        // Interpretation placeholder:
        // Reg 0: Flow rate (0.1 L/min or m3/h)
        // Reg 1..2: Total volume
        shared_flow_rate = (float)shared_meter_regs[0] / 10.0f;
        uint32_t rawVol = ((uint32_t)shared_meter_regs[1] << 16) | shared_meter_regs[2];
        shared_volume_l = (float)rawVol / 10.0f;
        meter_data_updated = true;
    } else {
        water_meter_ok = false;
    }
}
#endif

void doModbusScan() {
    logMsg("\n=== Scanning Modbus IDs 1 to 10 (Core 0) ===\n");
    bool foundAny = false;

    for (uint8_t id = 1; id <= 10; id++) {
        modbus.begin(id, Serial1);
        WS43B.rs485.clearRxBuffer();

        // ทดสอบ Function 0x03
        uint8_t res = modbus.readHoldingRegisters(0x0000, 1);
        if (res == modbus.ku8MBSuccess || res == modbus.ku8MBIllegalDataAddress) {
            char buf[64];
            snprintf(buf, sizeof(buf), " [FOUND] Slave ID %d responded (Func 0x03)!\n", id);
            logMsg(buf);
            MODBUS_SLAVE_ID = id;
            foundAny = true;
            break;
        }

        // ทดสอบ Function 0x04
        res = modbus.readInputRegisters(0x0000, 1);
        if (res == modbus.ku8MBSuccess || res == modbus.ku8MBIllegalDataAddress) {
            char buf[64];
            snprintf(buf, sizeof(buf), " [FOUND] Slave ID %d responded (Func 0x04)!\n", id);
            logMsg(buf);
            MODBUS_SLAVE_ID = id;
            foundAny = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(40));
    }

    if (!foundAny) {
        logMsg(" [!] No Slave replied. Tip: Try swapping wire A & B.\n");
    }

    modbus.begin(MODBUS_SLAVE_ID, Serial1);
    logMsg("============================================\n");
}

static void modbus_idle_yield() {
    vTaskDelay(pdMS_TO_TICKS(1));
}

// -----------------------------------------------------------------------------
// OMRON SYSDRIVE 3G3JZ Inverter & Pressure Modbus RTU Routines (Core 0)
// -----------------------------------------------------------------------------
static void sendInverterCmd(uint8_t slaveId, bool run, float targetFreq) {
    modbus.begin(slaveId, Serial1);
    modbus.idle(modbus_idle_yield);
    WS43B.rs485.clearRxBuffer();

    // Frequency command register 0x2102 (scale: 0.01 Hz)
    uint16_t fVal = (uint16_t)constrain((int)(targetFreq * 100.0f + 0.5f), 0, 5000);
    uint8_t res = modbus.writeSingleRegister(0x2102, fVal);

    if (res == modbus.ku8MBSuccess) {
        vTaskDelay(pdMS_TO_TICKS(10));
        WS43B.rs485.clearRxBuffer();

        // Control command register 0x2101 (Bit 5 ComCtrl, Bit 4 ComRef, Bit 0 Run)
        uint16_t ctrlVal = run ? 0x0031 : 0x0030;
        modbus.writeSingleRegister(0x2101, ctrlVal);
    }
}

static bool sendInverterAccDec(uint8_t slaveId, float accDecSec) {
    modbus.begin(slaveId, Serial1);
    modbus.idle(modbus_idle_yield);
    WS43B.rs485.clearRxBuffer();

    // Omron 3G3JZ: n1.09 (0x0109) = Accel Time 1, n1.10 (0x010A) = Decel Time 1
    // Range: 0.1s to 600.0s, scale: 0.1s -> value 1 to 6000
    uint16_t rawVal = (uint16_t)constrain((int)(accDecSec * 10.0f + 0.5f), 1, 6000);

    // 1. Write Acceleration Time 1 (0x0109)
    uint8_t res1 = modbus.writeSingleRegister(0x0109, rawVal);
    vTaskDelay(pdMS_TO_TICKS(10));
    WS43B.rs485.clearRxBuffer();

    // 2. Write Deceleration Time 1 (0x010A)
    uint8_t res2 = modbus.writeSingleRegister(0x010A, rawVal);

    if (res1 == modbus.ku8MBSuccess && res2 == modbus.ku8MBSuccess) {
        Serial.printf("[INV ID %d] Acc/Dec updated: %.1fs (raw: %u)\n", slaveId, accDecSec, rawVal);
        return true;
    } else {
        return false;
    }
}

static void pollInverter(uint8_t slaveId, volatile float &outFreq, volatile float &outCurr,
                         volatile uint16_t &outStatus, volatile bool &outRunning,
                         volatile bool &outAlarm, volatile bool &modbusOk,
                         uint8_t &failCount) {
    modbus.begin(slaveId, Serial1);
    modbus.idle(modbus_idle_yield);
    WS43B.rs485.clearRxBuffer();

    // Read 0x2120 - 0x2121 (Status bit and Error code)
    uint8_t res = modbus.readHoldingRegisters(0x2120, 2);
    if (res == modbus.ku8MBSuccess) {
        failCount = 0;
        modbusOk = true;
        uint16_t st = modbus.getResponseBuffer(0);
        outStatus = st;
        outRunning = (st & 0x0001) != 0;
        outAlarm = (st & 0x0018) != 0; // Bit 3 (Trip) or Bit 4 (Alarm)

        vTaskDelay(pdMS_TO_TICKS(10));
        WS43B.rs485.clearRxBuffer();

        // Read 0x2124 - 0x2126 (Output Frequency, Voltage, Current)
        res = modbus.readHoldingRegisters(0x2124, 3);
        if (res == modbus.ku8MBSuccess) {
            uint16_t rawF = modbus.getResponseBuffer(0);
            uint16_t rawC = modbus.getResponseBuffer(2);
            outFreq = (rawF > 600) ? ((float)rawF / 100.0f) : ((float)rawF / 10.0f);
            outCurr = (rawC > 300) ? ((float)rawC / 100.0f) : ((float)rawC / 10.0f);
        }
    } else {
        if (failCount < 5) failCount++;
        // ป้องกันไฟสถานะกระพริบติดๆ ดับๆ: ต้องพลาดติดต่อกัน 3 ครั้งขึ้นไปจึงจะตัดสินว่า Offline
        if (failCount >= 3) {
            modbusOk = false;
            outRunning = false;
        }
    }
}

static void pollPressureSensor() {
    static uint8_t pressFailCount = 0;
    modbus.begin(PRESSURE_SLAVE_ID, Serial1);
    modbus.idle(modbus_idle_yield);
    WS43B.rs485.clearRxBuffer();

    uint8_t result = modbus.readHoldingRegisters(PRESSURE_START_REG, 1);
    if (result == modbus.ku8MBIllegalFunction) {
        vTaskDelay(pdMS_TO_TICKS(5));
        WS43B.rs485.clearRxBuffer();
        result = modbus.readInputRegisters(PRESSURE_START_REG, 1);
    }

    if (result == modbus.ku8MBSuccess) {
        pressFailCount = 0;
        pressure_sensor_ok = true;
        uint16_t raw = modbus.getResponseBuffer(0);
        // ID6, Reg 0, 0 - 6.00 bar:
        float p = (raw > 600) ? ((float)raw / 1000.0f) : ((float)raw / 100.0f);
        shared_pressure_bar = p;
    } else {
        if (pressFailCount < 5) pressFailCount++;
        if (pressFailCount >= 3) {
            pressure_sensor_ok = false;
        }
    }
}

// -----------------------------------------------------------------------------
// Core 0 Dedicated Communication Task
// -----------------------------------------------------------------------------
void commTask(void *param) {
    Serial.printf("[TASK] Communication Task running on Core %d\n", xPortGetCoreID());

    // Give the IO slave the bus first at boot; a missing sensor must not block
    // the first manual output command for the Modbus timeout period.
    unsigned long lastPollTime = millis();
    unsigned long lastDIPollTime = 0;
    uint8_t local_last_di0 = 0xFF;
    uint8_t local_last_di1 = 0xFF;

    // Inverter PID controller states
    float pidIntegral = 0.0f;
    uint32_t sleepStartTime = 0;
    bool inSleep = false;
    unsigned long lastPressurePollTime = 0;
    unsigned long lastInverterPollTime = 0;
    unsigned long lastPidComputeTime = millis();
    bool lastP1Run = false;
    float lastP1Freq = -1.0f;
    bool lastP2Run = false;
    float lastP2Freq = -1.0f;

    while (true) {
        // 1. ส่งสถานะ output เมื่อมีการเปลี่ยนแปลง หรือครั้งแรกหลังเชื่อมต่อ
        if (rs485_output_synced && pendingIoMask != 0) {
            pendingIoMask = 0;
            rs485Io.begin(RS485_IO_SLAVE_ID, Serial1);
            rs485Io.setTransmitBuffer(0, shared_rs485_outputs & 0xFF);
            const uint8_t result = rs485Io.writeMultipleCoils(0, 8); // 0x0F, coils 0..7
            if (result == rs485Io.ku8MBSuccess)
                rs485_io_ok = true;
            else {
                pendingIoMask = 0xFF;
                rs485_io_ok = false;
            }
        }

        // 2. จัดการคำขอสแกนหา Slave ID (จากปุ่ม SCAN MB)
        if (requestScanModbus) {
            requestScanModbus = false;
            doModbusScan();
            lastPollTime = millis();
        }

        // 3. อ่านค่า Pressure Transmitter (Slave ID 6, Reg 0)
        const uint32_t pressPollInterval = pressure_sensor_ok ? DEFAULT_PRESSURE_POLL_MS : 2500;
        if (millis() - lastPressurePollTime >= pressPollInterval) {
            lastPressurePollTime = millis();
            pollPressureSensor();
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // 4. Inverter PID & Sleep Controller Logic (คำนวณและสั่งงานทุก 100ms)
        if (millis() - lastPidComputeTime >= 100) {
            const float dt = (millis() - lastPidComputeTime) / 1000.0f;
            lastPidComputeTime = millis();

            // ตรวจสอบระดับน้ำต่ำกว่า lowerLimit หรือไม่ (Protection)
            float levelCm = (float)shared_reg0 / 10.0f;
            bool levelOk = (levelCm >= waterSettings.lowerLimitCm);

            bool p1TargetRun = false;
            float p1TargetFreq = 0.0f;
            bool p2TargetRun = false;
            float p2TargetFreq = 0.0f;

            if (waterSettings.autoMode) {
                if (!levelOk) {
                    // น้ำในถังต่ำกว่า Lower Limit -> สั่งตัดปั๊มในโหมด auto เพื่อความปลอดภัย
                    inSleep = false;
                    sleepStartTime = 0;
                    p1TargetRun = false;
                    p2TargetRun = false;
                    shared_pid_target_freq = 0.0f;
                    shared_is_sleeping = false;
                } else if (waterSettings.sensorTypeSwitch) {
                    // --- AUTO MODE with PRESSURE SWITCH (จาก pressure_switch_io_pin) ---
                    inSleep = false;
                    sleepStartTime = 0;
                    shared_is_sleeping = false;

                    if (shared_pressure_switch) {
                        // Pressure switch ON (แรงดันตก/สั่งปั๊มทำงาน) -> Run pumps
                        float spd1 = constrain(waterSettings.manualSpeedPump1, waterSettings.sleepFreq, 50.0f);
                        float spd2 = constrain(waterSettings.manualSpeedPump2, waterSettings.sleepFreq, 50.0f);
                        shared_pid_target_freq = max(spd1, spd2);

                        if (waterSettings.autoPump1 && !shared_pump1_alarm) {
                            p1TargetRun = true;
                            p1TargetFreq = spd1;
                        }
                        if (waterSettings.autoPump2 && !shared_pump2_alarm) {
                            p2TargetRun = true;
                            p2TargetFreq = spd2;
                        }
                    } else {
                        // Pressure switch OFF (แรงดันได้ที่แล้ว) -> หยุดปั๊ม
                        p1TargetRun = false;
                        p1TargetFreq = 0.0f;
                        p2TargetRun = false;
                        p2TargetFreq = 0.0f;
                        shared_pid_target_freq = 0.0f;
                    }
                } else {
                    // --- AUTO PID MODE with RS485 SENSOR (Slave ID 6) ---
                    const float sp = waterSettings.setPressure;
                    const float pv = shared_pressure_bar;
                    const float err = sp - pv;

                    const float pTerm = waterSettings.pidKp * err;
                    pidIntegral += waterSettings.pidKi * err * dt;
                    pidIntegral = constrain(pidIntegral, 0.0f, 50.0f);

                    float rawCalc = pTerm + pidIntegral;

                    // ตรวจสอบเงื่อนไข Sleep: ความถี่ลงมาที่หรือต่ำกว่า SetSleepFreq เป็นเวลาที่กำหนดใน SetSleepDelay
                    if (rawCalc <= waterSettings.sleepFreq) {
                        if (sleepStartTime == 0) sleepStartTime = millis();
                        if (millis() - sleepStartTime >= (uint32_t)(waterSettings.sleepDelaySec * 1000UL)) {
                            inSleep = true;
                        }
                    } else {
                        sleepStartTime = 0;
                    }

                    // ตรวจสอบเงื่อนไข Wake-up: เมื่อแรงดันตกต่ำกว่า Setpoint (ต่ำกว่า 0.30 bar) ให้ตื่นขึ้นมาทำงานใหม่
                    if (inSleep) {
                        if (pv <= (sp - 0.30f)) {
                            inSleep = false;
                            sleepStartTime = 0;
                            pidIntegral = waterSettings.sleepFreq; // เริ่มต้นนุ่มนวล
                            rawCalc = waterSettings.sleepFreq;
                        }
                    }

                    float finalFreq = 0.0f;
                    if (inSleep) {
                        finalFreq = 0.0f;
                    } else {
                        // SetSleepFreq จะควบคุม ความเร็วต่ำสุดด้วย, ค่า max 50Hz
                        finalFreq = constrain(rawCalc, waterSettings.sleepFreq, 50.0f);
                    }

                    shared_pid_target_freq = finalFreq;
                    shared_is_sleeping = inSleep;

                    if (!inSleep && finalFreq >= 1.0f) {
                        if (waterSettings.autoPump1 && !shared_pump1_alarm) {
                            p1TargetRun = true;
                            p1TargetFreq = finalFreq;
                        }
                        if (waterSettings.autoPump2 && !shared_pump2_alarm) {
                            p2TargetRun = true;
                            p2TargetFreq = finalFreq;
                        }
                    }
                }
            } else {
                // --- MANUAL MODE ---
                inSleep = false;
                sleepStartTime = 0;
                shared_is_sleeping = false;
                shared_pid_target_freq = 0.0f;

                // SetSpeedPump1, SetSpeedPump2 สำหรับตั้ง speed เป็น Hz ค่า min กำหนดตามค่า SetSleepFreq ค่า max 50Hz
                float spd1 = constrain(waterSettings.manualSpeedPump1, waterSettings.sleepFreq, 50.0f);
                float spd2 = constrain(waterSettings.manualSpeedPump2, waterSettings.sleepFreq, 50.0f);

                if (waterSettings.manualPump1 && !shared_pump1_alarm) {
                    p1TargetRun = true;
                    p1TargetFreq = spd1;
                }
                if (waterSettings.manualPump2 && !shared_pump2_alarm) {
                    p2TargetRun = true;
                    p2TargetFreq = spd2;
                }
            }

            // ส่งคำสั่งไปยัง Inverter เมื่อสถานะเปลี่ยน หรือส่งย้ำทุก 3 วินาทีเฉพาะตอนที่ปั๊มกำลัง Run
            static uint32_t lastCmdSendTime = 0;
            const bool stateChanged = (p1TargetRun != lastP1Run || abs(p1TargetFreq - lastP1Freq) > 0.2f ||
                                       p2TargetRun != lastP2Run || abs(p2TargetFreq - lastP2Freq) > 0.2f);
            const bool periodicRefresh = (p1TargetRun || p2TargetRun) && (millis() - lastCmdSendTime >= 3000);
            if (stateChanged || periodicRefresh) {
                lastCmdSendTime = millis();
                lastP1Run = p1TargetRun;
                lastP1Freq = p1TargetFreq;
                lastP2Run = p2TargetRun;
                lastP2Freq = p2TargetFreq;

                if (p1TargetRun || stateChanged) {
                    sendInverterCmd(INVERTER_PUMP1_SLAVE_ID, p1TargetRun, p1TargetFreq);
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                if (p2TargetRun || (stateChanged && pump2_modbus_ok)) {
                    sendInverterCmd(INVERTER_PUMP2_SLAVE_ID, p2TargetRun, p2TargetFreq);
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }

            // ส่งค่า Acc/Dec ไปยัง Inverter เมื่อค่ามีการเปลี่ยนแปลง หรือเมื่อเริ่มต้น
            static float lastSentAccDecP1 = -1.0f;
            static float lastSentAccDecP2 = -1.0f;
            static uint32_t lastAccDecRetryTime = 0;
            const float currentAccDec = constrain(waterSettings.accDec, 0.1f, 600.0f);

            if (millis() - lastAccDecRetryTime >= 2000) {
                lastAccDecRetryTime = millis();
                if (abs(currentAccDec - lastSentAccDecP1) > 0.05f) {
                    if (sendInverterAccDec(INVERTER_PUMP1_SLAVE_ID, currentAccDec)) {
                        lastSentAccDecP1 = currentAccDec;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                if (pump2_modbus_ok && abs(currentAccDec - lastSentAccDecP2) > 0.05f) {
                    if (sendInverterAccDec(INVERTER_PUMP2_SLAVE_ID, currentAccDec)) {
                        lastSentAccDecP2 = currentAccDec;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }

        // 5. อ่านสถานะ Inverter 1 & Inverter 2 แยกอิสระ สลับคิวกันเพื่อไม่ให้แย่งบัสพร้อมกัน
        static unsigned long lastP1PollTime = 0;
        static unsigned long lastP2PollTime = 0;
        static uint8_t p1FailCount = 0;
        static uint8_t p2FailCount = 0;
        static bool initP2Offset = false;
        if (!initP2Offset) {
            initP2Offset = true;
            // เหลื่อมเวลาครึ่งรอบ เพื่อไม่ให้ Inverter 1 และ 2 ส่งคำถามชนกันในมิลลิวินาทีเดียวกัน
            lastP2PollTime = millis() + (DEFAULT_INVERTER_POLL_MS / 2);
        }

        const uint32_t p1Interval = pump1_modbus_ok ? DEFAULT_INVERTER_POLL_MS : 1000;
        const uint32_t p2Interval = pump2_modbus_ok ? DEFAULT_INVERTER_POLL_MS : 1000;

        if (millis() - lastP1PollTime >= p1Interval) {
            lastP1PollTime = millis();
            pollInverter(INVERTER_PUMP1_SLAVE_ID, shared_pump1_freq, shared_pump1_current,
                         shared_pump1_status, shared_pump1_running, shared_pump1_alarm, pump1_modbus_ok, p1FailCount);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (millis() - lastP2PollTime >= p2Interval) {
            lastP2PollTime = millis();
            pollInverter(INVERTER_PUMP2_SLAVE_ID, shared_pump2_freq, shared_pump2_current,
                         shared_pump2_status, shared_pump2_running, shared_pump2_alarm, pump2_modbus_ok, p2FailCount);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // 6. รอบอ่านค่า Water Level Sensor ตามรอบเวลา
#if !WATER_LEVEL_SIMULATION && WATER_LEVEL_SENSOR_PRESENT
        const uint32_t sensorPollInterval = MODBUS_POLL_MS;
        if (millis() - lastPollTime >= sensorPollInterval) {
            lastPollTime = millis();
            doModbusPoll();
        }
#endif
#if WATER_METER_SENSOR_PRESENT
        static unsigned long lastMeterPollTime = 0;
        if (millis() - lastMeterPollTime >= WATER_METER_POLL_MS) {
            lastMeterPollTime = millis();
            pollWaterMeter();
        }
#endif
        static unsigned long lastIoPollTime = 0;
        if (millis() - lastIoPollTime >= RS485_IO_POLL_MS) {
            lastIoPollTime = millis();
            pollRs485Io();
        }

        // 7. รับและประมวลผลเฟรม CAN Bus (TWAI)
        twai_message_t canRx;
        if (WS43B.can.receiveMessage(canRx, 2)) {
            char cbuf[128];
            int offset = snprintf(cbuf, sizeof(cbuf), "[CAN RX] ID:0x%X DLC:%d ", canRx.identifier, canRx.data_length_code);
            for (int i = 0; i < canRx.data_length_code && offset < (int)sizeof(cbuf) - 5; i++) {
                offset += snprintf(cbuf + offset, sizeof(cbuf) - offset, "%02X ", canRx.data[i]);
            }
            strncat(cbuf, "\n", sizeof(cbuf) - strlen(cbuf) - 1);
            logMsg(cbuf);
        }

        // 8. ตรวจสอบ Digital Inputs จาก CH422G (อ่านทุก 100ms เพื่อไม่แย่ง I2C bus กับทัชสกรีน)
        if (millis() - lastDIPollTime >= 100) {
            lastDIPollTime = millis();
            uint8_t inputs = EX_Read_All();
            bool current_di0 = (inputs >> DI0) & 1;
            bool current_di1 = (inputs >> DI1) & 1;

            if (current_di0 != local_last_di0) {
                local_last_di0 = current_di0;
                shared_di0 = current_di0;
                di0_changed = true;
                char dbuf[32];
                snprintf(dbuf, sizeof(dbuf), "[DIN0] -> %s\n", current_di0 ? "HIGH" : "LOW");
                logMsg(dbuf);
            }

            if (current_di1 != local_last_di1) {
                local_last_di1 = current_di1;
                shared_di1 = current_di1;
                di1_changed = true;
                char dbuf[32];
                snprintf(dbuf, sizeof(dbuf), "[DIN1] -> %s\n", current_di1 ? "HIGH" : "LOW");
                logMsg(dbuf);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Sleep 10ms to yield CPU
    }
}

// -----------------------------------------------------------------------------
// UI Event Handlers (Executed inside LVGL task on Core 1 - Instant & Smooth!)
// -----------------------------------------------------------------------------
void DO0_OUTPUT(lv_event_t *e) {
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    bool state = lv_obj_has_state(obj, LV_STATE_CHECKED);

    if (state) {
        DO0_ON;
    } else {
        DO0_OFF;
    }

    pendingCoilIndex = 0;
    pendingCoilState = state;

    uint8_t canData[2] = {0x00, (uint8_t)(state ? 1 : 0)};
    WS43B.can.send(0x200, canData, 2);

    logMsg(state ? "[DO0] -> ON\n" : "[DO0] -> OFF\n");
}

void DO1_OUTPUT(lv_event_t *e) {
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    bool state = lv_obj_has_state(obj, LV_STATE_CHECKED);

    if (state) {
        DO1_ON;
    } else {
        DO1_OFF;
    }

    pendingCoilIndex = 1;
    pendingCoilState = state;

    uint8_t canData[2] = {0x01, (uint8_t)(state ? 1 : 0)};
    WS43B.can.send(0x200, canData, 2);

    logMsg(state ? "[DO1] -> ON\n" : "[DO1] -> OFF\n");
}

static void set_output_state(uint8_t output, bool on) {
    if (output == 0) {
        if (on)
            DO0_ON;
        else
            DO0_OFF;
    } else {
        if (on)
            DO1_ON;
        else
            DO1_OFF;
    }
}

static void set_rs485_output(uint8_t coil, bool on) {
    if (coil > 7)
        return;
    const uint8_t mask = (uint8_t)(1u << coil);
    const bool current = (shared_rs485_outputs & mask) != 0;
    if (current == on)
        return;
    // Update the shadow immediately so a single control pass can queue
    // several different coils without one command replacing another.
    if (on) shared_rs485_outputs |= mask;
    else shared_rs485_outputs &= (uint8_t)~mask;
    pendingIoMask |= mask;
}

static float get_current_level_cm() {
#if WATER_LEVEL_SIMULATION
    return (simulatedLevelPct / 100.0f) * configuredLevelMaxCm();
#else
    return (float)shared_reg0 / 10.0f;
#endif
}

static float get_lower_limit_cm() {
    float limit = waterSettings.lowerLimitCm;
    if (ui_SetLowerLimit) {
        const char *text = lv_textarea_get_text(ui_SetLowerLimit);
        if (text && *text) {
            char *end = nullptr;
            const float val = strtof(text, &end);
            if (end != text && isfinite(val)) {
                limit = val;
            }
        }
    }
    return limit;
}

static void update_pump_status_outputs() {
    // 1. ไฟสถานะรัน Pump 1 (CH1) และ Pump 2 (CH2)
    set_rs485_output(pump1_run_io_pin, shared_pump1_running);
    set_rs485_output(pump2_run_io_pin, shared_pump2_running);

    // 2. ไฟ Alarm ของ Pump 1 (CH3) และ Pump 2 (CH4)
    // ติดเมื่อ Inverter แจ้ง Alarm/Trip หรือเกิด Overload จากสวิตช์ DI
    set_rs485_output(pump1_alarm_io_pin, shared_pump1_alarm || shared_pump1_overload);
    set_rs485_output(pump2_alarm_io_pin, shared_pump2_alarm || shared_pump2_overload);
}

static void update_auto_pumps_from_pressure(float levelCm) {
    // Inverter PID and sleep control is managed on Core 0 in commTask()
}

static bool update_automatic_outputs(float levelCm) {
    static bool valveOn = false;
    static uint32_t valveLowSince = 0;
    static uint32_t valveHighSince = 0;
    const uint32_t now = millis();
    const bool autoMode = ui_ManualAuto && lv_obj_has_state(ui_ManualAuto, LV_STATE_CHECKED);

    if (autoMode && !manualValveOverride) {
        if (!valveOn && levelCm <= waterSettings.valveOnCm) {
            if (valveLowSince == 0)
                valveLowSince = now;
            if (now - valveLowSince >= waterSettings.valvePeriodSec * 1000UL)
                valveOn = true;
        } else {
            valveLowSince = 0;
        }
        // SetOnValve is the fill target: close the valve after the level
        // reaches the configured value, using the same continuous period.
        if (valveOn && levelCm >= waterSettings.valveOnCm) {
            if (valveHighSince == 0)
                valveHighSince = now;
            if (now - valveHighSince >= waterSettings.valvePeriodSec * 1000UL)
                valveOn = false;
        } else {
            valveHighSince = 0;
        }
        set_rs485_output(valve_io_pin, valveOn);
    } else if (!autoMode) {
        valveOn = (shared_rs485_outputs & (1u << valve_io_pin)) != 0;
    }

    update_auto_pumps_from_pressure(levelCm);

    const bool lowCondition = levelCm <= waterSettings.alertMinCm;
    const bool highCondition = levelCm >= waterSettings.alertMaxCm;
    if (lowCondition) {
        if (alertLowSince == 0)
            alertLowSince = now;
    } else {
        alertLowSince = 0;
    }
    if (highCondition) {
        if (alertHighSince == 0)
            alertHighSince = now;
    } else {
        alertHighSince = 0;
    }

    const uint32_t requiredMs = waterSettings.alertPeriodSec * 1000UL;
    const bool timedOut = (alertLowSince != 0 && now - alertLowSince >= requiredMs) ||
                          (alertHighSince != 0 && now - alertHighSince >= requiredMs);
    alertOutputOn = timedOut;
    set_rs485_output(alarm_io_pin, alertOutputOn);

    update_pump_status_outputs();
    return valveOn;
}

static void set_pump_led(lv_obj_t *led, bool running, bool alarm) {
    if (!led)
        return;
    if (alarm) {
        lv_obj_set_style_bg_opa(led, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(led, lv_color_hex(0xF97316), LV_PART_MAIN | LV_STATE_DEFAULT); // Orange (Inverter alarm/trip)
        lv_obj_set_style_shadow_color(led, lv_color_hex(0xF97316), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(led, 16, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(led, LV_OPA_60, LV_PART_MAIN | LV_STATE_DEFAULT);
    } else if (running) {
        lv_obj_set_style_bg_opa(led, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(led, lv_color_hex(0x22C55E), LV_PART_MAIN | LV_STATE_DEFAULT); // Green
        lv_obj_set_style_shadow_color(led, lv_color_hex(0x22C55E), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(led, 16, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(led, LV_OPA_60, LV_PART_MAIN | LV_STATE_DEFAULT);
    } else {
        lv_obj_set_style_bg_opa(led, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(led, lv_color_hex(0x3F0F16), LV_PART_MAIN | LV_STATE_DEFAULT); // Dark / Off
        lv_obj_set_style_shadow_color(led, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(led, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(led, LV_OPA_0, LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

static void update_manual_leds(bool force) {
    const bool localDo0On = DO_Read(DO0) == 0;
    const bool localDo1On = DO_Read(DO1) == 0;

    static bool prev_meter485 = false;
    static bool prev_level485 = false;
    static bool prev_io485 = false;
    static bool prev_pump1_mb = false;
    static bool prev_pump2_mb = false;
    static bool prev_press_mb = false;
    static uint8_t prev_outputs = 0xFF;
    static uint8_t prev_inputs = 0xFF;
    static bool prev_di0 = false;
    static bool prev_di1 = false;
    static bool prev_do0 = false;
    static bool prev_do1 = false;

    bool ioChanged = force ||
                     (prev_meter485 != water_meter_ok) ||
                     (prev_level485 != lastRs485Ok) ||
                     (prev_io485 != rs485_io_ok) ||
                     (prev_pump1_mb != pump1_modbus_ok) ||
                     (prev_pump2_mb != pump2_modbus_ok) ||
                     (prev_press_mb != pressure_sensor_ok) ||
                     (prev_outputs != shared_rs485_outputs) ||
                     (prev_inputs != shared_rs485_inputs) ||
                     (prev_di0 != shared_di0) ||
                     (prev_di1 != shared_di1) ||
                     (prev_do0 != localDo0On) ||
                     (prev_do1 != localDo1On);

    if (ioChanged) {
        prev_meter485 = water_meter_ok;
        prev_level485 = lastRs485Ok;
        prev_io485 = rs485_io_ok;
        prev_pump1_mb = pump1_modbus_ok;
        prev_pump2_mb = pump2_modbus_ok;
        prev_press_mb = pressure_sensor_ok;
        prev_outputs = shared_rs485_outputs;
        prev_inputs = shared_rs485_inputs;
        prev_di0 = shared_di0;
        prev_di1 = shared_di1;
        prev_do0 = localDo0On;
        prev_do1 = localDo1On;

        auto set_monitor_alpha = [](lv_obj_t *obj, bool on) {
            if (obj)
                lv_obj_set_style_bg_opa(obj, on ? LV_OPA_COVER : 100, LV_PART_MAIN | LV_STATE_DEFAULT);
        };

        // PanelMonitor LEDs
        set_status_led(ui_Rs485LedMeter, water_meter_ok, 0x22C55E);         // Meter (Slave 1)
        set_status_led(ui_Rs485LedIo, rs485_io_ok, 0x22C55E);               // RS485 IO (Slave 2)
        set_status_led(ui_Rs485LedLevel, lastRs485Ok, 0x22C55E);            // Level Sensor (Slave 3)
        set_status_led(ui_Rs485LedPump1, pump1_modbus_ok, 0x22C55E);        // Inverter Pump 1 (Slave 4)
        set_status_led(ui_Rs485LedPump2, pump2_modbus_ok, 0x22C55E);        // Inverter Pump 2 (Slave 5)
        set_status_led(ui_Rs485LedPressure, pressure_sensor_ok, 0x22C55E);  // Pressure Sensor (Slave 6)
        set_status_led(ui_LedValve, (shared_rs485_outputs & (1u << valve_io_pin)) != 0, 0x22C55E);

        set_monitor_alpha(ui_LocalDI0, shared_di0);
        set_monitor_alpha(ui_LocalDI1, shared_di1);
        set_monitor_alpha(ui_LocalDO0, localDo0On);
        set_monitor_alpha(ui_LocalDO1, localDo1On);
        lv_obj_t *rs485Di[] = {ui_RS485DI1, ui_RS485DI2, ui_RS485DI3, ui_RS485DI4, ui_RS485DI5, ui_RS485DI6, ui_RS485DI7, ui_RS485DI8};
        lv_obj_t *rs485Do[] = {ui_RS485CH1, ui_RS485CH2, ui_RS485CH3, ui_RS485CH4, ui_RS485CH5, ui_RS485CH6, ui_RS485CH7, ui_RS485CH8};
        for (uint8_t i = 0; i < 8; ++i) {
            set_monitor_alpha(rs485Di[i], rs485_io_ok && ((shared_rs485_inputs >> i) & 1));
            set_monitor_alpha(rs485Do[i], rs485_io_ok && ((shared_rs485_outputs >> i) & 1));
        }
    }

    // Pump 1 & 2 LEDs (Yellow if alarm, Green if running, Dark/off if stopped)
    static int prev_p1_state = -1;
    static int prev_p2_state = -1;
    int p1_state = shared_pump1_alarm ? 2 : (shared_pump1_running ? 1 : 0);
    int p2_state = shared_pump2_alarm ? 2 : (shared_pump2_running ? 1 : 0);
    if (force || p1_state != prev_p1_state) {
        prev_p1_state = p1_state;
        set_pump_led(ui_LedPump1, shared_pump1_running, shared_pump1_alarm);
    }
    if (force || p2_state != prev_p2_state) {
        prev_p2_state = p2_state;
        set_pump_led(ui_LedPump2, shared_pump2_running, shared_pump2_alarm);
    }

    // Status text in ui_LabelPumpStatus (Auto panel) and ui_LabelPumpStatus1 (Manual panel):
    // Example: Pressure [ 2.45 bar ]   Pump 1  [ 38.5 Hz  3.8A ]  Pump 2  [ 38.5 Hz  3.8A ]
    char statusText[128];
    if (waterSettings.sensorTypeSwitch) {
        snprintf(statusText, sizeof(statusText),
                 "Pressure [ SW: %s ]   Pump 1  [ %.1f Hz  %.1fA ]  Pump 2  [ %.1f Hz  %.1fA ]",
                 shared_pressure_switch ? "ON" : "OFF",
                 shared_pump1_freq, shared_pump1_current,
                 shared_pump2_freq, shared_pump2_current);
    } else {
        snprintf(statusText, sizeof(statusText),
                 "Pressure [ %.2f bar ]   Pump 1  [ %.1f Hz  %.1fA ]  Pump 2  [ %.1f Hz  %.1fA ]",
                 shared_pressure_bar,
                 shared_pump1_freq, shared_pump1_current,
                 shared_pump2_freq, shared_pump2_current);
    }
    static char prevStatusText[128] = "";
    if (force || strcmp(prevStatusText, statusText) != 0) {
        strncpy(prevStatusText, statusText, sizeof(prevStatusText));
        if (ui_LabelPumpStatus) {
            lv_label_set_text(ui_LabelPumpStatus, statusText);
        }
        if (ui_LabelPumpStatus1) {
            lv_label_set_text(ui_LabelPumpStatus1, statusText);
        }
    }

    if (ui_SwitchAutoPump1 && lv_obj_has_state(ui_SwitchAutoPump1, LV_STATE_CHECKED) != waterSettings.autoPump1)
        lv_obj_set_state(ui_SwitchAutoPump1, LV_STATE_CHECKED, waterSettings.autoPump1);
    if (ui_SwitchAutoPump2 && lv_obj_has_state(ui_SwitchAutoPump2, LV_STATE_CHECKED) != waterSettings.autoPump2)
        lv_obj_set_state(ui_SwitchAutoPump2, LV_STATE_CHECKED, waterSettings.autoPump2);

    auto sync_manual_switch = [](lv_obj_t *sw, bool checked, bool disabled) {
        if (!sw) return;
        if (lv_obj_has_state(sw, LV_STATE_CHECKED) != checked)
            lv_obj_set_state(sw, LV_STATE_CHECKED, checked);
        const bool isDisabled = lv_obj_has_state(sw, LV_STATE_DISABLED);
        if (isDisabled != disabled) {
            if (disabled) {
                lv_obj_add_state(sw, LV_STATE_DISABLED);
                lv_obj_remove_flag(sw, LV_OBJ_FLAG_CLICKABLE);
            } else {
                lv_obj_remove_state(sw, LV_STATE_DISABLED);
                lv_obj_add_flag(sw, LV_OBJ_FLAG_CLICKABLE);
            }
        }
    };

    sync_manual_switch(ui_SwitchManualPump1, waterSettings.manualPump1, waterSettings.autoMode);
    sync_manual_switch(ui_SwitchManualPump2, waterSettings.manualPump2, waterSettings.autoMode);
    sync_manual_switch(ui_SwitchValve, waterSettings.manualValve, waterSettings.autoMode);

    if (ui_SwitchSensorType && lv_obj_has_state(ui_SwitchSensorType, LV_STATE_CHECKED) != waterSettings.sensorTypeSwitch)
        lv_obj_set_state(ui_SwitchSensorType, LV_STATE_CHECKED, waterSettings.sensorTypeSwitch);

    if (ui_ManualAuto && lv_obj_has_state(ui_ManualAuto, LV_STATE_CHECKED) != waterSettings.autoMode)
        lv_obj_set_state(ui_ManualAuto, LV_STATE_CHECKED, waterSettings.autoMode);
}

static void manual_output_event(lv_event_t *e) {
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    const bool state = lv_obj_has_state(obj, LV_STATE_CHECKED);
    void *userData = lv_event_get_user_data(e);
    const uint8_t output = (uint8_t)(uintptr_t)userData;

    // userData: 0 = Auto Pump 1, 1 = Auto Pump 2, 2 = Manual Pump 1, 3 = Manual Pump 2, 4 = Valve
    if (output == 0) {
        waterSettings.autoPump1 = state;
    } else if (output == 1) {
        waterSettings.autoPump2 = state;
    } else if (output == 2) {
        if (waterSettings.autoMode) {
            lv_obj_set_state(obj, LV_STATE_CHECKED, false);
            return;
        }
        waterSettings.manualPump1 = state;
    } else if (output == 3) {
        if (waterSettings.autoMode) {
            lv_obj_set_state(obj, LV_STATE_CHECKED, false);
            return;
        }
        waterSettings.manualPump2 = state;
    } else if (output == 4) {
        if (waterSettings.autoMode) {
            lv_obj_set_state(obj, LV_STATE_CHECKED, false);
            return;
        }
        waterSettings.manualValve = state;
        manualValveOverride = state;
        set_rs485_output(valve_io_pin, state);
    }
    save_water_settings();
    update_manual_leds();
}

static void local_output_test_event(lv_event_t *e) {
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    const uint8_t output = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    const bool state = !lv_obj_has_state(obj, LV_STATE_CHECKED);
    lv_obj_set_state(obj, LV_STATE_CHECKED, state);
    set_output_state(output, state);
    update_manual_leds();
}

static void rs485_output_test_event(lv_event_t *e) {
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    const uint8_t coil = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    const bool state = !lv_obj_has_state(obj, LV_STATE_CHECKED);
    lv_obj_set_state(obj, LV_STATE_CHECKED, state);
    set_rs485_output(coil, state);
    update_manual_leds();
}

static void select_panel(lv_obj_t *panel) {
    if (!panel)
        return;
    lv_obj_add_flag(ui_LineChart, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_PanelSettings, LV_OBJ_FLAG_HIDDEN);
    if (ui_PanelAuto) lv_obj_add_flag(ui_PanelAuto, LV_OBJ_FLAG_HIDDEN);
    if (ui_PanelManual) lv_obj_add_flag(ui_PanelManual, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_PanelMonitor, LV_OBJ_FLAG_HIDDEN);
    if (ui_PanelWiFiAndClock) lv_obj_add_flag(ui_PanelWiFiAndClock, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_HIDDEN);
}

static void select_chart(lv_event_t *) {
    select_panel(ui_LineChart);
    if (ui_LabelMenu) lv_label_set_text(ui_LabelMenu, "LEVEL & FLOW TREND (REAL-TIME)");
}

static void select_settings(lv_event_t *) {
    update_settings_ui();
    select_panel(ui_PanelSettings);
    if (ui_LabelMenu) lv_label_set_text(ui_LabelMenu, "SETTINGS CONTROL");
}

static void select_wifi_clock(lv_event_t *) {
    select_panel(ui_PanelWiFiAndClock);
    update_wifi_status();
    if (ui_LabelMenu) lv_label_set_text(ui_LabelMenu, "COMMUNICATION & POWER SETTINGS");
}

static void select_auto(lv_event_t *) {
    select_panel(ui_PanelAuto);
    update_settings_ui();
    update_mode_ui();
    update_manual_leds(true);
    if (ui_LabelMenu) lv_label_set_text(ui_LabelMenu, "AUTO PUMP CONTROL");
}

static void select_manual(lv_event_t *) {
    select_panel(ui_PanelManual);
    update_settings_ui();
    update_mode_ui();
    update_manual_leds(true);
    if (ui_LabelMenu) lv_label_set_text(ui_LabelMenu, "MANUAL PUMP CONTROL");
}

static void select_auto_manual_menu(lv_event_t *e) {
    if (waterSettings.autoMode) {
        select_auto(e);
    } else {
        select_manual(e);
    }
}

static void select_monitor(lv_event_t *) {
    select_panel(ui_PanelMonitor);
    update_manual_leds();
    if (ui_LabelMenu) lv_label_set_text(ui_LabelMenu, "MONITOR IO STATUS");
}

void toggleBacklight(lv_event_t *e) {
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    bool state = lv_obj_has_state(obj, LV_STATE_CHECKED);

    if (state) {
        BL_ON;
    } else {
        BL_OFF;
    }
    logMsg(state ? "[Backlight] -> ON\n" : "[Backlight] -> OFF\n");
}

void onButtonAction(lv_event_t *e) {
    requestScanModbus = true;
    logMsg("\n>>> Requested Modbus Scan <<<\n");
}

// -----------------------------------------------------------------------------
// Water-level UI
// -----------------------------------------------------------------------------
static lv_coord_t waterTankMaxHeight = 285; // Maximum height of the water tank in pixels (matches the UI design)

static void set_status_led(lv_obj_t *led, bool enabled, uint32_t on_color) {
    if (!led)
        return;
    lv_obj_set_style_bg_opa(led, enabled ? LV_OPA_COVER : 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(led, enabled ? lv_color_hex(on_color) : lv_color_hex(0x3F0F16), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(led, enabled ? lv_color_hex(on_color) : lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(led, enabled ? 16 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(led, enabled ? LV_OPA_60 : LV_OPA_0, LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void update_water_level_ui(float level_pct, float level_cm, float volume_l, float flow_rate, bool valve_on, bool pump1_on,
                                  bool pump2_on) {
    if (!ui_WaterTank)
        return;

    float clamped_pct = level_pct;
    if (clamped_pct < 0.0f) clamped_pct = 0.0f;
    if (clamped_pct > 100.0f) clamped_pct = 100.0f;

    char text[32];

    if (!lastRs485Ok) {
        lv_obj_set_height(ui_WaterTank, 0);
        if (ui_WaterTankLevel) {
            lv_label_set_text(ui_WaterTankLevel, "ERROR");
        }
        if (ui_LabelWaterCm) {
            lv_label_set_text(ui_LabelWaterCm, "Level: ERROR");
        }
        if (ui_LabelWaterStatus){
            lv_label_set_text(ui_LabelWaterStatus, "NOT CONNECTED");
            lv_obj_set_style_bg_color(ui_PanelWaterStatus, lv_color_hex(0x7F1D1D), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_color(ui_PanelWaterStatus, lv_color_hex(0xEF4444), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(ui_LabelWaterStatus, lv_color_hex(0xF87171), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    } else {
        const lv_coord_t water_height = (lv_coord_t)((waterTankMaxHeight * clamped_pct / 100.0f) + 0.5f);
        lv_obj_set_height(ui_WaterTank, water_height);

        if (ui_WaterTankLevel) {
            char tank_text[12];
            snprintf(tank_text, sizeof(tank_text), "%d%%", (int)(clamped_pct + 0.5f));
            lv_label_set_text(ui_WaterTankLevel, tank_text);
        }

        snprintf(text, sizeof(text), "Level: %.1f cm", level_cm);
        if (ui_LabelWaterCm)
            lv_label_set_text(ui_LabelWaterCm, text);
    }

    if (waterLevelChartSeries && ui_LineChart) {
        // Data1 follows the WaterTankLevel percentage exactly.
        lv_chart_set_next_value(ui_LineChart, waterLevelChartSeries, lastRs485Ok ? (int32_t)(clamped_pct + 0.5f) : 0);
        // Data2 follows LabelFlowRate. Missing flow data is represented by 0.
        if (flowRateChartSeries)
            lv_chart_set_next_value(ui_LineChart, flowRateChartSeries, (int32_t)(flow_rate >= 0.0f ? flow_rate : 0.0f));
        lv_chart_refresh(ui_LineChart);
    }

    snprintf(text, sizeof(text), "%.3f", volume_l);
    if (ui_LabelVolumn)
        lv_label_set_text(ui_LabelVolumn, text);

    snprintf(text, sizeof(text), "%.3f", flow_rate);
    if (ui_LabelFlowRate)
        lv_label_set_text(ui_LabelFlowRate, text);

    set_status_led(ui_LedValve, valve_on, 0x22C55E);
    set_pump_led(ui_LedPump1, shared_pump1_running, shared_pump1_alarm);
    set_pump_led(ui_LedPump2, shared_pump2_running, shared_pump2_alarm);

    if (level_cm < waterSettings.alertMinCm) {
        lv_label_set_text(ui_LabelWaterStatus, "LOW LEVEL");
        lv_obj_set_style_bg_color(ui_PanelWaterStatus, lv_color_hex(0x78350F), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(ui_PanelWaterStatus, lv_color_hex(0xF59E0B), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(ui_LabelWaterStatus, lv_color_hex(0xF59E0B), LV_PART_MAIN | LV_STATE_DEFAULT);
    } else if (level_cm >= waterSettings.alertMaxCm) {
        lv_label_set_text(ui_LabelWaterStatus, "HIGH LEVEL");
        lv_obj_set_style_bg_color(ui_PanelWaterStatus, lv_color_hex(0x7F1D1D), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(ui_PanelWaterStatus, lv_color_hex(0xEF4444), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(ui_LabelWaterStatus, lv_color_hex(0xF87171), LV_PART_MAIN | LV_STATE_DEFAULT);
    } else {
        lv_label_set_text(ui_LabelWaterStatus, "NORMAL LEVEL");
        lv_obj_set_style_bg_color(ui_PanelWaterStatus, lv_color_hex(0x064E3B), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(ui_PanelWaterStatus, lv_color_hex(0x10B981), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(ui_LabelWaterStatus, lv_color_hex(0x10B981), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}


static void update_water_level_simulation() {
#if WATER_LEVEL_SIMULATION
    const uint32_t now = millis();
    if (now - lastSimulationUpdateMs < 1000)
        return;
    lastSimulationUpdateMs = now;

    // Random walk: values move continuously without abrupt unrealistic jumps.
    simulatedLevelPct += (float)random(-15, 16) / 10.0f;
    if (simulatedLevelPct < 5.0f)
        simulatedLevelPct = 5.0f;
    if (simulatedLevelPct > 95.0f)
        simulatedLevelPct = 95.0f;

    const float level_pct = simulatedLevelPct;
    const float level_cm = (level_pct / 100.0f) * configuredLevelMaxCm();
    const float volume_l = 0.0f; // Water meter is not installed yet.
    const float flow_rate = 0.0f;
    const bool valve_on = update_automatic_outputs(level_cm);
    const bool autoMode = ui_ManualAuto && lv_obj_has_state(ui_ManualAuto, LV_STATE_CHECKED);
    const float lowerLimit = get_lower_limit_cm();
    const bool levelOk = level_cm >= lowerLimit;
    const bool pump1_on = autoMode && waterSettings.autoPump1 && shared_pressure_switch && levelOk && !shared_pump1_overload;
    const bool pump2_on = autoMode && waterSettings.autoPump2 && shared_pressure_switch && levelOk && !shared_pump2_overload;

    if (lvgl_port_lock(10)) {
        update_water_level_ui(level_pct, level_cm, volume_l, flow_rate, valve_on, pump1_on, pump2_on);
        lvgl_port_unlock();
    }
#endif
}

static void setup_water_level_chart() {
    if (!ui_LineChart)
        return;

    // The exported chart contains placeholder series. Hide them and create a
    // runtime-owned 50-point chart series for WaterTank level and flow.
    for (lv_chart_series_t *series = lv_chart_get_series_next(ui_LineChart, NULL);
         series != NULL;
         series = lv_chart_get_series_next(ui_LineChart, series)) {
        lv_chart_hide_series(ui_LineChart, series, true);
    }
    lv_chart_set_point_count(ui_LineChart, 50);
    waterLevelChartSeries = lv_chart_add_series(ui_LineChart, lv_color_hex(0x13C2D4), LV_CHART_AXIS_PRIMARY_Y);
    flowRateChartSeries = lv_chart_add_series(ui_LineChart, lv_color_hex(0xF59E0B), LV_CHART_AXIS_PRIMARY_Y);
    if (waterLevelChartSeries)
        lv_chart_set_all_values(ui_LineChart, waterLevelChartSeries, 0);
    if (flowRateChartSeries)
        lv_chart_set_all_values(ui_LineChart, flowRateChartSeries, 0);
    lv_chart_refresh(ui_LineChart);
}

void setup_ui_events() {
    lv_obj_add_event_cb(ui_HomeButton, select_chart, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_SettingsButton, select_settings, LV_EVENT_CLICKED, NULL);
    if (ui_ManualAutoButton)
        lv_obj_add_event_cb(ui_ManualAutoButton, select_auto_manual_menu, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_MonitorButton, select_monitor, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_WiFiAndClock, select_wifi_clock, LV_EVENT_CLICKED, NULL);
    // SquareLine marks the child images as clickable, so a touch on an icon
    // may target the image instead of bubbling to its button parent.
    lv_obj_add_event_cb(ui_Image1, select_chart, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Image2, select_settings, LV_EVENT_CLICKED, NULL);
    if (ui_Image3)
        lv_obj_add_event_cb(ui_Image3, select_auto_manual_menu, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Image4, select_monitor, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Image6, select_wifi_clock, LV_EVENT_CLICKED, NULL);

    // Navigation buttons between ui_PanelAuto and ui_PanelManual
    if (ui_ManualButton)
        lv_obj_add_event_cb(ui_ManualButton, select_manual, LV_EVENT_CLICKED, NULL);
    if (ui_Image7)
        lv_obj_add_event_cb(ui_Image7, select_manual, LV_EVENT_CLICKED, NULL);
    if (ui_AutoButton)
        lv_obj_add_event_cb(ui_AutoButton, select_auto, LV_EVENT_CLICKED, NULL);
    if (ui_Image8)
        lv_obj_add_event_cb(ui_Image8, select_auto, LV_EVENT_CLICKED, NULL);

    lv_obj_t *numericInputs[] = {
        ui_SetOnValve, ui_SetTankCapacity, ui_SetPeriodTimeContorl,
        ui_SetMinAlert, ui_SetMaxAlert, ui_SetPeriodTimeAlert,
        ui_SetLowerLimit, ui_SetPidKp, ui_SetPidKi, ui_SetAccDec,
        ui_SetPressure, ui_SetSleepFreq, ui_SetSleepDelay,
        ui_SetSpeedPump1, ui_SetSpeedPump2
    };
    for (lv_obj_t *inp : numericInputs) {
        if (!inp) continue;
        lv_obj_add_event_cb(inp, save_settings_from_ui, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_event_cb(inp, save_settings_from_ui, LV_EVENT_READY, NULL);
        lv_obj_add_event_cb(inp, open_numeric_keyboard, LV_EVENT_CLICKED, NULL);
    }
    lv_obj_add_event_cb(ui_Keyboard, close_numeric_keyboard, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(ui_Keyboard, close_numeric_keyboard, LV_EVENT_CANCEL, NULL);

    if (ui_SwitchAutoPump1)
        lv_obj_add_event_cb(ui_SwitchAutoPump1, manual_output_event, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)0);
    if (ui_SwitchAutoPump2)
        lv_obj_add_event_cb(ui_SwitchAutoPump2, manual_output_event, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)1);
    if (ui_SwitchManualPump1)
        lv_obj_add_event_cb(ui_SwitchManualPump1, manual_output_event, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)2);
    if (ui_SwitchManualPump2)
        lv_obj_add_event_cb(ui_SwitchManualPump2, manual_output_event, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)3);
    if (ui_SwitchValve)
        lv_obj_add_event_cb(ui_SwitchValve, manual_output_event, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)4);
    if (ui_SwitchSensorType)
        lv_obj_add_event_cb(ui_SwitchSensorType, sensor_type_event, LV_EVENT_VALUE_CHANGED, NULL);
    if (ui_ManualAuto)
        lv_obj_add_event_cb(ui_ManualAuto, save_mode_settings, LV_EVENT_VALUE_CHANGED, NULL);
    for (lv_obj_t *obj : {ui_LocalDO0, ui_LocalDO1})
        if (obj) lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ui_LocalDO0, local_output_test_event, LV_EVENT_CLICKED, (void *)(uintptr_t)0);
    lv_obj_add_event_cb(ui_LocalDO1, local_output_test_event, LV_EVENT_CLICKED, (void *)(uintptr_t)1);
    lv_obj_t *rs485Do[] = {ui_RS485CH1, ui_RS485CH2, ui_RS485CH3, ui_RS485CH4,
                           ui_RS485CH5, ui_RS485CH6, ui_RS485CH7, ui_RS485CH8};
    for (uint8_t i = 0; i < 8; ++i) {
        if (rs485Do[i]) lv_obj_add_flag(rs485Do[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(rs485Do[i], rs485_output_test_event, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }
    select_panel(ui_LineChart);
    if (ui_LabelMenu) lv_label_set_text(ui_LabelMenu, "LEVEL AND FLOW TREND (REAL-TIME)");
}

// -----------------------------------------------------------------------------
// Arduino Setup & Loop
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(400);

    Serial.println("\n========================================================");
    Serial.println(" Waveshare ESP32-S3-Touch-LCD-4.3B: High-Speed System");
    Serial.println("========================================================");

    // Initialize Wi-Fi while the internal heap is still available. LVGL uses
    // DMA-capable internal buffers, so starting Wi-Fi after the UI is built
    // can fail with ESP_ERR_NO_MEM (257).
    init_wifi_once();

    // 1. สร้าง Queue สำหรับส่งข้อความ Log ข้าม Core แบบ Thread-safe
    logQueue = xQueueCreate(25, sizeof(LogItem));

    // 2. เริ่มต้นหน้าจอและระบบสัมผัส (รันบน Core 1 ด้วย True Double Buffer DMA)
    lvgl_port_init();

    // 3. เริ่มต้นชิปขยายขา CH422G และช่อง MicroSD
    initCH422G();
    initSDCARD();

    // 4. เริ่มต้นพอร์ต RS485 (GPIO 44 TX, GPIO 43 RX) 9600 baud, 8E1
    bool rs485Ok = WS43B.rs485.begin(MODBUS_BAUD_RATE, SERIAL_8E1, WS43B_RS485_RX_PIN, WS43B_RS485_TX_PIN);

    // PCF85063 shares the board I2C pins with the touch/IO hardware.
    const bool rtcOk = rtc.begin(WS43B_I2C_ADDR_PCF85063);
    Serial.println(rtcOk ? "[RTC] PCF85063 detected at 0x51" : "[RTC] PCF85063 not detected");
    if (rtcOk) {
        RtcDateTime rtcValue;
        // A new PCF85063 can contain zero/invalid calendar fields. Seed it
        // once so the UI has a valid value; the user can then correct it from
        // SET CLOCK.
        if (!rtc.read(rtcValue)) {
            rtcValue = build_compile_datetime();
            Serial.println(rtc.write(rtcValue) ? "[RTC] Invalid value; seeded from compile time"
                                                : "[RTC] Unable to seed invalid value");
        }
    }

    // 5. เริ่มต้น ModbusMaster โดยผูกเข้ากับ Serial1
    modbus.begin(MODBUS_SLAVE_ID, Serial1);
    modbus.idle(modbus_idle_yield);
    rs485Io.begin(RS485_IO_SLAVE_ID, Serial1);
    rs485Io.idle(modbus_idle_yield);
#if WATER_METER_SENSOR_PRESENT
    waterMeter.begin(WATER_METER_SLAVE_ID, Serial1);
    waterMeter.idle(modbus_idle_yield);
#endif

    // 6. เริ่มต้น CAN Bus (TWAI: GPIO 15 TX, GPIO 16 RX)
    bool canOk = WS43B.can.begin(500000);

    // 7. เริ่มต้น UI หน้าจอ
    lvgl_port_lock(-1);
    ui_init();
    load_water_settings();
    update_settings_ui();
    update_mode_ui();
    setup_water_level_chart();
    setup_ui_events();
    setup_rtc_ui();
    setup_wifi_clock_ui();
    load_system_settings();   // โหลด autoBacklight, autoRestart, brightness
    update_manual_leds();

    lvgl_port_unlock();

    // 8. สร้าง FreeRTOS Background Task สำหรับงานสื่อสาร แยกไปรันที่ CORE 0 โดยเฉพาะ
    xTaskCreatePinnedToCore(commTask,        // Function task
                            "comm_task",     // Task name
                            8192,            // Stack size in bytes
                            NULL,            // Parameters
                            1,               // Priority (1 is above idle 0, yields nicely)
                            &commTaskHandle, // Task handle
                            0                // 🔥 รันที่ CORE 0
    );

    Serial.println("[SYSTEM] High-speed display pipeline active.");

    bootTimeMs = millis();
    autoRestartStartMs = millis();
    lastUserActivityMs = millis();

    DO0_OFF;
    DO1_OFF;
}

void loop() {
    if (isOtaUpdating) {
        handle_ota_loop();
        vTaskDelay(pdMS_TO_TICKS(1));
        return;
    }

    // 3. อัปเดตเกจวัดระดับน้ำและกราฟแนวโน้มแบบ Real-Time จากข้อมูล Modbus
    update_auto_pumps_from_pressure();
#if WATER_LEVEL_SIMULATION
    lastRs485Ok = true;
    update_water_level_simulation();
#else
    if (reg_data_updated) {
        reg_data_updated = false;
        // แปลงค่า Raw Modbus (สมมุติหน่วย mm เต็มถัง 2000 mm = 200.0 cm)
        float level_cm = (float)shared_reg0 / 10.0f;
        // Sensor value is in mm; convert to cm, then normalize against the
        // configured tank capacity instead of a hard-coded 200 cm tank.
        float level_pct = (level_cm / configuredLevelMaxCm()) * 100.0f;
        if (level_pct > 100.0f)
            level_pct = 100.0f;
        if (level_pct < 0.0f)
            level_pct = 0.0f;

        update_automatic_outputs(level_cm);
        if (lvgl_port_lock(10)) {
#if WATER_METER_SENSOR_PRESENT
            const float volume_l = water_meter_ok ? shared_volume_l : 0.0f;
            const float flow_rate = water_meter_ok ? shared_flow_rate : 0.0f;
#else
            const float volume_l = 0.0f; // Water meter is not installed yet.
            const float flow_rate = 0.0f;
#endif
            const bool valveOn = (shared_rs485_outputs & (1u << valve_io_pin)) != 0;
            const bool pump1On = (shared_rs485_outputs & (1u << pump1_io_pin)) != 0;
            const bool pump2On = (shared_rs485_outputs & (1u << pump2_io_pin)) != 0;
            update_water_level_ui(level_pct, level_cm, volume_l, flow_rate, valveOn, pump1On, pump2On);
            update_manual_leds();
            lvgl_port_unlock();
        }
    } else if (!lastRs485Ok) {
        static uint32_t lastErrorUiUpdate = 0;
        if (millis() - lastErrorUiUpdate >= 500) {
            lastErrorUiUpdate = millis();
            if (lvgl_port_lock(10)) {
                if (ui_WaterTankLevel) {
                    lv_label_set_text(ui_WaterTankLevel, "ERROR");
                }
                if (ui_WaterTank) {
                    lv_obj_set_height(ui_WaterTank, 0);
                }
                if (ui_LabelWaterCm) {
                    lv_label_set_text(ui_LabelWaterCm, "Level: ERROR");
                }
                update_manual_leds();
                lvgl_port_unlock();
            }
        }
    }
#endif

    update_pump_status_outputs();

    if (lvgl_port_lock(25)) {
        update_manual_leds();
        if (millis() - lastRtcUpdateMs >= 1000) {
            lastRtcUpdateMs = millis();
            update_rtc_ui();
            update_wifi_status();
        }
        lvgl_port_unlock();
    }

    // ---- Overload / Inverter Trip Protection ----
    if (shared_pump1_alarm && waterSettings.manualPump1 && !waterSettings.autoMode) {
        waterSettings.manualPump1 = false;
        save_water_settings();
        if (lvgl_port_lock(10)) {
            update_mode_ui();
            lvgl_port_unlock();
        }
        publish_mqtt_data();
    }
    if (shared_pump2_alarm && waterSettings.manualPump2 && !waterSettings.autoMode) {
        waterSettings.manualPump2 = false;
        save_water_settings();
        if (lvgl_port_lock(10)) {
            update_mode_ui();
            lvgl_port_unlock();
        }
        publish_mqtt_data();
    }

    handle_mqtt_loop();
    handle_ota_loop();

    // ---- Auto Backlight ----
    if (autoBacklightEnabled && isBacklightOn) {
        const uint32_t idleMs = millis() - lastUserActivityMs;
        if (idleMs >= autoBacklightMin * 60000UL) {
            BL_OFF;
            isBacklightOn = false;
            Serial.printf("[SYSTEM] Backlight OFF (idle %lu ms >= %lu min)\n", (unsigned long)idleMs, (unsigned long)autoBacklightMin);
        }
    }

    // ---- Auto Restart ----
    if (autoRestartEnabled) {
        const uint32_t elapsedMs = millis() - autoRestartStartMs;
        if (elapsedMs >= autoRestartMin * 60000UL) {
            Serial.printf("[SYSTEM] Auto restart triggered (after %lu min)\n", autoRestartMin);
            delay(500);
            ESP.restart();
        }
    }

    delay(10);
}
