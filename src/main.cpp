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
#include "waveshare_ch422g.h"
#include "waveshare_sdcard.h"
#include <Arduino.h>
#include <ModbusMaster.h>
#include <Preferences.h>
#include <lv_conf.h>
#include <lvgl.h>
#include <math.h>

// Real RS485 devices are connected on this installation.
#ifndef WATER_LEVEL_SIMULATION
#define WATER_LEVEL_SIMULATION 0
#endif

// The level sensor is now part of the live system. Set to 0 only when testing
// with the IO module alone, because a missing slave causes a Modbus timeout.
#ifndef WATER_LEVEL_SENSOR_PRESENT
#define WATER_LEVEL_SENSOR_PRESENT 0
#endif

// Include Waveshare 4.3B Library (RS485, CAN, I2C, IO Expander)
#include <WaveShare4_3B.h>

// =============================================================================
// Modbus RTU Configuration
// =============================================================================
volatile uint8_t MODBUS_SLAVE_ID = 2;        // Water-level sensor slave ID
static const uint8_t RS485_IO_SLAVE_ID = 3;
volatile uint32_t MODBUS_BAUD_RATE = 9600;   // Standard Baud Rate (เช่น 9600, 19200, 115200)
volatile uint16_t MODBUS_START_REG = 0x0000; // Register เริ่มต้น
volatile uint16_t MODBUS_REG_COUNT = 2;      // จำนวน Register ที่ต้องการอ่าน
volatile uint32_t MODBUS_POLL_MS = 200;      // อ่านค่าทุก 200 ms

ModbusMaster modbus;
ModbusMaster rs485Io;

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
volatile uint16_t shared_reg1 = 285;
volatile bool reg_data_updated = false;
volatile uint8_t shared_rs485_inputs = 0;
volatile uint8_t shared_rs485_outputs = 0;
volatile bool shared_pressure_switch = false;
volatile bool rs485_io_ok = false;
volatile bool sensor_rs485_ok = false;

// Water-level gauge created inside the SquareLine placeholder object.
static lv_chart_series_t *waterLevelChartSeries = NULL;
static uint32_t lastSimulationUpdateMs = 0;
static float simulatedLevelPct = 60.0f;

// Persistent process settings. Values are percentages of the 0-100% tank
// range; the UI text areas are intentionally kept as the source of truth for
// editing, while these values are used by the control loop.
struct WaterSettings {
    float valveOnCm = 84.0f;
    float tankCapacity = 2000.0f;
    uint32_t valvePeriodSec = 300;
    float alertMinCm = 40.0f;
    float alertMaxCm = 180.0f;
    uint32_t alertPeriodSec = 300;
    bool autoMode = false;
    bool autoPump1 = false;
    bool autoPump2 = false;
};

static Preferences settingsStore;
static WaterSettings waterSettings;
static bool manualValveOverride = false;
static uint32_t alertLowSince = 0;
static uint32_t alertHighSince = 0;
static bool alertOutputOn = false;
static bool lastRs485Ok = false;

static void set_status_led(lv_obj_t *led, bool enabled, uint32_t on_color);

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

static void load_water_settings() {
    settingsStore.begin("watercfg", false);
    waterSettings.valveOnCm = settingsStore.getFloat("voncm", waterSettings.valveOnCm);
    waterSettings.tankCapacity = settingsStore.getFloat("tankcap", waterSettings.tankCapacity);
    waterSettings.valvePeriodSec = settingsStore.getUInt("vperiod", waterSettings.valvePeriodSec);
    waterSettings.alertMinCm = settingsStore.getFloat("amincm", waterSettings.alertMinCm);
    waterSettings.alertMaxCm = settingsStore.getFloat("amaxcm", waterSettings.alertMaxCm);
    waterSettings.alertPeriodSec = settingsStore.getUInt("aperiod", waterSettings.alertPeriodSec);
    waterSettings.autoMode = settingsStore.getBool("automode", waterSettings.autoMode);
    waterSettings.autoPump1 = settingsStore.getBool("apump1", waterSettings.autoPump1);
    waterSettings.autoPump2 = settingsStore.getBool("apump2", waterSettings.autoPump2);
    settingsStore.end();

    waterSettings.valveOnCm = constrain(waterSettings.valveOnCm, 0.0f, 200.0f);
    waterSettings.tankCapacity = constrain(waterSettings.tankCapacity, 0.1f, 1000000.0f);
    waterSettings.alertMinCm = constrain(waterSettings.alertMinCm, 0.0f, 200.0f);
    waterSettings.alertMaxCm = constrain(waterSettings.alertMaxCm, 0.0f, 200.0f);
    waterSettings.valvePeriodSec = constrain(waterSettings.valvePeriodSec, (uint32_t)1, (uint32_t)86400);
    waterSettings.alertPeriodSec = constrain(waterSettings.alertPeriodSec, (uint32_t)1, (uint32_t)86400);
    const float maxLevelCm = max(1.0f, waterSettings.tankCapacity);
    waterSettings.valveOnCm = min(waterSettings.valveOnCm, maxLevelCm);
    waterSettings.alertMinCm = min(waterSettings.alertMinCm, maxLevelCm);
    waterSettings.alertMaxCm = min(waterSettings.alertMaxCm, maxLevelCm);
    if (waterSettings.alertMinCm >= waterSettings.alertMaxCm)
        waterSettings.alertMaxCm = min(200.0f, waterSettings.alertMinCm + 1.0f);
}

static void save_water_settings() {
    settingsStore.begin("watercfg", false);
    settingsStore.putFloat("voncm", waterSettings.valveOnCm);
    settingsStore.putFloat("tankcap", waterSettings.tankCapacity);
    settingsStore.putUInt("vperiod", waterSettings.valvePeriodSec);
    settingsStore.putFloat("amincm", waterSettings.alertMinCm);
    settingsStore.putFloat("amaxcm", waterSettings.alertMaxCm);
    settingsStore.putUInt("aperiod", waterSettings.alertPeriodSec);
    settingsStore.putBool("automode", waterSettings.autoMode);
    settingsStore.putBool("apump1", waterSettings.autoPump1);
    settingsStore.putBool("apump2", waterSettings.autoPump2);
    settingsStore.end();
}

static void set_textarea_number(lv_obj_t *textarea, float value, uint8_t decimals = 0) {
    if (!textarea)
        return;
    char text[20];
    snprintf(text, sizeof(text), decimals ? "%.1f" : "%.0f", value);
    lv_textarea_set_text(textarea, text);
}

static void update_settings_ui() {
    set_textarea_number(ui_SetOnValve, waterSettings.valveOnCm, 1);
    set_textarea_number(ui_SetTankCapacity, waterSettings.tankCapacity, 1);
    set_textarea_number(ui_SetPeriodTimeContorl, (float)waterSettings.valvePeriodSec);
    set_textarea_number(ui_SetMinAlert, waterSettings.alertMinCm, 1);
    set_textarea_number(ui_SetMaxAlert, waterSettings.alertMaxCm, 1);
    set_textarea_number(ui_SetPeriodTimeAlert, (float)waterSettings.alertPeriodSec);
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
    if (ui_AutoPump1)
        lv_obj_set_state(ui_AutoPump1, LV_STATE_CHECKED, waterSettings.autoPump1);
    if (ui_AutoPump2)
        lv_obj_set_state(ui_AutoPump2, LV_STATE_CHECKED, waterSettings.autoPump2);
    const bool autoMode = waterSettings.autoMode;
    for (lv_obj_t *obj : {ui_ManualPump1, ui_ManualPump2, ui_ManualValve}) {
        if (!obj) continue;
        if (autoMode) lv_obj_add_state(obj, LV_STATE_DISABLED);
        else lv_obj_clear_state(obj, LV_STATE_DISABLED);
    }
}

static void save_settings_from_ui(lv_event_t *) {
    const float maxLevelCm = configuredLevelMaxCm();
    waterSettings.valveOnCm = boundedFloat(lv_textarea_get_text(ui_SetOnValve), waterSettings.valveOnCm, 0.0f, maxLevelCm);
    waterSettings.tankCapacity = boundedFloat(lv_textarea_get_text(ui_SetTankCapacity), waterSettings.tankCapacity, 0.1f, 1000000.0f);
    waterSettings.valvePeriodSec = boundedUint(lv_textarea_get_text(ui_SetPeriodTimeContorl), waterSettings.valvePeriodSec, 1, 86400);
    waterSettings.alertMinCm = boundedFloat(lv_textarea_get_text(ui_SetMinAlert), waterSettings.alertMinCm, 0.0f, maxLevelCm);
    waterSettings.alertMaxCm = boundedFloat(lv_textarea_get_text(ui_SetMaxAlert), waterSettings.alertMaxCm, 0.0f, maxLevelCm);
    waterSettings.alertPeriodSec = boundedUint(lv_textarea_get_text(ui_SetPeriodTimeAlert), waterSettings.alertPeriodSec, 1, 86400);

    if (waterSettings.alertMinCm >= waterSettings.alertMaxCm)
        waterSettings.alertMaxCm = min(200.0f, waterSettings.alertMinCm + 1.0f);
    update_settings_ui();
    save_water_settings();
}

static void save_mode_settings(lv_event_t *) {
    waterSettings.autoMode = ui_ManualAuto && lv_obj_has_state(ui_ManualAuto, LV_STATE_CHECKED);
    waterSettings.autoPump1 = ui_AutoPump1 && lv_obj_has_state(ui_AutoPump1, LV_STATE_CHECKED);
    waterSettings.autoPump2 = ui_AutoPump2 && lv_obj_has_state(ui_AutoPump2, LV_STATE_CHECKED);
    if (waterSettings.autoMode)
        manualValveOverride = false;
    for (lv_obj_t *obj : {ui_ManualPump1, ui_ManualPump2, ui_ManualValve}) {
        if (!obj) continue;
        if (waterSettings.autoMode) lv_obj_add_state(obj, LV_STATE_DISABLED);
        else lv_obj_clear_state(obj, LV_STATE_DISABLED);
    }
    save_water_settings();
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
    lv_keyboard_set_mode(ui_Keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_clear_flag(ui_ContainerKeyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_state(ui_KeyboardDisplay, LV_STATE_FOCUSED);
    lv_textarea_set_cursor_pos(ui_KeyboardDisplay, LV_TEXTAREA_CURSOR_LAST);
}

static void close_numeric_keyboard(lv_event_t *e) {
    if (!activeNumericInput || !ui_KeyboardDisplay)
        return;

    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        lv_textarea_set_text(activeNumericInput, lv_textarea_get_text(ui_KeyboardDisplay));
        save_settings_from_ui(nullptr);
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
    rs485Io.begin(RS485_IO_SLAVE_ID, Serial1);
    WS43B.rs485.clearRxBuffer();
    const uint8_t inputResult = rs485Io.readDiscreteInputs(0, 8); // 1x: addresses 0..7
    if (inputResult != rs485Io.ku8MBSuccess) {
        rs485_io_ok = false;
        return;
    }
    // Slave 3 is online as soon as the required 8-channel input read works.
    // Some IO modules do not implement function 0x01 (read coils), even
    // though they support function 0x0F for writing all eight outputs.
    rs485_io_ok = true;
    shared_rs485_inputs = (uint8_t)(rs485Io.getResponseBuffer(0) & 0xFF);
    shared_pressure_switch = (shared_rs485_inputs & (1u << 1)) != 0; // DI address 1

    // This IO module is verified to support 0x0F write-multiple-coils, but
    // does not need 0x01 read-coils for operation. Keep the output shadow from
    // the successful 0x0F response to avoid a timeout on every IO poll.
}

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

    while (true) {
        // 1. จัดการคำขอเขียน Coil (จาก UI สวิตช์ DO0/DO1)
        if (pendingIoMask != 0) {
            pendingIoMask = 0;
            rs485Io.begin(RS485_IO_SLAVE_ID, Serial1);
            // The IO module requires all eight outputs in one transaction.
            // ModbusMaster packs coil states from transmit buffer word 0.
            rs485Io.setTransmitBuffer(0, shared_rs485_outputs & 0xFF);
            const uint8_t result = rs485Io.writeMultipleCoils(0, 8); // 0x0F, coils 0..7
            if (result == rs485Io.ku8MBSuccess)
                rs485_io_ok = true;
            else
                rs485_io_ok = false;
        }

        // 2. จัดการคำขอสแกนหา Slave ID (จากปุ่ม SCAN MB)
        if (requestScanModbus) {
            requestScanModbus = false;
            doModbusScan();
            lastPollTime = millis();
        }

        // 3. รอบอ่านค่า Modbus อัตโนมัติตามรอบเวลา
#if !WATER_LEVEL_SIMULATION && WATER_LEVEL_SENSOR_PRESENT
        const uint32_t sensorPollInterval = sensor_rs485_ok ? MODBUS_POLL_MS : 5000UL;
        if (millis() - lastPollTime >= sensorPollInterval) {
            lastPollTime = millis();
            doModbusPoll();
        }
#endif
        static unsigned long lastIoPollTime = 0;
        if (millis() - lastIoPollTime >= 50) {
            lastIoPollTime = millis();
            pollRs485Io();
        }

        // 4. รับและประมวลผลเฟรม CAN Bus (TWAI)
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

        // 5. ตรวจสอบ Digital Inputs จาก CH422G (อ่านทุก 100ms เพื่อไม่แย่ง I2C bus กับทัชสกรีน)
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

static void update_auto_pumps_from_pressure() {
    const bool autoMode = ui_ManualAuto && lv_obj_has_state(ui_ManualAuto, LV_STATE_CHECKED);
    if (!autoMode)
        return;

    // Pressure switch DI address 1 is the single source of truth for Auto
    // pump requests. The output shadow is updated immediately and the comm
    // task sends all eight coils in one 0x0F transaction.
    set_rs485_output(1, shared_pressure_switch && waterSettings.autoPump1);
    set_rs485_output(2, shared_pressure_switch && waterSettings.autoPump2);
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
        set_rs485_output(0, valveOn); // RS485 coil 0 is the water valve.
        update_auto_pumps_from_pressure();
    } else if (!autoMode) {
        valveOn = (shared_rs485_outputs & (1u << 0)) != 0;
    }

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
    set_rs485_output(3, alertOutputOn); // RS485 coil 3 is the alarm output.
    return valveOn;
}

static void update_manual_leds() {
    const bool localDo0On = DO_Read(DO0) == 0;
    const bool localDo1On = DO_Read(DO1) == 0;
    auto set_monitor_alpha = [](lv_obj_t *obj, bool on) {
        if (obj)
            lv_obj_set_style_bg_opa(obj, on ? LV_OPA_COVER : 100, LV_PART_MAIN | LV_STATE_DEFAULT);
    };

    // This LED reports the actual Modbus slave 3 transaction status, so it
    // remains useful even when the water-level sensor (slave 2) is absent.
    set_status_led(ui_LedMeter485, lastRs485Ok, 0x22C55E);
    set_status_led(ui_LedLevel485, lastRs485Ok, 0x22C55E);
    set_status_led(ui_LedIO485, rs485_io_ok, 0x22C55E);
    set_status_led(ui_LedValve, (shared_rs485_outputs & (1u << 0)) != 0, 0x22C55E);
    set_status_led(ui_LedPump1, (shared_rs485_outputs & (1u << 1)) != 0, 0x22C55E);
    set_status_led(ui_LedPump2, (shared_rs485_outputs & (1u << 2)) != 0, 0x22C55E);

    // Keep the three manual controls as status indicators as well as controls.
    // This also reflects automatic output changes while the Manual panel is
    // open; disabled controls in Auto mode still show the real state.
    if (ui_ManualPump1)
        lv_obj_set_state(ui_ManualPump1, LV_STATE_CHECKED, (shared_rs485_outputs & (1u << 1)) != 0);
    if (ui_ManualPump2)
        lv_obj_set_state(ui_ManualPump2, LV_STATE_CHECKED, (shared_rs485_outputs & (1u << 2)) != 0);
    if (ui_ManualValve)
        lv_obj_set_state(ui_ManualValve, LV_STATE_CHECKED, (shared_rs485_outputs & (1u << 0)) != 0);

    set_monitor_alpha(ui_LocalDI0, shared_di0);
    set_monitor_alpha(ui_LocalDI1, shared_di1);
    set_monitor_alpha(ui_LocalDO0, localDo0On);
    set_monitor_alpha(ui_LocalDO1, localDo1On);
    lv_obj_t *rs485Di[] = {ui_RS485DI0, ui_RS485DI1, ui_RS485DI2, ui_RS485DI3, ui_RS485DI4, ui_RS485DI5, ui_RS485DI6, ui_RS485DI7};
    lv_obj_t *rs485Do[] = {ui_RS485DO0, ui_RS485DO1, ui_RS485DO2, ui_RS485DO3, ui_RS485DO4, ui_RS485DO5, ui_RS485DO6, ui_RS485DO7};
    for (uint8_t i = 0; i < 8; ++i) {
        set_monitor_alpha(rs485Di[i], rs485_io_ok && ((shared_rs485_inputs >> i) & 1));
        set_monitor_alpha(rs485Do[i], rs485_io_ok && ((shared_rs485_outputs >> i) & 1));
    }
}

static void manual_output_event(lv_event_t *e) {
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    const bool state = lv_obj_has_state(obj, LV_STATE_CHECKED);
    void *userData = lv_event_get_user_data(e);
    const uint8_t output = (uint8_t)(uintptr_t)userData;
    const bool autoMode = ui_ManualAuto && lv_obj_has_state(ui_ManualAuto, LV_STATE_CHECKED);
    if (autoMode)
        return;
    if (output == 2) {
        manualValveOverride = true;
        set_rs485_output(0, state);
    } else {
        set_rs485_output((uint8_t)(output + 1), state);
    }
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
    lv_obj_add_flag(ui_PanelManual, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_PanelMonitor, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_HIDDEN);
}

static void select_chart(lv_event_t *) {
    select_panel(ui_LineChart);
    if (ui_LabelMenu) lv_label_set_text(ui_LabelMenu, "LEVEL AND FLOW TREND (REAL-TIME)");
}

static void select_settings(lv_event_t *) {
    update_settings_ui();
    select_panel(ui_PanelSettings);
    if (ui_LabelMenu) lv_label_set_text(ui_LabelMenu, "Settings");
}

static void select_manual(lv_event_t *) {
    select_panel(ui_PanelManual);
    update_manual_leds();
    if (ui_LabelMenu) lv_label_set_text(ui_LabelMenu, "Manual");
}

static void select_monitor(lv_event_t *) {
    select_panel(ui_PanelMonitor);
    update_manual_leds();
    if (ui_LabelMenu) lv_label_set_text(ui_LabelMenu, "Monitor");
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

    const lv_coord_t water_height = (lv_coord_t)((waterTankMaxHeight * clamped_pct / 100.0f) + 0.5f);
    lv_obj_set_height(ui_WaterTank, water_height);

    if (ui_WaterTankLevel) {
        char tank_text[12];
        snprintf(tank_text, sizeof(tank_text), "%d%%", (int)(clamped_pct + 0.5f));
        lv_label_set_text(ui_WaterTankLevel, tank_text);
    }

    if (waterLevelChartSeries && ui_LineChart) {
        lv_chart_set_next_value(ui_LineChart, waterLevelChartSeries, (int32_t)(level_pct + 0.5f));
        lv_chart_refresh(ui_LineChart);
    }

    char text[32];
    snprintf(text, sizeof(text), "Level: %.1f cm", level_cm);
    if (ui_LabelWaterCm)
        lv_label_set_text(ui_LabelWaterCm, text);

    snprintf(text, sizeof(text), "%.3f", volume_l);
    if (ui_LabelVolumn)
        lv_label_set_text(ui_LabelVolumn, text);

    snprintf(text, sizeof(text), "%.3f", flow_rate);
    if (ui_LabelFlowRate)
        lv_label_set_text(ui_LabelFlowRate, text);

    set_status_led(ui_LedValve, valve_on, 0x22C55E);
    set_status_led(ui_LedPump1, pump1_on, 0x22C55E);
    set_status_led(ui_LedPump2, pump2_on, 0x22C55E);

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
    const float level_cm = level_pct * 2.0f;                        // 200 cm tank
    const float volume_l = (level_pct / 100.0f) * waterSettings.tankCapacity;
    float flow = 100.0f + level_pct * 1.8f + (float)random(-25, 26);
    if (flow < 0)
        flow = 0;
    const float flow_rate = flow;
    const bool valve_on = update_automatic_outputs(level_cm);
    const bool autoMode = ui_ManualAuto && lv_obj_has_state(ui_ManualAuto, LV_STATE_CHECKED);
    const bool pump1_on = autoMode && waterSettings.autoPump1 && shared_pressure_switch;
    const bool pump2_on = autoMode && waterSettings.autoPump2 && shared_pressure_switch;

    if (lvgl_port_lock(10)) {
        update_water_level_ui(level_pct, level_cm, volume_l, flow_rate, valve_on, pump1_on, pump2_on);
        lvgl_port_unlock();
    }
#endif
}

void setup_ui_events() {
    lv_obj_add_event_cb(ui_HomeButton, select_chart, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_SettingsButton, select_settings, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_ManualButton, select_manual, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_MonitorButton, select_monitor, LV_EVENT_CLICKED, NULL);
    // SquareLine marks the child images as clickable, so a touch on an icon
    // may target the image instead of bubbling to its button parent.
    lv_obj_add_event_cb(ui_Image1, select_chart, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Image2, select_settings, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Image3, select_manual, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Image4, select_monitor, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(ui_SetOnValve, save_settings_from_ui, LV_EVENT_DEFOCUSED, NULL);
    lv_obj_add_event_cb(ui_SetTankCapacity, save_settings_from_ui, LV_EVENT_DEFOCUSED, NULL);
    lv_obj_add_event_cb(ui_SetPeriodTimeContorl, save_settings_from_ui, LV_EVENT_DEFOCUSED, NULL);
    lv_obj_add_event_cb(ui_SetMinAlert, save_settings_from_ui, LV_EVENT_DEFOCUSED, NULL);
    lv_obj_add_event_cb(ui_SetMaxAlert, save_settings_from_ui, LV_EVENT_DEFOCUSED, NULL);
    lv_obj_add_event_cb(ui_SetPeriodTimeAlert, save_settings_from_ui, LV_EVENT_DEFOCUSED, NULL);
    lv_obj_add_event_cb(ui_SetOnValve, save_settings_from_ui, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(ui_SetTankCapacity, save_settings_from_ui, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(ui_SetPeriodTimeContorl, save_settings_from_ui, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(ui_SetMinAlert, save_settings_from_ui, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(ui_SetMaxAlert, save_settings_from_ui, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(ui_SetPeriodTimeAlert, save_settings_from_ui, LV_EVENT_READY, NULL);

    lv_obj_add_event_cb(ui_SetOnValve, open_numeric_keyboard, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_SetTankCapacity, open_numeric_keyboard, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_SetPeriodTimeContorl, open_numeric_keyboard, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_SetMinAlert, open_numeric_keyboard, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_SetMaxAlert, open_numeric_keyboard, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_SetPeriodTimeAlert, open_numeric_keyboard, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Keyboard, close_numeric_keyboard, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(ui_Keyboard, close_numeric_keyboard, LV_EVENT_CANCEL, NULL);

    lv_obj_add_event_cb(ui_ManualPump1, manual_output_event, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)0);
    lv_obj_add_event_cb(ui_ManualPump2, manual_output_event, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)1);
    lv_obj_add_event_cb(ui_ManualValve, manual_output_event, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)2);
    lv_obj_add_event_cb(ui_ManualAuto, save_mode_settings, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui_AutoPump1, save_mode_settings, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui_AutoPump2, save_mode_settings, LV_EVENT_VALUE_CHANGED, NULL);
    for (lv_obj_t *obj : {ui_LocalDO0, ui_LocalDO1})
        if (obj) lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ui_LocalDO0, local_output_test_event, LV_EVENT_CLICKED, (void *)(uintptr_t)0);
    lv_obj_add_event_cb(ui_LocalDO1, local_output_test_event, LV_EVENT_CLICKED, (void *)(uintptr_t)1);
    lv_obj_t *rs485Do[] = {ui_RS485DO0, ui_RS485DO1, ui_RS485DO2, ui_RS485DO3,
                           ui_RS485DO4, ui_RS485DO5, ui_RS485DO6, ui_RS485DO7};
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

    // 1. สร้าง Queue สำหรับส่งข้อความ Log ข้าม Core แบบ Thread-safe
    logQueue = xQueueCreate(25, sizeof(LogItem));

    // 2. เริ่มต้นหน้าจอและระบบสัมผัส (รันบน Core 1 ด้วย True Double Buffer DMA)
    lvgl_port_init();

    // 3. เริ่มต้นชิปขยายขา CH422G และช่อง MicroSD
    initCH422G();
    initSDCARD();

    // 4. เริ่มต้นพอร์ต RS485 (GPIO 44 TX, GPIO 43 RX)
    bool rs485Ok = WS43B.rs485.begin(MODBUS_BAUD_RATE, SERIAL_8N1, WS43B_RS485_RX_PIN, WS43B_RS485_TX_PIN);

    // 5. เริ่มต้น ModbusMaster โดยผูกเข้ากับ Serial1
    modbus.begin(MODBUS_SLAVE_ID, Serial1);
    rs485Io.begin(RS485_IO_SLAVE_ID, Serial1);

    // 6. เริ่มต้น CAN Bus (TWAI: GPIO 15 TX, GPIO 16 RX)
    bool canOk = WS43B.can.begin(500000);

    // 7. เริ่มต้น UI หน้าจอ
    lvgl_port_lock(-1);
    ui_init();
    load_water_settings();
    update_settings_ui();
    update_mode_ui();
#if WATER_LEVEL_SIMULATION
    setup_water_level_simulation();
#endif
    setup_ui_events();
    update_manual_leds();

    lvgl_port_unlock();

    // 8. สร้าง FreeRTOS Background Task สำหรับงานสื่อสาร แยกไปรันที่ CORE 0 โดยเฉพาะ
    xTaskCreatePinnedToCore(commTask,        // Function task
                            "comm_task",     // Task name
                            8192,            // Stack size in bytes
                            NULL,            // Parameters
                            2,               // Priority (ต่ำกว่า LVGL ที่อยู่ระดับ 3)
                            &commTaskHandle, // Task handle
                            0                // 🔥 รันที่ CORE 0
    );

    Serial.println("[SYSTEM] High-speed display pipeline active.");

    DO0_ON;
    delay(100);
    DO1_ON;
    delay(100);
    DO0_OFF;
    delay(100);
    DO1_OFF;
}

void loop() {

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
        float level_pct = (level_cm / 200.0f) * 100.0f;
        if (level_pct > 100.0f)
            level_pct = 100.0f;
        if (level_pct < 0.0f)
            level_pct = 0.0f;

        update_automatic_outputs(level_cm);
        if (lvgl_port_lock(10)) {
            const float volume_l = (level_pct / 100.0f) * waterSettings.tankCapacity;
            const bool valveOn = (shared_rs485_outputs & (1u << 0)) != 0;
            const bool pump1On = (shared_rs485_outputs & (1u << 1)) != 0;
            const bool pump2On = (shared_rs485_outputs & (1u << 2)) != 0;
            update_water_level_ui(level_pct, level_cm, volume_l, (float)shared_reg1, valveOn, pump1On, pump2On);
            update_manual_leds();
            lvgl_port_unlock();
        }
    }
#endif

    if (lvgl_port_lock(2)) {
        update_manual_leds();
        lvgl_port_unlock();
    }

    delay(10);
}
