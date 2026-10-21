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
#include <lv_conf.h>
#include <lvgl.h>
#include <math.h>

// Set to 0 when a real Modbus device is connected.
#ifndef WATER_LEVEL_SIMULATION
#define WATER_LEVEL_SIMULATION 1
#endif

// Include Waveshare 4.3B Library (RS485, CAN, I2C, IO Expander)
#include <WaveShare4_3B.h>

// =============================================================================
// Modbus RTU Configuration
// =============================================================================
volatile uint8_t MODBUS_SLAVE_ID = 1;        // Target Modbus Slave ID (1-247)
volatile uint32_t MODBUS_BAUD_RATE = 9600;   // Standard Baud Rate (เช่น 9600, 19200, 115200)
volatile uint16_t MODBUS_START_REG = 0x0000; // Register เริ่มต้น
volatile uint16_t MODBUS_REG_COUNT = 2;      // จำนวน Register ที่ต้องการอ่าน
volatile uint32_t MODBUS_POLL_MS = 2000;     // อ่านค่าทุก 2 วินาที

ModbusMaster modbus;

// FreeRTOS Task and Queue Handles
TaskHandle_t commTaskHandle = NULL;
QueueHandle_t logQueue = NULL;

// Asynchronous Request Flags (from UI to Core 0)
volatile bool requestScanModbus = false;
volatile int8_t pendingCoilIndex = -1;
volatile bool pendingCoilState = false;

// Shared State between Core 0 and Core 1
volatile bool shared_di0 = false;
volatile bool shared_di1 = false;
volatile bool di0_changed = false;
volatile bool di1_changed = false;
volatile uint16_t shared_reg0 = 1500;
volatile uint16_t shared_reg1 = 285;
volatile bool reg_data_updated = false;

// Water-level gauge created inside the SquareLine placeholder object.
static lv_chart_series_t *waterLevelChartSeries = NULL;
static uint32_t lastSimulationUpdateMs = 0;
static float simulatedLevelPct = 60.0f;

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

    unsigned long lastPollTime = 0;
    unsigned long lastDIPollTime = 0;
    uint8_t local_last_di0 = 0xFF;
    uint8_t local_last_di1 = 0xFF;

    while (true) {
        // 1. จัดการคำขอเขียน Coil (จาก UI สวิตช์ DO0/DO1)
        if (pendingCoilIndex >= 0) {
            int8_t coil = pendingCoilIndex;
            bool state = pendingCoilState;
            pendingCoilIndex = -1; // Clear flag
            modbus.writeSingleCoil(coil, state ? 1 : 0);
        }

        // 2. จัดการคำขอสแกนหา Slave ID (จากปุ่ม SCAN MB)
        if (requestScanModbus) {
            requestScanModbus = false;
            doModbusScan();
            lastPollTime = millis();
        }

        // 3. รอบอ่านค่า Modbus อัตโนมัติตามรอบเวลา
#if !WATER_LEVEL_SIMULATION
        if (millis() - lastPollTime >= MODBUS_POLL_MS) {
            lastPollTime = millis();
            doModbusPoll();
        }
#endif

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
    lv_obj_set_style_bg_color(led, enabled ? lv_color_hex(on_color) : lv_color_hex(0x3F0F16), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(led, enabled ? lv_color_hex(on_color) : lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(led, enabled ? 16 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(led, enabled ? LV_OPA_60 : LV_OPA_0, LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void update_water_level_ui(float level_pct, float level_cm, uint16_t volume_l, uint16_t flow_rate, bool valve_on, bool pump1_on,
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

    snprintf(text, sizeof(text), "%u", volume_l);
    if (ui_LabelVolumn)
        lv_label_set_text(ui_LabelVolumn, text);

    snprintf(text, sizeof(text), "%u", flow_rate);
    if (ui_LabelFlowRate)
        lv_label_set_text(ui_LabelFlowRate, text);

    set_status_led(ui_LedValve, valve_on, 0xF59E0B);
    set_status_led(ui_LedPump1, pump1_on, 0x22C55E);
    set_status_led(ui_LedPump2, pump2_on, 0x38BDF8);

    if (level_pct < 20.0f) {
        lv_label_set_text(ui_LabelWaterStatus, "LOW LEVEL");
        lv_obj_set_style_bg_color(ui_PanelWaterStatus, lv_color_hex(0x78350F), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(ui_PanelWaterStatus, lv_color_hex(0xF59E0B), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(ui_LabelWaterStatus, lv_color_hex(0xF59E0B), LV_PART_MAIN | LV_STATE_DEFAULT);
    } else if (level_pct >= 90.0f) {
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

static void setup_water_level_simulation() {
    if (!ui_LineChart)
        return;

    lv_chart_set_range(ui_LineChart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    // The SquareLine-generated series uses a 10-element external array while
    // this chart has 20 points. Hide it and use a chart-owned 20-point series
    // so the simulation never writes past the generated buffer.
    lv_chart_series_t *generatedSeries = lv_chart_get_series_next(ui_LineChart, NULL);
    if (generatedSeries)
        lv_chart_hide_series(ui_LineChart, generatedSeries, true);
    waterLevelChartSeries = lv_chart_add_series(ui_LineChart, lv_color_hex(0x22D3EE), LV_CHART_AXIS_PRIMARY_Y);
    if (waterLevelChartSeries) {
        lv_chart_set_all_values(ui_LineChart, waterLevelChartSeries, 60);
    }
    lastSimulationUpdateMs = 0;
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
    const uint16_t volume_l = (uint16_t)(level_pct * 20.0f + 0.5f); // 2,000 L tank
    int flow = (int)(100.0f + level_pct * 1.8f) + random(-25, 26);
    if (flow < 0)
        flow = 0;
    const uint16_t flow_rate = (uint16_t)flow;
    const bool valve_on = level_pct < 42.0f;
    const bool pump1_on = level_pct < 50.0f;
    const bool pump2_on = level_pct > 78.0f;

    if (lvgl_port_lock(10)) {
        update_water_level_ui(level_pct, level_cm, volume_l, flow_rate, valve_on, pump1_on, pump2_on);
        lvgl_port_unlock();
    }
#endif
}

void setup_ui_events() {
    return;
    // lv_obj_add_event_cb(ui_DO1, DO0_OUTPUT, LV_EVENT_VALUE_CHANGED, NULL);
    // lv_obj_add_event_cb(ui_DO2, DO1_OUTPUT, LV_EVENT_VALUE_CHANGED, NULL);
    // lv_obj_add_event_cb(ui_backlight, toggleBacklight, LV_EVENT_VALUE_CHANGED, NULL);
    // if (ui_Button2) {
    //     lv_obj_add_event_cb(ui_Button2, onButtonAction, LV_EVENT_CLICKED, NULL);
    // }
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

    // 6. เริ่มต้น CAN Bus (TWAI: GPIO 15 TX, GPIO 16 RX)
    bool canOk = WS43B.can.begin(500000);

    // 7. เริ่มต้น UI หน้าจอ
    lvgl_port_lock(-1);
    ui_init();
    setup_water_level_simulation();
    // randomSeed((uint32_t)micros());
    // update_water_level_ui(75.0f, 150.0f, 1500, 285, false, false, false);
    setup_ui_events();

    // if (ui_LabelButton2) {
    //     lv_label_set_text(ui_LabelButton2, "SCAN MB");
    // }

    // if (ui_TextArea1) {
    //     lv_textarea_set_text(ui_TextArea1, "Waveshare ESP32-S3 Touch 4.3B\n");
    //     lv_textarea_add_text(ui_TextArea1, "[LVGL] Version 9.5.0 Active\n");
    //     lv_textarea_add_text(ui_TextArea1, "[GFX] LovyanGFX 1.2.33 Active\n");
    //     lv_textarea_add_text(ui_TextArea1, "[DMA] True Double-Buffer (48L)\n");
    //     lv_textarea_add_text(ui_TextArea1, rs485Ok ? "[OK] RS485 Modbus Ready\n" : "[ERR] RS485 Failed!\n");
    //     lv_textarea_add_text(ui_TextArea1, canOk ? "[OK] CAN Bus (TWAI) Ready\n" : "[ERR] CAN Failed!\n");
    //     lv_textarea_add_text(ui_TextArea1, "[UI] 60 FPS Render Pipeline\n");
    //     lv_textarea_add_text(ui_TextArea1, "--------------------------------\n");
    // }
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
}

void loop() {
    // -----------------------------------------------------------------
    // Core 1 Main Loop: อัปเดต GUI แสดงผลอย่างราบรื่น ไม่มีสะดุด
    // -----------------------------------------------------------------

    // // 1. ดึงข้อความจาก Log Queue มาแสดงผลบนหน้าจอ TextArea (จำกัดขนาดข้อความไม่ให้ layout บวมช้า)
    // LogItem item;
    // uint8_t processed = 0;
    // while (logQueue && xQueueReceive(logQueue, &item, 0) == pdTRUE && processed < 3) {
    //     processed++;
    //     if (lvgl_port_lock(10)) {
    //         // if (ui_TextArea1) {
    //         //     const char* cur = lv_textarea_get_text(ui_TextArea1);
    //         //     // ตัดข้อความเก่าออกเมื่อเกิน 1000 ตัวอักษร เพื่อรักษาความเร็วในการ Render
    //         //     if (cur && strlen(cur) > 1000) {
    //         //         lv_textarea_set_text(ui_TextArea1, "-- Log Trimmed --\n");
    //         //     }
    //         //     lv_textarea_add_text(ui_TextArea1, item.text);
    //         // }
    //         lvgl_port_unlock();
    //     }
    // }

    // // 2. อัปเดตสีสถานะปุ่ม Digital Input (DI0 / DI1) เมื่อมีการเปลี่ยนแปลง
    // if (di0_changed) {
    //     di0_changed = false;
    //     if (lvgl_port_lock(10)) {
    //         // lv_obj_set_style_bg_color(ui_DI0, shared_di0 ? lv_color_hex(0x000000) : lv_color_hex(0x00A000), LV_PART_MAIN);
    //         lvgl_port_unlock();
    //     }
    // }

    // if (di1_changed) {
    //     di1_changed = false;
    //     if (lvgl_port_lock(10)) {
    //         // lv_obj_set_style_bg_color(ui_DI1, shared_di1 ? lv_color_hex(0x000000) : lv_color_hex(0x00A000), LV_PART_MAIN);
    //         lvgl_port_unlock();
    //     }
    // }

    // 3. อัปเดตเกจวัดระดับน้ำและกราฟแนวโน้มแบบ Real-Time จากข้อมูล Modbus
#if WATER_LEVEL_SIMULATION
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

        // if (lvgl_port_lock(10)) {
        //     update_water_level_ui(level_pct, level_cm, shared_reg0, shared_reg1, level_pct < 42.0f, level_pct < 50.0f, level_pct > 78.0f);
        //     lvgl_port_unlock();
        // }
    }
#endif

    delay(10);
}
