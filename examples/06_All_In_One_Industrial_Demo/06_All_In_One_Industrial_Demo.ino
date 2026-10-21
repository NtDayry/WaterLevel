/**
 * @file 06_All_In_One_Industrial_Demo.ino
 * @brief Comprehensive Industrial Telemetry Demo for Waveshare ESP32-S3-Touch-LCD-4.3B.
 * 
 * Demonstrates:
 *   - Unified initialization using `WS43B.begin()`
 *   - RS485 sending periodic Modbus / telemetry data
 *   - CAN bus listening and echoing status frames
 *   - I2C device status polling
 *   - CH422G backlight management and Digital Input monitoring
 */

#include <Arduino.h>
#include <WaveShare4_3B.h>

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================================");
    Serial.println("  Waveshare ESP32-S3-Touch-LCD-4.3B Industrial Demo");
    Serial.println("  Interfaces: RS485 | CAN (TWAI) | I2C | IO Expander");
    Serial.println("========================================================");

    // Initialize all hardware peripherals in one single call:
    // RS485: 9600 baud (GPIO44 TX, GPIO43 RX)
    // CAN:   500 kbps  (GPIO15 TX, GPIO16 RX)
    // I2C:   100 kHz   (GPIO8 SDA, GPIO9 SCL)
    // Backlight automatically enabled via CH422G!
    if (WS43B.begin(9600, 500000, 100000)) {
        Serial.println("[OK] All industrial peripherals initialized successfully!");
    } else {
        Serial.println("[!] One or more peripherals reported an init warning.");
    }

    // Run an initial I2C scan to display connected sensors
    WS43B.i2c.scan(Serial);

    Serial.println("\nSystem running. Listening on RS485 and CAN...");
}

unsigned long lastTelemetry = 0;
uint32_t cycle = 0;

void loop() {
    // -------------------------------------------------------------
    // 1. Process Incoming CAN Frames
    // -------------------------------------------------------------
    twai_message_t canMsg;
    if (WS43B.can.receiveMessage(canMsg, 5)) {
        Serial.print("[CAN RX] ");
        WaveShare4_3B_CAN::printMessage(canMsg, Serial);

        // Echo CAN message over RS485 as notification
        WS43B.rs485.print("CAN MSG ID: 0x");
        WS43B.rs485.println(canMsg.identifier, HEX);
    }

    // -------------------------------------------------------------
    // 2. Process Incoming RS485 Data
    // -------------------------------------------------------------
    if (WS43B.rs485.available()) {
        Serial.print("[RS485 RX] ");
        while (WS43B.rs485.available()) {
            char c = (char)WS43B.rs485.read();
            Serial.print(c);
        }
        Serial.println();
    }

    // -------------------------------------------------------------
    // 3. Periodic Telemetry Broadcast (Every 2 seconds)
    // -------------------------------------------------------------
    if (millis() - lastTelemetry >= 2000) {
        lastTelemetry = millis();
        cycle++;

        // Read Isolated Digital Inputs
        bool din0 = WS43B.io.readDigitalInput(0);
        bool din1 = WS43B.io.readDigitalInput(1);

        // A. Broadcast over RS485
        String rs485Packet = "TELEMETRY #" + String(cycle) +
                             " | DIN0:" + String(din0 ? "1" : "0") +
                             " | DIN1:" + String(din1 ? "1" : "0") +
                             " | UPTIME:" + String(millis() / 1000) + "s\r\n";
        WS43B.rs485.print(rs485Packet);
        Serial.print("[RS485 TX] " + rs485Packet);

        // B. Broadcast over CAN bus (ID 0x300, 8 bytes)
        uint8_t canData[8];
        canData[0] = (uint8_t)(cycle >> 8);
        canData[1] = (uint8_t)(cycle & 0xFF);
        canData[2] = din0 ? 1 : 0;
        canData[3] = din1 ? 1 : 0;
        canData[4] = (uint8_t)((millis() / 1000) >> 8);
        canData[5] = (uint8_t)((millis() / 1000) & 0xFF);
        canData[6] = 0xAA;
        canData[7] = 0x55;

        if (WS43B.can.send(0x300, canData, 8, false)) {
            Serial.println("[CAN TX] Sent telemetry frame ID 0x300");
        } else {
            Serial.println("[CAN TX] Send failed (check termination/bus)");
            WS43B.can.recoverBus();
        }
    }
}
