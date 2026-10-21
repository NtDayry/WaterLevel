/**
 * @file 02_RS485_Modbus_Read.ino
 * @brief Modbus RTU Master query example for Waveshare ESP32-S3-Touch-LCD-4.3B.
 * 
 * Hardware Connections on Terminal Block P1:
 *   - Pin 9:  RS485_A (D+)
 *   - Pin 10: RS485_B (D-)
 *   - Pin 2:  GND
 * 
 * Demonstrates:
 *   1. Sending Modbus RTU Read Holding Registers request (Function 0x03)
 *   2. Automatically appending Modbus CRC16
 *   3. Receiving response, verifying CRC16, and extracting register values
 */

#include <Arduino.h>
#include <WaveShare4_3B.h>

const uint8_t SLAVE_ID = 1;         // Target Modbus Slave ID
const uint16_t START_REG = 0x0000;  // Register address to read
const uint16_t REG_COUNT = 2;       // Read 2 registers (4 bytes)

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n--- Waveshare ESP32-S3-Touch-LCD-4.3B: Modbus RTU Master ---");

    // Initialize RS485 (usually 9600 baud, 8N1 for standard Modbus sensors)
    WS43B.rs485.begin(9600);
    Serial.println("[OK] Modbus RTU Master ready on RS485.");
}

unsigned long lastQuery = 0;

void loop() {
    // Query every 2 seconds
    if (millis() - lastQuery >= 2000) {
        lastQuery = millis();

        Serial.print("[Modbus Master] Sending Read Request to Slave ID ");
        Serial.print(SLAVE_ID);
        Serial.print(", Start Reg: 0x");
        Serial.print(START_REG, HEX);
        Serial.print(", Count: ");
        Serial.println(REG_COUNT);

        // Send Modbus Function 0x03 (Read Holding Registers)
        WS43B.rs485.sendModbusReadHolding(SLAVE_ID, START_REG, REG_COUNT);

        // Receive response with 300ms timeout
        uint8_t rxBuffer[64];
        size_t rxLen = 0;

        if (WS43B.rs485.receiveModbusFrame(rxBuffer, sizeof(rxBuffer), rxLen, 300)) {
            Serial.print("[Modbus Master] Valid Response Received! Payload Bytes: ");
            Serial.println(rxLen);

            // Print raw payload
            Serial.print("Raw: ");
            for (size_t i = 0; i < rxLen; i++) {
                if (rxBuffer[i] < 0x10) Serial.print("0");
                Serial.print(rxBuffer[i], HEX);
                Serial.print(" ");
            }
            Serial.println();

            // Verify Function Code and Byte Count
            if (rxBuffer[0] == SLAVE_ID && rxBuffer[1] == 0x03) {
                uint8_t byteCount = rxBuffer[2];
                Serial.print("Data Bytes: ");
                Serial.println(byteCount);

                for (uint8_t r = 0; r < byteCount / 2; r++) {
                    uint16_t regVal = ((uint16_t)rxBuffer[3 + r * 2] << 8) | rxBuffer[4 + r * 2];
                    Serial.print("  Register [");
                    Serial.print(START_REG + r);
                    Serial.print("] = ");
                    Serial.print(regVal);
                    Serial.print(" (0x");
                    Serial.print(regVal, HEX);
                    Serial.println(")");
                }
            } else if (rxBuffer[1] & 0x80) {
                // Modbus Exception Response
                Serial.print("[!] Modbus Exception Code: 0x");
                Serial.println(rxBuffer[2], HEX);
            }
        } else {
            Serial.println("[Timeout / Error] No valid Modbus response or CRC mismatch.");
        }
    }
}
