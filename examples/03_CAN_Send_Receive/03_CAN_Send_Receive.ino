/**
 * @file 03_CAN_Send_Receive.ino
 * @brief CAN Bus (TWAI) Send and Receive example for Waveshare ESP32-S3-Touch-LCD-4.3B.
 * 
 * Hardware Connections on Terminal Block P1:
 *   - Pin 7: CANL (CAN Low)
 *   - Pin 8: CANH (CAN High)
 *   - Pin 2: GND
 * 
 * Note:
 *   - Transceiver: Onboard TJA1051T/3
 *   - Terminating Resistor: Enable the onboard 120-ohm switch (SW2) if this board is at the bus end.
 */

#include <Arduino.h>
#include <WaveShare4_3B.h>

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n--- Waveshare ESP32-S3-Touch-LCD-4.3B: CAN Bus (TWAI) Demo ---");

    // Initialize CAN at 500 kbps (GPIO15 TX, GPIO16 RX)
    if (WS43B.can.begin(500000)) {
        Serial.println("[OK] CAN (TWAI) initialized at 500 kbps.");
    } else {
        Serial.println("[ERR] CAN Initialization Failed!");
    }
}

unsigned long lastSend = 0;
uint32_t packetCount = 0;

void loop() {
    // 1. Send CAN frame every 1 second
    if (millis() - lastSend >= 1000) {
        lastSend = millis();
        packetCount++;

        // Prepare test payload
        uint8_t txData[8];
        txData[0] = (uint8_t)(packetCount >> 24);
        txData[1] = (uint8_t)(packetCount >> 16);
        txData[2] = (uint8_t)(packetCount >> 8);
        txData[3] = (uint8_t)(packetCount & 0xFF);
        txData[4] = 0xAA;
        txData[5] = 0xBB;
        txData[6] = 0xCC;
        txData[7] = 0xDD;

        uint32_t canId = 0x123; // Standard 11-bit ID

        if (WS43B.can.send(canId, txData, 8, false)) {
            Serial.print("[CAN TX] Sent frame ID 0x123, Count: ");
            Serial.println(packetCount);
        } else {
            Serial.println("[CAN TX] Error sending frame! Check bus connection / termination.");
            WS43B.can.recoverBus();
        }
    }

    // 2. Check for incoming CAN frames
    twai_message_t rxMsg;
    if (WS43B.can.receiveMessage(rxMsg, 10)) {
        Serial.print("[CAN RX] ");
        WaveShare4_3B_CAN::printMessage(rxMsg, Serial);
    }
}
