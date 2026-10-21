/**
 * @file 01_RS485_Echo_Test.ino
 * @brief RS485 Transmit and Echo / Receive Test for Waveshare ESP32-S3-Touch-LCD-4.3B.
 * 
 * Hardware Connections on Terminal Block P1:
 *   - Pin 9:  RS485_A (D+)
 *   - Pin 10: RS485_B (D-)
 *   - Pin 2 or 14: GND (recommended for common reference)
 * 
 * Note: SP3485EN has automatic hardware direction control. No DE/RE pin is needed.
 * Turn on the onboard 120-ohm termination switch if this board is at the end of the RS485 bus.
 */

#include <Arduino.h>
#include <WaveShare4_3B.h>

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n--- Waveshare ESP32-S3-Touch-LCD-4.3B: RS485 Test ---");

    // Initialize RS485 at 9600 baud, 8N1
    if (WS43B.rs485.begin(9600)) {
        Serial.println("[OK] RS485 Initialized at 9600 baud.");
    } else {
        Serial.println("[ERR] RS485 Initialization Failed!");
    }

    WS43B.rs485.println("Hello from Waveshare ESP32-S3-Touch-LCD-4.3B over RS485!");
}

unsigned long lastSend = 0;
uint32_t counter = 0;

void loop() {
    // 1. Periodically send a message every 2 seconds
    if (millis() - lastSend >= 2000) {
        lastSend = millis();
        counter++;

        String msg = "Ping #" + String(counter) + " (millis: " + String(millis()) + ")\r\n";
        WS43B.rs485.print(msg);
        Serial.print("[RS485 TX] Sent: ");
        Serial.print(msg);
    }

    // 2. Receive incoming data from RS485 and print to USB Serial
    if (WS43B.rs485.available()) {
        Serial.print("[RS485 RX] Received: ");
        while (WS43B.rs485.available()) {
            char c = (char)WS43B.rs485.read();
            Serial.print(c);
            // Echo back to RS485
            WS43B.rs485.write(c);
        }
        Serial.println();
    }
}
