/**
 * @file 05_IO_Expander_And_Digital_IO.ino
 * @brief Test for CH422G IO Expander: Backlight control & Industrial Digital IO.
 * 
 * Hardware Connections on Terminal Block P1:
 *   - Pin 13: DI_COM (Digital Input Common Reference)
 *   - Pin 15: DIN0   (Isolated Digital Input 0)
 *   - Pin 16: DIN1   (Isolated Digital Input 1)
 *   - Pin 11: DOUT0  (Isolated Digital Output 0)
 *   - Pin 12: DOUT1  (Isolated Digital Output 1)
 */

#include <Arduino.h>
#include <WaveShare4_3B.h>

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n--- Waveshare ESP32-S3-Touch-LCD-4.3B: IO Expander & Industrial IO ---");

    // Initialize I2C first
    WS43B.i2c.begin();

    // Initialize CH422G IO Expander
    if (WS43B.io.begin(WS43B.i2c.getWire())) {
        Serial.println("[OK] CH422G IO Expander initialized.");
    } else {
        Serial.println("[ERR] CH422G not responding!");
    }

    // Turn backlight ON
    WS43B.io.setBacklight(true);
    Serial.println("[OK] Screen Backlight turned ON.");
}

unsigned long lastToggle = 0;
bool backlightState = true;

void loop() {
    // 1. Read Isolated Digital Inputs (DIN0, DIN1)
    bool di0 = WS43B.io.readDigitalInput(0);
    bool di1 = WS43B.io.readDigitalInput(1);

    Serial.print("DIN0: ");
    Serial.print(di0 ? "HIGH" : "LOW");
    Serial.print(" | DIN1: ");
    Serial.println(di1 ? "HIGH" : "LOW");

    // 2. Toggle backlight every 3 seconds as demonstration
    if (millis() - lastToggle >= 3000) {
        lastToggle = millis();
        backlightState = !backlightState;
        WS43B.io.setBacklight(backlightState);
        Serial.print(">>> Backlight state toggled: ");
        Serial.println(backlightState ? "ON" : "OFF");
    }

    delay(500);
}
