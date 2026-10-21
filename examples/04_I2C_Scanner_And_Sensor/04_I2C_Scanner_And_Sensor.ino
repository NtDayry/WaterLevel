/**
 * @file 04_I2C_Scanner_And_Sensor.ino
 * @brief I2C Scanner and External Device Test for Waveshare ESP32-S3-Touch-LCD-4.3B.
 * 
 * Hardware Connections on Terminal Block P1:
 *   - Pin 3: I2C_VCC (Power output: 3.3V or 5V selected via onboard jumper)
 *   - Pin 4: GND
 *   - Pin 5: D_SDA (GPIO 8)
 *   - Pin 6: D_SCL (GPIO 9)
 * 
 * The I2C bus is shared between onboard ICs (GT911 Touch, CH422G IO Expander)
 * and the external terminal pins.
 */

#include <Arduino.h>
#include <WaveShare4_3B.h>

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n--- Waveshare ESP32-S3-Touch-LCD-4.3B: I2C Scanner ---");

    // Initialize I2C bus at 100 kHz (GPIO 8 SDA, GPIO 9 SCL)
    if (WS43B.i2c.begin(100000)) {
        Serial.println("[OK] I2C bus initialized.");
    } else {
        Serial.println("[ERR] I2C initialization failed!");
    }

    // Run bus scan and print all detected chips
    std::vector<uint8_t> devices = WS43B.i2c.scan(Serial);

    Serial.println("\nExpected Onboard Devices:");
    Serial.println("  - 0x5D / 0x14 : GT911 Capacitive Touch Controller");
    Serial.println("  - 0x24 / 0x38 : CH422G IO Expander (System / Output registers)");
    Serial.println("  - 0x51        : PCF85063 RTC (if fitted)");
    Serial.println("Any other addresses are external sensors wired to D_SDA / D_SCL.\n");
}

unsigned long lastCheck = 0;

void loop() {
    // Periodically re-scan every 5 seconds to detect dynamically plugged sensors
    if (millis() - lastCheck >= 5000) {
        lastCheck = millis();
        WS43B.i2c.scan(Serial);
    }
}
