/**
 * @file WaveShare4_3B_I2C.h
 * @brief I2C Bus Manager and Scanner for Waveshare ESP32-S3-Touch-LCD-4.3B.
 * Manages the shared I2C bus between onboard devices (Touch, IO Expander, RTC)
 * and the external screw terminal pins (D_SDA Pin 5, D_SCL Pin 6).
 */

#ifndef WAVESHARE_4_3B_I2C_H
#define WAVESHARE_4_3B_I2C_H

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include "WaveShare4_3B_Config.h"

class WaveShare4_3B_I2C {
public:
    WaveShare4_3B_I2C(TwoWire& wire = Wire);
    ~WaveShare4_3B_I2C();

    /**
     * @brief Initialize I2C bus with board default SDA/SCL pins.
     * @param frequency Clock frequency in Hz (default 100000 Hz, supports up to 400000 Hz).
     * @param sdaPin SDA GPIO (default WS43B_I2C_SDA_PIN = 8).
     * @param sclPin SCL GPIO (default WS43B_I2C_SCL_PIN = 9).
     * @return true if initialized successfully.
     */
    bool begin(uint32_t frequency = WS43B_I2C_DEFAULT_FREQ,
               int sdaPin = WS43B_I2C_SDA_PIN,
               int sclPin = WS43B_I2C_SCL_PIN);

    /**
     * @brief Check if a specific I2C device acknowledges.
     * @param address 7-bit slave address.
     * @return true if ACK received.
     */
    bool isDeviceConnected(uint8_t address);

    /**
     * @brief Scan the entire I2C bus (0x01 to 0x7F) and print detected devices.
     * Identifies known onboard hardware (Touch, IO expander, RTC) and external sensors.
     * @param out Stream to output report (default Serial).
     * @return List of detected 7-bit addresses.
     */
    std::vector<uint8_t> scan(Stream &out = Serial);

    /**
     * @brief Helper to write an 8-bit register on an I2C device.
     */
    bool writeRegister8(uint8_t devAddr, uint8_t regAddr, uint8_t value);

    /**
     * @brief Helper to read an 8-bit register from an I2C device.
     */
    bool readRegister8(uint8_t devAddr, uint8_t regAddr, uint8_t &value);

    /**
     * @brief Helper to write a 16-bit register on an I2C device.
     */
    bool writeRegister16(uint8_t devAddr, uint8_t regAddr, uint16_t value, bool msbFirst = true);

    /**
     * @brief Helper to read a 16-bit register from an I2C device.
     */
    bool readRegister16(uint8_t devAddr, uint8_t regAddr, uint16_t &value, bool msbFirst = true);

    /**
     * @brief Helper to write multiple bytes to a register.
     */
    bool writeBytes(uint8_t devAddr, uint8_t regAddr, const uint8_t *data, size_t length);

    /**
     * @brief Helper to read multiple bytes from a register.
     */
    bool readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *buffer, size_t length);

    /**
     * @brief Get underlying TwoWire instance to pass to external sensor libraries.
     */
    TwoWire& getWire() { return _wire; }

    /**
     * @brief Get human-readable description for known I2C devices on this board.
     */
    static const char* getDeviceDescription(uint8_t address);

private:
    TwoWire& _wire;
    bool _isInitialized;
    int _sdaPin;
    int _sclPin;
    uint32_t _frequency;
};

#endif // WAVESHARE_4_3B_I2C_H
