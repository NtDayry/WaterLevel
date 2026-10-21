/**
 * @file WaveShare4_3B_IOExpander.h
 * @brief Driver for onboard CH422G I2C IO Expander on Waveshare ESP32-S3-Touch-LCD-4.3B.
 * Controls LCD backlight, LCD reset, capacitive touch reset, SD card CS,
 * and the isolated Digital Inputs (DIN0, DIN1) & Outputs (DOUT0, DOUT1).
 */

#ifndef WAVESHARE_4_3B_IO_EXPANDER_H
#define WAVESHARE_4_3B_IO_EXPANDER_H

#include <Arduino.h>
#include <Wire.h>
#include "WaveShare4_3B_Config.h"

class WaveShare4_3B_IOExpander {
public:
    WaveShare4_3B_IOExpander(TwoWire& wire = Wire);
    ~WaveShare4_3B_IOExpander();

    /**
     * @brief Initialize CH422G IO Expander and set default pin states.
     * Turns on the display backlight by default.
     * @param wire TwoWire instance (default Wire).
     * @return true if initialized and responding on I2C.
     */
    bool begin(TwoWire& wire = Wire);

    /**
     * @brief Control LCD display enable & backlight (EXIO2 / DISP).
     * @param on true to turn on backlight, false to turn off.
     */
    void setBacklight(bool on);

    /**
     * @brief Control Capacitive Touch GT911 Reset pin (EXIO1 / TP_RST).
     * @param active true to pull into reset (LOW), false to release reset (HIGH).
     */
    void setTouchReset(bool active);

    /**
     * @brief Control ST7701 LCD Display Controller Reset pin (EXIO3 / LCD_RST).
     * @param active true to pull into reset (LOW), false to release reset (HIGH).
     */
    void setLCDReset(bool active);

    /**
     * @brief Control MicroSD Card Chip Select pin (EXIO4 / SD_CS).
     * @param active true to select (LOW), false to deselect (HIGH).
     */
    void setSDCS(bool active);

    /**
     * @brief Read state of isolated Digital Input on the external terminal block.
     * @param channel 0 for DIN0 (P1 pin 15 / EXIO0), 1 for DIN1 (P1 pin 16 / EXIO5).
     * @return true if input is HIGH/Active, false if LOW/Inactive.
     */
    bool readDigitalInput(uint8_t channel);

    /**
     * @brief Set state of individual expander pin (0 to 7).
     * @param pin Pin index (0 to 7).
     * @param level HIGH (1) or LOW (0).
     */
    bool writePin(uint8_t pin, bool level);

    /**
     * @brief Read state of individual expander pin (0 to 7).
     * @param pin Pin index (0 to 7).
     * @return Pin level HIGH (1) or LOW (0).
     */
    bool readPin(uint8_t pin);

    /**
     * @brief Write 8-bit output bitmask to all expanded pins.
     * @param outputMask 8-bit state.
     */
    bool writeAll(uint8_t outputMask);

    /**
     * @brief Read all 8 input pins at once.
     * @return 8-bit input bitmask.
     */
    uint8_t readAll();

    /**
     * @brief Get current cached output states.
     */
    uint8_t getCachedOutput() const { return _outputState; }

private:
    TwoWire& _wire;
    bool _isInitialized;
    uint8_t _outputState;
};

#endif // WAVESHARE_4_3B_IO_EXPANDER_H
