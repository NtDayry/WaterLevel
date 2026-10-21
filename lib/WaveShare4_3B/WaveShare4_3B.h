/**
 * @file WaveShare4_3B.h
 * @brief Main umbrella header and unified manager class for Waveshare ESP32-S3-Touch-LCD-4.3B.
 * Provides unified access to RS485, CAN (TWAI), I2C, and CH422G IO Expander.
 * 
 * Hardware Documentation: https://docs.waveshare.com/ESP32-S3-Touch-LCD-4.3B
 */

#ifndef WAVESHARE_4_3B_H
#define WAVESHARE_4_3B_H

#include <Arduino.h>
#include "WaveShare4_3B_Config.h"
#include "WaveShare4_3B_RS485.h"
#include "WaveShare4_3B_CAN.h"
#include "WaveShare4_3B_I2C.h"
#include "WaveShare4_3B_IOExpander.h"

class WaveShare4_3B {
public:
    WaveShare4_3B()
        : rs485(Serial1), can(), i2c(Wire), io(Wire)
    {
    }

    /**
     * @brief Initialize all essential board peripherals:
     *   1. I2C Bus (GPIO8 SDA, GPIO9 SCL)
     *   2. CH422G IO Expander (Enables Screen Backlight & releases resets)
     *   3. RS485 (GPIO44 TX, GPIO43 RX at specified baud, default 9600)
     *   4. CAN Bus (GPIO15 TX, GPIO16 RX at specified baud, default 500k)
     * 
     * @param rs485Baud RS485 baud rate (default 9600)
     * @param canBaud CAN Bus speed in bps (default 500000)
     * @param i2cFreq I2C clock frequency (default 100000 Hz)
     * @return true if all peripherals initialized successfully.
     */
    bool begin(unsigned long rs485Baud = WS43B_RS485_DEFAULT_BAUD,
               uint32_t canBaud = WS43B_CAN_DEFAULT_BAUD,
               uint32_t i2cFreq = WS43B_I2C_DEFAULT_FREQ)
    {
        bool ok = true;

        // 1. Initialize I2C
        if (!i2c.begin(i2cFreq, WS43B_I2C_SDA_PIN, WS43B_I2C_SCL_PIN)) {
            ok = false;
        }

        // 2. Initialize IO Expander (turns on screen backlight)
        if (!io.begin(i2c.getWire())) {
            ok = false;
        }

        // 3. Initialize RS485
        if (!rs485.begin(rs485Baud, SERIAL_8N1, WS43B_RS485_RX_PIN, WS43B_RS485_TX_PIN)) {
            ok = false;
        }

        // 4. Initialize CAN
        if (!can.begin(canBaud, (gpio_num_t)WS43B_CAN_TX_PIN, (gpio_num_t)WS43B_CAN_RX_PIN)) {
            ok = false;
        }

        return ok;
    }

    // Peripherals
    WaveShare4_3B_RS485      rs485;
    WaveShare4_3B_CAN        can;
    WaveShare4_3B_I2C        i2c;
    WaveShare4_3B_IOExpander io;
};

// Global convenience instance
extern WaveShare4_3B WS43B;

#endif // WAVESHARE_4_3B_H
