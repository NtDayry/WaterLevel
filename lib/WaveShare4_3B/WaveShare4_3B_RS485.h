/**
 * @file WaveShare4_3B_RS485.h
 * @brief RS485 communication wrapper with automatic direction control and Modbus helpers
 * for Waveshare ESP32-S3-Touch-LCD-4.3B.
 */

#ifndef WAVESHARE_4_3B_RS485_H
#define WAVESHARE_4_3B_RS485_H

#include <Arduino.h>
#include "WaveShare4_3B_Config.h"

class WaveShare4_3B_RS485 : public Stream {
public:
    WaveShare4_3B_RS485(HardwareSerial& serial = Serial1);
    ~WaveShare4_3B_RS485();

    /**
     * @brief Initialize RS485 hardware serial interface.
     * @param baud Baud rate (e.g. 9600, 19200, 38400, 115200). Default is 9600.
     * @param config Serial configuration (e.g. SERIAL_8N1, SERIAL_8E1). Default is SERIAL_8N1.
     * @param rxPin RX GPIO (default WS43B_RS485_RX_PIN = 43).
     * @param txPin TX GPIO (default WS43B_RS485_TX_PIN = 44).
     * @return true if initialized successfully.
     */
    bool begin(unsigned long baud = WS43B_RS485_DEFAULT_BAUD,
               uint32_t config = SERIAL_8N1,
               int rxPin = WS43B_RS485_RX_PIN,
               int txPin = WS43B_RS485_TX_PIN);

    /**
     * @brief De-initialize the serial interface.
     */
    void end();

    // Stream interface implementation
    virtual int available() override;
    virtual int read() override;
    virtual int peek() override;
    virtual void flush() override;
    virtual size_t write(uint8_t byte) override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;

    using Print::write; // pull in write(const char *str) etc.

    /**
     * @brief Send raw byte array over RS485 bus.
     */
    size_t sendBytes(const uint8_t *buffer, size_t size);

    /**
     * @brief Clear receive buffer.
     */
    void clearRxBuffer();

    /**
     * @brief Read data into buffer until specified length or timeout.
     * @param buffer Destination buffer.
     * @param length Expected number of bytes.
     * @param timeoutMs Timeout in milliseconds.
     * @return Number of bytes actually read.
     */
    size_t readBytesTimeout(uint8_t *buffer, size_t length, unsigned long timeoutMs = 200);

    // =========================================================================
    // Modbus RTU Support Functions
    // =========================================================================

    /**
     * @brief Compute standard Modbus RTU CRC16.
     * @param data Byte buffer.
     * @param length Number of bytes.
     * @return 16-bit CRC with low byte first.
     */
    static uint16_t calculateModbusCRC16(const uint8_t *data, size_t length);

    /**
     * @brief Send a Modbus RTU frame. The function automatically computes and
     * appends the 2-byte CRC16 at the end of the payload.
     * @param payload Modbus frame excluding CRC.
     * @param payloadLength Length of frame excluding CRC.
     * @return Number of total bytes sent (payloadLength + 2).
     */
    size_t sendModbusFrame(const uint8_t *payload, size_t payloadLength);

    /**
     * @brief Receive a Modbus RTU frame and verify its CRC16.
     * @param buffer Destination buffer.
     * @param maxLen Maximum buffer capacity.
     * @param[out] receivedLen Actual number of valid payload bytes received (excluding CRC).
     * @param timeoutMs Time to wait for response in milliseconds.
     * @return true if valid frame with matching CRC16 was received.
     */
    bool receiveModbusFrame(uint8_t *buffer, size_t maxLen, size_t &receivedLen, unsigned long timeoutMs = 200);

    /**
     * @brief Helper to send a Modbus RTU Read Holding Registers request (Function 0x03).
     * @param slaveId Slave device address (1-247).
     * @param startReg Starting register address (0-65535).
     * @param regCount Number of registers to read (1-125).
     * @return true if request was sent.
     */
    bool sendModbusReadHolding(uint8_t slaveId, uint16_t startReg, uint16_t regCount);

    /**
     * @brief Helper to send a Modbus RTU Write Single Register request (Function 0x06).
     * @param slaveId Slave device address.
     * @param regAddr Register address.
     * @param regValue 16-bit value to write.
     * @return true if request was sent.
     */
    bool sendModbusWriteSingle(uint8_t slaveId, uint16_t regAddr, uint16_t regValue);

    /**
     * @brief Get underlying HardwareSerial instance.
     */
    HardwareSerial& getSerial() { return _serial; }

private:
    HardwareSerial& _serial;
    bool _isInitialized;
    int _rxPin;
    int _txPin;
};

#endif // WAVESHARE_4_3B_RS485_H
