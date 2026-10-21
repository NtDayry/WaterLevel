/**
 * @file WaveShare4_3B_RS485.cpp
 * @brief Implementation of RS485 communication wrapper for Waveshare ESP32-S3-Touch-LCD-4.3B.
 */

#include "WaveShare4_3B_RS485.h"

WaveShare4_3B_RS485::WaveShare4_3B_RS485(HardwareSerial& serial)
    : _serial(serial), _isInitialized(false),
      _rxPin(WS43B_RS485_RX_PIN), _txPin(WS43B_RS485_TX_PIN)
{
}

WaveShare4_3B_RS485::~WaveShare4_3B_RS485() {
    end();
}

bool WaveShare4_3B_RS485::begin(unsigned long baud, uint32_t config, int rxPin, int txPin) {
    _rxPin = rxPin;
    _txPin = txPin;
    
    // Initialize the hardware serial with the RS485 transceiver pins
    _serial.begin(baud, config, _rxPin, _txPin);
    _isInitialized = true;
    
    // Clear initial garbage on lines
    clearRxBuffer();
    return true;
}

void WaveShare4_3B_RS485::end() {
    if (_isInitialized) {
        _serial.end();
        _isInitialized = false;
    }
}

int WaveShare4_3B_RS485::available() {
    return _serial.available();
}

int WaveShare4_3B_RS485::read() {
    return _serial.read();
}

int WaveShare4_3B_RS485::peek() {
    return _serial.peek();
}

void WaveShare4_3B_RS485::flush() {
    _serial.flush();
}

size_t WaveShare4_3B_RS485::write(uint8_t byte) {
    return _serial.write(byte);
}

size_t WaveShare4_3B_RS485::write(const uint8_t *buffer, size_t size) {
    return _serial.write(buffer, size);
}

size_t WaveShare4_3B_RS485::sendBytes(const uint8_t *buffer, size_t size) {
    size_t written = _serial.write(buffer, size);
    _serial.flush(); // Ensure everything shifted out through hardware auto-direction
    return written;
}

void WaveShare4_3B_RS485::clearRxBuffer() {
    while (_serial.available() > 0) {
        _serial.read();
    }
}

size_t WaveShare4_3B_RS485::readBytesTimeout(uint8_t *buffer, size_t length, unsigned long timeoutMs) {
    size_t count = 0;
    unsigned long start = millis();
    while ((millis() - start < timeoutMs) && (count < length)) {
        if (_serial.available()) {
            buffer[count++] = (uint8_t)_serial.read();
            start = millis(); // Reset timeout on character received (inter-character timeout)
        }
    }
    return count;
}

uint16_t WaveShare4_3B_RS485::calculateModbusCRC16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

size_t WaveShare4_3B_RS485::sendModbusFrame(const uint8_t *payload, size_t payloadLength) {
    if (!payload || payloadLength == 0) return 0;

    uint16_t crc = calculateModbusCRC16(payload, payloadLength);
    uint8_t crcLow = (uint8_t)(crc & 0xFF);
    uint8_t crcHigh = (uint8_t)((crc >> 8) & 0xFF);

    clearRxBuffer(); // Clear any old incoming data before sending request

    size_t written = _serial.write(payload, payloadLength);
    written += _serial.write(crcLow);
    written += _serial.write(crcHigh);
    _serial.flush();

    return written;
}

bool WaveShare4_3B_RS485::receiveModbusFrame(uint8_t *buffer, size_t maxLen, size_t &receivedLen, unsigned long timeoutMs) {
    receivedLen = 0;
    if (!buffer || maxLen < 4) return false; // Minimum Modbus frame: Addr(1) + Func(1) + CRC(2) = 4 bytes

    unsigned long start = millis();
    size_t idx = 0;

    // Read bytes with inter-frame timeout
    while (millis() - start < timeoutMs) {
        if (_serial.available() > 0) {
            if (idx < maxLen) {
                buffer[idx++] = (uint8_t)_serial.read();
            } else {
                _serial.read(); // Drop overflowing bytes
            }
            start = millis(); // Reset timer for inter-character timeout
        } else if (idx > 0) {
            // Check for silence gap (~3.5 characters, roughly 3.5ms at 9600baud)
            delayMicroseconds(3500);
            if (_serial.available() == 0) {
                // Bus is silent, frame finished
                break;
            }
        }
    }

    if (idx < 4) return false;

    // Verify CRC
    uint16_t receivedCRC = (uint16_t)buffer[idx - 2] | ((uint16_t)buffer[idx - 1] << 8);
    uint16_t calculatedCRC = calculateModbusCRC16(buffer, idx - 2);

    if (receivedCRC == calculatedCRC) {
        receivedLen = idx - 2; // Payload excluding CRC
        return true;
    }

    return false;
}

bool WaveShare4_3B_RS485::sendModbusReadHolding(uint8_t slaveId, uint16_t startReg, uint16_t regCount) {
    uint8_t request[6];
    request[0] = slaveId;
    request[1] = 0x03; // Function: Read Holding Registers
    request[2] = (uint8_t)((startReg >> 8) & 0xFF);
    request[3] = (uint8_t)(startReg & 0xFF);
    request[4] = (uint8_t)((regCount >> 8) & 0xFF);
    request[5] = (uint8_t)(regCount & 0xFF);

    return sendModbusFrame(request, sizeof(request)) == 8;
}

bool WaveShare4_3B_RS485::sendModbusWriteSingle(uint8_t slaveId, uint16_t regAddr, uint16_t regValue) {
    uint8_t request[6];
    request[0] = slaveId;
    request[1] = 0x06; // Function: Write Single Register
    request[2] = (uint8_t)((regAddr >> 8) & 0xFF);
    request[3] = (uint8_t)(regAddr & 0xFF);
    request[4] = (uint8_t)((regValue >> 8) & 0xFF);
    request[5] = (uint8_t)(regValue & 0xFF);

    return sendModbusFrame(request, sizeof(request)) == 8;
}
