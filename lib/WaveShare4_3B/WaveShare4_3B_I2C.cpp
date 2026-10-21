/**
 * @file WaveShare4_3B_I2C.cpp
 * @brief Implementation of I2C Bus Manager and Scanner for Waveshare ESP32-S3-Touch-LCD-4.3B.
 */

#include "WaveShare4_3B_I2C.h"

WaveShare4_3B_I2C::WaveShare4_3B_I2C(TwoWire& wire)
    : _wire(wire), _isInitialized(false),
      _sdaPin(WS43B_I2C_SDA_PIN), _sclPin(WS43B_I2C_SCL_PIN),
      _frequency(WS43B_I2C_DEFAULT_FREQ)
{
}

WaveShare4_3B_I2C::~WaveShare4_3B_I2C() {
}

bool WaveShare4_3B_I2C::begin(uint32_t frequency, int sdaPin, int sclPin) {
    _frequency = frequency;
    _sdaPin = sdaPin;
    _sclPin = sclPin;

    bool ok = _wire.begin(_sdaPin, _sclPin, _frequency);
    if (ok) {
        _wire.setClock(_frequency);
        _isInitialized = true;
    }
    return ok;
}

bool WaveShare4_3B_I2C::isDeviceConnected(uint8_t address) {
    _wire.beginTransmission(address);
    return (_wire.endTransmission() == 0);
}

const char* WaveShare4_3B_I2C::getDeviceDescription(uint8_t address) {
    switch (address) {
        case WS43B_I2C_ADDR_GT911_PRIMARY:
            return "GT911 (Capacitive Touch Screen Controller, Primary)";
        case WS43B_I2C_ADDR_GT911_ALT:
            return "GT911 (Capacitive Touch Screen Controller, Alternate)";
        case WS43B_I2C_ADDR_CH422G_SYS:
            return "CH422G (IO Expander - System Parameter)";
        case WS43B_I2C_ADDR_CH422G_OUT:
            return "CH422G (IO Expander - Output Control)";
        case WS43B_I2C_ADDR_CH422G_IN:
            return "CH422G (IO Expander - Input Read)";
        case WS43B_I2C_ADDR_PCF85063:
            return "PCF85063 (Real-Time Clock RTC, if equipped)";
        default:
            return "External I2C Device (Connected to Terminal D_SDA / D_SCL)";
    }
}

std::vector<uint8_t> WaveShare4_3B_I2C::scan(Stream &out) {
    std::vector<uint8_t> foundDevices;

    out.println("=================================================");
    out.println("  WaveShare ESP32-S3-Touch-LCD-4.3B I2C Bus Scan");
    out.print("  SDA: GPIO"); out.print(_sdaPin);
    out.print(" | SCL: GPIO"); out.print(_sclPin);
    out.print(" | Freq: "); out.print(_frequency / 1000); out.println(" kHz");
    out.println("=================================================");

    int count = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        _wire.beginTransmission(addr);
        uint8_t error = _wire.endTransmission();

        if (error == 0) {
            foundDevices.push_back(addr);
            count++;
            out.print(" [OK] Found device at 0x");
            if (addr < 16) out.print("0");
            out.print(addr, HEX);
            out.print(" -> ");
            out.println(getDeviceDescription(addr));
        } else if (error == 4) {
            out.print(" [!] Unknown error at 0x");
            if (addr < 16) out.print("0");
            out.println(addr, HEX);
        }
    }

    out.println("-------------------------------------------------");
    out.print("Scan complete. Total devices detected: ");
    out.println(count);
    out.println("=================================================");

    return foundDevices;
}

bool WaveShare4_3B_I2C::writeRegister8(uint8_t devAddr, uint8_t regAddr, uint8_t value) {
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    _wire.write(value);
    return (_wire.endTransmission() == 0);
}

bool WaveShare4_3B_I2C::readRegister8(uint8_t devAddr, uint8_t regAddr, uint8_t &value) {
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    if (_wire.endTransmission(false) != 0) return false;

    if (_wire.requestFrom(devAddr, (uint8_t)1) == 1) {
        value = (uint8_t)_wire.read();
        return true;
    }
    return false;
}

bool WaveShare4_3B_I2C::writeRegister16(uint8_t devAddr, uint8_t regAddr, uint16_t value, bool msbFirst) {
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    if (msbFirst) {
        _wire.write((uint8_t)((value >> 8) & 0xFF));
        _wire.write((uint8_t)(value & 0xFF));
    } else {
        _wire.write((uint8_t)(value & 0xFF));
        _wire.write((uint8_t)((value >> 8) & 0xFF));
    }
    return (_wire.endTransmission() == 0);
}

bool WaveShare4_3B_I2C::readRegister16(uint8_t devAddr, uint8_t regAddr, uint16_t &value, bool msbFirst) {
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    if (_wire.endTransmission(false) != 0) return false;

    if (_wire.requestFrom(devAddr, (uint8_t)2) == 2) {
        uint8_t b1 = (uint8_t)_wire.read();
        uint8_t b2 = (uint8_t)_wire.read();
        if (msbFirst) {
            value = ((uint16_t)b1 << 8) | b2;
        } else {
            value = ((uint16_t)b2 << 8) | b1;
        }
        return true;
    }
    return false;
}

bool WaveShare4_3B_I2C::writeBytes(uint8_t devAddr, uint8_t regAddr, const uint8_t *data, size_t length) {
    if (!data && length > 0) return false;
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    for (size_t i = 0; i < length; i++) {
        _wire.write(data[i]);
    }
    return (_wire.endTransmission() == 0);
}

bool WaveShare4_3B_I2C::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *buffer, size_t length) {
    if (!buffer || length == 0) return false;
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    if (_wire.endTransmission(false) != 0) return false;

    size_t received = _wire.requestFrom(devAddr, (uint8_t)length);
    if (received == length) {
        for (size_t i = 0; i < length; i++) {
            buffer[i] = (uint8_t)_wire.read();
        }
        return true;
    }
    return false;
}
