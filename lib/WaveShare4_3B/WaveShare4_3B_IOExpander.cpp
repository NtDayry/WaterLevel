/**
 * @file WaveShare4_3B_IOExpander.cpp
 * @brief Implementation of driver for CH422G on Waveshare ESP32-S3-Touch-LCD-4.3B.
 */

#include "WaveShare4_3B_IOExpander.h"

WaveShare4_3B_IOExpander::WaveShare4_3B_IOExpander(TwoWire& wire)
    : _wire(wire), _isInitialized(false),
      // Default state: Backlight(EXIO2)=1, TP_RST(EXIO1)=1, LCD_RST(EXIO3)=1, SD_CS(EXIO4)=1
      _outputState((1 << WS43B_EXIO_DISP) | (1 << WS43B_EXIO_TP_RST) |
                   (1 << WS43B_EXIO_LCD_RST) | (1 << WS43B_EXIO_SD_CS))
{
}

WaveShare4_3B_IOExpander::~WaveShare4_3B_IOExpander() {
}

bool WaveShare4_3B_IOExpander::begin(TwoWire& wire) {
    _wire = wire;

    // 1. Enable CH422G IO Output Mode
    // Write 0x01 to address 0x24 (Set system parameters: IO mode enabled)
    _wire.beginTransmission(WS43B_I2C_ADDR_CH422G_SYS);
    _wire.write(0x01);
    uint8_t err = _wire.endTransmission();
    if (err != 0) {
        return false;
    }

    _isInitialized = true;

    // 2. Set default pin states (Turn on backlight, release all resets)
    writeAll(_outputState);

    return true;
}

bool WaveShare4_3B_IOExpander::writeAll(uint8_t outputMask) {
    _outputState = outputMask;
    _wire.beginTransmission(WS43B_I2C_ADDR_CH422G_OUT);
    _wire.write(_outputState);
    return (_wire.endTransmission() == 0);
}

uint8_t WaveShare4_3B_IOExpander::readAll() {
    _wire.requestFrom((uint8_t)WS43B_I2C_ADDR_CH422G_IN, (uint8_t)1);
    if (_wire.available()) {
        return (uint8_t)_wire.read();
    }
    return 0;
}

bool WaveShare4_3B_IOExpander::writePin(uint8_t pin, bool level) {
    if (pin > 7) return false;
    if (level) {
        _outputState |= (1 << pin);
    } else {
        _outputState &= ~(1 << pin);
    }
    return writeAll(_outputState);
}

bool WaveShare4_3B_IOExpander::readPin(uint8_t pin) {
    if (pin > 7) return false;
    uint8_t val = readAll();
    return (val & (1 << pin)) != 0;
}

void WaveShare4_3B_IOExpander::setBacklight(bool on) {
    writePin(WS43B_EXIO_DISP, on);
}

void WaveShare4_3B_IOExpander::setTouchReset(bool active) {
    // Active LOW reset: LOW = in reset, HIGH = running
    writePin(WS43B_EXIO_TP_RST, !active);
}

void WaveShare4_3B_IOExpander::setLCDReset(bool active) {
    // Active LOW reset: LOW = in reset, HIGH = running
    writePin(WS43B_EXIO_LCD_RST, !active);
}

void WaveShare4_3B_IOExpander::setSDCS(bool active) {
    // Active LOW chip select: LOW = selected, HIGH = deselected
    writePin(WS43B_EXIO_SD_CS, !active);
}

bool WaveShare4_3B_IOExpander::readDigitalInput(uint8_t channel) {
    uint8_t pin = (channel == 0) ? WS43B_EXIO_DI0 : WS43B_EXIO_DI1;
    // The optocoupler input typically inverts or passes signal
    return readPin(pin);
}
