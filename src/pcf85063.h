#pragma once

#include <Arduino.h>
#include <Wire.h>

struct RtcDateTime {
    uint16_t year = 2026;
    uint8_t month = 1;
    uint8_t day = 1;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
};

class Pcf85063 {
public:
    bool begin(TwoWire &wire, uint8_t address = 0x51);
    bool available() const { return _available; }
    bool read(RtcDateTime &dateTime);
    bool write(const RtcDateTime &dateTime);

private:
    TwoWire *_wire = nullptr;
    uint8_t _address = 0x51;
    bool _available = false;
};

