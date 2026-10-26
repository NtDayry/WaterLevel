#include "pcf85063.h"

namespace {
uint8_t fromBcd(uint8_t value) { return (uint8_t)((value >> 4) * 10 + (value & 0x0F)); }
uint8_t toBcd(uint8_t value) { return (uint8_t)(((value / 10) << 4) | (value % 10)); }

bool validDateTime(const RtcDateTime &value) {
    return value.year >= 2000 && value.year <= 2099 && value.month >= 1 && value.month <= 12 &&
           value.day >= 1 && value.day <= 31 && value.hour <= 23 && value.minute <= 59 && value.second <= 59;
}
}

bool Pcf85063::begin(TwoWire &wire, uint8_t address) {
    _wire = &wire;
    _address = address;
    _wire->beginTransmission(_address);
    _available = _wire->endTransmission() == 0;
    return _available;
}

bool Pcf85063::read(RtcDateTime &dateTime) {
    if (!_wire || !_available)
        return false;

    _wire->beginTransmission(_address);
    _wire->write(0x04); // seconds register; 0x03 is the RAM byte
    if (_wire->endTransmission(false) != 0 || _wire->requestFrom(_address, (uint8_t)7) != 7)
        return false;

    const uint8_t seconds = (uint8_t)_wire->read();
    const uint8_t minutes = (uint8_t)_wire->read();
    const uint8_t hours = (uint8_t)_wire->read();
    const uint8_t days = (uint8_t)_wire->read();
    (void)_wire->read(); // weekday
    const uint8_t months = (uint8_t)_wire->read();
    const uint8_t years = (uint8_t)_wire->read();

    dateTime.second = fromBcd(seconds & 0x7F);
    dateTime.minute = fromBcd(minutes & 0x7F);
    dateTime.hour = fromBcd(hours & 0x3F);
    dateTime.day = fromBcd(days & 0x3F);
    dateTime.month = fromBcd(months & 0x1F);
    dateTime.year = (uint16_t)(2000 + fromBcd(years));
    return validDateTime(dateTime);
}

bool Pcf85063::write(const RtcDateTime &dateTime) {
    if (!_wire || !_available || !validDateTime(dateTime))
        return false;

    _wire->beginTransmission(_address);
    _wire->write(0x04);
    _wire->write(toBcd(dateTime.second));
    _wire->write(toBcd(dateTime.minute));
    _wire->write(toBcd(dateTime.hour));
    _wire->write(toBcd(dateTime.day));
    _wire->write((uint8_t)0); // weekday; PCF85063 recalculates no weekday itself
    _wire->write(toBcd(dateTime.month));
    _wire->write(toBcd((uint8_t)(dateTime.year - 2000)));
    return _wire->endTransmission() == 0;
}

