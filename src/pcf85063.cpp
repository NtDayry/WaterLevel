#include "pcf85063.h"
#include <LovyanGFX.hpp>
#include "waveshare_ch422g.h"

namespace {
uint8_t fromBcd(uint8_t value) { return (uint8_t)((value >> 4) * 10 + (value & 0x0F)); }
uint8_t toBcd(uint8_t value) { return (uint8_t)(((value / 10) << 4) | (value % 10)); }

bool validDateTime(const RtcDateTime &value) {
    return value.year >= 2000 && value.year <= 2099 && value.month >= 1 && value.month <= 12 &&
           value.day >= 1 && value.day <= 31 && value.hour <= 23 && value.minute <= 59 && value.second <= 59;
}
}

bool Pcf85063::begin(TwoWire &wire, uint8_t address) {
    (void)wire;
    return begin(address);
}

bool Pcf85063::begin(uint8_t address) {
    _address = address;
    if (i2c_bus_lock(50)) {
        uint8_t reg = 0x04;
        uint8_t dummy = 0;
        _available = lgfx::i2c::transactionWriteRead(0, _address, &reg, 1, &dummy, 1, 400000).has_value();
        i2c_bus_unlock();
    }
    return _available;
}

bool Pcf85063::read(RtcDateTime &dateTime) {
    if (!_available)
        return false;

    uint8_t buf[7] = {0};
    uint8_t reg = 0x04;
    bool ok = false;
    if (i2c_bus_lock(50)) {
        ok = lgfx::i2c::transactionWriteRead(0, _address, &reg, 1, buf, 7, 400000).has_value();
        i2c_bus_unlock();
    }
    if (!ok)
        return false;

    dateTime.second = fromBcd(buf[0] & 0x7F);
    dateTime.minute = fromBcd(buf[1] & 0x7F);
    dateTime.hour = fromBcd(buf[2] & 0x3F);
    dateTime.day = fromBcd(buf[3] & 0x3F);
    // buf[4] is weekday
    dateTime.month = fromBcd(buf[5] & 0x1F);
    dateTime.year = (uint16_t)(2000 + fromBcd(buf[6]));
    return validDateTime(dateTime);
}

bool Pcf85063::write(const RtcDateTime &dateTime) {
    if (!_available || !validDateTime(dateTime))
        return false;

    uint8_t data[8];
    data[0] = 0x04;
    data[1] = toBcd(dateTime.second);
    data[2] = toBcd(dateTime.minute);
    data[3] = toBcd(dateTime.hour);
    data[4] = toBcd(dateTime.day);
    data[5] = 0; // weekday
    data[6] = toBcd(dateTime.month);
    data[7] = toBcd((uint8_t)(dateTime.year - 2000));

    bool ok = false;
    if (i2c_bus_lock(50)) {
        ok = lgfx::i2c::transactionWrite(0, _address, data, 8, 400000).has_value();
        i2c_bus_unlock();
    }
    return ok;
}
