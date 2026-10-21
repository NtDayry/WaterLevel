#include "waveshare_ch422g.h"
#include <Wire.h>

#define I2C_FREQ 400000

uint8_t EXIO_state = 0b11111111;
uint8_t DO_state   = 0b00000000;

uint8_t DI0_shadow_state = 0;
uint8_t DI1_shadow_state = 0;

#define CH422G_CONFIG_WRITE_ENABLE 0b00000001
#define CH422G_CONFIG_READ_ENABLE  0b00000010

//-------------------------------------------------------
// INIT
void initCH422G() {
    // set WRITE mode
    uint8_t config = CH422G_CONFIG_WRITE_ENABLE;
    lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_SET, &config, 1, I2C_FREQ);
    delay(10);

    // set EXIO state
    lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_IO, &EXIO_state, 1, I2C_FREQ);
    delay(10);

    // OC OFF
    uint8_t oc_state = 0x0F;
    lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_OC, &oc_state, 1, I2C_FREQ);
    delay(10);
}

//-------------------------------------------------------
// DO WRITE
void DO_Write(uint8_t pin, bool state) {
    if (state)
        DO_state |= (1 << pin);
    else
        DO_state &= ~(1 << pin);

    lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_OC, &DO_state, 1, I2C_FREQ);
}

//-------------------------------------------------------
// DO READ
uint8_t DO_Read(uint8_t pin) {
    return (DO_state >> pin) & 0x01;
}

//-------------------------------------------------------
// READ ALL EXIO (แนะนำให้ใช้ตัวนี้)
uint8_t EX_Read_All() {

    uint8_t inputs = 0;

    // 1. set READ mode
    uint8_t read_config = CH422G_CONFIG_READ_ENABLE;
    if (!lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_SET, &read_config, 1, I2C_FREQ)) {
        Serial.println("EX Read: Config failed");
        return 0;
    }

    // 2. read data
    if (!lgfx::i2c::transactionRead(I2C_PORT, CH422G_ADDR_RD_IO, &inputs, 1, I2C_FREQ)) {
        Serial.println("EX Read: Read failed");
        return 0;
    }

    // 3. restore WRITE mode
    uint8_t write_config = CH422G_CONFIG_WRITE_ENABLE;
    lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_SET, &write_config, 1, I2C_FREQ);

    return inputs;
}

//-------------------------------------------------------
// READ PIN
bool EX_Read(uint8_t pin) {
    uint8_t inputs = EX_Read_All();
    return (inputs >> pin) & 0x01;
}

//-------------------------------------------------------
// EX WRITE
void EX_Write(uint8_t pin, bool state) {

    // set WRITE mode
    uint8_t write_config = CH422G_CONFIG_WRITE_ENABLE;
    if (!lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_SET, &write_config, 1, I2C_FREQ)) {
        Serial.println("EX Write: Config failed");
        return;
    }

    // update shadow
    if (state)
        EXIO_state |= (1 << pin);
    else
        EXIO_state &= ~(1 << pin);

    // write IO
    if (!lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_IO, &EXIO_state, 1, I2C_FREQ)) {
        Serial.println("EX Write: Data failed");
    }
}