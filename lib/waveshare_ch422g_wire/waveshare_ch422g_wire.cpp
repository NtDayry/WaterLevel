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
    Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    Wire.setClock(I2C_FREQ);

    // Enable WRITE mode ก่อน
    uint8_t config = CH422G_CONFIG_WRITE_ENABLE;
    Wire.beginTransmission(CH422G_ADDR_WR_SET);
    Wire.write(config);
    Wire.endTransmission();
    delay(10);

    // ตั้ง EXIO (1 = input / high)
    Wire.beginTransmission(CH422G_ADDR_WR_IO);
    Wire.write(EXIO_state);
    Wire.endTransmission();
    delay(10);

    // OC OFF
    uint8_t oc_state = 0x0F;
    Wire.beginTransmission(CH422G_ADDR_WR_OC);
    Wire.write(oc_state);
    Wire.endTransmission();
    delay(10);
}

//-------------------------------------------------------
// DO WRITE
void DO_Write(uint8_t pin, bool state) {
    if (state)
        DO_state |= (1 << pin);
    else
        DO_state &= ~(1 << pin);

    Wire.beginTransmission(CH422G_ADDR_WR_OC);
    Wire.write(DO_state);
    if (Wire.endTransmission() != 0) {
        Serial.println("DO Write failed");
    }
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
    Wire.beginTransmission(CH422G_ADDR_WR_SET);
    Wire.write(read_config);
    if (Wire.endTransmission() != 0) {
        Serial.println("EX Read: Config set failed");
        return 0;
    }

    // 2. read data
    Wire.requestFrom(CH422G_ADDR_RD_IO, 1);
    if (Wire.available()) {
        inputs = Wire.read();
    } else {
        Serial.println("EX Read: No data");
    }

    // 3. restore WRITE mode
    uint8_t write_config = CH422G_CONFIG_WRITE_ENABLE;
    Wire.beginTransmission(CH422G_ADDR_WR_SET);
    Wire.write(write_config);
    Wire.endTransmission();

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
    Wire.beginTransmission(CH422G_ADDR_WR_SET);
    Wire.write(write_config);
    if (Wire.endTransmission() != 0) {
        Serial.println("EX Write: Config failed");
        return;
    }

    // update shadow
    if (state)
        EXIO_state |= (1 << pin);
    else
        EXIO_state &= ~(1 << pin);

    // write IO
    Wire.beginTransmission(CH422G_ADDR_WR_IO);
    Wire.write(EXIO_state);
    if (Wire.endTransmission() != 0) {
        Serial.println("EX Write: Data failed");
    }
}