#include "waveshare_ch422g.h"

uint8_t EXIO_state = 0b11111111;
uint8_t DO_state   = 0b00000000;

uint8_t DI0_shadow_state = 0;
uint8_t DI1_shadow_state = 0;

static SemaphoreHandle_t i2cMux = NULL;

bool i2c_bus_lock(int timeout_ms) {
    if (i2cMux == NULL) {
        i2cMux = xSemaphoreCreateRecursiveMutex();
    }
    if (i2cMux == NULL) return false;
    TickType_t t = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(i2cMux, t) == pdTRUE;
}

void i2c_bus_unlock() {
    if (i2cMux != NULL) {
        xSemaphoreGiveRecursive(i2cMux);
    }
}

//-------------------------------------------------------
// INIT
void initCH422G() {
    if (i2cMux == NULL) {
        i2cMux = xSemaphoreCreateRecursiveMutex();
    }

    if (i2c_bus_lock(200)) {
        // Set WRITE mode (IO output enable = 1)
        uint8_t config = CH422G_CONFIG_WRITE_ENABLE;
        lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_SET, &config, 1, I2C_FREQ);
        delay(10);

        // Set EXIO state (Backlight ON, resets released, DIN0/DIN1 pull-ups enabled)
        EXIO_state = 0b11111111;
        lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_IO, &EXIO_state, 1, I2C_FREQ);
        delay(10);

        // OC OFF
        uint8_t oc_state = 0x0F;
        lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_OC, &oc_state, 1, I2C_FREQ);
        delay(10);

        i2c_bus_unlock();
    }
}

//-------------------------------------------------------
// DO WRITE
void DO_Write(uint8_t pin, bool state) {
    if (state)
        DO_state |= (1 << pin);
    else
        DO_state &= ~(1 << pin);

    if (i2c_bus_lock(50)) {
        lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_OC, &DO_state, 1, I2C_FREQ);
        i2c_bus_unlock();
    }
}

//-------------------------------------------------------
// DO READ
uint8_t DO_Read(uint8_t pin) {
    return (DO_state >> pin) & 0x01;
}

//-------------------------------------------------------
// READ ALL EXIO
// Reads physical IO states directly from register 0x26.
// NEVER toggle register 0x24 to READ mode, because that disables IO_OE (bit 0)
// and tri-states all pins, which turns off LCD backlight and resets ST7701!
uint8_t EX_Read_All() {
    uint8_t inputs = 0;

    if (i2c_bus_lock(20)) {
        if (!lgfx::i2c::transactionRead(I2C_PORT, CH422G_ADDR_RD_IO, &inputs, 1, I2C_FREQ)) {
            // If read failed, keep last known state rather than zeroing
            inputs = EXIO_state;
        }
        i2c_bus_unlock();
    } else {
        inputs = EXIO_state;
    }

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
    // update shadow
    if (state)
        EXIO_state |= (1 << pin);
    else
        EXIO_state &= ~(1 << pin);

    // Safety guard: TP_RST (pin 1) and LCD_RST (pin 3) must always remain HIGH (not in reset)
    EXIO_state |= (1 << TP_RST) | (1 << LCD_RST);

    if (i2c_bus_lock(50)) {
        // Ensure WRITE mode is active
        uint8_t write_config = CH422G_CONFIG_WRITE_ENABLE;
        lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_SET, &write_config, 1, I2C_FREQ);

        // Write IO states (bit 2 = LCD_BL)
        lgfx::i2c::transactionWrite(I2C_PORT, CH422G_ADDR_WR_IO, &EXIO_state, 1, I2C_FREQ);
        i2c_bus_unlock();
    }
}
