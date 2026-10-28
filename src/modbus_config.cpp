#include "modbus_config.h"

// =============================================================================
// Modbus RTU Configuration
// =============================================================================
volatile uint32_t MODBUS_BAUD_RATE = DEFAULT_MODBUS_BAUD_RATE;   // Standard Baud Rate (เช่น 9600, 19200, 115200)

// 1. Water-level sensor
volatile uint8_t MODBUS_SLAVE_ID = DEFAULT_LEVEL_SLAVE_ID;        // Water-level sensor slave ID
volatile uint16_t MODBUS_START_REG = DEFAULT_LEVEL_START_REG;     // Water level register address
volatile uint16_t MODBUS_REG_COUNT = DEFAULT_LEVEL_REG_COUNT;     // Read level only
volatile uint32_t MODBUS_POLL_MS = DEFAULT_LEVEL_POLL_MS;         // อ่านระดับน้ำทุก 1 วินาที

// 2. RS485 IO module
const uint8_t RS485_IO_SLAVE_ID = DEFAULT_RS485_IO_SLAVE_ID;      // RS485 IO slave ID
const uint32_t RS485_IO_POLL_MS = DEFAULT_RS485_IO_POLL_MS;       // Polling interval 500ms

// 3. Water meter sensor (Slave ID 1, Registers 0-4)
volatile uint8_t WATER_METER_SLAVE_ID = DEFAULT_METER_SLAVE_ID;       // Water meter slave ID
volatile uint16_t WATER_METER_START_REG = DEFAULT_METER_START_REG;   // Water meter start register address
volatile uint16_t WATER_METER_REG_COUNT = DEFAULT_METER_REG_COUNT;   // Read count (registers 0-4 = 5 registers)
volatile uint32_t WATER_METER_POLL_MS = DEFAULT_METER_POLL_MS;       // Polling interval 1000ms
