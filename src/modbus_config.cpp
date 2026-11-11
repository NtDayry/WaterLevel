#include "modbus_config.h"

// =============================================================================
// Modbus RTU Configuration
// =============================================================================
volatile uint32_t MODBUS_BAUD_RATE = DEFAULT_MODBUS_BAUD_RATE;   // 9600 baud

// 1. Water meter sensor (Slave ID 1, Registers 0-4)
volatile uint8_t WATER_METER_SLAVE_ID = DEFAULT_METER_SLAVE_ID;
volatile uint16_t WATER_METER_START_REG = DEFAULT_METER_START_REG;
volatile uint16_t WATER_METER_REG_COUNT = DEFAULT_METER_REG_COUNT;
volatile uint32_t WATER_METER_POLL_MS = DEFAULT_METER_POLL_MS;

// 2. Water-level sensor
volatile uint8_t MODBUS_SLAVE_ID = DEFAULT_LEVEL_SLAVE_ID;        // Water-level sensor slave ID
volatile uint16_t MODBUS_START_REG = DEFAULT_LEVEL_START_REG;     // Water level register address
volatile uint16_t MODBUS_REG_COUNT = DEFAULT_LEVEL_REG_COUNT;     // Read level only
volatile uint32_t MODBUS_POLL_MS = DEFAULT_LEVEL_POLL_MS;

// 3. RS485 IO module
const uint8_t RS485_IO_SLAVE_ID = DEFAULT_RS485_IO_SLAVE_ID;      // RS485 IO slave ID
const uint32_t RS485_IO_POLL_MS = DEFAULT_RS485_IO_POLL_MS;       // Polling interval 500ms

// 4. Inverter OMRON SYSDRIVE 3G3JZ (Pump 1: ID 4, Pump 2: ID 5)
volatile uint8_t INVERTER_PUMP1_SLAVE_ID = DEFAULT_INVERTER_PUMP1_SLAVE_ID;
volatile uint8_t INVERTER_PUMP2_SLAVE_ID = DEFAULT_INVERTER_PUMP2_SLAVE_ID;

// 5. Pressure Transmitter (Slave ID 6, Reg 0)
volatile uint8_t PRESSURE_SLAVE_ID = DEFAULT_PRESSURE_SLAVE_ID;
volatile uint16_t PRESSURE_START_REG = DEFAULT_PRESSURE_START_REG;
