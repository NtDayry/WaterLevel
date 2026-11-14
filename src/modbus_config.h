#pragma once

#include <Arduino.h>

// =============================================================================
// Feature Switches
// =============================================================================
// Set to 1 to simulate water level without physical sensor
#ifndef WATER_LEVEL_SIMULATION
#define WATER_LEVEL_SIMULATION 0
#endif

// Set to 0 if water level sensor is not physically connected
#ifndef WATER_LEVEL_SENSOR_PRESENT
#define WATER_LEVEL_SENSOR_PRESENT 1
#endif

// Water meter sensor presence switch (default 0: disabled until connected)
#ifndef WATER_METRER_SENSOR_PRESENT
#define WATER_METRER_SENSOR_PRESENT 0
#endif

#ifndef WATER_METER_SENSOR_PRESENT
#define WATER_METER_SENSOR_PRESENT WATER_METRER_SENSOR_PRESENT
#endif

// =============================================================================
// Modbus RTU Initial Default Values
// =============================================================================
#define DEFAULT_MODBUS_BAUD_RATE     9600
#define DEFAULT_MODBUS_CONFIG        SERIAL_8E1   // 9600 baud, 8 data bits, Even parity, 1 stop bit

// 1. Water Meter Sensor Defaults (Slave ID 1, Reg 0-4)
#define DEFAULT_METER_SLAVE_ID       1
#define DEFAULT_METER_START_REG      0x0000
#define DEFAULT_METER_REG_COUNT      5
#define DEFAULT_METER_POLL_MS        1000

// 2. RS485 IO Module Defaults (Slave ID 2)
#define DEFAULT_RS485_IO_SLAVE_ID    2
#define DEFAULT_RS485_IO_POLL_MS     500

// 3. Water Level Sensor Defaults (Slave ID 3)
#define DEFAULT_LEVEL_SLAVE_ID       3
#define DEFAULT_LEVEL_START_REG      0x0006
#define DEFAULT_LEVEL_REG_COUNT      1
#define DEFAULT_LEVEL_POLL_MS        1000

// 4. Inverter OMRON SYSDRIVE 3G3JZ (Pump 1: ID 4, Pump 2: ID 5)
#define DEFAULT_INVERTER_PUMP1_SLAVE_ID 4
#define DEFAULT_INVERTER_PUMP2_SLAVE_ID 5
#define DEFAULT_INVERTER_POLL_MS        250

// 5. Modbus Pressure Transmitter (Slave ID 6, Reg 0, 0-6.00 bar)
#define DEFAULT_PRESSURE_SLAVE_ID    6
#define DEFAULT_PRESSURE_START_REG   0x0000
#define DEFAULT_PRESSURE_POLL_MS     200

// =============================================================================
// Modbus RTU Runtime Configuration Variables
// =============================================================================
extern volatile uint32_t MODBUS_BAUD_RATE;

// Water-level sensor
extern volatile uint8_t MODBUS_SLAVE_ID;
extern volatile uint16_t MODBUS_START_REG;
extern volatile uint16_t MODBUS_REG_COUNT;
extern volatile uint32_t MODBUS_POLL_MS;

// RS485 IO module
extern const uint8_t RS485_IO_SLAVE_ID;
extern const uint32_t RS485_IO_POLL_MS;

// Water meter sensor
extern volatile uint8_t WATER_METER_SLAVE_ID;
extern volatile uint16_t WATER_METER_START_REG;
extern volatile uint16_t WATER_METER_REG_COUNT;
extern volatile uint32_t WATER_METER_POLL_MS;

// Inverter OMRON SYSDRIVE 3G3JZ
extern volatile uint8_t INVERTER_PUMP1_SLAVE_ID;
extern volatile uint8_t INVERTER_PUMP2_SLAVE_ID;

// Pressure sensor
extern volatile uint8_t PRESSURE_SLAVE_ID;
extern volatile uint16_t PRESSURE_START_REG;
