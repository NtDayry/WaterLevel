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
// Supports both WATER_METRER_SENSOR_PRESENT and WATER_METER_SENSOR_PRESENT
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

// 1. Water Meter Sensor Defaults (Slave ID 1, Reg 0-4)
#define DEFAULT_METER_SLAVE_ID       1
#define DEFAULT_METER_START_REG      0x0000
#define DEFAULT_METER_REG_COUNT      5
#define DEFAULT_METER_POLL_MS        1000

// 2. Water Level Sensor Defaults
#define DEFAULT_LEVEL_SLAVE_ID       2
#define DEFAULT_LEVEL_START_REG      0x0006
#define DEFAULT_LEVEL_REG_COUNT      1
#define DEFAULT_LEVEL_POLL_MS        1000

// 3. RS485 IO Module Defaults
#define DEFAULT_RS485_IO_SLAVE_ID    3
#define DEFAULT_RS485_IO_POLL_MS     500

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
