#pragma once

#include <Arduino.h>

// Persistent settings used by the water-level & pump PID controller.
struct WaterSettings {
    float valveOnCm = 84.0f;
    float tankCapacity = 2000.0f;
    float valvePeriodSec = 300.0f;
    float alertMinCm = 40.0f;
    float alertMaxCm = 180.0f;
    uint32_t alertPeriodSec = 300;
    float lowerLimitCm = 30.0f;

    // Pump & Valve Control Modes
    bool autoMode = false;
    bool sensorTypeSwitch = false;  // false = RS485 sensor (ID 6), true = pressure switch (pressure_switch_io_pin)
    bool autoPump1 = true;          // Auto mode: enable pump 1
    bool autoPump2 = true;          // Auto mode: enable pump 2
    bool manualPump1 = false;       // Manual mode: run pump 1
    bool manualPump2 = false;       // Manual mode: run pump 2
    bool manualValve = false;       // Manual mode: open refill valve
    float manualSpeedPump1 = 35.0f; // Manual mode: speed pump 1 in Hz (min: sleepFreq, max: 50.0)
    float manualSpeedPump2 = 35.0f; // Manual mode: speed pump 2 in Hz (min: sleepFreq, max: 50.0)

    // Inverter PID Parameters (OMRON SYSDRIVE 3G3JZ)
    float pidKp = 2.0f;
    float pidKi = 0.5f;
    float pidKd = 0.0f;          // Fixed constant 0
    float accDec = 10.0f;        // Acc & Dec time in seconds
    float setPressure = 2.50f;   // Setpoint pressure in bar
    float sleepFreq = 25.0f;     // Sleep frequency in Hz
    uint32_t sleepDelaySec = 10; // Sleep delay in seconds
};

extern WaterSettings waterSettings;

void load_water_settings();
void save_water_settings();
