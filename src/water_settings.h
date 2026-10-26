#pragma once

#include <Arduino.h>

// Persistent settings used by the water-level controller.
struct WaterSettings {
    float valveOnCm = 84.0f;
    float tankCapacity = 2000.0f;
    float valvePeriodSec = 300.0f;
    uint32_t pumpDelaySec = 0;
    float alertMinCm = 40.0f;
    float alertMaxCm = 180.0f;
    uint32_t alertPeriodSec = 300;
    bool autoMode = false;
    bool autoPump1 = false;
    bool autoPump2 = false;
};

extern WaterSettings waterSettings;

void load_water_settings();
void save_water_settings();
