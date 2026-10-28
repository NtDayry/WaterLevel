#include "water_settings.h"

#include <Preferences.h>

namespace {
Preferences store;
}

WaterSettings waterSettings;

void load_water_settings() {
    store.begin("watercfg", false);
    waterSettings.valveOnCm = store.getFloat("voncm", waterSettings.valveOnCm);
    waterSettings.tankCapacity = store.getFloat("tankcap", waterSettings.tankCapacity);
    waterSettings.valvePeriodSec = store.getFloat("vperiod", waterSettings.valvePeriodSec);
    waterSettings.pumpDelaySec = store.getUInt("pumpdelay", waterSettings.pumpDelaySec);
    waterSettings.alertMinCm = store.getFloat("amincm", waterSettings.alertMinCm);
    waterSettings.alertMaxCm = store.getFloat("amaxcm", waterSettings.alertMaxCm);
    waterSettings.alertPeriodSec = store.getUInt("aperiod", waterSettings.alertPeriodSec);
    waterSettings.autoMode = store.getBool("automode", waterSettings.autoMode);
    waterSettings.autoPump1 = store.getBool("apump1", waterSettings.autoPump1);
    waterSettings.autoPump2 = store.getBool("apump2", waterSettings.autoPump2);
    waterSettings.manualPump1 = store.getBool("mpump1", waterSettings.manualPump1);
    waterSettings.manualPump2 = store.getBool("mpump2", waterSettings.manualPump2);
    waterSettings.manualValve = store.getBool("mvalve", waterSettings.manualValve);
    store.end();

    waterSettings.tankCapacity = constrain(waterSettings.tankCapacity, 0.1f, 1000000.0f);
    const float maxLevelCm = max(1.0f, waterSettings.tankCapacity);
    waterSettings.valveOnCm = constrain(waterSettings.valveOnCm, 0.0f, maxLevelCm);
    waterSettings.alertMinCm = constrain(waterSettings.alertMinCm, 0.0f, maxLevelCm);
    waterSettings.alertMaxCm = constrain(waterSettings.alertMaxCm, 0.0f, maxLevelCm);
    waterSettings.valvePeriodSec = constrain(waterSettings.valvePeriodSec, 0.1f, 86400.0f);
    waterSettings.pumpDelaySec = constrain(waterSettings.pumpDelaySec, (uint32_t)0, (uint32_t)86400);
    waterSettings.alertPeriodSec = constrain(waterSettings.alertPeriodSec, (uint32_t)1, (uint32_t)86400);

    if (waterSettings.alertMinCm >= waterSettings.alertMaxCm)
        waterSettings.alertMaxCm = min(maxLevelCm, waterSettings.alertMinCm + 1.0f);
}

void save_water_settings() {
    store.begin("watercfg", false);
    store.putFloat("voncm", waterSettings.valveOnCm);
    store.putFloat("tankcap", waterSettings.tankCapacity);
    store.putFloat("vperiod", waterSettings.valvePeriodSec);
    store.putUInt("pumpdelay", waterSettings.pumpDelaySec);
    store.putFloat("amincm", waterSettings.alertMinCm);
    store.putFloat("amaxcm", waterSettings.alertMaxCm);
    store.putUInt("aperiod", waterSettings.alertPeriodSec);
    store.putBool("automode", waterSettings.autoMode);
    store.putBool("apump1", waterSettings.autoPump1);
    store.putBool("apump2", waterSettings.autoPump2);
    store.putBool("mpump1", waterSettings.manualPump1);
    store.putBool("mpump2", waterSettings.manualPump2);
    store.putBool("mvalve", waterSettings.manualValve);
    store.end();
}
