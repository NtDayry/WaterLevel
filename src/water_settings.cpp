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
    waterSettings.alertMinCm = store.getFloat("amincm", waterSettings.alertMinCm);
    waterSettings.alertMaxCm = store.getFloat("amaxcm", waterSettings.alertMaxCm);
    waterSettings.alertPeriodSec = store.getUInt("aperiod", waterSettings.alertPeriodSec);
    waterSettings.lowerLimitCm = store.getFloat("lowlmt", waterSettings.lowerLimitCm);
    waterSettings.autoMode = store.getBool("automode", waterSettings.autoMode);
    waterSettings.sensorTypeSwitch = store.getBool("snrtype", waterSettings.sensorTypeSwitch);
    waterSettings.autoPump1 = store.getBool("autop1", waterSettings.autoPump1);
    waterSettings.autoPump2 = store.getBool("autop2", waterSettings.autoPump2);
    waterSettings.manualPump1 = store.getBool("manp1", waterSettings.manualPump1);
    waterSettings.manualPump2 = store.getBool("manp2", waterSettings.manualPump2);
    waterSettings.manualValve = store.getBool("manval", waterSettings.manualValve);
    waterSettings.manualSpeedPump1 = store.getFloat("manspd1", waterSettings.manualSpeedPump1);
    waterSettings.manualSpeedPump2 = store.getFloat("manspd2", waterSettings.manualSpeedPump2);

    // Inverter PID settings
    waterSettings.pidKp = store.getFloat("pidkp", waterSettings.pidKp);
    waterSettings.pidKi = store.getFloat("pidki", waterSettings.pidKi);
    waterSettings.accDec = store.getFloat("accdec", waterSettings.accDec);
    waterSettings.setPressure = store.getFloat("setpress", waterSettings.setPressure);
    waterSettings.sleepFreq = store.getFloat("sleepfreq", waterSettings.sleepFreq);
    waterSettings.sleepDelaySec = store.getUInt("sleepdel", waterSettings.sleepDelaySec);
    store.end();

    waterSettings.tankCapacity = constrain(waterSettings.tankCapacity, 0.1f, 1000000.0f);
    const float maxLevelCm = max(1.0f, waterSettings.tankCapacity);
    waterSettings.valveOnCm = constrain(waterSettings.valveOnCm, 0.0f, maxLevelCm);
    waterSettings.alertMinCm = constrain(waterSettings.alertMinCm, 0.0f, maxLevelCm);
    waterSettings.alertMaxCm = constrain(waterSettings.alertMaxCm, 0.0f, maxLevelCm);
    waterSettings.lowerLimitCm = constrain(waterSettings.lowerLimitCm, 0.0f, maxLevelCm);
    waterSettings.valvePeriodSec = constrain(waterSettings.valvePeriodSec, 0.1f, 86400.0f);
    waterSettings.alertPeriodSec = constrain(waterSettings.alertPeriodSec, (uint32_t)1, (uint32_t)86400);

    waterSettings.pidKp = constrain(waterSettings.pidKp, 0.0f, 100.0f);
    waterSettings.pidKi = constrain(waterSettings.pidKi, 0.0f, 100.0f);
    waterSettings.pidKd = 0.0f;
    waterSettings.accDec = constrain(waterSettings.accDec, 0.1f, 3600.0f);
    waterSettings.setPressure = constrain(waterSettings.setPressure, 0.0f, 6.0f);
    waterSettings.sleepFreq = constrain(waterSettings.sleepFreq, 0.0f, 50.0f);
    waterSettings.sleepDelaySec = constrain(waterSettings.sleepDelaySec, (uint32_t)0, (uint32_t)3600);
    waterSettings.manualSpeedPump1 = constrain(waterSettings.manualSpeedPump1, waterSettings.sleepFreq, 50.0f);
    waterSettings.manualSpeedPump2 = constrain(waterSettings.manualSpeedPump2, waterSettings.sleepFreq, 50.0f);

    if (waterSettings.alertMinCm >= waterSettings.alertMaxCm)
        waterSettings.alertMaxCm = min(maxLevelCm, waterSettings.alertMinCm + 1.0f);
}

void save_water_settings() {
    store.begin("watercfg", false);
    store.putFloat("voncm", waterSettings.valveOnCm);
    store.putFloat("tankcap", waterSettings.tankCapacity);
    store.putFloat("vperiod", waterSettings.valvePeriodSec);
    store.putFloat("amincm", waterSettings.alertMinCm);
    store.putFloat("amaxcm", waterSettings.alertMaxCm);
    store.putUInt("aperiod", waterSettings.alertPeriodSec);
    store.putFloat("lowlmt", waterSettings.lowerLimitCm);
    store.putBool("automode", waterSettings.autoMode);
    store.putBool("snrtype", waterSettings.sensorTypeSwitch);
    store.putBool("autop1", waterSettings.autoPump1);
    store.putBool("autop2", waterSettings.autoPump2);
    store.putBool("manp1", waterSettings.manualPump1);
    store.putBool("manp2", waterSettings.manualPump2);
    store.putBool("manval", waterSettings.manualValve);
    store.putFloat("manspd1", waterSettings.manualSpeedPump1);
    store.putFloat("manspd2", waterSettings.manualSpeedPump2);

    store.putFloat("pidkp", waterSettings.pidKp);
    store.putFloat("pidki", waterSettings.pidKi);
    store.putFloat("accdec", waterSettings.accDec);
    store.putFloat("setpress", waterSettings.setPressure);
    store.putFloat("sleepfreq", waterSettings.sleepFreq);
    store.putUInt("sleepdel", waterSettings.sleepDelaySec);
    store.end();
}
