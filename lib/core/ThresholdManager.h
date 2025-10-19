#pragma once

#include <map>
#include <string>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

struct Thresholds {
    float MAP_WAKEUP_PERCENT;
    float BOOST_TPS_ON;
    float BOOST_MAP_ON;
    float BOOST_TPS_OFF;
    float BOOST_MAP_OFF;
    float BEAM_TPS_ON;
    float BEAM_MAP_ON;
    float BEAM_TPS_OFF;
    float BEAM_MAP_OFF;

};

class ThresholdManager {
public:
    bool begin();
    static ThresholdManager& getInstance() {
        static ThresholdManager inst;
        return inst;
        }

    Thresholds getThresholds() const;
    bool setThreshold(const std::string& key, float value);
    bool save();
    bool reset();

    std::vector<std::string> listKeys() const;

    void debugDump(const char* prefix = "") const;
    void recalculateOffThresholds();

private:
    std::map<std::string, float> thresholds;
    mutable portMUX_TYPE thresholdMux = portMUX_INITIALIZER_UNLOCKED;


    void loadDefaults();
    bool loadFromNVS();
    bool saveToNVS();
};
