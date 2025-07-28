#pragma once

#include <map>
#include <string>
#include <vector>

struct Thresholds {
    float MAP_WAKEUP_PERCENT;
    float INJ_TPS_ON;
    float INJ_MAP_ON;
    float INJ_TPS_OFF;
    float INJ_MAP_OFF;
    float VORTEX_TPS_ON;
    float VORTEX_MAP_ON;
    float VORTEX_TPS_OFF;
};

class ThresholdManager {
public:
    bool begin();

    Thresholds getThresholds() const;
    bool setThreshold(const std::string& key, float value);
    bool save();
    bool reset();

    std::vector<std::string> listKeys() const;

private:
    std::map<std::string, float> thresholds;

    void loadDefaults();
    bool loadFromNVS();
    bool saveToNVS();
};
