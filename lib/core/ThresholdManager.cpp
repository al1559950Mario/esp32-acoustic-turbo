#include "ThresholdManager.h"
#include <Preferences.h>
#include <Arduino.h>

static constexpr const char* NVS_NAMESPACE = "thresholds";

bool ThresholdManager::begin() {
    if (!loadFromNVS()) {
        loadDefaults();
        return saveToNVS();
    }
    return true;
}

Thresholds ThresholdManager::getThresholds() const {
    Thresholds t;
    t.MAP_WAKEUP_PERCENT = thresholds.at("MAP_WAKEUP_PERCENT");
    t.INJ_TPS_ON        = thresholds.at("INJ_TPS_ON");
    t.INJ_MAP_ON        = thresholds.at("INJ_MAP_ON");
    t.INJ_TPS_OFF       = thresholds.at("INJ_TPS_OFF");
    t.INJ_MAP_OFF       = thresholds.at("INJ_MAP_OFF");
    t.VORTEX_TPS_ON      = thresholds.at("VORTEX_TPS_ON");
    t.VORTEX_MAP_ON      = thresholds.at("VORTEX_MAP_ON");
    t.VORTEX_TPS_OFF     = thresholds.at("VORTEX_TPS_OFF");
    return t;
}

bool ThresholdManager::setThreshold(const std::string& key, float value) {
    auto it = thresholds.find(key);
    if (it != thresholds.end()) {
        it->second = value;
        return true;
    }
    return false;
}

bool ThresholdManager::save() {
    return saveToNVS();
}

bool ThresholdManager::reset() {
    loadDefaults();
    return saveToNVS();
}

std::vector<std::string> ThresholdManager::listKeys() const {
    std::vector<std::string> keys;
    for (const auto& pair : thresholds) {
        keys.push_back(pair.first);
    }
    return keys;
}

void ThresholdManager::loadDefaults() {
    thresholds.clear();

    // Umbral mínimo de presión (MAP) para pasar de OFF a IDLE
    thresholds["MAP_WAKEUP_PERCENT"] = 5.0f;

    // Umbrales para activar la inyección acústica
    thresholds["INJ_TPS_ON"]         = 10.0f;   // % TPS mínimo para iniciar
    thresholds["INJ_MAP_ON"]         = 40.0f;   // % MAP mínimo para iniciar

    // Umbrales para detener la inyección acústica
    thresholds["INJ_TPS_OFF"]        = 8.0f;    // % TPS para apagar
    thresholds["INJ_MAP_OFF"]        = 30.0f;   // % MAP para apagar

    // Umbrales para activar el vortex
    thresholds["VORTEX_TPS_ON"]       = 45.0f;   // % TPS para activar
    thresholds["VORTEX_MAP_ON"]       = 75.0f;   // % MAP para activar (presión alta)

    // Umbral para apagar el vortex
    thresholds["VORTEX_TPS_OFF"]      = 30.0f;   // % TPS para apagar vortex
}


bool ThresholdManager::loadFromNVS() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        Serial.println("ERROR: No se pudo abrir NVS para lectura");
        return false;
    }

    // Si alguna clave falta, no cargamos (datos incompletos)
    for (const auto& key : listKeys()) {
        if (!prefs.isKey(key.c_str())) {
            prefs.end();
            return false;
        }
    }

    for (auto& pair : thresholds) {
        pair.second = prefs.getFloat(pair.first.c_str(), -1.0f);
        if (pair.second < 0.0f) {
            prefs.end();
            return false;
        }
    }

    prefs.end();
    return true;
}

bool ThresholdManager::saveToNVS() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println("ERROR: No se pudo abrir NVS para escritura");
        return false;
    }

    for (const auto& pair : thresholds) {
        prefs.putFloat(pair.first.c_str(), pair.second);
    }

    prefs.end();
    return true;
}
