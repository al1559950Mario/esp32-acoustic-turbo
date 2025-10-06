#include "ThresholdManager.h"
#include <Preferences.h>
#include <Arduino.h>

static constexpr const char* NVS_NAMESPACE = "thresholds";

bool ThresholdManager::begin() {
    Serial.println("[ThresholdManager] begin()");
    loadDefaults();  // Cargar llaves y valores por defecto primero
    Serial.println("[ThresholdManager] Defaults loaded:");
    for (const auto &p : thresholds) Serial.printf("  %s = %.3f\n", p.first.c_str(), p.second);

    if (!loadFromNVS()) {
        Serial.println("[ThresholdManager] loadFromNVS() failed. Will save defaults to NVS.");
        bool saved = saveToNVS();
        Serial.printf("[ThresholdManager] saveToNVS returned %d\n", saved ? 1 : 0);
        return saved;
    }

    Serial.println("[ThresholdManager] Loaded thresholds from NVS:");
    for (const auto &p : thresholds) Serial.printf("  %s = %.3f\n", p.first.c_str(), p.second);
    return true;
}

Thresholds ThresholdManager::getThresholds() const {
    Thresholds t;
    // Añadimos debug mínimo aquí para facilitar trazado cuando se llama.
    t.MAP_WAKEUP_PERCENT = thresholds.at("MAP_WAKEUP_PERCENT");
    t.INJ_MAP_ON        = thresholds.at("INJ_MAP_ON");
    t.INJ_TPS_ON        = thresholds.at("INJ_TPS_ON");
    t.INJ_TPS_OFF       = thresholds.at("INJ_TPS_OFF");
    t.INJ_MAP_OFF       = thresholds.at("INJ_MAP_OFF");
    t.VORTEX_TPS_ON      = thresholds.at("VORTEX_TPS_ON");
    t.VORTEX_MAP_ON      = thresholds.at("VORTEX_MAP_ON");
    t.VORTEX_TPS_OFF     = thresholds.at("VORTEX_TPS_OFF");
    t.VORTEX_MAP_OFF      = thresholds.at("VORTEX_MAP_OFF");

    return t;
}

bool ThresholdManager::setThreshold(const std::string& key, float value) {
    auto it = thresholds.find(key);
    if (it != thresholds.end()) {
        it->second = value;
        Serial.printf("[ThresholdManager] setThreshold OK: %s = %.3f\n", key.c_str(), value);
        return true;
    }

    Serial.printf("[ThresholdManager] setThreshold FAILED: key not found: '%s'\n", key.c_str());
    for (const auto &p : thresholds) Serial.printf("  '%s'\n", p.first.c_str());
    return false;
}

bool ThresholdManager::save() {
    Serial.println("[ThresholdManager] save() called");
    return saveToNVS();
}

bool ThresholdManager::reset() {
    Serial.println("[ThresholdManager] reset() called");
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
    thresholds["MAP_WAKEUP_PERCENT"] = 10.0f;

    // Umbrales para activar la inyección acústica
    thresholds["INJ_TPS_ON"]  = 50.0f;   // % TPS mínimo para iniciar  ← añadido
    thresholds["INJ_MAP_ON"]         = 50.0f;   // % MAP mínimo para iniciar

    // Umbrales para detener la inyección acústica
    thresholds["INJ_TPS_OFF"]        = 40.0f;    // % TPS para apagar
    thresholds["INJ_MAP_OFF"]        = 40.0f;   // % MAP para apagar

    // Umbrales para activar el vortex
    thresholds["VORTEX_TPS_ON"]       = 95.0f;   // % TPS para activar
    thresholds["VORTEX_MAP_ON"]       = 95.0f;   // % MAP para activar (presión alta)

    // Umbral para apagar el vorte
    thresholds["VORTEX_TPS_OFF"]      = 80.0f;   // % TPS para apagar vortex
    thresholds["VORTEX_MAP_OFF"]      = 80.0f;   // % TPS para apagar vortex
}

bool ThresholdManager::loadFromNVS() {
    Serial.println("[ThresholdManager] loadFromNVS() starting");
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        Serial.println("[ThresholdManager] ERROR: No se pudo abrir NVS para lectura");
        return false;
    }

    // Si alguna clave falta, no cargamos (datos incompletos)
    for (const auto& key : listKeys()) {
        bool exists = prefs.isKey(key.c_str());
        Serial.printf("[ThresholdManager] key check: %s -> %d\n", key.c_str(), exists ? 1 : 0);
        if (!exists) {
            prefs.end();
            Serial.printf("[ThresholdManager] loadFromNVS FAIL: missing key '%s'\n", key.c_str());
            return false;
        }
    }

    for (auto& pair : thresholds) {
        float v = prefs.getFloat(pair.first.c_str(), -1.0f);
        Serial.printf("[ThresholdManager] read %s -> %.3f\n", pair.first.c_str(), v);
        if (v < 0.0f) {
            prefs.end();
            Serial.printf("[ThresholdManager] loadFromNVS FAIL: invalid value for '%s' (%.3f)\n", pair.first.c_str(), v);
            return false;
        }
        pair.second = v;
    }

    prefs.end();
    Serial.println("[ThresholdManager] loadFromNVS OK");
    return true;
}

bool ThresholdManager::saveToNVS() {
    Serial.println("[ThresholdManager] saveToNVS() starting");
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println("[ThresholdManager] ERROR: No se pudo abrir NVS para escritura");
        return false;
    }

    for (const auto& pair : thresholds) {
        prefs.putFloat(pair.first.c_str(), pair.second);
        Serial.printf("[ThresholdManager] wrote %s = %.3f\n", pair.first.c_str(), pair.second);
    }

    prefs.end();
    Serial.println("[ThresholdManager] saveToNVS OK");
    return true;
}

void ThresholdManager::debugDump(const char* prefix) const {
    Serial.printf("== Thresholds dump: %s ==\n", prefix ? prefix : "");
    for (const auto &p : thresholds) {
        Serial.printf("  %s = %.3f\n", p.first.c_str(), p.second);
    }
}
