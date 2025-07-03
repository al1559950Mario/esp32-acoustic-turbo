#include "DebugManager.h"
#include <string.h>  // Para strlen()

// Cantidad total de señales simulables
constexpr int OVERRIDE_COUNT = 4;

int DebugManager::index(DebugTarget target) const {
  return static_cast<int>(target);
}

void DebugManager::enableOverride(DebugTarget target, float value) {
  int i = index(target);
  if (i >= 0 && i < OVERRIDE_COUNT) {
    overrides[i].activo = true;
    overrides[i].valor  = value;
  }
}

void DebugManager::enableOverride(DebugTarget target) {
  enableOverride(target, 0.0f);  // Valor por defecto si no se indica
}

void DebugManager::disableOverride(DebugTarget target) {
  int i = index(target);
  if (i >= 0 && i < OVERRIDE_COUNT) {
    overrides[i].activo = false;
    overrides[i].valor  = 0.0f;
  }
}

void DebugManager::disableAll() {
  for (int i = 0; i < OVERRIDE_COUNT; ++i) {
    overrides[i].activo = false;
    overrides[i].valor  = 0.0f;
  }
}

bool DebugManager::hasOverride(DebugTarget target) const {
  int i = index(target);
  return (i >= 0 && i < OVERRIDE_COUNT) ? overrides[i].activo : false;
}

float DebugManager::getValue(DebugTarget target) const {
  int i = index(target);
  return (i >= 0 && i < OVERRIDE_COUNT) ? overrides[i].valor : 0.0f;
}

bool DebugManager::turboOverride() const {
  return hasOverride(DebugTarget::TURBO);
}

bool DebugManager::acousticOverride() const {
  return hasOverride(DebugTarget::INYECTOR);
}

float DebugManager::getLevel() const {
  return getValue(DebugTarget::INYECTOR);
}

void DebugManager::updateFromSerial(Stream& serial) {
  if (!serial.available()) return;

  String line = serial.readStringUntil('\n');
  line.trim();

  setIfPresent(line, "tps:", DebugTarget::TPS);
  setIfPresent(line, "map:", DebugTarget::MAP);
  setIfPresent(line, "turbo:", DebugTarget::TURBO);
  setIfPresent(line, "iny:", DebugTarget::INYECTOR);
}

void DebugManager::setIfPresent(const String& line,
                                const char* prefix,
                                DebugTarget target) {
  int start = line.indexOf(prefix);
  if (start < 0) return;

  int valStart = start + strlen(prefix);
  int valEnd   = line.indexOf(',', valStart);
  if (valEnd < 0) valEnd = line.length();

  String valStr = line.substring(valStart, valEnd);
  float valor   = valStr.toFloat();

  if (target == DebugTarget::TURBO || target == DebugTarget::INYECTOR) {
    if (valor <= 0.0f)
      disableOverride(target);
    else
      enableOverride(target, valor);
  } else {
    enableOverride(target, valor);
  }
}
