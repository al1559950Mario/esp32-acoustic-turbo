#pragma once
#include "MAPSensor.h"
#include "TPSSensor.h"
#include <Preferences.h>

enum class CalibStep {
  TPS_MIN,
  TPS_MAX,
  MAP_MIN,
  MAP_MAX
};

class CalibrationManager {
public:
  static CalibrationManager& getInstance();

  void begin();

  // Nueva función para activar modo debug hardcodeado
  void enableDebugMode(bool enable) { debugMode = enable; }

  bool loadCalibration();
  void loadDebugCalibration();

  void clearCalibration();

  bool saveCalibration();         // guarda mapMin, mapMax, tpsMin, tpsMax
  void runMAPCalibration(MAPSensor& sensor, bool simulacionActiva);
  void runTPSCalibration(TPSSensor& sensor, bool simulacionActiva);

  uint16_t getMAPMin() const;
  uint16_t getMAPMax() const;
  uint16_t getTPSMin() const;
  uint16_t getTPSMax() const;

private:
  CalibrationManager() = default;
  Preferences prefs;

  uint16_t mapMin = 0, mapMax = 0;
  uint16_t tpsMin = 0, tpsMax = 0;

  bool debugMode = false;  // Modo debug hardcodeado

  void saveStep(CalibStep step, uint16_t value);
};
