#pragma once

#include <Arduino.h>

/**
 * CalibrationManager
 * Encargado de capturar, guardar y recuperar los valores de calibración
 * de sensores analógicos como el MAP y TPS. Se apoya en NVS para
 * almacenamiento persistente entre reinicios.
 */
class CalibrationManager {
public:
  static CalibrationManager& getInstance();

  void begin();

  // Guardar y cargar calibración desde NVS
  bool loadCalibration();
  bool saveCalibration();

  // Calibración interactiva desde consola o BLE
  void runMAPCalibration(class MAPSensor& sensor);
  void runTPScalibration(class TPSSensor& sensor);

  // Getters públicos usados por sensores
  uint16_t getMAPMin() const;
  uint16_t getMAPMax() const;
  uint16_t getTPSMin() const;
  uint16_t getTPSMax() const;

private:
  // Rango calibrado de valores crudos (ADC)
  uint16_t mapMin = 0, mapMax = 4095;
  uint16_t tpsMin = 0, tpsMax = 4095;

  CalibrationManager() = default; // Singleton
};
