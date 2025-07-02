#include "CalibrationManager.h"
#include "MAPSensor.h"
#include "TPSSensor.h"
#include <Preferences.h>

static Preferences prefs; // Objeto persistente de NVS

/**
 * Devuelve la instancia única (singleton)
 */
CalibrationManager& CalibrationManager::getInstance() {
  static CalibrationManager instance;
  return instance;
}

/**
 * Inicializa la partición NVS donde se guardan datos de calibración.
 */
void CalibrationManager::begin() {
  prefs.begin("calib", false);  // false = lectura/escritura
}

/**
 * Carga los datos previamente almacenados en NVS.
 * Comprueba integridad básica de los valores.
 */
bool CalibrationManager::loadCalibration() {
  mapMin = prefs.getUShort("map_min", 0);
  mapMax = prefs.getUShort("map_max", 4095);
  tpsMin = prefs.getUShort("tps_min", 0);
  tpsMax = prefs.getUShort("tps_max", 4095);

  bool valid = (mapMax > mapMin) && (tpsMax > tpsMin);
  Serial.printf(">> Calibración cargada: MAP[%u–%u], TPS[%u–%u] %s\n",
                mapMin, mapMax, tpsMin, tpsMax, valid ? "(OK)" : "(inválido)");
  return valid;
}

/**
 * Guarda los valores actuales de calibración en NVS.
 */
bool CalibrationManager::saveCalibration() {
  prefs.putUShort("map_min", mapMin);
  prefs.putUShort("map_max", mapMax);
  prefs.putUShort("tps_min", tpsMin);
  prefs.putUShort("tps_max", tpsMax);
  prefs.end();  // Finaliza sesión NVS
  return true;
}

/**
 * Rutina guiada para calibrar el sensor MAP.
 * Instrucciones:
 *  - motor apagado → mapMax (presión atmosférica)
 *  - motor en ralentí → mapMin (vacío estable)
 */
void CalibrationManager::runMAPCalibration(MAPSensor& sensor) {
  Serial.println("\n=== 🧭 Calibración de sensor MAP ===");
  delay(1000);

  Serial.println("[1] Enciende la llave (motor apagado): midiendo presión atmosférica");
  for (int i = 5; i > 0; --i) {
    uint16_t raw = sensor.readRaw();
    float volt = raw * 3.3f / 4095.0f;
    Serial.printf("  -> ADC: %4u (%.2f V)  [%ds]\n", raw, volt, i);
    delay(1000);
  }
  mapMax = sensor.readRaw();  // presión más alta posible
  Serial.printf("✔️  mapMax capturado = %u\n\n", mapMax);

  Serial.println("[2] Enciende el motor y déjalo en ralentí: midiendo vacío");
  for (int i = 5; i > 0; --i) {
    uint16_t raw = sensor.readRaw();
    float volt = raw * 3.3f / 4095.0f;
    Serial.printf("  -> ADC: %4u (%.2f V)  [%ds]\n", raw, volt, i);
    delay(1000);
  }
  mapMin = sensor.readRaw();  // vacío estable a ralentí
  Serial.printf("✔️  mapMin capturado = %u\n", mapMin);

  Serial.printf("✅ Calibración MAP completada: min=%u, max=%u\n\n", mapMin, mapMax);
}

/**
 * Rutina guiada para calibrar el sensor TPS.
 * Instrucciones:
 *  - pedal suelto → tpsMin
 *  - pedal a fondo → tpsMax
 */
void CalibrationManager::runTPScalibration(TPSSensor& sensor) {
  Serial.println("\n=== 🧭 Calibración de sensor TPS ===");
  delay(1000);

  Serial.println("[1] Asegúrate de que el pedal esté totalmente SUELTO");
  for (int i = 5; i > 0; --i) {
    uint16_t raw = sensor.readRaw();
    float volt = raw * 3.3f / 4095.0f;
    Serial.printf("  -> ADC: %4u (%.2f V)  [%ds]\n", raw, volt, i);
    delay(1000);
  }
  tpsMin = sensor.readRaw();
  Serial.printf("✔️  tpsMin capturado = %u\n\n", tpsMin);

  Serial.println("[2] Pisa el pedal de acelerador A FONDO y mantenlo presionado");
  for (int i = 5; i > 0; --i) {
    uint16_t raw = sensor.readRaw();
    float volt = raw * 3.3f / 4095.0f;
    Serial.printf("  -> ADC: %4u (%.2f V)  [%ds]\n", raw, volt, i);
    delay(1000);
  }
  tpsMax = sensor.readRaw();
  Serial.printf("✔️  tpsMax capturado = %u\n", tpsMax);

  Serial.printf("✅ Calibración TPS completada: min=%u, max=%u\n\n", tpsMin, tpsMax);
}

uint16_t CalibrationManager::getMAPMin() const { return mapMin; }
uint16_t CalibrationManager::getMAPMax() const { return mapMax; }
uint16_t CalibrationManager::getTPSMin() const { return tpsMin; }
uint16_t CalibrationManager::getTPSMax() const { return tpsMax; }
