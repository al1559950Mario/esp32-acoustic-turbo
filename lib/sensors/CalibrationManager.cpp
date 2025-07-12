#include "CalibrationManager.h"
#include <Arduino.h>

CalibrationManager& CalibrationManager::getInstance() {
  static CalibrationManager inst;
  return inst;
}

void CalibrationManager::begin() {
  prefs.begin("calib", false);
  prefs.end();
}

// Si no existen las 4 claves, pide calibración
bool CalibrationManager::loadCalibration() {
  prefs.begin("calib", false);
  bool ready = prefs.isKey("map_min")
            && prefs.isKey("map_max")
            && prefs.isKey("tps_min")
            && prefs.isKey("tps_max");
  if (!ready) {
    Serial.println(">> No hay datos de calibración. Ejecute calibración.");
    prefs.end();
    return false;
  }

  mapMin = prefs.getUShort("map_min");
  mapMax = prefs.getUShort("map_max");
  tpsMin = prefs.getUShort("tps_min");
  tpsMax = prefs.getUShort("tps_max");
  prefs.end();

  bool valid = mapMax > mapMin && tpsMax > tpsMin;
  Serial.printf(">> Calibración cargada: MAP[%u–%u], TPS[%u–%u] %s\n",
                mapMin, mapMax, tpsMin, tpsMax,
                valid ? "(OK)" : "(inválido)");
  return valid;
}

void CalibrationManager::loadDebugCalibration() {
  tpsMin = static_cast<uint16_t>((0.5f / 3.3f) * 4095);
  tpsMax = static_cast<uint16_t>((2.25f / 3.3f) * 4095);
  mapMin = static_cast<uint16_t>((3.05f / 3.3f) * 4095);
  mapMax = static_cast<uint16_t>((3.26f / 3.3f) * 4095);

  Serial.println(">> Calibración DEBUG cargada (valores hardcodeados).");
}


// Borra NVS y valores en RAM
void CalibrationManager::clearCalibration() {
  prefs.begin("calib", false);
  prefs.clear();
  prefs.end();
  
  mapMin = mapMax = tpsMin = tpsMax = 0;
  Serial.println(">> Umbrales borrados. Requiere calibración.");
}

// Graba los 4 valores actuales
bool CalibrationManager::saveCalibration() {
  prefs.begin("calib", false);
  prefs.putUShort("map_min", mapMin);
  prefs.putUShort("map_max", mapMax);
  prefs.putUShort("tps_min", tpsMin);
  prefs.putUShort("tps_max", tpsMax);
  prefs.end();
  Serial.println(">> Valores de calibración guardados en NVS.");
  return true;
}

// Guarda un solo paso inmediatamente
void CalibrationManager::saveStep(CalibStep step, uint16_t value) {
  prefs.begin("calib", false);
  switch(step) {
    case CalibStep::MAP_MAX: prefs.putUShort("map_max", value); break;
    case CalibStep::MAP_MIN: prefs.putUShort("map_min", value); break;
    case CalibStep::TPS_MIN: prefs.putUShort("tps_min", value); break;
    case CalibStep::TPS_MAX: prefs.putUShort("tps_max", value); break;
  }
  prefs.end();
  Serial.printf(">> Paso %d guardado: %u\n", int(step), value);
}

// Función para esperar ENTER y descartar secuencias de escape o caracteres extraños
bool waitForEnter() {
  while (true) {
    if (Serial.available()) {
      char ch = Serial.read();

      if (ch == '\r') {  // ENTER en Windows suele ser '\r\n'
        // Limpiar posible '\n' siguiente
        delay(5);
        while (Serial.available()) {
          char nextChar = Serial.peek();
          if (nextChar == '\n') Serial.read();
          else break;
        }
        return true;
      }
      else if (ch == 27) {  // ESC: limpiar secuencia escape completa
        delay(10);
        while (Serial.available()) Serial.read();
      }
      // Ignorar otros caracteres
    }
    delay(10);
  }
}

// Captura en tiempo real el máximo y mínimo de MAP
void CalibrationManager::runMAPCalibration(MAPSensor& sensor) {
  Serial.println(F("\n=== Calibración MAP (Max then Min) ==="));
  delay(500);

  // 1) Captura MAP_MAX (motor apagado)
  Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F(" [1] Motor apagado: capturando valor máximo"));
  Serial.println(F("     >> Presiona ENTER cuando el valor en consola sea adecuado"));
  Serial.println(F("     >> O espera 20 segundos para avanzar automáticamente"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));

  uint16_t candidateMax = 0;
  uint16_t raw = 0;
  unsigned long startTime = millis();

  while (true) {
    raw = sensor.readRaw();
    candidateMax = max(candidateMax, raw);
    Serial.printf("\r    raw=%4u | candidatoMax=%4u", raw, candidateMax);
    delay(200);

    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.length() == 0) break;
    }
    unsigned long now = millis();
    if (now - startTime >= 20000) {
      Serial.println("\n>> Tiempo agotado, avanzando automáticamente.");
      break;
    }
  }
  while (Serial.available()) Serial.read();  // limpia buffer por seguridad

  delay(250);
  mapMax = candidateMax;
  Serial.printf("\n✔ mapMax = %u\n", mapMax);
  saveStep(CalibStep::MAP_MAX, mapMax);

  // 2) Captura MAP_MIN (motor en ralentí)
  Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F(" [2] Motor en ralentí: capturando valor mínimo"));
  Serial.println(F("     >> Presiona ENTER cuando el valor en consola sea adecuado"));
  Serial.println(F("     >> O espera 20 segundos para avanzar automáticamente"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));

  uint16_t candidateMin = UINT16_MAX;
  raw = 0;
  startTime = millis();

  while (true) {
    raw = sensor.readRaw();
    candidateMin = min(candidateMin, raw);
    Serial.printf("\r    raw=%4u | candidatoMin=%4u", raw, candidateMin);
    delay(200);

    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.length() == 0) break;
    }
    unsigned long now = millis();
    if (now - startTime >= 20000) {
      Serial.println("\n>> Tiempo agotado, avanzando automáticamente.");
      break;
    }
  }
  while (Serial.available()) Serial.read();  // limpia buffer por seguridad

  delay(250);
  mapMin = candidateMin;
  Serial.printf("\n✔ mapMin = %u\n", mapMin);
  saveStep(CalibStep::MAP_MIN, mapMin);
}

// Captura en tiempo real TPS_MIN y TPS_MAX
void CalibrationManager::runTPSCalibration(TPSSensor& sensor) {
  Serial.println(F("\n=== Calibración TPS (Min then Max) ==="));
  delay(500);

  // 1) TPS_MIN (pedal suelto)
  Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F(" [1] Pedal suelto: capturando valor mínimo"));
  Serial.println(F("     >> Presiona ENTER cuando el valor en consola sea adecuado"));
  Serial.println(F("     >> O espera 20 segundos para avanzar automáticamente"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));

  uint16_t candidateMin = UINT16_MAX;
  uint16_t raw = 0;
  unsigned long startTime = millis();

  while (true) {
    raw = sensor.readRaw();
    candidateMin = min(candidateMin, raw);
    Serial.printf("\r    raw=%4u | candidatoMin=%4u", raw, candidateMin);
    delay(200);

    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.length() == 0) break;
    }
    unsigned long now = millis();
    if (now - startTime >= 20000) {
      Serial.println("\n>> Tiempo agotado, avanzando automáticamente.");
      break;
    }
  }
  while (Serial.available()) Serial.read();
  delay(250);
  tpsMin = candidateMin;
  Serial.printf("\n✔ tpsMin = %u\n", tpsMin);
  saveStep(CalibStep::TPS_MIN, tpsMin);

  // 2) TPS_MAX (pedal a fondo)
  Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F(" [2] Pedal a fondo: capturando valor máximo"));
  Serial.println(F("     >> Presiona ENTER cuando el valor en consola sea adecuado"));
  Serial.println(F("     >> O espera 20 segundos para avanzar automáticamente"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));

  uint16_t candidateMax = 0;
  raw = 0;
  startTime = millis();

  while (true) {
    raw = sensor.readRaw();
    candidateMax = max(candidateMax, raw);
    Serial.printf("\r    raw=%4u | candidatoMax=%4u", raw, candidateMax);
    delay(200);

    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.length() == 0) break;
    }
    unsigned long now = millis();
    if (now - startTime >= 20000) {
      Serial.println("\n>> Tiempo agotado, avanzando automáticamente.");
      break;
    }
  }
  while (Serial.available()) Serial.read();  // limpia buffer por seguridad

  delay(250);
  tpsMax = candidateMax;
  Serial.printf("\n✔ tpsMax = %u\n", tpsMax);
  saveStep(CalibStep::TPS_MAX, tpsMax);

  Serial.printf("\n✅ Calibración TPS completada: min=%u, max=%u\n\n", tpsMin, tpsMax);
}


// Getters
uint16_t CalibrationManager::getMAPMin() const { return mapMin; }
uint16_t CalibrationManager::getMAPMax() const { return mapMax; }
uint16_t CalibrationManager::getTPSMin() const { return tpsMin; }
uint16_t CalibrationManager::getTPSMax() const { return tpsMax; }

