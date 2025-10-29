#include "CalibrationManager.h"
#include <Arduino.h>

CalibrationManager& CalibrationManager::getInstance() {
  static CalibrationManager inst;
  return inst;
} 

void CalibrationManager::begin(SensorManager* _sensors) {
  if (prefsMutex == nullptr) {
    prefsMutex = xSemaphoreCreateMutex();
  }

  // Toque rápido al NVS para inicializar partición sin bloquear
  if (prefsMutex && xSemaphoreTake(prefsMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
    prefs.begin("calib", true); // readonly touch
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }

  sensors = _sensors;
  currentStep = CalibStep::TPS_MIN;
  calibrationDone = loadCalibration();
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
  if (prefsMutex && xSemaphoreTake(prefsMutex, portMAX_DELAY) != pdTRUE) return;

  prefs.begin("calib", false);
  prefs.remove("map_min");
  prefs.remove("map_max");
  prefs.remove("tps_min");
  prefs.remove("tps_max");
  prefs.end();

  mapMin = mapMax = tpsMin = tpsMax = 0;
  calibrationDone = false;
  currentStep = CalibStep::TPS_MIN;

  xSemaphoreGive(prefsMutex);
  Serial.println(">> Umbrales borrados. Requiere calibración.");
}

// Si no existen las 4 claves, pide calibración
bool CalibrationManager::loadCalibration() {
  if (prefsMutex && xSemaphoreTake(prefsMutex, portMAX_DELAY) != pdTRUE) return false;

  prefs.begin("calib", false);
  bool ready = prefs.isKey("map_min")
            && prefs.isKey("map_max")
            && prefs.isKey("tps_min")
            && prefs.isKey("tps_max");

  if (!ready) {
    prefs.end();
    mapMin = mapMax = tpsMin = tpsMax = 0;
    calibrationDone = false;
    xSemaphoreGive(prefsMutex);
    Serial.println(">> No hay datos de calibración. Ejecute calibración.");
    return false;
  }

  mapMin = prefs.getUShort("map_min");
  mapMax = prefs.getUShort("map_max");
  tpsMin = prefs.getUShort("tps_min");
  tpsMax = prefs.getUShort("tps_max");
  prefs.end();

  xSemaphoreGive(prefsMutex);

  bool valid = mapMax > mapMin && tpsMax > tpsMin;
  calibrationDone = valid;
  return valid;
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
  if (prefsMutex && xSemaphoreTake(prefsMutex, portMAX_DELAY) != pdTRUE) return;

  // Actualizar RAM inmediatamente
  switch(step) {
    case CalibStep::MAP_MAX: mapMax = value; break;
    case CalibStep::MAP_MIN: mapMin = value; break;
    case CalibStep::TPS_MIN: tpsMin = value; break;
    case CalibStep::TPS_MAX: tpsMax = value; break;
  }

  // Persistir solo la clave modificada
  prefs.begin("calib", false);
  switch(step) {
    case CalibStep::MAP_MAX: prefs.putUShort("map_max", value); break;
    case CalibStep::MAP_MIN: prefs.putUShort("map_min", value); break;
    case CalibStep::TPS_MIN: prefs.putUShort("tps_min", value); break;
    case CalibStep::TPS_MAX: prefs.putUShort("tps_max", value); break;
  }
  prefs.end();

  if (prefsMutex) xSemaphoreGive(prefsMutex);
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

bool CalibrationManager::runAutoCalibration(SensorManager& sensors, bool simulacionActiva) {
  static bool initialized = false;
  static unsigned long startTime = 0;
  static uint16_t tpsMinCandidate = UINT16_MAX;
  static uint16_t tpsMaxCandidate = 0;
  static uint16_t mapMinCandidate = UINT16_MAX;
  static uint16_t mapMaxCandidate = 0;

  if (!initialized) {
    Serial.println(F("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
    Serial.println(F(" CALIBRACIÓN EN PROGRESO (20s)"));
    Serial.println(F("  >> Enciende la llave sin motor, presiona el pedal a fondo"));
    Serial.println(F("  >> Enciende el motor."));
    Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
    startTime = millis();
    initialized = true;
  }

  // Leer sensores
  uint16_t tpsRaw = sensors.readTPSRawCached();
  uint16_t mapRaw = sensors.readMAPRawCached();

  // Actualizar candidatos
  tpsMinCandidate = min(tpsMinCandidate, tpsRaw);
  tpsMaxCandidate = max(tpsMaxCandidate, tpsRaw);
  mapMinCandidate = min(mapMinCandidate, mapRaw);
  mapMaxCandidate = max(mapMaxCandidate, mapRaw);

  // Mostrar en consola
  float tpsVolts = sensors.representVoltsFromRaw(tpsRaw);
  float tpsMinVolts = sensors.representVoltsFromRaw(tpsMinCandidate);
  float tpsMaxVolts = sensors.representVoltsFromRaw(tpsMaxCandidate);

  float mapVolts = sensors.representVoltsFromRaw(mapRaw);
  float mapMinVolts = sensors.representVoltsFromRaw(mapMinCandidate);
  float mapMaxVolts = sensors.representVoltsFromRaw(mapMaxCandidate);

  Serial.printf(
    "\rTPS=%.2fV [%.2f ⇄ %.2f] | MAP=%.2fV [%.2f ⇄ %.2f]   ",
    tpsVolts, tpsMinVolts, tpsMaxVolts,
    mapVolts, mapMinVolts, mapMaxVolts
  );

  if (millis() - startTime >= 10000) {
    Serial.println(F("\n\n>> Tiempo finalizado. Guardando calibración..."));

    // Guardar en miembros internos
    tpsMin = tpsMinCandidate;
    tpsMax = tpsMaxCandidate;
    mapMin = mapMinCandidate;
    mapMax = mapMaxCandidate;

    // Guardar en NVS
    saveStep(CalibStep::TPS_MIN, tpsMin);
    saveStep(CalibStep::TPS_MAX, tpsMax);
    saveStep(CalibStep::MAP_MIN, mapMin);
    saveStep(CalibStep::MAP_MAX, mapMax);

    // Reset y marcar como terminado
    initialized = false;
    calibrationDone = true;

    Serial.println(F("✔ Calibración completada y almacenada."));
    return true;
  }
  return false;
}

// Getters
uint16_t CalibrationManager::getMAPMin() const {
  uint16_t v = 0;
  if (prefsMutex && xSemaphoreTake(prefsMutex, portMAX_DELAY) == pdTRUE) {
    v = mapMin;
    xSemaphoreGive(prefsMutex);
  } else {
    v = mapMin; // fallback raro
  }
  return v;
}

uint16_t CalibrationManager::getMAPMaxRaw() const {
  uint16_t v = 0;
  if (prefsMutex && xSemaphoreTake(prefsMutex, portMAX_DELAY) == pdTRUE) {
    v = mapMax;
    xSemaphoreGive(prefsMutex);
  } else v = mapMax;
  return v;
}

uint16_t CalibrationManager::getTPSMin() const {
  uint16_t v = 0;
  if (prefsMutex && xSemaphoreTake(prefsMutex, portMAX_DELAY) == pdTRUE) {
    v = tpsMin;
    xSemaphoreGive(prefsMutex);
  } else v = tpsMin;
  return v;
}

uint16_t CalibrationManager::getTPSMaxRaw() const {
  uint16_t v = 0;
  if (prefsMutex && xSemaphoreTake(prefsMutex, portMAX_DELAY) == pdTRUE) {
    v = tpsMax;
    xSemaphoreGive(prefsMutex);
  } else v = tpsMax;
  return v;
}

void CalibrationManager::update(bool sim) {
  if (calibrationDone || sensors == nullptr) {
    return;
  }
  simulation = sim;

  // Llamar runAutoCalibration y verificar si terminó
  if (runAutoCalibration(*sensors, sim)) {
    calibrationDone = true;
    saveCalibration();
    loadCalibration();
    Serial.print("Favor de reiniciar el dispositivo o apagar el carro para aplicar cambios.");
  }
}



