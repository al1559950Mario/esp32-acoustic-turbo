#include "SensorManager.h"


void SensorManager::begin(uint8_t pinMAP, uint8_t pinTPS) {
  mapSensor.begin(pinMAP);
  tpsSensor.begin(pinTPS);
}

float SensorManager::readVacuum_inHg() {
  return vacuum_inHg;
}

float SensorManager::readLoadTPSPercent() {
  return tpsLoadPercent;
}

uint16_t SensorManager::readMAPRaw() {
  return mapSensor.readRaw();
}

uint16_t SensorManager::readTPSRaw() {
  return tpsSensor.readRaw();
}

float SensorManager::readMAPVolts() {
  return mapSensor.readVolts();
}

float SensorManager::readTPSVolts() {
  return tpsSensor.readVolts();
}

bool SensorManager::isTPSValid() {
  return tpsSensor.isValidReading();
}

MAPSensor& SensorManager::getMAP() {
  return mapSensor;
}

TPSSensor& SensorManager::getTPS() {
  return tpsSensor;
}

float SensorManager::readMAPLoadPercent() {
  return mapLoadPercent;
}

float SensorManager::representVoltsFromRaw(uint16_t raw) const {
  return (raw * 3.3f) / 4095.0f;
}

void SensorManager::update() {
  uint16_t rawMAP = mapSensor.readRaw();
  uint16_t rawTPS = tpsSensor.readRaw();

  // Filtro IIR al raw directamente
  filteredRawMAP = alpha * rawMAP + (1 - alpha) * filteredRawMAP;
  filteredRawTPS = alpha * rawTPS + (1 - alpha) * filteredRawTPS;

  //Porcentaje absoluto
  mapLoadPercent = mapSensor.convertRawToPercent((uint16_t)filteredRawMAP);
  tpsLoadPercent = tpsSensor.convertRawToPercent((uint16_t)filteredRawTPS);
}

void SensorManager::enableSimulacion() {
  simulacionActiva = true;
}

void SensorManager::disableSimulacion() {
  simulacionActiva = false;
  mapSensor.disableSimulation();
  tpsSensor.disableSimulation();
}

bool SensorManager::isSimulation() {
  return simulacionActiva;
}

float SensorManager::getRelativeTPSLoad(uint16_t tpsInitial, uint16_t max) {
  if (max <= tpsInitial) return 0.0f;

  float norm = ((float)filteredRawTPS - tpsInitial) / (max - tpsInitial);
  return constrain(norm, 0.0f, 1.0f) * 100.0f;
}

float SensorManager::getRelativeMAPLoad(uint16_t mapInitial, uint16_t max) {
  if (max <= mapInitial) return 0.0f;

  float norm = ((float)filteredRawMAP - mapInitial) / (max - mapInitial);
  return constrain(norm, 0.0f, 1.0f) * 100.0f;
}
