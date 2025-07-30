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
  uint16_t rawMAP = mapSensor.readRaw();  // lectura directa
  uint16_t rawTPS = tpsSensor.readRaw();  // lectura directa

  mapLoadPercent = mapSensor.convertRawToPercent(rawMAP);
  tpsLoadPercent = tpsSensor.convertRawToPercent(rawTPS);;
    // Filtro IIR aplicado a porcentaje
  filteredMAP = alpha * mapLoadPercent + (1 - alpha) * filteredMAP;
  filteredTPS = alpha * tpsLoadPercent + (1 - alpha) * filteredTPS;
  mapLoadPercent = filteredMAP;
  tpsLoadPercent = filteredTPS;
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