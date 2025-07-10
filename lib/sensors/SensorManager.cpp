#include "SensorManager.h"

void SensorManager::begin(uint8_t pinMAP, uint8_t pinTPS) {
  mapSensor.begin(pinMAP);
  tpsSensor.begin(pinTPS);
}

float SensorManager::readVacuum_inHg() {
  return vacuum_inHg;
}

float SensorManager::readTPSPercent() {
  return tpsPercent;
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

// 👇 Esta función es opcional si quieres actualizar valores cada ciclo
void SensorManager::update() {
  vacuum_inHg = mapSensor.readVacuum_inHg();
  tpsPercent  = tpsSensor.readPorcent();
}
