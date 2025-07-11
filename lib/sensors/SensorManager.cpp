#include "SensorManager.h"
#include "ISRManager.h"
 

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


void SensorManager::update() {
  // Usa los valores rápidos leídos por el ISR
  uint16_t rawMAP = ISRManager::getInstance()->getCachedMAPRaw();
  uint16_t rawTPS = ISRManager::getInstance()->getCachedTPSRaw();

  vacuum_inHg = mapSensor.convertRawToHg(rawMAP);
  tpsPercent  = tpsSensor.convertRawToPercent(rawTPS);
}
