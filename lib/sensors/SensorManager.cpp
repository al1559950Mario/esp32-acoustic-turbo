#include "SensorManager.h"
#include "ISRManager.h"
 

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
  Serial.println("[DEBUG] SensorManager::update() llamada");

  // Verifica si está activa la simulación MAP
  bool sim = isSimulation();

  Serial.print("[DEBUG] Simulación TPS activa: ");
  Serial.println(sim ? "Sí" : "No");

  uint16_t rawMAP = sim
                      ? mapSensor.getSimulatedRaw()
                      : ISRManager::getInstance()->getCachedMAPRaw();

  uint16_t rawTPS = sim
                      ? tpsSensor.getSimulatedRaw()
                      : ISRManager::getInstance()->getCachedTPSRaw();

  Serial.print("[DEBUG] rawMAP: "); Serial.println(rawMAP);
  Serial.print("[DEBUG] rawTPS: "); Serial.println(rawTPS);

  mapLoadPercent = mapSensor.convertRawToPercent(rawMAP);
  tpsLoadPercent = tpsSensor.convertRawToPercent(rawTPS);
}



void SensorManager::enableSimulacion() {
  simulacionActiva = true;
  mapSensor.enableSimulation();  
  tpsSensor.enableSimulation();
}

void SensorManager::disableSimulacion() {
  simulacionActiva = false;
  mapSensor.disableSimulation();
  tpsSensor.disableSimulation();
}

bool SensorManager::isSimulation() {
  return simulacionActiva;
}