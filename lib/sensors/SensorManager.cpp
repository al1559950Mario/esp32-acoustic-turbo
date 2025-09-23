#include "SensorManager.h"


void SensorManager::begin(uint8_t pinPressureData, uint8_t pinPressureSCK, uint8_t pinSDA, uint8_t pinSCL) {
  Wire.begin(pinSDA, pinSCL);
  if (!ads.begin()) {
    Serial.println("❌ ADS1115 no detectado");
  } else {
    ads.setGain(GAIN_TWOTHIRDS);
    adsReady = true;  
    Serial.println("✅ ADS1115 listo");
  }
  mapSensor.begin(1, &ads);  // A1 para MAP
  tpsSensor.begin(0, &ads);  // A0 para TPS
  pressureSensor.begin(pinPressureData, pinPressureSCK);
}


float SensorManager::readVacuum_inHg() {
  return vacuum_inHg;
}

float SensorManager::readTPSLoadPercent() {
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
  int16_t rawMAP = ads.readADC_SingleEnded(1);
  int16_t rawTPS = ads.readADC_SingleEnded(0); 

    // Debug: valores crudos
  Serial.print("[DEBUG] rawMAP = "); Serial.print(rawMAP);
  Serial.print(" | rawTPS = "); Serial.println(rawTPS);

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


float SensorManager::getRelativeTPSLoad(uint16_t tpsInitial) {
  if (tpsInitial >= 100) return 0.0f;
  float norm = ((float)tpsLoadPercent - tpsInitial) / (100.0f - tpsInitial);
  return constrain(norm, 0.0f, 1.0f) * 100.0f;
}

float SensorManager::getRelativeMAPLoad(uint16_t mapInitialPercent) {
  if (mapInitialPercent >= 100) return 0.0f;
  float norm = ((float)mapLoadPercent - mapInitialPercent) / (100.0f - mapInitialPercent);
  return constrain(norm, 0.0f, 1.0f) * 100.0f;
}

float SensorManager::readPressure_kPa() {
    return pressureSensor.readPressure_kPa();
}

long SensorManager::readPressureRaw() {
    return pressureSensor.readRaw();
}