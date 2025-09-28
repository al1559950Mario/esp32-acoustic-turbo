#include "MAPSensor.h"
#include "CalibrationManager.h"


void MAPSensor::begin(uint8_t adsChannel, Adafruit_ADS1115* adsPtr) {
  _adsChannel = adsChannel;
  _ads = adsPtr;
}

uint16_t MAPSensor::readRaw() {
  if (modoSimulacion) return rawSimulado;
  return _ads->readADC_SingleEnded(_adsChannel);
}

float MAPSensor::readNormalized() {
  uint16_t raw = readRaw();
  uint16_t min = CalibrationManager::getInstance().getMAPMin();
  uint16_t max = CalibrationManager::getInstance().getMAPMaxRaw();

  if (max <= min) return 0.0f;

  return constrain((float)(raw - min) / (max - min), 0.0f, 1.0f);
}

float MAPSensor::readVacuum_inHg() {
  float norm = readNormalized();
  constexpr float vacMin = -18.0f;
  constexpr float vacMax = 0.0f;
  return vacMin + norm * (vacMax - vacMin);
}

float MAPSensor::readVolts() {
  if (modoSimulacion) return (rawSimulado * 3.3f) / 4095.0f;
  int16_t raw = readRaw();
  return raw * 0.1875f / 1000.0f;  // GAIN_TWOTHIRDS
}

float MAPSensor::convertRawToHg(uint16_t raw) {
  constexpr float vacMin = -18.0f;
  constexpr float vacMax = 0.0f;
  float norm = (float)raw / 4095.0f;
  return vacMin + norm * (vacMax - vacMin);
}

float MAPSensor::convertRawToPercent(uint16_t raw) {
  uint16_t min = CalibrationManager::getInstance().getMAPMin();
  uint16_t max = CalibrationManager::getInstance().getMAPMaxRaw();

  if (max <= min) {
    // Calibración inválida
    return 0.0f;
  }

  if (raw <= min) {
    return 0.0f;  // por debajo del mínimo
  }

  if (raw >= max) {
    return 100.0f;  // por encima del máximo
  }

  float norm = (float)(raw - min) / (max - min);
  float percent = norm * 100.0f;

  return percent;
}


float MAPSensor::readMAPLoadPercent() {
  uint16_t raw = readRaw();
  uint16_t min = CalibrationManager::getInstance().getMAPMin();
  uint16_t max = CalibrationManager::getInstance().getMAPMaxRaw();

  if (max <= min) return 0.0f;

  float percent = 100.0f * (float)(raw - min) / (float)(max - min);
  return percent;
}

 
