#include "MAFSensor.h"
#include "CalibrationManager.h"

void MAFSensor::begin(uint8_t adsChannel, Adafruit_ADS1115* adsPtr) {
  _adsChannel = adsChannel;
  _ads = adsPtr;
}

uint16_t MAFSensor::readRaw() {
  if (modoSimulacion) return rawSimulado;
  return _ads->readADC_SingleEnded(_adsChannel);
}

float MAFSensor::readNormalized() {
  uint16_t raw = readRaw();
  uint16_t min = CalibrationManager::getInstance().getTPSMin();
  uint16_t max = CalibrationManager::getInstance().getTPSMaxRaw();

  if (max <= min || raw < min) return 0.0f;

  return constrain(1.0f - (float)(raw - min) / (max - min), 0.0f, 1.0f);
}

float MAFSensor::readPorcent() {
  return readNormalized() * 100.0f;
}

float MAFSensor::readVolts() {
  if (modoSimulacion) {
    return (rawSimulado * 3.3f) / 4095.0f;
  }
  int16_t raw = readRaw();
  return raw * 0.1875f / 1000.0f;
}

bool MAFSensor::isValidReading() {
  uint16_t raw = readRaw();
  return (raw >= 50 && raw <= 4045);
}

float MAFSensor::convertRawToPercent(uint16_t raw) {
  uint16_t min = CalibrationManager::getInstance().getTPSMin();
  uint16_t max = CalibrationManager::getInstance().getTPSMaxRaw();

  if (max <= min || raw < min) return 0.0f;

  float norm = (float)(raw - min) / (max - min);
  return constrain(norm, 0.0f, 1.0f) * 100.0f;
}
