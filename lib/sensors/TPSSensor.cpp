// TPSSensor.cpp (implementación con ISR minimalista)
#include "TPSSensor.h"
#include "CalibrationManager.h"


void TPSSensor::begin(uint8_t adsChannel, Adafruit_ADS1115* adsPtr) {
  _adsChannel = adsChannel;
  _ads = adsPtr;
}

uint16_t TPSSensor::readRaw() {
  if (modoSimulacion) return rawSimulado;
  return _ads->readADC_SingleEnded(_adsChannel);
}


float TPSSensor::readNormalized() {
  uint16_t raw = readRaw();
  uint16_t min = CalibrationManager::getInstance().getTPSMin();
  uint16_t max = CalibrationManager::getInstance().getTPSMaxRaw();

  if (max <= min || raw < min || raw > max) return 0.0f;

  return constrain(1.0f - (float)(raw - min) / (max - min), 0.0f, 1.0f);
}

float TPSSensor::readPorcent() {
  return readNormalized() * 100.0f;
}

float TPSSensor::readVolts() {
  if (modoSimulacion) {
    return (rawSimulado * 3.3f) / 4095.0f;
  }
  int16_t raw = readRaw();
  return raw * 0.1875f / 1000.0f;  // GAIN_TWOTHIRDS
}

bool TPSSensor::isValidReading() {
  uint16_t raw = readRaw();
  return (raw >= 50 && raw <= 4045);
}


float TPSSensor::convertRawToPercent(uint16_t raw) {
  uint16_t min = CalibrationManager::getInstance().getTPSMin();
  uint16_t max = CalibrationManager::getInstance().getTPSMaxRaw();

  if (max <= min || raw < min || raw > max) return 0.0f;

  float norm = (float)(raw - min) / (max - min);
  return constrain(norm, 0.0f, 1.0f) * 100.0f;
}
