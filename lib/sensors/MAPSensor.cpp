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

  //Serial.print("[DEBUG] MAP Raw: "); Serial.println(raw);
  //Serial.print("[DEBUG] MAP Min: "); Serial.println(min);
  //Serial.print("[DEBUG] MAP Max: "); Serial.println(max);

  if (max <= min || raw < min || raw > max) {
    //Serial.println("[DEBUG] MAP fuera de rango o calibración inválida. Retornando 0.0%");
    return 0.0f;
  }

  float norm = (float)(raw - min) / (max - min);
  float percent = constrain(norm, 0.0f, 1.0f) * 100.0f;

  //Serial.print("[DEBUG] MAP Load %: "); Serial.println(percent, 2);

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

 
