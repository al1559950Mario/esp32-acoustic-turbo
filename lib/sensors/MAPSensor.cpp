#include "MAPSensor.h"
#include "CalibrationManager.h"

void MAPSensor::begin(uint8_t analogPin) {
  _pin = analogPin;
  pinMode(_pin, INPUT);
}

uint16_t MAPSensor::readRaw() {
  if (modoSimulacion) {
    return rawSimulado;
  }
  return analogRead(_pin);
}

float MAPSensor::readNormalized() {
  uint16_t raw = readRaw();
  uint16_t min = CalibrationManager::getInstance().getMAPMin();
  uint16_t max = CalibrationManager::getInstance().getMAPMax();

  if (max <= min) return 0.0f;

  return constrain((float)(raw - min) / (max - min), 0.0f, 1.0f);
}

float MAPSensor::readVacuum_inHg() {
  float norm = readNormalized();
  constexpr float vacMin = -18.0f;
  constexpr float vacMax = 0.0f;
  return vacMin + norm * (vacMax - vacMin);
}

float MAPSensor::readVolts() const {
  if (modoSimulacion) {
    //Serial.println("[MAP] Leyendo valor simulado");
    //Serial.printf("[MAP] Leyendo valor simulado: %u raw -> %.2f V\n", rawSimulado, (rawSimulado * 3.3f) / 4095.0f);


    return (rawSimulado * 3.3f) / 4095.0f;
  }
  uint16_t raw = analogRead(_pin);
  return (raw * 3.3f) / 4095.0f;
}

adc1_channel_t MAPSensor::pinToADCChannel(uint8_t gpio) {
  switch (gpio) {
    case 36: return ADC1_CHANNEL_0;
    case 37: return ADC1_CHANNEL_1;
    case 38: return ADC1_CHANNEL_2;
    case 39: return ADC1_CHANNEL_3;
    case 32: return ADC1_CHANNEL_4;
    case 33: return ADC1_CHANNEL_5;
    case 34: return ADC1_CHANNEL_6;
    case 35: return ADC1_CHANNEL_7;
    default: return ADC1_CHANNEL_MAX;
  }
}

uint16_t MAPSensor::readRawISR() {
  adc1_channel_t channel = pinToADCChannel(_pin);
  if (channel == ADC1_CHANNEL_MAX) {
    return 0;
  }
  return adc1_get_raw(channel);
}

float MAPSensor::convertRawToHg(uint16_t raw) {
  constexpr float vacMin = -18.0f;
  constexpr float vacMax = 0.0f;
  float norm = (float)raw / 4095.0f;
  return vacMin + norm * (vacMax - vacMin);
}

float MAPSensor::convertRawToPercent(uint16_t raw) {
  uint16_t min = CalibrationManager::getInstance().getMAPMin();
  uint16_t max = CalibrationManager::getInstance().getMAPMax();

  if (max <= min || raw < min || raw > max) return 0.0f;

  float norm = (float)(raw - min) / (max - min);
  return constrain(norm, 0.0f, 1.0f) * 100.0f;
}


float MAPSensor::readMAPLoadPercent() {
  uint16_t raw = readRaw();
  uint16_t min = CalibrationManager::getInstance().getMAPMin();  // vacío máximo (ralentí)
  uint16_t max = CalibrationManager::getInstance().getMAPMax();  // atmósfera

  if (max <= min) return 0.0f;

  float percent = 100.0f * (float)(raw - min) / (float)(max - min);
  return percent;  // intencionalmente sin constrain
}
