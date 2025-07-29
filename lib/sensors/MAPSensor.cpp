#include "MAPSensor.h"
#include "CalibrationManager.h"
#include "driver/adc.h"
#include "ADCUtils.h"

void MAPSensor::begin(uint8_t analogPin) {
  _pin = analogPin;
  cachedRaw = 0;
  pinMode(_pin, INPUT);

  adc1_channel_t channel = pinToADCChannel(_pin);
  if (channel == ADC1_CHANNEL_MAX) {
    Serial.println("Error: GPIO inválido para ADC1.");
    return;
  }

  if (channel != ADC1_CHANNEL_MAX) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);
  }
}

uint16_t MAPSensor::readRaw() {
  if (modoSimulacion) return rawSimulado;
  if (_pin == 0xFF) {
    Serial.println("ERROR: MAPSensor pin no inicializado!");
    return 0;
  }
  return cachedRaw;
}

void MAPSensor::updateCacheFromISR() {
  adc1_channel_t channel = pinToADCChannel(_pin);
  if (channel != ADC1_CHANNEL_MAX) {
    cachedRaw = adc1_get_raw(channel);
  }
}

uint16_t MAPSensor::readRawISR() {
  adc1_channel_t channel = pinToADCChannel(_pin);
  if (channel == ADC1_CHANNEL_MAX) {
    return 0;
  }
  return adc1_get_raw(channel);
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
  if (modoSimulacion) return (rawSimulado * 3.3f) / 4095.0f;
  uint16_t raw = analogRead(_pin);
  return (raw * 3.3f) / 4095.0f;
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

  Serial.print("[DEBUG] MAP Raw: "); Serial.println(raw);
  Serial.print("[DEBUG] MAP Min: "); Serial.println(min);
  Serial.print("[DEBUG] MAP Max: "); Serial.println(max);

  if (max <= min || raw < min || raw > max) {
    Serial.println("[DEBUG] MAP fuera de rango o calibración inválida. Retornando 0.0%");
    return 0.0f;
  }

  float norm = (float)(raw - min) / (max - min);
  float percent = constrain(norm, 0.0f, 1.0f) * 100.0f;

  Serial.print("[DEBUG] MAP Load %: "); Serial.println(percent, 2);

  return percent;
}

float MAPSensor::readMAPLoadPercent() {
  uint16_t raw = readRaw();
  uint16_t min = CalibrationManager::getInstance().getMAPMin();
  uint16_t max = CalibrationManager::getInstance().getMAPMax();

  if (max <= min) return 0.0f;

  float percent = 100.0f * (float)(raw - min) / (float)(max - min);
  return percent;
}

 
