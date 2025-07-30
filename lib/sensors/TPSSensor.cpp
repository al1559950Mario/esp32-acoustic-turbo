// TPSSensor.cpp (implementación con ISR minimalista)
#include "TPSSensor.h"
#include "CalibrationManager.h"
#include "driver/adc.h"
#include "ADCUtils.h"  // si usas utilidades ADC específicas

void TPSSensor::begin(uint8_t analogPin) {
  _pin = analogPin;
  cachedRaw = 0;  // inicializar lectura cacheada
  pinMode(_pin, INPUT);

  adc1_channel_t channel = pinToADCChannel(_pin);
  if (channel != ADC1_CHANNEL_MAX) {
    adc1_config_width(ADC_WIDTH_BIT_12);  // Resolución a 12 bits (0-4095)
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);  // Atenuación para 3.3V
  }
}

uint16_t TPSSensor::readRaw() {
  if (modoSimulacion) {
    return rawSimulado;
  }
  if (_pin == 0xFF) {
    Serial.println("ERROR: TPSSensor pin no inicializado!");
    return 0;
  }

  cachedRaw = analogRead(_pin);  // <-- lectura directa

  return cachedRaw;
}


float TPSSensor::readNormalized() {
  uint16_t raw = readRaw();
  uint16_t min = CalibrationManager::getInstance().getTPSMin();
  uint16_t max = CalibrationManager::getInstance().getTPSMax();

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
  if (_pin == 0xFF) {
    Serial.println("ERROR: TPSSensor pin no inicializado en readVolts!");
    return 0.0f;
  }
  uint16_t raw = analogRead(_pin);

  if (raw <= 5) return 0.0f;
  if (raw >= 4090) return 3.3f;
  return raw * 3.3f / 4095.0f;
}

bool TPSSensor::isValidReading() {
  uint16_t raw = readRaw();
  return (raw >= 50 && raw <= 4045);
}


float TPSSensor::convertRawToPercent(uint16_t raw) {
  uint16_t min = CalibrationManager::getInstance().getTPSMin();
  uint16_t max = CalibrationManager::getInstance().getTPSMax();

  if (max <= min || raw < min || raw > max) return 0.0f;

  float norm = (float)(raw - min) / (max - min);
  return constrain(norm, 0.0f, 1.0f) * 100.0f;
}
