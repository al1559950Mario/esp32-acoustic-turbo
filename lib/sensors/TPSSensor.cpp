#include "TPSSensor.h"
#include "CalibrationManager.h"

void TPSSensor::begin(uint8_t analogPin) {
  _pin = analogPin;
  pinMode(_pin, INPUT);
}

uint16_t TPSSensor::readRaw() {
  if (_pin == 0xFF) {
    Serial.println("ERROR: TPSSensor pin no inicializado!");
    return 0;
  }
  uint16_t raw = analogRead(_pin);
  Serial.printf("TPSSensor readRaw, pin: %d, value: %u\n", _pin, raw);

  constexpr uint16_t RAW_ERR_LOW  = 50;
  constexpr uint16_t RAW_ERR_HIGH = 4045;

  if (raw < RAW_ERR_LOW) return 0;
  if (raw > RAW_ERR_HIGH) return 4095;
  return raw;
}

float TPSSensor::readNormalized() {
  uint16_t raw = isSimulationActive() ? getSimulatedRaw() : analogRead(_pin);
  uint16_t min = CalibrationManager::getInstance().getTPSMin();
  uint16_t max = CalibrationManager::getInstance().getTPSMax();

  if (max <= min || raw < min || raw > max) return 0.0f;

  return constrain((float)(raw - min) / (max - min), 0.0f, 1.0f);
}

float TPSSensor::readPorcent() {
  return readNormalized() * 100.0f;
}

// Aquí está la versión modificada: 
// NO se llama a readRaw(), se usa analogRead directamente
float TPSSensor::readVolts() {
  if (_pin == 0xFF) {
    Serial.println("ERROR: TPSSensor pin no inicializado en readVolts!");
    return 0.0f;
  }
  uint16_t raw = analogRead(_pin);
  //Serial.printf("TPSSensor readVolts (test), pin: %d, raw: %u\n", _pin, raw);

  if (raw <= 5) return 0.0f;
  if (raw >= 4090) return 3.3f;
  return raw * 3.3f / 4095.0f;
}

bool TPSSensor::isValidReading() {
  uint16_t raw = analogRead(_pin);
  return (raw >= 50 && raw <= 4045);
}
