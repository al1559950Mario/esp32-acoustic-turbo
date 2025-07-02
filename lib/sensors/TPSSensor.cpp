#include "TPSSensor.h"
#include "CalibrationManager.h"

/**
 * Inicializa el pin analógico asignado al TPS.
 * @param analogPin Número de pin GPIO del ESP32 asignado a la entrada analógica.
 */
void TPSSensor::begin(uint8_t analogPin) {
  _pin = analogPin;
  pinMode(_pin, INPUT);
}

/**
 * Realiza una lectura cruda del valor analógico (ADC) del TPS.
 * @return Valor de 0 a 4095 según apertura del pedal.
 */
uint16_t TPSSensor::readRaw() {
  return analogRead(_pin);
}

/**
 * Devuelve el valor normalizado entre 0.0 y 1.0 en base a los valores de calibración.
 */
float TPSSensor::readNormalized() {
  uint16_t raw = readRaw();
  uint16_t min = CalibrationManager::getInstance().getTPSMin();
  uint16_t max = CalibrationManager::getInstance().getTPSMax();

  if (max <= min) return 0.0f;

  return constrain((float)(raw - min) / (max - min), 0.0f, 1.0f);
}

/**
 * Convierte la lectura a porcentaje de apertura del acelerador (0–100%).
 * Ideal para determinar umbrales de activación.
 * @return Porcentaje estimado (0.0 – 100.0 %)
 */
float TPSSensor::readPct() {
  return readNormalized() * 100.0f;
}
