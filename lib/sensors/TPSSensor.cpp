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
 * Protege contra lecturas erráticas fuera de rango.
 * @return Valor confiable de 0 a 4095.
 */
uint16_t TPSSensor::readRaw() {
  uint16_t raw = analogRead(_pin);

  if (raw < 50 || raw > 4045) {
  return CalibrationManager::getInstance().getTPSMin();  // ← más seguro que 0
  }


  return raw;
}

/**
 * Devuelve el valor normalizado entre 0.0 y 1.0 en base a los valores de calibración.
 * Evita división por cero y lecturas fuera de rango calibrado.
 */
float TPSSensor::readNormalized() {
  uint16_t raw = readRaw();
  uint16_t min = CalibrationManager::getInstance().getTPSMin();
  uint16_t max = CalibrationManager::getInstance().getTPSMax();

  if (max <= min || raw < min || raw > max) {
    //Serial.println(">> TPSSensor fuera de rango de calibración");
    return 0.0f;
  }

  return constrain((float)(raw - min) / (max - min), 0.0f, 1.0f);
}

/**
 * Convierte la lectura a porcentaje de apertura del acelerador (0–100%)
 */
float TPSSensor::readPct() {
  return readNormalized() * 100.0f;
}

/**
 * Convierte lectura cruda del ADC a volts reales en el divisor.
 * Protege contra valores extremos.
 */
float TPSSensor::readVolts() const {
  uint16_t raw = analogRead(_pin);
  if (raw < 50 || raw > 4045) {
    uint16_t fallback = CalibrationManager::getInstance().getTPSMin();
    return (fallback * 3.3f) / 4095.0f;
  }


  return (raw * 3.3f) / 4095.0f;
}

/**
 * Verifica si la lectura actual del TPS es confiable.
 * Útil para evitar transiciones de estado con datos corruptos.
 */
bool TPSSensor::isValidReading() {
  uint16_t raw = analogRead(_pin);
  return (raw >= 50 && raw <= 4045);
}
