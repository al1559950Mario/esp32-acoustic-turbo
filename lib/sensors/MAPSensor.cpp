#include "MAPSensor.h"
#include "CalibrationManager.h"

/**
 * Inicializa el pin analógico del sensor MAP.
 * @param analogPin Número de pin GPIO del ESP32 asignado a la entrada analógica.
 */
void MAPSensor::begin(uint8_t analogPin) {
  _pin = analogPin;
  pinMode(_pin, INPUT);
}

/**
 * Realiza una lectura sin procesar del ADC del pin MAP.
 * @return Valor crudo de 0 a 4095 (12 bits del ADC del ESP32).
 */
uint16_t MAPSensor::readRaw() {
  return analogRead(_pin);
}

/**
 * Devuelve el valor normalizado del sensor MAP entre 0.0 y 1.0.
 * Utiliza valores de calibración guardados en NVS.
 */
float MAPSensor::readNormalized() {
  uint16_t raw = isSimulationActive() ? getSimulatedRaw() : analogRead(_pin);
  uint16_t min = CalibrationManager::getInstance().getMAPMin();
  uint16_t max = CalibrationManager::getInstance().getMAPMax();

  if (max <= min) return 0.0f; // Evita división por cero o datos corruptos

  return constrain((float)(raw - min) / (max - min), 0.0f, 1.0f);
}

/**
 * Convierte la lectura MAP a kilopascales (kPa) utilizando una interpolación sobre el rango físico estimado.
 * @return Presión estimada del múltiple en kPa (usualmente entre 20 y 105).
 */
float MAPSensor::readkPa() {
  float norm = readNormalized();
  const float kpaMin = 20.0f;   // Vacío en deceleración (~ -18 inHg)
  const float kpaMax = 105.0f;  // Presión atmosférica o aceleración a fondo
  return kpaMin + norm * (kpaMax - kpaMin);
}

float MAPSensor::readVolts() const {
  uint16_t raw = analogRead(_pin);
  return (raw * 3.3f) / 4095.0f;
}
