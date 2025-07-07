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

  if (max <= min) return 0.0f; // Evita división por cero o calibración corrupta

  return constrain((float)(raw - min) / (max - min), 0.0f, 1.0f);
}

/**
 * Calcula vacío relativo en pulgadas de mercurio (inHg) usando el valor normalizado.
 * Se asume que el rango físico calibrado va de –18 inHg (máximo vacío) a 0 inHg (presión atmosférica).
 * @return Vacío estimado en inHg entre –18.0f y 0.0f
 */
float MAPSensor::readVacuum_inHg() {
  float norm = readNormalized();
  constexpr float vacMin = -18.0f; // Máximo vacío
  constexpr float vacMax = 0.0f;   // Sin vacío (atmósfera)
  return vacMin + norm * (vacMax - vacMin);
}

/**
 * Convierte la lectura cruda del sensor a voltaje real.
 * @return Voltaje entre 0.0V y 3.3V para depuración o monitoreo.
 */
float MAPSensor::readVolts() const {
  uint16_t raw = analogRead(_pin);
  return (raw * 3.3f) / 4095.0f;
}
