#pragma once

#include <Arduino.h>
#include <SimulableSensor.h>

/**
 * MAPSensor
 * Lee el sensor de presión absoluta del múltiple y lo convierte a vacío relativo en pulgadas de mercurio (inHg).
 */
class MAPSensor : public SimulableSensor {
public:
  void begin(uint8_t analogPin);             // Inicializa el pin analógico
  uint16_t readRaw();                        // Lectura cruda (0–4095)
  float readNormalized();                    // Valor calibrado entre 0.0 y 1.0
  float readVacuum_inHg();                   // Vacío relativo en pulgadas de mercurio (–18 a 0 inHg)
  float readVolts() const;                   // Voltaje del sensor (útil para depuración)
  uint16_t readRawISR();
  float convertRawToHg(uint16_t raw);


private:
  uint8_t _pin;
};
