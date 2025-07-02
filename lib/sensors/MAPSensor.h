#pragma once

#include <Arduino.h>

/**
 * MAPSensor
 * Lee el sensor de presión absoluta del múltiple y lo convierte a kPa o valor normalizado.
 */
class MAPSensor {
public:
  void begin(uint8_t analogPin);
  uint16_t readRaw();     // Lectura en crudo (0–4095)
  float readkPa();        // Conversión calibrada a kPa
  float readNormalized(); // Valor entre 0.0 y 1.0

private:
  uint8_t _pin;
};
