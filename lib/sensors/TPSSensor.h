#pragma once

#include <Arduino.h>

/**
 * TPSSensor
 * Lee la señal del potenciómetro del pedal de acelerador y la escala a %.
 */
class TPSSensor {
public:
  void begin(uint8_t analogPin);
  uint16_t readRaw();     // Lectura en crudo (0–4095)
  float readPct();        // Conversión calibrada a % (0.0 – 100.0)
  float readNormalized(); // Valor entre 0.0 y 1.0

private:
  uint8_t _pin;
};
