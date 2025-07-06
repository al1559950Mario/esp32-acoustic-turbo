#pragma once

#include <Arduino.h>

/**
 * TPSSensor
 * Lee el sensor de posición del acelerador y convierte la lectura en crudo, voltaje, porcentaje o valor normalizado.
 * Incluye validaciones para proteger contra lecturas fuera de rango y fallos por calibración incorrecta.
 */
class TPSSensor {
public:
  void begin(uint8_t analogPin);      // Inicializa el pin analógico

  uint16_t readRaw() const;                 // Lectura cruda (0–4095) con validación de rango
  float readNormalized();            // Valor entre 0.0 y 1.0 según calibración
  float readPct();                   // Porcentaje estimado (0.0–100.0%)
  float readVolts() const;           // Conversión directa a volts
  bool isValidReading();             // Verifica si la lectura está dentro de los límites seguros

private:
  uint8_t _pin;
};
