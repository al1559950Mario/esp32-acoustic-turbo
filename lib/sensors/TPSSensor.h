// TPSSensor.h (asegúrate de incluir esto en tu header)
#pragma once
#include <Arduino.h>
#include <SimulableSensor.h>
#include "driver/adc.h"  // 👈 NECESARIO para adc1_channel_t
#include "ADCUtils.h"



class TPSSensor : public SimulableSensor {
public:
  void begin(uint8_t analogPin);

  uint16_t readRaw();           // Ahora devuelve lectura cacheada
  float readNormalized();
  float readPorcent();
  float readVolts();
  bool isValidReading();
  float convertRawToPercent(uint16_t raw);

  void enableSimulation() { modoSimulacion = true; }
  void disableSimulation() { modoSimulacion = false; }

private:
  uint8_t _pin = 0xFF;
  volatile uint16_t _raw = 0;  // lectura cacheada desde ISR
};
