#pragma once

#include <Arduino.h>
#include "SimulableSensor.h"
#include "driver/adc.h"

class MAPSensor : public SimulableSensor {
public:
  void begin(uint8_t analogPin);
  uint16_t readRaw();
  float readNormalized();
  float readVacuum_inHg();
  float readVolts() const;
  uint16_t readRawISR();
  float convertRawToHg(uint16_t raw);
  void enableSimulation() { modoSimulacion = true; }
  void disableSimulation() { modoSimulacion = false; }

private:
  uint8_t _pin;
  adc1_channel_t pinToADCChannel(uint8_t gpio);
};
