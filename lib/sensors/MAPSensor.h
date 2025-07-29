#pragma once
#include <Arduino.h>
#include <SimulableSensor.h>


class MAPSensor : public SimulableSensor {
public:
  void begin(uint8_t analogPin);

  uint16_t readRaw();            // Devuelve el valor cacheado actualizado desde ISR

  float readNormalized();
  float readVacuum_inHg();
  float readVolts() const;
  float readMAPLoadPercent();

  float convertRawToHg(uint16_t raw);
  float convertRawToPercent(uint16_t raw);

private:
  uint8_t _pin = 0xFF;
  volatile uint16_t cachedRaw = 0;
};
