#pragma once

#include <Arduino.h>
#include <SimulableSensor.h>
#include <Adafruit_ADS1X15.h>

/**
 * @brief Sensor de flujo másico (MAF) basado en el código previo de TPS.
 *
 * Externa la misma API que tenía el sensor de TPS para permitir reutilizar
 * la lógica existente con el nuevo hardware sin necesidad de reescrituras
 * profundas.
 */
class MAFSensor : public SimulableSensor {
public:
  void begin(uint8_t adsChannel, Adafruit_ADS1115* adsPtr);

  uint16_t readRaw();
  float readNormalized();
  float readPorcent();
  float readVolts();
  bool isValidReading();
  float convertRawToPercent(uint16_t raw);

  void enableSimulation() { modoSimulacion = true; }
  void disableSimulation() { modoSimulacion = false; }

private:
  uint8_t _pin = 0xFF;
  volatile uint16_t _raw = 0;
  uint8_t _adsChannel = 0xFF;
  Adafruit_ADS1115* _ads = nullptr;
};
