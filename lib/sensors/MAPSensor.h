#pragma once
#include <Arduino.h>
#include <SimulableSensor.h>
#include <Adafruit_ADS1X15.h>



class MAPSensor : public SimulableSensor {
public:
  void begin(uint8_t adsChannel, Adafruit_ADS1115* adsPtr);
  uint16_t readRaw();            // Devuelve el valor cacheado actualizado desde ISR

  float readNormalized();
  float readVacuum_inHg();
  float readVolts();
  float readMAPLoadPercent();

  float convertRawToHg(uint16_t raw);
  float convertRawToPercent(uint16_t raw);

private:
  uint8_t _pin = 0xFF;
  volatile uint16_t cachedRaw = 0;
  uint8_t _adsChannel = 0xFF;
  Adafruit_ADS1115* _ads = nullptr;
};
