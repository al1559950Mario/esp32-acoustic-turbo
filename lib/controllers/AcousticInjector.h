#pragma once
#include <Arduino.h>

class AcousticInjector {
public:
  AcousticInjector() = default;
  void begin(uint8_t dacPin, uint8_t relayPin);
  void start(int level);
  void stop();

private:
  uint8_t _dacPin = 0;
  uint8_t _relayPin = 0;
};
