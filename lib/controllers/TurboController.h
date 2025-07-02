#pragma once
#include <Arduino.h>

class TurboController {
public:
  TurboController() = default;
  void begin(uint8_t relayPin);
  void update(float tpsPct, float mapKPa);

private:
  uint8_t _pin = 0;
};
