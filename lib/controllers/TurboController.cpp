#include "TurboController.h"

void TurboController::begin(uint8_t relayPin) {
  _pin = relayPin;
  pinMode(_pin, OUTPUT);
}

void TurboController::update(float tpsPct, float mapKPa) {
  // stub
}
