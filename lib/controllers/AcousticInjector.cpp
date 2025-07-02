#include "AcousticInjector.h"

void AcousticInjector::begin(uint8_t dacPin, uint8_t relayPin) {
  _dacPin = dacPin;
  _relayPin = relayPin;
  pinMode(_relayPin, OUTPUT);
}

void AcousticInjector::start(int level) {
  // stub
}

void AcousticInjector::stop() {
  // stub
}
