#include "TurboController.h"

void TurboController::begin(uint8_t pinRelay) {
  relayPin = pinRelay;
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);  // Asegura que el turbo arranque apagado
  active = false;
}

void TurboController::start() {
  if (!active) {
    digitalWrite(relayPin, HIGH);  // Activa el relé
    active = true;
    // Serial.println(">> Turbo ON");
  }
}

void TurboController::stop() {
  if (active) {
    digitalWrite(relayPin, LOW);  // Desactiva el relé
    active = false;
    // Serial.println(">> Turbo OFF");
  }
}

bool TurboController::isOn() const {
  return active;
}
