#include "TurboController.h"

void VortexController::begin(uint8_t pinRelay) {
  relayPin = pinRelay;
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);  // Asegura que el turbo arranque apagado
  active = false;
}

void VortexController::start() {
  if (!active) {
    digitalWrite(relayPin, HIGH);  // Activa el relé
    active = true;
    // Serial.println(">> Turbo ON");
  }
}

void VortexController::stop() {
  if (active) {
    digitalWrite(relayPin, LOW);  // Desactiva el relé
    active = false;
    // Serial.println(">> Turbo OFF");
  }
}

bool VortexController::isOn() const {
  return active;
}

void VortexController::updatePowerLevel(float level) {
  if (!active) return;

  // 🚧 Futuro: aplicar PWM, DAC o lógica de control variable
  // Por ahora no hace nada
}

bool VortexController::isActive() const {
  return active;  // ← o lo que estés usando para rastrear el estado actual
}
