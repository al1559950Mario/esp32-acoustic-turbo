#include "ActuatorManager.h"

void ActuatorManager::begin(uint8_t turboRelayPin, uint8_t acousticDacPin, uint8_t acousticRelayPin) {
  turbo.begin(turboRelayPin);
  injector.begin(acousticDacPin, acousticRelayPin);

  // Apagar ambos al inicio
  turbo.stop();
  injector.stop();
}

void ActuatorManager::update() {
  injector.update();
  //turbo.updatePowerLevel(level);
  // Aquí podrías añadir lógica para turbo si la necesitas en update (ahora no tiene)
}

void ActuatorManager::startTurbo() {
  turbo.start();
}

void ActuatorManager::stopAll() {
  turbo.stop();
  injector.stop();
}

void ActuatorManager::stopTurbo() {
  turbo.stop();
}

void ActuatorManager::setTurboLevel(float level) {
  turbo.updatePowerLevel(level);
}

bool ActuatorManager::isTurboOn() const {
  return turbo.isOn();
}

void ActuatorManager::startAcoustic(float level) {
  injector.start(level);
}

void ActuatorManager::stopAcoustic() {
  injector.stop();
}

void ActuatorManager::setAcousticLevel(float level) {
  injector.setLevel(level);
}

bool ActuatorManager::isAcousticOn() const {
  return injector.isActive();
}

TurboController& ActuatorManager::getTurboController() {
  return turbo;
}

AcousticInjector& ActuatorManager::getAcousticInjector() {
  return injector;
}
