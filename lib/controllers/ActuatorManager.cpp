#include "ActuatorManager.h"

void ActuatorManager::begin(uint8_t turboRelayPin, uint8_t acousticDacPin, uint8_t acousticRelayPin) {
  vortex.begin(turboRelayPin);
  injector.begin(acousticDacPin, acousticRelayPin);

  // Apagar ambos al inicio
  vortex.stop();
  injector.stop();
}

void ActuatorManager::update() {
  injector.update();
  //turbo.updatePowerLevel(level);
  // Aquí podrías añadir lógica para turbo si la necesitas en update (ahora no tiene)
}

void ActuatorManager::startVortex() {
  vortex.start();
}

void ActuatorManager::stopAll() {
  vortex.stop();
  injector.stop();
}

void ActuatorManager::stopVortex() {
  vortex.stop();
}

void ActuatorManager::setVortexLevel(float level) {
  vortex.updatePowerLevel(level);
}

bool ActuatorManager::isTurboOn() const {
  return vortex.isOn();
}

void ActuatorManager::startAcoustic(float level) {
  injector.start(level);
}

void ActuatorManager::stopAcoustic() {
  injector.stop();
}

void ActuatorManager::setAcousticParameters(float level, float mapLoadPercent) {
  float freq = AcousticInjector::mapLoadToWaveFrequency(mapLoadPercent);
  injector.updateWaveFrequency(freq);
  injector.setLevel(level);
}


bool ActuatorManager::isAcousticOn() const {
  return injector.isActive();
}

VortexController& ActuatorManager::getVortexController() {
  return vortex;
}

AcousticInjector& ActuatorManager::getAcousticInjector() {
  return injector;
}
