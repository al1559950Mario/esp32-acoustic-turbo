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
  // Asegúrate de tener los thresholds antes de usarlos
  auto thresholds = thresholdManager->getThresholds();

  // Calcular frecuencia según carga MAP
  float freq = AcousticInjector::mapLoadToWaveFrequency(mapLoadPercent);
  injector.updateWaveFrequency(freq);

  // Ajustar el nivel tomando en cuenta el umbral INJ_TPS_ON
  float injTPSon = thresholds.INJ_TPS_ON;
  float rawTPS = level * 100.0f;

  float adjustedLevel = (rawTPS - injTPSon) / (100.0f - injTPSon);
  adjustedLevel = constrain(adjustedLevel, 0.0f, 1.0f);

  injector.setLevel(adjustedLevel);
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
