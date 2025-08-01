#pragma once

#include "VortexController.h"
#include "AcousticInjector.h"
#include "ThresholdManager.h"

class ActuatorManager {
public:
  ActuatorManager() = default;

  // Inicializa ambos actuadores con sus pines respectivos
  void begin(uint8_t turboRelayPin, uint8_t acousticDacPin, uint8_t acousticRelayPin);

  // Actualiza lógica interna (por ejemplo, rampas, timers)
  void update();

  void stopAll();
  // Control Vortex
  void startVortex();
  void stopVortex();
  bool isTurboOn() const;
  void setVortexLevel(float level);

  // Control Acoustic Injector
  void startAcoustic(float level);
  void stopAcoustic();
  void setAcousticParameters(float level, float mapLoadPercent);
  bool isAcousticOn() const;

  VortexController& getVortexController();
  AcousticInjector& getAcousticInjector();
  

private:
  VortexController vortex;
  AcousticInjector injector;
  ThresholdManager* thresholdManager = nullptr;  ///< Puntero al gestor de umbrales

};
