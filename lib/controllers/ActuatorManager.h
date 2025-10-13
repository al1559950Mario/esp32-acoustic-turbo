#pragma once

#include "VortexController.h"
#include "AcousticInjector.h"
#include "ThresholdManager.h"

class ActuatorManager {
public:
  ActuatorManager() = default;

  // Inicializa ambos actuadores con sus pines respectivos
  void begin( uint8_t turboPwmPin, uint8_t turboPwmChannel,
                            uint8_t turboSensePin,
                            uint8_t acousticDacPin);

  // Actualiza lógica interna (por ejemplo, rampas, timers)
  void updateInjector();

  void stopAll();
  // Control Vortex
  void startVortex();
  void stopVortex();
  bool isTurboOn() const;
  void setVortexLevel(float, float);

  // Control Acoustic Injector
  void startAcoustic(float level);
  void stopAcoustic();
  void setAcousticParameters(float level, float mapLoadPercent);
  bool isAcousticOn() const;
  float getCurrentFrequency();
  float getAcousticLevel();
  float getTurboLevel();

  VortexController& getVortexController();
  AcousticInjector& getAcousticInjector();
  float getTurboSense();

  

private:
  VortexController vortex;
  AcousticInjector injector;
  ThresholdManager* thresholdManager = nullptr;  ///< Puntero al gestor de umbrales

};