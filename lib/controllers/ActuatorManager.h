#pragma once

#include "TurboController.h"
#include "AcousticInjector.h"

class ActuatorManager {
public:
  ActuatorManager() = default;

  // Inicializa ambos actuadores con sus pines respectivos
  void begin(uint8_t turboRelayPin, uint8_t acousticDacPin, uint8_t acousticRelayPin);

  // Actualiza lógica interna (por ejemplo, rampas, timers)
  void update();

  void stopAll();
  // Control Turbo
  void startTurbo();
  void stopTurbo();
  bool isTurboOn() const;
  void setTurboLevel(float level);

  // Control Acoustic Injector
  void startAcoustic(float level);
  void stopAcoustic();
  void setAcousticLevel(float level, float mapLoadPercent);
  bool isAcousticOn() const;

  TurboController& getTurboController();
  AcousticInjector& getAcousticInjector();

private:
  TurboController turbo;
  AcousticInjector injector;
};
