#pragma once

#include "VortexController.h"
#include "AcousticInjector.h"
#include "ThresholdManager.h"
#include "../audio/PCM5102Driver.h"

class ActuatorManager {
public:
  ActuatorManager() = default;

  // Inicializa ambos actuadores con sus pines respectivos
  void begin(uint8_t rEnPin, uint8_t turboPwmPin, uint8_t turboPwmChannel,
                            uint8_t turboSensePin,
                            uint8_t acousticDacPin);

  // Actualiza lógica interna (por ejemplo, rampas, timers)
  void updateInjector();

  void stopAll();
  // Control Vortex
  void startVortex();
  void stopVortex();
  bool isTurboOn() const;
  void updateVortexLevel(float mafLevel, float mapLevel);

  // Control Acoustic Injector
  void startAcoustic(float level, float dMAFdt);
  void stopAcoustic();
  void setAcousticParameters(float mafLevel, float mapLoadPercent);
  bool isAcousticOn() const;
  float getCurrentFrequency();
  float getAcousticLevel();
  float getTurboLevel();

  VortexController& getVortexController();
  AcousticInjector& getAcousticInjector();
  float getTurboSense();
  bool decayFinished() const;

  // Minimal PCM/I2S sine test helper
  void testminimalPCMDAC();

  // Continuous pure sine via I2S pipeline
  void startPureSine(uint32_t freqHz = 1000, float amplitude = 0.6f);
  void stopPureSine();

  // ISR pipeline sine (uses AcousticInjector ISR)
  void startISRSine(uint32_t freqHz = 1000, float level = 0.3f);
  void stopISRSine();

  // Exponer stats de audio
  void getAudioStats(uint32_t& dropsFromISR, uint32_t& underruns, size_t& minAvail, size_t& maxAvail) const;
  void resetAudioStats();

  

private:
  VortexController vortex;
  AcousticInjector injector;
  PCM5102Driver pcm5102;
  ThresholdManager* thresholdManager = nullptr;  ///< Puntero al gestor de umbrales

};
