#pragma once

#include <Arduino.h>
#include "SensorManager.h"
#include "AcousticInjector.h"

class ISRManager {
public:
  void begin(SensorManager* sensors, AcousticInjector* injector);
  void start();
  void stop();

  uint16_t getCachedMAPRaw();  
  uint16_t getCachedTPSRaw();
  static ISRManager* getInstance();

  static void IRAM_ATTR onTimerISR();

private:
  static ISRManager* _instance;

  SensorManager* _sensors = nullptr;
  AcousticInjector* _injector = nullptr;

  hw_timer_t* _timer = nullptr;

  // Solo RAW en la ISR
  volatile uint16_t cachedMAPRaw = 0;
  volatile uint16_t cachedTPSRaw = 0;
};
