#pragma once

#include <Arduino.h>
#include "driver/dac.h"


class AcousticInjector {
public:
  static constexpr uint8_t TABLE_SIZE = 64;
  static constexpr uint16_t SAMPLE_RATE = 8000; // Hz
  static constexpr float RAMP_STEP = 0.01f;

  void begin(uint8_t dacPin, uint8_t relayPin);
  void start(float level);
  void stop();
  void setLevel(float level);
  void update();               // Ramp de nivel
  void applyPendingDAC();      // ← ¡NUEVO! Aplica DAC fuera del ISR
  uint8_t getCurrentDAC() const;
  bool isActive() const;
  static void IRAM_ATTR onTimer();
  void testRelay(bool);
  bool isRelayActive() const;
  void test();  // Ejecuta una prueba rápida del sonido acústico

private:
  uint8_t  _dacPin = 0;
  uint8_t  _relayPin = 0;
  uint8_t  _index = 0;
  float    _level = 0.0f;
  float    _targetLevel = 0.0f;
  uint8_t  _lastDACValue = 128;
  dac_channel_t _dacChannel;
  hw_timer_t* _timer = nullptr;

  static const uint8_t _sineTable[TABLE_SIZE];
};
