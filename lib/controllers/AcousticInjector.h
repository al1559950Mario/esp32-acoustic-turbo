#pragma once

#include <Arduino.h>
#include "driver/dac.h"

class AcousticInjector {
public:
  static constexpr uint8_t TABLE_SIZE = 16;
  static constexpr uint32_t SAMPLE_RATE = 64000;  // 64 kHz para alta fidelidad
  static constexpr float RAMP_STEP = 0.01f;

  void begin(uint8_t dacPin, uint8_t relayPin);
  void start(float level);
  void stop();
  void setLevel(float level);
  void update();               // Rampa de nivel
  void applyPendingDAC();      // Aplica valor DAC calculado fuera del ISR
  uint8_t getCurrentDAC() const;
  bool isActive() const;
  static void IRAM_ATTR onTimer();
  void testRelay(bool);
  bool isRelayActive() const;
  void test();  // Prueba rápida del sonido acústico
  void emitResonant(float level); // Señal por fase acumulada

  static AcousticInjector* _instance;

private:
  uint8_t  _dacPin = 0;
  uint8_t  _relayPin = 0;
  uint8_t  _index = 0;      // índice para tabla seno (solo para modo tabla)
  float    _level = 0.0f;
  float    _targetLevel = 0.0f;
  uint8_t  _lastDACValue = 128;
  dac_channel_t _dacChannel;
  hw_timer_t* _timer = nullptr;
  volatile uint8_t _levelInt = 0;  // nivel escalado 0-255 para ISR


  // Tabla seno 16 muestras para ISR rápido (0-255)
  static const uint8_t _sineTable[TABLE_SIZE];
};
