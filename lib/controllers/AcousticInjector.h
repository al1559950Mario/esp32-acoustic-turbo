#pragma once

#include <Arduino.h>
#include "driver/dac.h"

class AcousticInjector {
public:
  static constexpr uint8_t TABLE_SIZE = 16;
  static constexpr uint32_t SAMPLE_RATE = 64000;  // 64 kHz para alta fidelidad
  // Paso de rampa para suavizar cambios en el nivel (_level).
  // Modificar este valor para hacer la transición más lenta (valor menor) o más rápida (valor mayor).
  static constexpr float RAMP_STEP = 0.01f;

  void begin(uint8_t dacPin, uint8_t relayPin);
  void start(float level);
  void stop();
  void setLevel(float level);
  void update();               // Rampa de nivel
  void IRAM_ATTR applyPendingDAC(); // ✅ Safe para llamar desde interrupción
  uint8_t getCurrentDAC() const;
  bool isActive() const;
  static void IRAM_ATTR onTimer();
  void testRelay(bool);
  bool isRelayActive() const;
  void test();  // Prueba rápida del sonido acústico
  void emitResonant(float level); // Señal por fase acumulada
  void testSimple();
  void updateWaveFrequency(float freqHz);  // Cambiar nombre para aclarar que es por onda completa
  static float mapLoadToWaveFrequency(float mapLoadPercent);
  float getLevel() const { return _level; }
  float getFrequency() const { return _currentFrequency; }


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
  float _currentFrequency = 0.0f;


  // Tabla seno 16 muestras para ISR rápido (0-255)
  static const uint8_t _sineTable[TABLE_SIZE];
};
