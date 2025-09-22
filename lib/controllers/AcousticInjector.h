#pragma once

#include <Arduino.h>
#include "driver/dac.h"

class AcousticInjector {
public:
  enum FrequencyRangeOption : uint8_t {
      RANGE_1 = 0,  // 4400 – 5100 Hz
      RANGE_2 = 1,  // 5100 – 5800 Hz
      RANGE_3 = 2,   // 5800 – 6500 Hz
      RANGE_4 = 3 //FULL RANGE
  };
  void setFrequencyRangeOption(FrequencyRangeOption option);
  FrequencyRangeOption getFrequencyRangeOption() const;
  static constexpr uint8_t TABLE_SIZE = 64;
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
  void setTargetFrequency(float freq) {
    _targetFrequency = freq;
  }
  void updateWaveFrequency(float freqHz);  // Cambiar nombre para aclarar que es por onda completa
  float mapLoadToWaveFrequency(float mapLoadPercent);
  float getLevel() const { return _level; }
  float getFrequency() const { return _currentFrequency; }
  float AcousticInjector::getFreqMin() const { return _freqMin; }
  float AcousticInjector::getFreqMax() const { return _freqMax; }



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
  float _currentFrequency = 4400.0f;
  float _targetFrequency = 0.0f;
  static constexpr uint8_t PHASE_FRAC = 16;   
  static_assert((1 << PHASE_FRAC) > 0, "PHASE_FRAC ok");

  volatile uint32_t _phaseAcc = 0;
  volatile uint32_t _phaseStep = 0; 
  static constexpr float FREQ_RAMP_STEP = 1.0f; // Hz por llamada a update()
  static constexpr float DEFAULT_SAMPLE_RATE = 32000.0f; // tasa de muestreo segura
  FrequencyRangeOption _freqOption = RANGE_3;
  float _freqMin = 5500.0f;
  float _freqMax = 6500.0f;


  // Tabla seno 16 muestras para ISR rápido (0-255)
  static uint8_t _sineTable[TABLE_SIZE];
  
};