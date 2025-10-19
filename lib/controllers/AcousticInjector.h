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

  void begin(uint8_t dacPin);
  void start(float level);
  void stop();
  void setLevel(float level);
  void update();               // Rampa de nivel
  void IRAM_ATTR applyPendingDAC(); // ✅ Safe para llamar desde interrupción
  uint8_t getCurrentDAC() const;
  bool isActive() const;
  static void IRAM_ATTR onTimer();
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
  float getFreqMin() const { return _freqMin; }
  float getFreqMax() const { return _freqMax; }
  void startDecay(uint32_t  nowMillis);
  void setDecayParameters(uint32_t durationMs, float avgMAPLevel,
                        float gFast, float gSustain, uint32_t tFastMs, uint32_t tSustainMs);
  void setDecayLevel(float level);   // nivel base que dispara la resonancia (0..1)
  void updateDecayState();           // llamados desde loop/task para recalcular envelope/resonator


  static AcousticInjector* _instance;

private:
  uint8_t  _dacPin = 0;
  uint8_t  _index = 0;      // índice para tabla seno (solo para modo tabla)
  float    _level = 0.0f;
  float    _targetLevel = 0.0f;
  uint8_t  _lastDACValue = 128;
  dac_channel_t _dacChannel;
  hw_timer_t* _timer = nullptr;
  volatile uint8_t _levelInt = 0;  // nivel escalado 0-255 para ISR
  float _currentFrequency = 2000.0f;
  float _targetFrequency = 2000.0f;
  float _lastFrequency = 2000.0f;
  static constexpr uint8_t PHASE_FRAC = 16;   
  static_assert((1 << PHASE_FRAC) > 0, "PHASE_FRAC ok");
  bool _active = false;

  volatile uint32_t _phaseAcc = 0;
  volatile uint32_t _phaseStep = 0; 
  static constexpr uint8_t TABLE_SIZE = 64;
  static constexpr uint32_t SAMPLE_RATE = 64000;  // 64 kHz para alta fidelidad
  // Paso de rampa para suavizar cambios en el nivel (_level).
  // Modificar este valor para hacer la transición más lenta (valor menor) o más rápida (valor mayor).
  static constexpr float RAMP_STEP = 0.01f;
  static constexpr float FREQ_RAMP_STEP = 05.0f; // Hz por llamada a update()
  static constexpr float DEFAULT_SAMPLE_RATE = 32000.0f; // tasa de muestreo segura
  FrequencyRangeOption _freqOption = RANGE_3;
  float _freqMin = 2000.0f;
  float _freqMax = 6500.0f;
  bool _skipSmoothStep = false;
  void resetInternal() {
  // Barrido
  _phaseAcc     = 0;
  _phaseStep    = 0;
  _index        = 0;
  // Nivel
  _level        = 0.0f;
  _targetLevel  = 0.0f;
  _levelInt     = 0;
  _lastDACValue = 128;
  _skipSmoothStep = false;
  // Frecuencia (si quieres reiniciar a la última cargada en begin())
  // _currentFrequency = _freqMin;  
  // _targetFrequency  = _freqMin;  
}
  unsigned long _decayStartMillis; // marca de tiempo al iniciar DECAY

  // Tabla seno 16 muestras para ISR rápido (0-255)
  static uint8_t _sineTable[TABLE_SIZE];

  volatile bool _inDecay = false;
  volatile uint32_t _resPhaseStep = 0; // paso de fase del resonador para la ISR
  volatile float _decayMix = 0.7f;              // mezcla resonador vs principal (0..1)
  uint32_t _decayDurationMs = 1200;    // default
  float _decayResFreq = 4000.0f;       // freq resonante por defecto
  volatile uint64_t _resPhaseAcc = 0;

  float _gFast;
  float _gSustain = 0.0f;       // 0..1
  uint32_t _tFastMs;
  uint32_t _tSustainMs;
  uint8_t _zeroCount = 0;
  volatile uint16_t _decayLevelMulInt = 65535; // 0..65535 multiplicador de level
  volatile uint16_t _decayEnvInt16 = 0;        // opción: 16-bit envelope si usas   
  volatile uint16_t _decayMixInt16;

// constantes de diseño — ajusta estos valores para afinar el ring down
static constexpr float POW_ALPHA = 1.0f;     // Qué controla: curvatura perceptual de la cola (exponente).
                                             // Rango típico: 1.0 .. 2.5
                                             // Cómo ajustarlo: subir (+0.1) para caída más rápida; bajar (-0.1) para cola más suave.

static constexpr float SOFTCLIP_BETA = 0.5f; // Qué controla: agresividad del soft‑clip aplicado a picos.
                                             // Rango típico: 0.1 .. 1.0
                                             // Cómo ajustarlo: subir para limitar picos; bajar para preservar dinámica.

static constexpr float PERCEPT_EXP = 0.75f;  // Qué controla: mapeo perceptual para niveles (curva de respuesta).
                                             // Rango típico: 0.5 .. 1.2
                                             // Cómo ajustarlo: valores <1.0 realzan bajos perceptualmente; >1.0 atenúa bajos.

const float MIX_FALL = 0.6f;                 // Qué controla: cuánto cae _decayMix durante la cola (0..1).
                                             // Rango típico: 0.2 .. 0.9
                                             // Cómo ajustarlo: subir para mayor énfasis en resonador; bajar para mantener el principal.

const float FADE_START = 0.80f;              // Qué controla: punto normalizado para empezar el fade final (0..1 del env).
                                             // Rango típico: 0.5 .. 0.95
                                             // Cómo ajustarlo: bajar para empezar fade antes; subir para mantener más tiempo.

const uint8_t ZERO_COUNT_TO_END = 4;         // Qué controla: frames consecutivos por debajo del umbral para declarar fin.
                                             // Rango típico: 1 .. 16
                                             // Cómo ajustarlo: subir para ser conservador contra ruido; bajar para terminar más rápido.

const uint8_t MAX_STEP_UP = 12;              // Qué controla: máximo incremento por tick en ramp 8‑bit.
                                             // Rango típico: 4 .. 16
                                             // Cómo ajustarlo: reducir para suavizar subidas; aumentar para respuesta más rápida.

const uint8_t MAX_STEP_DOWN = 8;             // Qué controla: máximo decremento por tick en ramp 8‑bit.
                                             // Rango típico: 4 .. 12
                                             // Cómo ajustarlo: reducir para evitar caídas abruptas; aumentar para apagados más directos.

const uint16_t MAX_STEP_UP16 = 3072;         // Qué controla: máximo incremento por tick en ramp 16‑bit (≈ MAX_STEP_UP * 256).
                                             // Cómo ajustarlo: mantener proporcional a MAX_STEP_UP.

const uint16_t MAX_STEP_DOWN16 = 2048;       // Qué controla: máximo decremento por tick en ramp 16‑bit (≈ MAX_STEP_DOWN * 256).
                                             // Cómo ajustarlo: mantener proporcional a MAX_STEP_DOWN.

};