#pragma once

#include <Arduino.h>
#include "driver/dac.h"

/*
  AcousticInjector.h

  Header con la definición de la clase y DOCUMENTACIÓN en línea
  para las constantes que ya existen en tu clase. NO se crean
  nuevas constantes ni se cambian nombres. Cada constante tiene
  una línea clara sobre qué controla, rango sugerido y cómo ajustarla.
*/

class AcousticInjector {
public:
  enum FrequencyRangeOption : uint8_t {
      RANGE_1 = 0,  // 4400 – 5100 Hz
      RANGE_2 = 1,  // 5100 – 5800 Hz
      RANGE_3 = 2,  // 5800 – 6500 Hz
      RANGE_4 = 3   // FULL RANGE
  };
  void setFrequencyRangeOption(FrequencyRangeOption option);
  FrequencyRangeOption getFrequencyRangeOption() const;

  void begin(uint8_t dacPin);
  void start(float level, float dTPSdt);
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
  void updateDecayState();           // llamados desde loop/task para recalcular envelope/resonator
  float freqGainFactor(float hz);

  bool isInDecay() const;

  // Getter público para la FSM
  bool AcousticInjector::decayFinished() const {
      return _decayFinished;
  }

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
  float _targetFrequency = 0.0f;

  uint32_t _lastUpdateMs = 0;     // timestamp de última update
  float _currentLogFreq = 0.0f;   // ln(freq) actual; inicializar en constructor

  // Constante tau (ms) para suavizado exponencial en log-domain; controla velocidad de respuesta de la rampa de frecuencia.
  // Valores típicos: 30..120 ms (menor = ataque más rápido; mayor = transición más suave).
  static constexpr float SLEW_TAU_MS = 250.0f;


  // Resolución de fase
  // PHASE_FRAC: bits fraccionales en acumulador de fase.
  // Qué controla: resolución de frecuencia en phaseStep/phaseAcc.
  // Rango sugerido: 12..20. Aumentar mejora resolución; reducir para usar menos rango.
  static constexpr uint8_t PHASE_FRAC = 16;
  static_assert((1 << PHASE_FRAC) > 0, "PHASE_FRAC ok");
  bool _active = false;

  volatile uint32_t _phaseAcc = 0;
  volatile uint32_t _phaseStep = 0;

  // Tabla seno y tamaño
  // TABLE_SIZE: número de entradas en la tabla seno usada por ISR (potencia de 2 recomendada).
  // Efecto: mayor TABLE_SIZE = mejor forma de onda; coste: más RAM y acceso más lento si muy grande.
  static constexpr uint8_t TABLE_SIZE = 64;

  // Tasas de muestreo
  // SAMPLE_RATE: tasa objetivo (Hz) para la ISR/DAC. Mantener 64000 para alta fidelidad.
  static constexpr uint32_t SAMPLE_RATE = 64000;  // 64 kHz para alta fidelidad

  // Paso de rampa para suavizar cambios en el nivel (_level).
  // RAMP_STEP: cuanto cambia _level por llamada a update().
  // Ajuste empírico: 0.005..0.02 = suave; >0.02 = respuesta más rápida.
  static constexpr float LEVEL_RAMP_STEP = 0.005f;
  float _lastTargetLevel = 0.0f;

  FrequencyRangeOption _freqOption = RANGE_3;

  float _freqMin = 3500.0f;
  float _freqMax = 6500.0f;

  // miembros a añadir en AcousticInjector (header)
  bool _forceSweep = false;
  uint32_t _sweepStartMs = 0;
  uint32_t _levelSweepStartMs = 0;
  uint32_t _preIdleStartMs = 0;


  bool _levelSweepInitDone = false; // indica que ya hicimos el sweep inicial desde la última activación

  //static constexpr uint32_t FORCE_SWEEP_TIME_MS = 260;    // 30..150 ms, 60 ms = rápido pero estable
  static constexpr float FORCE_SWEEP_START_HZ = 2000.0f;  // inicio fijo del sweep
  static constexpr float FORCE_FINAL_FREQ_MIN = 3500.0f;  // objetivo mínimo final (3.5 kHz)
  static constexpr float FORCE_FINAL_FREQ_MAX = 7000.0f;  // objetivo máximo final (7 kHz)


  // promedio de frecuencia (uso: smoothing / telemetría)
  float _avgFrequency = 0.0f;

  // RES_ZERO_THRESH16: umbral en escala 16-bit para considerar señal "casi cero".
  // RES_AMP_ZERO_THRESH: umbral de amplitud adicional para conteo de zeros; ajusta según ruido.
  const uint16_t RES_ZERO_THRESH16 = 64u; // existente
  const uint16_t RES_AMP_ZERO_THRESH = 256u; // nuevo, ajustar según tu rango

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

  // Tabla seno 0..255 para ISR rápido
  static uint8_t _sineTable[TABLE_SIZE];

  // Estado del resonador/decay
  volatile bool _inDecay = false;
  volatile bool _decayFinished = false;
  volatile uint32_t _resPhaseStep = 0; // paso de fase del resonador para la ISR
  volatile float _decayMix;              // mezcla resonador vs principal (0..1)
  uint32_t _decayDurationMs;    // default
  float _decayResFreq;       // freq resonante por defecto
  volatile uint64_t _resPhaseAcc = 0;

  float _gFast;
  float _gSustain = 0.0f;       // 0..1
  uint32_t _tFastMs;
  uint32_t _tSustainMs;
  uint8_t _zeroCount = 0;

  // Envelope/mezcla en enteros para ISR
  volatile uint16_t _decayLevelMulInt = 65535; // 0..65535 multiplicador de level
  static volatile uint16_t _decayEnvInt16;        // opción: 16-bit envelope si usas   
  static volatile uint16_t _decayMixInt16;
  float _decayControlLevel = 0.0f;
  volatile uint32_t _resTargetStep; // objetivo calculado por update
  volatile uint16_t _resAmpInt16 = 0; // 0..65535, usado por ISR
  volatile uint16_t _decayMixSnapshot16;
  // último level snapshot usado por el decay (0.0 .. 1.0)
  float _decayLastLevelSnapshot = 0.0f;

  // ----------------------------
  // constantes de diseño — explicadas y cómo ajustarlas
  // ----------------------------


  // ZERO_COUNT_TO_END: frames consecutivos bajo umbral para declarar fin del decay.
  // Ajuste práctico: 1..16. Mayor = conservador contra ruido; menor = termina más rápido.
  const uint8_t ZERO_COUNT_TO_END = 226;

  // MAX_STEP_UP / MAX_STEP_DOWN: límites por tick en la rampa 8‑bit.
  // Qué controla: velocidad máxima de subida/bajada del valor 8‑bit enviado al DAC.
  // Efecto: valores bajos = transiciones muy suaves; valores altos = respuesta más rápida, posible click.
  // Rango práctico:
  //  - Suave: 4..8
  //  - Equilibrio (por defecto): 8..16
  //  - Rápido: 16..32
  // Reglas rápidas:
  //  - Si oyes clicks al subir, baja MAX_STEP_UP.
  //  - Si la subida es demasiado lenta, sube MAX_STEP_UP en +2 hasta que sea aceptable.
  //  - Para apagados bruscos, reduce MAX_STEP_DOWN; para apagados rápidos, aumentalo.
  const uint8_t MAX_STEP_UP = 2;    // recomendado inicial: 8..16
  const uint8_t MAX_STEP_DOWN = 2;  // recomendado inicial: 8..24


  // MAX_STEP_UP16 / MAX_STEP_DOWN16: versiones 16‑bit (proporcionales a 8‑bit).
  // Implementación segura: mantener proporcionalidad exacta = valor8 * 256.
  const uint16_t MAX_STEP_UP16   = static_cast<uint16_t>(MAX_STEP_UP   * 256u);   // 8 * 256 = 2048
  const uint16_t MAX_STEP_DOWN16 = static_cast<uint16_t>(MAX_STEP_DOWN * 256u);   // 8 * 256 = 2048

  // RES_STEP_DELTA_MAX: cuánto puede cambiar el paso de fase del resonador en una sola actualización.
  // - Qué controla: la "velocidad" máxima de ajuste de la frecuencia interna del resonador.
  // - Efecto:
  //     valores bajos  -> cambios muy suaves en la frecuencia; elimina clicks pero ralentiza tracking.
  //     valores altos -> respuesta rápida a cambios de objetivo; puede introducir detune audible o clicks.
  // - Rangos sugeridos:
  //     Muy suave / barridos largos:   256 .. 1024
  //     Equilibrio (recomendado):     1024 .. 4096
  //     Rápido / transientes:         4096 .. 8192
  // - Reglas prácticas:
  //     * Si oyes clicks al cambiar freq, reduce a la mitad y prueba otra vez.
  //     * Para sweeps largos usa RES_STEP_DELTA_MAX_SLEW (p. ej. 1024).
  static constexpr uint32_t RES_STEP_DELTA_MAX_SLEW = 16u; // pruebe 1024 para sweeps largos y suaves

  static uint32_t _logLastMs;
  static const uint32_t LOG_INTERVAL_MS = 1; // intervalo telemetría; aumentar si el logging afecta audio
  float _dTPSdtEntry = 0.0f;

  // ======== FASE PRE-IDLE (Simulación de arranque turbo comprimido) ========
// Debe ir después de detectar la activación y ANTES del sweep de LEVEL

// Ajustes principales de la fase Pre-Idle (C – Wobble Idle)

// Duración total de la fase pre-idle (ms)
// 2000–6000 recomendado (2–6 segundos)
// Efecto: más tiempo = turbo “cargando” más realista antes del sweep
static constexpr uint32_t PRE_IDLE_TIME_MS = 2000;

// Nivel máximo audible durante pre-idle (0.02–0.10 típico)
// Efecto: mayor → más audible; menor → más sutil
static constexpr float PRE_IDLE_MAX = 0.001f;

// Exponente de curva inicial (1.5–4.0 recomendado)
// Efecto: mayor → arranque más silencioso y sube tarde; menor → sube más lineal
static constexpr float PRE_IDLE_CURVE_EXP = 4.0f;

// Intensidad del “wobble” (0.01–0.08 típico)
// Efecto: variaciones tipo compresión; mayores = más notoria vibración
static constexpr float PRE_IDLE_WOBBLE_STRENGTH = 0.08f;

// Velocidad del wobble (rads/seg) — 4–14 típico
// Efecto: menor = vibración lenta grave; mayor = vibración rápida
static constexpr float PRE_IDLE_WOBBLE_SPEED = 8.0f;

// --- Peak capture during BEAM ---
float _peakLevelDuringBeam = 0.0f;       // Máximo level alcanzado en BEAM (0.0 - 1.0)
float _peakFreqDuringBeam = 0.0f;        // Máxima frecuencia alcanzada en BEAM (Hz)
uint32_t _lastDecayPrintMs = 0;



};
