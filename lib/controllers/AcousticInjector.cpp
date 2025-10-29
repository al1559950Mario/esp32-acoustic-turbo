#include "AcousticInjector.h"
#include "driver/dac.h"
#include <math.h>
#include "StateMachine.h"
#include <cmath>   // para std::isfinite, std::isnan, std::isinf

uint32_t AcousticInjector::_logLastMs = 0;
// Definiciones de miembros static declarados en AcousticInjector.h
volatile uint16_t AcousticInjector::_decayEnvInt16  = 0;
volatile uint16_t AcousticInjector::_decayMixInt16  = 0;

// constantes de easing exposicional precalculadas
static const float a       = 0.5f;      
static const float expAmin = 1.0f;      
static const float expA    = expf(a); 

AcousticInjector* AcousticInjector::_instance = nullptr;

// Nuevo:
uint8_t AcousticInjector::_sineTable[AcousticInjector::TABLE_SIZE] = {
  128, 140, 153, 165, 177, 188, 198, 207,
  215, 222, 227, 231, 234, 235, 235, 234,
  231, 227, 222, 215, 207, 198, 188, 177,
  165, 153, 140, 128, 115, 102,  90,  78,
   67,  57,  48,  40,  33,  28,  24,  21,
   20,  20,  21,  24,  28,  33,  40,  48,
   57,  67,  78,  90, 102, 115, 128, 140,
  153, 165, 177, 188, 198, 207, 215, 222
};

void AcousticInjector::begin(uint8_t dacPin) {
  _instance = this;

  _dacPin = dacPin;

  _dacChannel = (_dacPin == 25) ? DAC_CHANNEL_1 : DAC_CHANNEL_2;
  dac_output_enable(_dacChannel);

  _level = 0.0f;
  _targetLevel = 0.0f;
  _index = 0;
  _levelInt = 0;

  // construir tabla seno en RAM
  for (int i = 0; i < TABLE_SIZE; ++i) {
    float s = sinf(2.0f * PI * (float)i / (float)TABLE_SIZE);
    int v = (int)roundf(128.0f + s * 127.0f);
    v = constrain(v, 0, 255);
    _sineTable[i] = (uint8_t)v;
  }

  // configurar timer a sampleRate fijo (counts = us porque prescaler 80 -> 1MHz)
  _timer = timerBegin(2, 80, true); // Timer 2, 1 MHz
  timerAttachInterrupt(_timer, &AcousticInjector::onTimer, true);

  float sampleRate = SAMPLE_RATE; // p.ej. 64kHz
  float periodPerSample = 1e6f / sampleRate; // μs
  timerAlarmWrite(_timer, static_cast<uint32_t>(periodPerSample), true);
  timerAlarmDisable(_timer);

  // init phase values
  _phaseAcc = 0;
  _phaseStep = 0;
  _levelInt = (uint8_t)(_level * 255.0f);

}

void AcousticInjector::start(float level, float dTPSdt) {
  // 1) Reinicio total (fase, nivel, índices)
  resetInternal();
  _active      = true;
  _targetLevel = constrain(level, 0.0f, 1.0f);
  _level       = _targetLevel;
  _levelInt    = uint8_t(_level * 255.0f);

  // 2) Configurar inicio y objetivo de frecuencia
  const float startFreq  = 2000.0f;
  float       targetFreq = _targetFrequency;

  _dTPSdtEntry = constrain(dTPSdt, -10.0f, 10.0f); // proteger

  // Si el caller no inicializó _targetFrequency, usar un fallback razonable
  if (!(std::isfinite(targetFreq) && targetFreq > 0.0f)) {
    targetFreq = _decayResFreq; // fallback a la freq del resonador preconfigurada
    _targetFrequency = targetFreq;
  }

  _currentFrequency = startFreq;
  _targetFrequency  = targetFreq;

  // 3) Calcular primer phaseStep en startFreq
  updateWaveFrequency(_currentFrequency);

  // 4) Arrancar timer/DAC y seed de decay de forma atómica
  timerWrite(_timer, 0);
  delay(1);
  timerAlarmEnable(_timer);

  noInterrupts();
  // seed de envelope desde nivel float convertido a 16-bit
  _decayEnvInt16     = uint16_t(constrain(_level * 65535.0f, 0.0f, 65535.0f));
  _decayControlLevel = _level;
  // asegurar mix inicial > 0 si _decayMix tiene valor
  _decayMixInt16     = uint16_t(constrain(_decayMix * 65535.0f, 1.0f, 65535.0f));
  interrupts();
}

void AcousticInjector::stop() {
  resetInternal();
  _active = false;
  timerAlarmDisable(_timer);
  dac_output_voltage(_dacChannel, 128);
  _level = 0.0f;
  _targetLevel = 0.0f;
  _levelInt = 0;
}

void AcousticInjector::setLevel(float level) {
  level = constrain(level, 0.0f, 1.0f);
  float a = 0.5f; // controla la aceleración al final
  float curvedLevel = (exp(a * level) - 1.0f) / (exp(a) - 1.0f);
  _targetLevel = curvedLevel;
}

void AcousticInjector::update() {
  if (!_active) return;
  // Escalar FREQ_RAMP_STEP dinámicamente según dTPSdt
  float slopeNorm = constrain((_dTPSdtEntry + 10.0f) / 20.0f, 0.0f, 1.0f);
  float rampStepHz = 5.0f + 5.0f * slopeNorm; // lineal 5–10 Hz

  // --- Frecuencia ---
  float diffF = _targetFrequency - _currentFrequency;
  if (fabs(diffF) < rampStepHz) {
    _currentFrequency = _targetFrequency;
  } else {
    _currentFrequency += (diffF > 0 ? rampStepHz : -rampStepHz);
  }

  updateWaveFrequency(_currentFrequency);

  // --- Nivel (tu rampa original) ---
  float diffL = _targetLevel - _level;
  if (fabs(diffL) < RAMP_STEP) {
    _level = _targetLevel;
  } else {
    _level += (diffL > 0 ? RAMP_STEP : -RAMP_STEP);
  }
  _levelInt = uint8_t(_level * 255);

  float A = constrain(_level, 0.0f, 1.0f);   // amplitud 0..1
  float energy = A * A;                      // proxy energía 0..1
  // opcional: ajustar por frecuencia (si no quieres corrección, usa G = 1.0f)
  float G = freqGainFactor(_currentFrequency); // 0..~1
  float effective = energy * G;               // energía corregida
  // shaping perceptual opcional (suaviza respuesta): uso sqrt para que pequeños A "pesen" más
  float decayMix = effective;         // o decayMix = effective;
  // suavizado/exponential moving average (mantener estable entre updates)
  const float alpha = 0.15f;
  _decayMix = _decayMix * (1.0f - alpha) + decayMix * alpha;
  // escribir 16-bit para uso en startDecay/updateDecayState
  noInterrupts();
  _decayMixInt16 = uint16_t(constrain(_decayMix * 65535.0f, 0.0f, 65535.0f));
  interrupts();
}

void IRAM_ATTR AcousticInjector::onTimer() {
  if (!_instance) return;

  // avance de fase principal (fixed point)
  _instance->_phaseAcc += _instance->_phaseStep;

  // Índice entero y fracción para interpolación principal
  uint32_t idx = (_instance->_phaseAcc >> PHASE_FRAC) & (TABLE_SIZE - 1);
  uint32_t nextIdx = (idx + 1) & (TABLE_SIZE - 1);
  uint32_t frac = _instance->_phaseAcc & ((1ULL << PHASE_FRAC) - 1);

  uint8_t sample1 = _instance->_sineTable[idx];
  uint8_t sample2 = _instance->_sineTable[nextIdx];
  int32_t delta = int32_t(sample2) - int32_t(sample1);
  int32_t interp = int32_t(sample1) + int32_t((delta * frac) >> PHASE_FRAC);

  // Salidas y estado compartido: leer copias atómicas de volátiles
  noInterrupts();
  uint16_t env16_local     = _instance->_decayEnvInt16;
  uint16_t mix16_local     = _instance->_decayMixInt16;
  uint16_t levelMul16_local= _instance->_decayLevelMulInt;
  uint8_t  level8_local    = _instance->_levelInt;      // 0..255 versión ISR
  uint32_t resPhaseAcc_local = _instance->_resPhaseAcc;
  uint32_t resPhaseStep_local= _instance->_resPhaseStep;
  dac_channel_t dacCh_local   = _instance->_dacChannel;  
  bool     inDecay_local   = _instance->_inDecay;
  interrupts();

  // Nivel principal (signed centered) y salida inmediata si no hay decay
  int32_t centered_p = interp - 128;
  if (!inDecay_local) {
    int32_t principal = 128 + ((centered_p * int32_t(level8_local)) >> 8);
    if (principal < 0) principal = 0;
    if (principal > 255) principal = 255;
    uint8_t output = uint8_t(principal);
    dac_output_voltage(dacCh_local, output);
    _instance->_lastDACValue = output;
    return;
  }

  // ---------- DECAY activo: resonador y mezcla (enteros, ISR friendly) ----------
  // Avanzar fase del resonador en copia local y escribir de vuelta atómicamente
  resPhaseAcc_local += resPhaseStep_local;
  uint32_t newResPhaseAcc = resPhaseAcc_local;
  noInterrupts();
  _instance->_resPhaseAcc = newResPhaseAcc;
  interrupts();

  // Índices y fracción para interpolación del resonador
  uint32_t rIdx = (resPhaseAcc_local >> PHASE_FRAC) & (TABLE_SIZE - 1);
  uint32_t rNext = (rIdx + 1) & (TABLE_SIZE - 1);
  uint32_t rFrac = resPhaseAcc_local & ((1ULL << PHASE_FRAC) - 1);
  uint8_t r1 = _instance->_sineTable[rIdx];
  uint8_t r2 = _instance->_sineTable[rNext];
  int32_t rDelta = int32_t(r2) - int32_t(r1);
  int32_t rInterp = int32_t(r1) + int32_t((rDelta * rFrac) >> PHASE_FRAC);
  int32_t centered_r = rInterp - 128;
  // --- start replacement: use precomputed resAmp and additive mix ---
  // Leer resAmp16 precomputado (0..65535) y mapear a ganancia 0..255
  uint16_t resAmp16_local = _instance->_resAmpInt16; // volatile precomputed in updateDecayState
  uint8_t resGain8 = uint8_t((uint32_t(resAmp16_local) * 255u) >> 16u);

  // Opcional: shaping perceptual (comentar si quieres lineal)
  // uint8_t resGainShaped = uint8_t(powf(float(resGain8) / 255.0f, 0.6f) * 255.0f);
  // Usar resGainShaped en lugar de resGain8 si aplicas shaping

  // Resonador escalado (signed) usando ganancia 0..255
  int32_t resonatorScaled = (int32_t(centered_r) * int32_t(resGain8)) >> 8; // signed

  // Principal escalado (igual que antes)
  int32_t scaledLevel = (int32_t(level8_local) * int32_t(levelMul16_local)) >> 16; // 0..255
  if (scaledLevel < 0) scaledLevel = 0;
  if (scaledLevel > 255) scaledLevel = 255;
  int32_t principalScaled = (int32_t(centered_p) * scaledLevel) >> 8; // signed

  // Mezcla aditiva: no atenuar el principal con resAmp
  int32_t mixed_signed = principalScaled + resonatorScaled;

  // Remap y clip una sola vez
  int32_t out = mixed_signed + 128;
  if (out < 0) out = 0;
  if (out > 255) out = 255;
  uint8_t output = uint8_t(out);

  dac_output_voltage(dacCh_local, output);
  _instance->_lastDACValue = output;
  // --- end replacement ---
}

void IRAM_ATTR AcousticInjector::applyPendingDAC() {
  uint8_t raw = _sineTable[_index];
  int16_t delta = (int16_t)raw - 128;
  int16_t modulated = 128 + ((delta * _levelInt) >> 8);
  uint8_t output = constrain(modulated, 0, 255);
  dac_output_voltage(_dacChannel, output);
  _lastDACValue = output;

  _index = (_index + 1) % TABLE_SIZE;
}

uint8_t AcousticInjector::getCurrentDAC() const {
  return _lastDACValue;
}

bool AcousticInjector::isActive() const {
  return _active;
}

void AcousticInjector::test() {
  Serial.println(F("🔊 Prueba acústica iniciada..."));
  start(1.0f, 0.0f);

  for (int i = 0; i < 250; i++) {
    update();
    delay(20);
    if (i % 50 == 0) {
      Serial.printf("Nivel actual: %.2f\n", _level);
    }
  }

  stop();
  Serial.println(F("✅ Prueba finalizada."));
}

void AcousticInjector::emitResonant(float level) {
  Serial.println(F("🌼 Emitiendo señal resonante por fase acumulada (5s)..."));

  const float freq = 6370.0f;
  const float amplitude = 127.0f * constrain(level, 0.0f, 1.0f);
  const uint8_t bias = 128;
  const float sampleRate = 64000.0f;
  const float dPhase = 2.0f * PI * freq / sampleRate;

  const int sampleCount = (int)(5.0f * sampleRate);
  float phase = 0.0f;

  for (int i = 0; i < sampleCount; ++i) {
    float value = bias + amplitude * sinf(phase);
    dac_output_voltage(_dacChannel, constrain((int)value, 0, 255));
    phase += dPhase;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;
    delayMicroseconds(15);
  }

  dac_output_voltage(_dacChannel, bias);
  Serial.println(F("✅ Señal por fase acumulada finalizada."));
}

void AcousticInjector::testSimple() {
  Serial.println(F("🔊 Test simple iniciado"));
  start(1.0f, 0.0f);

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    update();
    delay(20);
  }

  stop();
  Serial.println(F("✅ Test simple finalizado"));
}

float AcousticInjector::mapLoadToWaveFrequency(float level) {
  level = constrain(level, 0.0f, 1.0f);
  float curvedLevel = (expf(a * level) - expAmin) / (expA - expAmin);

    // Normalizar: -10 a +10 → 0 a 1
  float slopeNorm = constrain((_dTPSdtEntry + 10.0f) / 20.0f, 0.0f, 1.0f);

  // Interpolar entre 3500 y 5000 Hz
  float dynamicFreqMin = 3500.0f + (5000.0f - 3500.0f) * slopeNorm;

  float logMin  = logf(dynamicFreqMin);
  float logMax  = logf(_freqMax);
  float logFreq = logMin + curvedLevel * (logMax - logMin);
  return expf(logFreq);
}

void AcousticInjector::updateWaveFrequency(float freqHz) {
    if (!_timer) return;

    const float sr = SAMPLE_RATE;  // p.ej. 32000
    _currentFrequency = freqHz;

    // Acumulador y contador estáticos o miembros de clase
    static float _freqSum = 0.0f;
    static uint32_t _freqCount = 0;

    // Actualiza promedio
    _freqSum += freqHz;
    _freqCount++;
    _avgFrequency = _freqSum / _freqCount;  // <-- variable miembro de clase

    // cálculo fixed-point para phaseStep
    double step = freqHz * TABLE_SIZE * (1ULL << PHASE_FRAC) / sr;
    uint32_t newStep = (step < 1.0) ? 1 : uint32_t(round(step));

    _phaseStep = newStep;

    // reprograma periodo de timer a sample-period igual
    float periodUs = 1e6f / sr;
    timerAlarmWrite(_timer, uint32_t(periodUs), true);
}

void AcousticInjector::setFrequencyRangeOption(FrequencyRangeOption option) {
    _freqOption = option;

    switch(option) {
        case RANGE_1:
            _freqMin = 4400.0f;
            _freqMax = 5100.0f;
            break;
        case RANGE_2:
            _freqMin = 5100.0f;
            _freqMax = 5800.0f;
            break;
        case RANGE_3:
            _freqMin = 5800.0f;
            _freqMax = 6500.0f;
            break;
        case RANGE_4:
            _freqMin = 2000.0f;
            _freqMax = 6500.0f;
            break;            
        default:
            _freqMin = 5800.0f;
            _freqMax = 6500.0f;
            break;
    }
}

AcousticInjector::FrequencyRangeOption AcousticInjector::getFrequencyRangeOption() const {
  return _freqOption;
}

void AcousticInjector::startDecay(uint32_t now) {
  // snapshots locales primero (sin bloquear largo tiempo)
  uint16_t env_snapshot;
  uint16_t mix_snapshot;
  uint32_t phaseAcc_snapshot;
  uint32_t phaseStep_snapshot;

  // leer estado actual de forma segura
  noInterrupts();
  env_snapshot = uint16_t(constrain(_level * 65535.0f, 0.0f, 65535.0f));
  // preferimos la mezcla precomputada si ya existe; si no, calculamos a partir de level y G(f)
  uint16_t decayMixInt16_local = _decayMixInt16; // puede ser 0
  phaseAcc_snapshot = _resPhaseAcc;
  phaseStep_snapshot = _resPhaseStep;
  // marcar entrada en decay
  _inDecay = true;
  _decayStartMillis = now;
  _zeroCount = 0;
  interrupts();

  // escribir snapshot atómico en variables usadas por decay y precompute de amplitude inicial
  noInterrupts();
  _decayEnvInt16 = env_snapshot;
  _decayMixSnapshot16 = decayMixInt16_local;
  _resPhaseAcc = phaseAcc_snapshot;   // conservar fase
  _resPhaseStep = phaseStep_snapshot; // congelar paso de freq al inicio
  // precompute resAmp initial = (mix * env) >> 16
  _resAmpInt16 = uint16_t((uint32_t(decayMixInt16_local) * uint32_t(env_snapshot)) >> 16u);
  // asegurar multiplicador de nivel por defecto
  _decayLevelMulInt = uint16_t(65535u);
  interrupts();
  // justo después de interrupts() final en startDecay
Serial.printf("DBG:startDecay t=%lu env=%u mix=%u resAmp=%u phaseStep=%u phaseAcc=%u\n",
              now,
              env_snapshot,
              decayMixInt16_local,
              _resAmpInt16,
              phaseStep_snapshot,
              phaseAcc_snapshot);

}

void AcousticInjector::setDecayParameters(uint32_t durationMs, float avgTPSLevel,
                                         float gFast, float gSustain, uint32_t tFastMs, uint32_t tSustainMs) {

  _decayDurationMs = max<uint32_t>(1u, durationMs);

  // mezcla base del resonador en float
  float mix = 0.2f + 0.6f * powf(avgTPSLevel, 1.2f);
  _decayMix = constrain(mix, 0.0f, 1.0f);

  _decayResFreq = _currentFrequency;

  // guardar gains y tiempos
  _gFast = constrain(gFast, 0.0f, 1.0f);
  _gSustain = constrain(gSustain, 0.0f, 1.0f);
  _tFastMs = max<uint32_t>(5u, tFastMs);
  _tSustainMs = max<uint32_t>(10u, tSustainMs);

  // precomputar resonator step y escribir atómicamente
  const float sr = float(SAMPLE_RATE);
  double step = _decayResFreq * TABLE_SIZE * (1ULL << PHASE_FRAC) / sr;
  uint32_t newStep = (step < 1.0) ? 1u : uint32_t(round(step));

  noInterrupts();
  _resPhaseStep = newStep;
  // actualizar la versión en 16-bit de mix para la ISR
  _decayMixInt16 = uint16_t(constrain(_decayMix * 65535.0f, 0.0f, 65535.0f));
  interrupts();
  // justo después de interrupts() final en setDecayParameters
uint16_t mix16 = uint16_t(constrain(_decayMix * 65535.0f, 0.0f, 65535.0f));
Serial.printf("DBG:setDecay dur=%u TPSf=%0.3f mix=%0.4f mix16=%u freq=%0.1f newStep=%u\n",
              _decayDurationMs,
              avgTPSLevel,
              _decayMix,
              mix16,
              _decayResFreq,
              newStep);

}

void AcousticInjector::updateDecayState() {
  if (!_inDecay) return;

  // --- COPIAR AL INICIO TODO EL ESTADO COMPARTIDO ATÓMICAMENTE ---
  noInterrupts();
  uint16_t env16_copy       = _decayEnvInt16;
  uint16_t mix16_copy       = _decayMixInt16;
  uint32_t curStepAtomic    = _resPhaseStep;
  uint32_t decayDur_copy    = _decayDurationMs;
  bool     inDecay_copy     = _inDecay;
  interrupts();
  /*
  // después de interrupts() que siguen a la copia inicial
  Serial.printf("DBG:st_cp env=%u mix=%u curStep=%u dur=%u inDecay=%d\n",
                env16_copy, mix16_copy, curStepAtomic, decayDur_copy, inDecay_copy);

  */
  

  if (!inDecay_copy) return;

  // --- Reemplazo: usar decay control level para romper punto fijo ---
  // _decayControlLevel fue guardado en setDecayParameters (0..1)
  // permitimos una pequeña mezcla con env para adaptar si hace falta
  const float CONTROL_WEIGHT = 0.9f; // 0.9 -> 90% control, 10% env feedback
  float envF = float(env16_copy) / 65535.0f; // 0..1 (copia local)
  float controlF;
  // leer control de forma segura (noInterrupts ya fue usado arriba; asumimos copia local no disponible aquí)
  // _decayControlLevel es float y fue escrito en setDecayParameters con interrupts alrededor
  controlF = constrain(_decayControlLevel, 0.0f, 1.0f);

  // clipped ahora una mezcla que favorece el control inicial para evitar punto fijo
  float clipped = CONTROL_WEIGHT * controlF + (1.0f - CONTROL_WEIGHT) * envF;
  // asegurar rango
  clipped = constrain(clipped, 0.0f, 1.0f);

  // --- Reemplazo: usar _decayResFreq como top y un piso mínimo para la rampa ---
  const float MIN_FREQ_HZ = 2000.0f; // frecuencia mínima permitida en el sweep (ajustable)
  // _decayResFreq fue fijada en startDecay / setDecayParameters como la freq al inicio del BEAM
  float topFreqHz = _decayResFreq;

  // calcular piso: usamos un floor fijo o un valor derivado (ej. freqMin) que nunca supere el top
  float freqFloorHz = MIN_FREQ_HZ;
  // asegurar que el piso no sea mayor que el top
  if (freqFloorHz > topFreqHz) freqFloorHz = topFreqHz;

  // ahora targetFreqHz interpola entre top (clipped==1) y floor (clipped==0)
  float targetFreqHz = clipped * topFreqHz + (1.0f - clipped) * freqFloorHz;

  // convertir targetFreqHz a step entero y forzar mínimo en steps
  const double sr = double(SAMPLE_RATE);
  double stepFloat = double(targetFreqHz) * double(TABLE_SIZE) * double(1ULL << PHASE_FRAC) / sr;
  uint32_t computedTarget = (stepFloat < 1.0) ? 1u : uint32_t(round(stepFloat));

  // forzar computedTarget >= minStep correspondiente a MIN_FREQ_HZ
  uint32_t minStep = uint32_t(round(double(MIN_FREQ_HZ) * double(TABLE_SIZE) * double(1ULL << PHASE_FRAC) / sr));
  if (computedTarget < minStep) computedTarget = minStep;

  // --- inicio parche rampa dinámica (reemplaza la sección anterior) ---
  // target tal cual (permitir descenso); usar computedTarget directamente
  uint32_t targetStep = computedTarget;

  // calcular diff absoluto y decidir delta dinámico según tiempo de sweep
  uint32_t nextStep;
  if (curStepAtomic == targetStep) {
    nextStep = curStepAtomic;
  } else {
    // distancia a recorrer
    uint32_t diff = (curStepAtomic > targetStep) ? (curStepAtomic - targetStep) : (targetStep - curStepAtomic);

  // Parámetros para sweep más largo
  const uint32_t MIN_SWEEP_MS = 200u;     // mínimo 200 ms
  const uint32_t MAX_SWEEP_MS = 5000u;    // hasta 5 s para sweeps largos
  const uint32_t CONTROL_PERIOD_MS = 20u; // periodo estimado entre llamadas a updateDecayState

  // multiplicador para alargar respecto a decayDur_copy (usar >1 para alargar)
  const float DECAY_TO_SWEEP_MULT = 3.5f; // sweep = 150% de decayDur_copy por defecto

  // energía influye menos para no acelerar demasiado cuando envF es alto
  const float ENERGY_K = 0.8f;

  // calcular base a partir del decayDuration y el multiplicador
  uint32_t baseSweep = uint32_t(float(decayDur_copy) * DECAY_TO_SWEEP_MULT);
  if (baseSweep < MIN_SWEEP_MS) baseSweep = MIN_SWEEP_MS;

  // modula ligeramente por energía (cuando hay mucha energía, no acelerar demasiado)
  uint32_t sweepMs = uint32_t(float(baseSweep) / (1.0f + ENERGY_K * envF));

  // clamp a rango seguro
  if (sweepMs < MIN_SWEEP_MS) sweepMs = MIN_SWEEP_MS;
  if (sweepMs > MAX_SWEEP_MS) sweepMs = MAX_SWEEP_MS;

  // Estimar actualizaciones de control (más updates => pasos más pequeños)
  uint32_t updatesEstimate = max<uint32_t>(1u, sweepMs / CONTROL_PERIOD_MS);

  // calcular stepDelta dinámico y limitarlo por RES_STEP_DELTA_MAX_SLEW (ajústalo pequeño)
  uint32_t stepDeltaDynamic = (diff / updatesEstimate) + 1u;
  uint32_t stepDelta = (stepDeltaDynamic > RES_STEP_DELTA_MAX_SLEW) ? RES_STEP_DELTA_MAX_SLEW : stepDeltaDynamic;


    // aplicar la dirección correcta (bajar o subir según target)
    if (curStepAtomic > targetStep) {
      // decrecer hacia target
      if (stepDelta > diff) stepDelta = diff;
      nextStep = curStepAtomic - stepDelta;
    } else {
      // aumentar hacia target
      if (stepDelta > diff) stepDelta = diff;
      nextStep = curStepAtomic + stepDelta;
    }
    /*
      // antes del noInterrupts() que escribe _resPhaseStep/_resTargetStep
    Serial.printf("DBG:step cur=%u tgt=%u next=%u diff=%u sweepMs=%u upEst=%u delta=%u\n",
                  curStepAtomic, targetStep, nextStep,
                  (curStepAtomic>targetStep)?(curStepAtomic-targetStep):(targetStep-curStepAtomic),
                  sweepMs, updatesEstimate, stepDelta);

    */
    
  }

  // escribir nuevo paso y objetivo atómicamente
  noInterrupts();
  _resPhaseStep  = nextStep;
  _resTargetStep = targetStep;
  _currentFrequency = targetFreqHz; // monitoriza el objetivo float
  interrupts();

  // FORZAR mix mínimo si env tiene energía (usa la copia local)
  const uint16_t ENV_THRESHOLD = 4096u;
  const uint16_t MIX_MIN = 8192u;
  if (mix16_copy == 0 && env16_copy > ENV_THRESHOLD) {
    noInterrupts();
    _decayMixInt16 = MIX_MIN;
    interrupts();
    mix16_copy = MIX_MIN; // mantener coherencia local
  }
  // justo después de asignar mix16_copy = MIX_MIN (o tras el interrupts() que lo escribe)
  //Serial.printf("DBG:mix_fix mix16=%u env16=%u\n", mix16_copy, env16_copy);

  // --- fin parche rampa dinámica ---


  // calcular multiplicador del principal desde clipped (una sola declaración)
  uint16_t levelMulTarget16 = uint16_t(constrain((1.0f - clipped) * 65535.0f, 0.0f, 65535.0f));
  uint8_t  levelInt8Target  = uint8_t(levelMulTarget16 >> 8u); // 16->8 scaling
  if (clipped <= 0.01f) levelInt8Target = 0;


  noInterrupts();
  _decayLevelMulInt = levelMulTarget16; // 16-bit para cálculos fuera/ISR
  _levelInt        = levelInt8Target;   // 8-bit usado por la ISR (volatile uint8_t)
  _level = clipped;
  interrupts();

  /*
  // después del interrupts() que actualiza _decayLevelMulInt/_levelInt/_level
  Serial.printf("DBG:level mul16=%u level8=%u levelF=%.3f\n",
                levelMulTarget16, levelInt8Target, clipped);

  */
  
  
  // rampa 16-bit de la envolvente sin estancamiento (usar copia coherente para target16)
  uint16_t target16 = uint16_t(constrain(clipped * 65535.0f, 0.0f, 65535.0f));

  noInterrupts();
  uint16_t cur16 = _decayEnvInt16;
  interrupts();

  uint16_t next16 = cur16;
  const uint16_t MAX_STEP_UP16 = 1024u;
  const uint16_t MAX_STEP_DOWN16 = 4096u;
  if (target16 > cur16) {
    uint16_t diff = target16 - cur16;
    uint16_t step = (diff > MAX_STEP_UP16) ? MAX_STEP_UP16 : diff;
    next16 = cur16 + step;
  } else if (target16 < cur16) {
    uint16_t diff = cur16 - target16;
    uint16_t step = (diff > MAX_STEP_DOWN16) ? MAX_STEP_DOWN16 : diff;
    // forzar decremento mínimo para evitar plateaus
    const uint16_t MIN_STEP_DOWN16_FORCE = 512u;
    if (step < MIN_STEP_DOWN16_FORCE && diff > MIN_STEP_DOWN16_FORCE) step = MIN_STEP_DOWN16_FORCE;
    next16 = cur16 - step;
  }

    // Precompute resAmp for ISR: resAmp = (mix * env) >> 16
  uint16_t resAmpInt16 = uint16_t((uint32_t(mix16_copy) * uint32_t(next16)) >> 16u);

  /*
  // justo antes del noInterrupts() que guarda _decayEnvInt16 y _resAmpInt16
  Serial.printf("DBG:env next=%u cur=%u resAmp_calc=%u mix16=%u mix_snap=%u\n",
                next16, cur16, resAmpInt16, mix16_copy, _decayMixSnapshot16);

  */
  
  noInterrupts();
  _decayEnvInt16 = next16;
  // usa la mezcla congelada tomada en startDecay para evitar reexcitación en vivo
  uint16_t mix_for_calc = _decayMixSnapshot16 ? _decayMixSnapshot16 : _decayMixInt16;
  _resAmpInt16 = uint16_t((uint32_t(mix_for_calc) * uint32_t(next16)) >> 16u);
  interrupts();

bool levelNearZero = (next16 <= RES_ZERO_THRESH16);
bool ampNearZero   = (_resAmpInt16 <= RES_AMP_ZERO_THRESH);
if (levelNearZero || ampNearZero) { _zeroCount = min<uint8_t>(_zeroCount + 1, 255); }
else { _zeroCount = 0; }


  // logging periódico (fuera de sección crítica)
  uint32_t nowLog = millis();
  if ((nowLog - _logLastMs) >= LOG_INTERVAL_MS) {
    _logLastMs = nowLog;
    noInterrupts();
    uint32_t env16_log    = _decayEnvInt16;
    uint32_t mix16_log    = _decayMixInt16;
    uint32_t levelMulLog  = _decayLevelMulInt;
    uint32_t resStepLog   = _resPhaseStep;
    uint32_t mainStepLog  = _phaseStep;
    uint16_t lastDAC      = _lastDACValue;
    interrupts();

    float clippedF = float(env16_log) / 65535.0f;
    uint64_t tmp64_log = uint64_t(mix16_log) * uint64_t(env16_log);
    uint32_t resAmp16 = uint32_t((tmp64_log * 65535u) >> 32u);
    double resHz = double(resStepLog) * double(SAMPLE_RATE) / double(TABLE_SIZE * (1ULL << PHASE_FRAC));
    double mainHz = double(mainStepLog) * double(SAMPLE_RATE) / double(TABLE_SIZE * (1ULL << PHASE_FRAC));
    
    // reemplaza o añade a la Serial.printf existente en logging periódico
    Serial.printf("t=%lu clipped=%.3f level8=%u resHz=%.1f resAmp=%u targetHz=%.1f zeroCnt=%u mainHz=%.1f mainStep=%u\n",
      nowLog, clippedF, _levelInt, resHz, resAmp16, targetFreqHz, _zeroCount, mainHz, mainStepLog);

  
    
  }

  // finalizar decay si nivel baja
  if (levelNearZero || (_zeroCount >= ZERO_COUNT_TO_END)) {
    noInterrupts();
    _inDecay = false;
    _decayEnvInt16 = 0;
    _zeroCount = 0;
    interrupts();
    return;
  }
}

// simple frequency correction  (values 0..1). Ajustá según pruebas.
float AcousticInjector::freqGainFactor(float hz) {
  // tabla de ejemplo: (f, G)
  // por defecto asumimos ligera mayor eficiencia en medio ~3.5kHz
  const float f1 = 2000.0f, g1 = 0.85f;
  const float f2 = 3500.0f, g2 = 1.0f;
  const float f3 = 6500.0f, g3 = 0.75f;

  if (hz <= f1) return g1;
  if (hz >= f3) return g3;
  if (hz <= f2) {
    // interp f1..f2
    float t = (hz - f1) / (f2 - f1);
    return g1 + t * (g2 - g1);
  } else {
    // interp f2..f3
    float t = (hz - f2) / (f3 - f2);
    return g2 + t * (g3 - g2);
  }
}

bool AcousticInjector::isInDecay() const {
  return _inDecay;
}
