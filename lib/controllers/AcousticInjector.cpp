#include "AcousticInjector.h"
#include "driver/dac.h"
#include <math.h>
#include "StateMachine.h"

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

  float sampleRate = DEFAULT_SAMPLE_RATE; // p.ej. 64kHz
  float periodPerSample = 1e6f / sampleRate; // μs
  timerAlarmWrite(_timer, static_cast<uint32_t>(periodPerSample), true);
  timerAlarmDisable(_timer);

  // init phase values
  _phaseAcc = 0;
  _phaseStep = 0;
  _levelInt = (uint8_t)(_level * 255.0f);

}


void AcousticInjector::start(float level) {
  // 1) Reinicio total (fase, nivel, índices…)
  resetInternal();
  _active      = true;
  _targetLevel = constrain(level, 0.0f, 1.0f);
  _level       = _targetLevel;
  _levelInt    = uint8_t(_level * 255);

  // 2) Configurar “punto de partida” y “objetivo”
  const float startFreq  = 2000.0f;               // arranque en 2 kHz
  float       targetFreq = _targetFrequency;      // debe venir de mapLoadToWaveFrequency()

  _currentFrequency = startFreq;
  _targetFrequency  = targetFreq;

  // 3) Calcular primer phaseStep en 2 kHz
  updateWaveFrequency(_currentFrequency);

  // 4) Arrancar timer/DAC
  timerWrite(_timer, 0);
  delay(1);
  timerAlarmEnable(_timer);
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
  float a = 8.0f; // controla la aceleración al final
  float curvedLevel = (exp(a * level) - 1.0f) / (exp(a) - 1.0f);
  _targetLevel = curvedLevel;
}

void AcousticInjector::update() {
  if (!_active) return;

  // --- Frecuencia ---
  if (_currentFrequency < _targetFrequency) {
    _currentFrequency = min(_currentFrequency + FREQ_RAMP_STEP,
                            _targetFrequency);
    updateWaveFrequency(_currentFrequency);
  }

  // --- Nivel (tu rampa original) ---
  float diffL = _targetLevel - _level;
  if (fabs(diffL) < RAMP_STEP) {
    _level = _targetLevel;
  } else {
    _level += (diffL > 0 ? RAMP_STEP : -RAMP_STEP);
  }
  _levelInt = uint8_t(_level * 255);
}




void IRAM_ATTR AcousticInjector::onTimer() {
  if (!_instance) return;

  // avance de fase principal
  _instance->_phaseAcc += _instance->_phaseStep;

  // Índice entero y siguiente para interpolar (principal)
  uint32_t idx = (_instance->_phaseAcc >> PHASE_FRAC) & (TABLE_SIZE - 1);
  uint32_t nextIdx = (idx + 1) & (TABLE_SIZE - 1);

  // Fracción para interpolar
  uint32_t frac = _instance->_phaseAcc & ((1ULL << PHASE_FRAC) - 1);

  uint8_t sample1 = _instance->_sineTable[idx];
  uint8_t sample2 = _instance->_sineTable[nextIdx];

  int16_t delta = (int16_t)sample2 - (int16_t)sample1;
  uint16_t interp = (uint16_t)sample1 + ((delta * frac) >> PHASE_FRAC);

  // Modulación de nivel (0-255) — tu lógica intacta
  int16_t centered = (int16_t)interp - 128;
  int16_t principal = 128 + ((centered * _instance->_levelInt) >> 8);

  // Si no hay decay, comportamiento original
  if (!_instance->_inDecay) {
    uint8_t output = (uint8_t)constrain(principal, 0, 255);
    dac_output_voltage(_instance->_dacChannel, output);
    _instance->_lastDACValue = output;
    return;
  }

  // ---------- DECAY activo: calcular resonador y mezclar ----------
  // avanzar fase del resonador usando step precomputado
  _instance->_resPhaseAcc += _instance->_resPhaseStep;

  uint32_t rIdx = (_instance->_resPhaseAcc >> PHASE_FRAC) & (TABLE_SIZE - 1);
  uint32_t rNext = (rIdx + 1) & (TABLE_SIZE - 1);
  uint32_t rFrac = _instance->_resPhaseAcc & ((1ULL << PHASE_FRAC) - 1);
  uint8_t r1 = _instance->_sineTable[rIdx];
  uint8_t r2 = _instance->_sineTable[rNext];
  int16_t rDelta = (int16_t)r2 - (int16_t)r1;
  uint16_t rInterp = (uint16_t)r1 + ((rDelta * rFrac) >> PHASE_FRAC);
  int16_t centeredR = (int16_t)rInterp - 128;

  // leer envelope atómico (0..255) y normalizar
  uint8_t env = _instance->_decayEnvInt;
  float envNorm = float(env) / 255.0f;

  // amplitud del resonador en enteros 0..255 proporcional a envNorm y decayMix
  uint16_t resAmpInt = uint16_t(envNorm * (_instance->_decayMix * 255.0f));
  int16_t resonatorVal = 128 + ((centeredR * int32_t(resAmpInt)) >> 8);

  // atenuar la señal principal según mix * envNorm (no la borramos)
  float principalAttenuation = 1.0f - (_instance->_decayMix * envNorm);
  int16_t principalMixed = 128 + int16_t(((centered * _instance->_levelInt) >> 8) * principalAttenuation);

  // mezclar y saturar
  int32_t mixed = int32_t(principalMixed) + int32_t(resonatorVal) - 128;
  uint8_t output = (uint8_t)constrain(mixed, 0, 255);

  dac_output_voltage(_instance->_dacChannel, output);
  _instance->_lastDACValue = output;
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
  start(1.0f);

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
  start(1.0f);

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
    // 🔹 Aplicar curva exponencial SOLO al level
    float a = 4.0f; // controla la aceleración al final
    float curvedLevel = (exp(a * level) - 1.0f) / (exp(a) - 1.0f);

    float logMin = logf(_freqMin);
    float logMax = logf(_freqMax);
    float logFreq = logMin + curvedLevel * (logMax - logMin);

    return expf(logFreq);
}

void AcousticInjector::updateWaveFrequency(float freqHz) {
    if (!_timer) return;

    const float sr = DEFAULT_SAMPLE_RATE;       // p.ej. 32000
    _currentFrequency = freqHz;

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

void AcousticInjector::setDecayParameters(uint32_t durationMs, float lastMAPLevel) {
  float resFreqHz = _freqMin + (_freqMax - _freqMin) * constrain(lastMAPLevel, 0.0f, 1.0f);
  _decayDurationMs = max<uint32_t>(1, durationMs);

    // - eased: menos mezcla en cargas bajas, más en altas
  float mix = 0.4f + 0.6f * powf(lastMAPLevel, 1.8f); // adjust exponent for curve
  _decayMix = constrain(mix, 0.0f, 1.0f);
  _decayResFreq = constrain(resFreqHz, 100.0f, 30000.0f);

  // calcular paso de fase resonador en términos de SAMPLE_RATE / PHASE_FRAC
  const float sr = (float)SAMPLE_RATE;
  double step = _decayResFreq * TABLE_SIZE * (1ULL << PHASE_FRAC) / sr;
  uint32_t newStep = (step < 1.0) ? 1 : uint32_t(round(step));
  noInterrupts();
  _resPhaseStep = newStep;
  interrupts();
}

void AcousticInjector::setDecayLevel(float level) {
  // level esperado 0..1 (ActuatorManager pasa lastMAPLevel)
  float l = constrain(level, 0.0f, 1.0f);
  // inicializamos envelope (0..255) proporcional al nivel
  noInterrupts();
  _decayEnvInt = uint8_t(l * 255.0f);
  interrupts();
}

void AcousticInjector::updateDecayState() {
  if (!_inDecay) return;

  uint32_t now = millis();
  uint32_t elapsed = (now >= _decayStartMillis) ? (now - _decayStartMillis) : 0;
  float t = float(elapsed) / float(_decayDurationMs);
  t = constrain(t, 0.0f, 1.0f);

  // envelope exponencial simple: env = exp(-elapsed / tau), tau = duration/3
  float tau = float(max<uint32_t>(1, _decayDurationMs)) / 3.0f;
  float env = expf(-float(elapsed) / tau);
  // shape perceptual ligeramente (opcional pero probado)
  env = powf(env, 0.9f);

  uint8_t envInt = uint8_t(constrain(env * 255.0f, 0.0f, 255.0f));
  noInterrupts();
  _decayEnvInt = envInt;
  if (elapsed >= _decayDurationMs) {
    _inDecay = false;
    _decayEnvInt = 0;
  }
  interrupts();
}
