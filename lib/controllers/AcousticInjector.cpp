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

void AcousticInjector::start(float level, float dMAFdt) {
  // 1) Reinicio total (fase, nivel, índices)
  resetInternal();
  _active      = true;
  _inDecay = false;
  _targetLevel = constrain(level, 0.0f, 1.0f);
  if (_targetLevel < SWEEP_LOW_START_LEVEL) _targetLevel = SWEEP_LOW_START_LEVEL;
  _level       = SWEEP_LOW_START_LEVEL;
  _levelAtSweepStart = SWEEP_LOW_START_LEVEL;
  _levelInt    = uint8_t(_level * 255.0f);

  // 2) Configurar inicio y objetivo de frecuencia
  uint32_t nowMs = millis();
  float    targetFreq = mapLoadToWaveFrequency(_targetLevel);
  _lastUpdateMs       = nowMs;
  _forceSweep         = true;
  _sweepStartMs       = nowMs;
  _preIdleStartMs     = nowMs;
  _postSweepReleasePending = false;
  _levelSweepInitDone = true;
  _levelSweepStartMs  = nowMs;


  _dMAFdtEntry = constrain(dMAFdt, -10.0f, 10.0f); // proteger

  if (!(std::isfinite(targetFreq) && targetFreq > 0.0f)) {
    targetFreq = max(_freqMin, FORCE_FINAL_FREQ_MIN);
  }
  _targetFrequency = targetFreq;

  float sweepGoal = max(_freqMin, FORCE_SWEEP_START_HZ);
  sweepGoal = constrain(sweepGoal, FORCE_FINAL_FREQ_MIN, FORCE_FINAL_FREQ_MAX);
  float postGoal = constrain(targetFreq, FORCE_FINAL_FREQ_MIN, FORCE_FINAL_FREQ_MAX);
  _sweepTargetFrequency      = sweepGoal;
  _postSweepTargetFrequency  = postGoal;

  _currentFrequency = FORCE_SWEEP_START_HZ;
  _targetFrequency  = targetFreq;
  _currentLogFreq   = logf(max(1.0f, _currentFrequency));

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
  _inDecay = false;
  timerAlarmDisable(_timer);
  dac_output_voltage(_dacChannel, 128);
  _level = 0.0f;
  _targetLevel = 0.0f;
  _levelInt = 0;
}

void AcousticInjector::setLevel(float level) {
  level = constrain(level, 0.0f, 1.0f);

  _targetLevel = constrain(max(level, SWEEP_LOW_START_LEVEL), 0.0f, 1.0f);

  float mappedFreq = mapLoadToWaveFrequency(_targetLevel);
  if (std::isfinite(mappedFreq) && mappedFreq > 0.0f) {
    _targetFrequency          = mappedFreq;
    _postSweepTargetFrequency = mappedFreq;
  }
}

void AcousticInjector::update() {
  if (!_active) {
    _levelSweepInitDone = false;    // reset para la próxima activación
    // FIX: aseguramos rearmar pre-idle en próxima activación
    _preIdleStartMs = 0;
    return;
  }

  // Parámetros del sweep inicial
  const uint32_t forceSweepTimeMs = FORCE_SWEEP_TIME_MS;

  // ------ calcular dt ------
  uint32_t now = millis();
  uint32_t elapsedMs = (_lastUpdateMs == 0) ? 1 : (now - _lastUpdateMs);
  _lastUpdateMs = now;
  float dt = float(elapsedMs) * 0.001f; // segundos

  const float sweepStepLimitHz = FORCE_SWEEP_STEP_LIMIT_HZ;
  const float sweepPivotSpan   = max(1.0f, FORCE_FINAL_FREQ_MIN - FORCE_SWEEP_START_HZ);
  const bool  beamRangeActive  = !_inDecay;
  const float dynamicMinHz     = beamRangeActive ? max(_freqMin, FORCE_FINAL_FREQ_MIN)
                                                 : FORCE_SWEEP_START_HZ;
  const float dynamicMaxHz     = beamRangeActive ? min(_freqMax, FORCE_FINAL_FREQ_MAX)
                                                 : FORCE_FINAL_FREQ_MAX;
  auto computeMaxDeltaPerUpdate = [&](float freqHz, float baseStepHz) -> float {
    float normalized = 1.0f - ((freqHz - FORCE_SWEEP_START_HZ) / sweepPivotSpan);
    normalized = constrain(normalized, 0.0f, 1.0f);
    float boostedHz = baseStepHz * (1.0f + normalized * FORCE_SWEEP_NEAR_START_GAIN);
    return boostedHz * dt;
  };

  uint32_t elapsedPreIdle = PRE_IDLE_TIME_MS;
  bool      preIdleActive = false;

  if (_forceSweep) {
  float sweepGoal = max(_freqMin, FORCE_SWEEP_START_HZ);
  if (!(std::isfinite(sweepGoal) && sweepGoal > 0.0f)) {
    sweepGoal = dynamicMinHz;
  }
  _sweepTargetFrequency     = constrain(sweepGoal, dynamicMinHz, dynamicMaxHz);
  _postSweepTargetFrequency = _targetFrequency;
  }

  // ------ determinar desiredFreq (sweep o map) ------
  float demandFreq = _currentFrequency;
  float sweepDynamicStep = sweepStepLimitHz;
  if (_forceSweep) {
    if (_sweepStartMs == 0) {
      _sweepStartMs = now;
    }
    uint32_t elapsedSweep = (now >= _sweepStartMs) ? (now - _sweepStartMs) : 0;
    float rawT = constrain(float(elapsedSweep) / float(forceSweepTimeMs), 0.0f, 1.0f);
    float t = powf(rawT, FORCE_SWEEP_SHAPE_EXP);

    float freqTarget = FORCE_SWEEP_START_HZ;
    float sweepSpan  = _sweepTargetFrequency - FORCE_SWEEP_START_HZ;
    float accel = powf(t, 1.25f);
    if (fabsf(sweepSpan) > 1.0f) {
      freqTarget = FORCE_SWEEP_START_HZ + accel * sweepSpan;
    } else {
      freqTarget = _sweepTargetFrequency;
    }
    freqTarget = constrain(freqTarget, FORCE_SWEEP_START_HZ, dynamicMaxHz);
    float dynamicStep = sweepStepLimitHz * (1.0f - accel) + 15.0f * accel;
    sweepDynamicStep = dynamicStep;
    demandFreq = freqTarget;

    bool sweepTargetReached = (fabsf(_currentFrequency - _sweepTargetFrequency) <= FORCE_SWEEP_EXIT_TOL_HZ) ||
                              (fabsf(freqTarget - _sweepTargetFrequency) <= FORCE_SWEEP_EXIT_TOL_HZ) ||
                              (freqTarget >= (_freqMin - FORCE_SWEEP_EXIT_TOL_HZ));
    if (t >= 1.0f || sweepTargetReached) {
      _forceSweep = false;
      _sweepStartMs = 0;
      _postSweepTargetFrequency = _targetFrequency;
      _postSweepReleasePending  = true;
      _currentLogFreq = logf(max(_currentFrequency, 1.0f));
    }
  } else {
    float steadyTarget;
    if (_postSweepReleasePending) {
      steadyTarget               = _postSweepTargetFrequency;
      _postSweepReleasePending   = false;
    } else if (_targetFrequency > 0.0f) {
      steadyTarget = _targetFrequency;
    } else if (beamRangeActive) {
      float levelForFreq = max(_targetLevel, _level);
      steadyTarget = mapLoadToWaveFrequency(levelForFreq);
    } else {
      steadyTarget = mapLoadToWaveFrequency(_level);
    }
    demandFreq = steadyTarget;
  }
  if (_forceSweep &&
      (fabsf(_currentFrequency - _sweepTargetFrequency) <= FORCE_SWEEP_EXIT_TOL_HZ ||
       _currentFrequency >= (_freqMin - FORCE_SWEEP_EXIT_TOL_HZ))) {
    _forceSweep = false;
    _sweepStartMs = 0;
    _postSweepTargetFrequency = _targetFrequency;
    _postSweepReleasePending  = true;
  }

  if (beamRangeActive) {
    demandFreq = constrain(demandFreq, dynamicMinHz, dynamicMaxHz);
  } else {
    demandFreq = max(demandFreq, 1.0f);
  }

  float sonicShotLevelTarget = 0.0f;
  auto applySonicShot = [&](float& freqTarget, float& levelTarget) -> bool {
    bool overriding = false;
    const float pivotLow  = FORCE_SWEEP_START_HZ;
    const float pivotHigh = dynamicMinHz;
    float delta = freqTarget - _currentFrequency;
    bool wantsUp = (delta >= SONIC_SHOT_MIN_DELTA_HZ) &&
                   (fabsf(_currentFrequency - pivotLow) <= SONIC_SHOT_ZONE_HZ);
    bool wantsDown = (-delta >= SONIC_SHOT_MIN_DELTA_HZ) &&
                     (fabsf(_currentFrequency - pivotHigh) <= SONIC_SHOT_ZONE_HZ);
    bool cooldownReady = (now - _sonicShotLastMs) >= SONIC_SHOT_COOLDOWN_MS;
    bool levelEligible = (_targetLevel >= SONIC_SHOT_MIN_LEVEL);

    if (!_sonicShotActive && cooldownReady && (wantsUp || wantsDown) && levelEligible) {
      _sonicShotActive = true;
      _sonicShotDirectionUp = wantsUp;
      _sonicShotStartMs = now;
      _sonicShotLastMs = now;
    }

    if (_sonicShotActive) {
      float elapsed = float(now - _sonicShotStartMs);
      float duration = float(SONIC_SHOT_DURATION_MS);
      float tShot = (duration > 0.0f) ? constrain(elapsed / duration, 0.0f, 1.0f) : 1.0f;
      float shaped = 1.0f - powf(1.0f - tShot, SONIC_SHOT_EASE_EXP);
      float origin = _sonicShotDirectionUp ? pivotLow : pivotHigh;
      float pivotTarget = _sonicShotDirectionUp ? pivotHigh : pivotLow;
      float overshoot = (_sonicShotDirectionUp ? 1.0f : -1.0f) * SONIC_SHOT_OVERSHOOT_HZ;
      float shotTarget = pivotTarget + overshoot;
      shotTarget = constrain(shotTarget, FORCE_SWEEP_START_HZ, dynamicMaxHz);
      freqTarget = origin + (shotTarget - origin) * shaped;
      levelTarget = max(levelTarget,
                        SWEEP_LOW_START_LEVEL
                          + shaped * (SONIC_SHOT_LEVEL_PEAK - SWEEP_LOW_START_LEVEL));
      overriding = true;
      if (tShot >= 1.0f) {
        _sonicShotActive = false;
      }
    }
    return overriding;
  };

  bool sonicShotOverride = false;
  if (beamRangeActive) {
    sonicShotOverride = applySonicShot(demandFreq, sonicShotLevelTarget);
  } else {
    _sonicShotActive = false;
  }

  float desiredFreq = demandFreq;
  if (_forceSweep && !sonicShotOverride) {
    float maxDelta = computeMaxDeltaPerUpdate(_currentFrequency, sweepDynamicStep);
    float delta = desiredFreq - _currentFrequency;
    delta = constrain(delta, -maxDelta, maxDelta);
    desiredFreq = _currentFrequency + delta;
  }

  // ------ SLEW en log-domain (one-pole) ------
  float finalFreq = _currentFrequency;
  if (sonicShotOverride) {
    finalFreq = constrain(desiredFreq, FORCE_SWEEP_START_HZ, dynamicMaxHz);
    _currentLogFreq = logf(fmaxf(finalFreq, 1.0f));
  } else if (beamRangeActive) {
    float tauFreq = SLEW_TAU_MS * 0.001f;
    float alphaFreq = expf(-dt / max(1e-6f, tauFreq));
    float targetLog = logf(max(desiredFreq, 1.0f));
    _currentLogFreq = targetLog + alphaFreq * (_currentLogFreq - targetLog);
    finalFreq = expf(_currentLogFreq);
    if (finalFreq < dynamicMinHz) {
      finalFreq = dynamicMinHz;
      _currentLogFreq = logf(dynamicMinHz);
    } else if (finalFreq > dynamicMaxHz) {
      finalFreq = dynamicMaxHz;
      _currentLogFreq = logf(dynamicMaxHz);
    }
  } else {
    finalFreq = max(desiredFreq, 1.0f);
    _currentLogFreq = logf(max(finalFreq, 1.0f));
  }
  _currentFrequency = finalFreq;
  updateWaveFrequency(finalFreq);

  // ----- LEVEL transitions -----
  static constexpr uint32_t LEVEL_SWEEP_TIME_MS = 2000;
  float tLevel = constrain(_targetLevel, 0.0f, 1.0f);
  float sweepLogProgress = 0.0f;
  bool  sweepLoggingActive = false;

  if (preIdleActive) {
    float tIdle = float(elapsedPreIdle) / float(PRE_IDLE_TIME_MS);
    float baseIdle = PRE_IDLE_MIN_LEVEL
                   + powf(tIdle, PRE_IDLE_CURVE_EXP) * (PRE_IDLE_MAX - PRE_IDLE_MIN_LEVEL);
    float wobble = sinf(tIdle * PRE_IDLE_WOBBLE_SPEED * 2.0f * PI)
                   * PRE_IDLE_WOBBLE_STRENGTH * baseIdle;
    _level = constrain(baseIdle + wobble, PRE_IDLE_MIN_LEVEL, PRE_IDLE_MAX);
    _levelInt = uint8_t(_level * 255.0f);
    return;
  }

  if (_forceSweep) {
    float spanHz = max(1.0f, max(_freqMin, FORCE_SWEEP_START_HZ) - FORCE_SWEEP_START_HZ);
    float freqProgress = (_currentFrequency - FORCE_SWEEP_START_HZ) / spanHz;
    freqProgress = constrain(freqProgress, 0.0f, 1.0f);
    _level = SWEEP_LOW_START_LEVEL
           + freqProgress * (SWEEP_LOW_END_LEVEL - SWEEP_LOW_START_LEVEL);
    _level = constrain(_level, SWEEP_LOW_START_LEVEL, SWEEP_LOW_END_LEVEL);
    _levelInt = uint8_t(constrain(_level * 255.0f, 0.0f, 255.0f));
    if (_levelInt < 1) _levelInt = 1;
    sweepLogProgress = freqProgress;
    sweepLoggingActive = true;
  } else {
    float levelTau = LEVEL_SMOOTH_TAU_MS * 0.001f;
    float alphaLevel = 1.0f - expf(-dt / max(1e-4f, levelTau));
    _level += (tLevel - _level) * alphaLevel;
    _level = constrain(_level, 0.0f, 1.0f);
    _levelInt = uint8_t(constrain(_level * 255.0f, 0.0f, 255.0f));
  }

  if (sonicShotLevelTarget > 0.0f) {
    float boosted = constrain(sonicShotLevelTarget, 0.0f, 1.0f);
    if (boosted > _level) {
      _level = boosted;
      _levelInt = uint8_t(constrain(_level * 255.0f, 0.0f, 255.0f));
      if (_levelInt < 1) _levelInt = 1;
    }
  }
  /*
  static uint32_t _sweepDbgLast = 0;
  if (sweepLoggingActive) {
    uint32_t nowDbg = millis();
    if (nowDbg - _sweepDbgLast >= 20) {
      _sweepDbgLast = nowDbg;
      Serial.printf("[AI:SWEEP] t=%lu L=%.4f tgt=%.4f freq=%.1f sweepT=%.2f\n",
                    (unsigned long)nowDbg,
                    double(_level),
                    double(_targetLevel),
                    double(_currentFrequency),
                    double(sweepLogProgress));
    }
  }

  */
  
  // --- DECAY MIX / ENVELOPE (igual que antes) ---
  float A = constrain(_level, 0.0f, 1.0f);
  float energy = A * A;
  float G = freqGainFactor(_currentFrequency);
  float effective = energy * G;
  float decayMix = effective;
  const float alpha = 0.15f;
  _decayMix = _decayMix * (1.0f - alpha) + decayMix * alpha;

  noInterrupts();
  _decayMixInt16 = uint16_t(constrain(_decayMix * 65535.0f, 0.0f, 65535.0f));
  interrupts();

  // Capture BEAM peaks
  _peakLevelDuringBeam = (_level > _peakLevelDuringBeam) ? _level : _peakLevelDuringBeam;
  _peakFreqDuringBeam  = (_currentFrequency > _peakFreqDuringBeam) ? _currentFrequency : _peakFreqDuringBeam;
/*
    // ---------------------------------------------------------------
  // LOG cada 20 ms (telemetría compacta)
  static uint32_t _lastLogMs = 0;
  if (now - _lastLogMs >= 20) {           // cada 20 ms
    _lastLogMs = now;

    Serial.print("[AI]t:");
    Serial.print(now);
    Serial.print("ms | f:");
    Serial.print(_currentFrequency, 1);
    Serial.print("Hz | L:");
    Serial.print(_level, 3);
    Serial.print(" | dMix:");
    Serial.print(_decayMix, 3);
    Serial.print(" | phase:");
    Serial.print(_levelSweepInitDone ? "SWP" : "PIDL");
    Serial.print(" | act:");
    Serial.print(_active);
    Serial.println();
  }

*/

}

void IRAM_ATTR AcousticInjector::onTimer() {
  if (!_instance) return;

  // Si el sistema no está activo, reiniciamos para preparar arranque limpio
  if (!_instance->_active) {
    _instance->_phaseAcc = 0;
    _instance->_resPhaseAcc = 0;
    _instance->_decayEnvInt16 = 0;
    _instance->_resAmpInt16 = 0;
    _instance->_levelInt = 0;
    dac_output_voltage(_instance->_dacChannel, 128); // salida neutra DAC
    _instance->_lastDACValue = 128;
    return;
  }
  // =========================================================

  // avance de fase principal (fixed point)
  _instance->_phaseAcc += _instance->_phaseStep;

  // Índice entero y fracción para interpolación principal
  const uint32_t idx      = (_instance->_phaseAcc >> PHASE_FRAC) & (TABLE_SIZE - 1);
  const uint32_t nextIdx  = (idx + 1) & (TABLE_SIZE - 1);
  const uint32_t frac     = _instance->_phaseAcc & ((1ULL << PHASE_FRAC) - 1);

  const uint8_t  sample1  = _instance->_sineTable[idx];
  const uint8_t  sample2  = _instance->_sineTable[nextIdx];
  const int32_t  delta    = int32_t(sample2) - int32_t(sample1);
  const int32_t  interp   = int32_t(sample1) + int32_t((delta * frac) >> PHASE_FRAC);
  const int32_t  centered_p = interp - 128;

  // Salidas y estado compartido: leer copias atómicas de volátiles
  uint16_t env16_local      = _instance->_decayEnvInt16;
  uint16_t levelMul16_local = _instance->_decayLevelMulInt;
  uint8_t  level8_local     = _instance->_levelInt;
  uint32_t resPhaseAcc_local = _instance->_resPhaseAcc;
  uint32_t resPhaseStep_local= _instance->_resPhaseStep;
  dac_channel_t dacCh_local  = _instance->_dacChannel;  
  bool     inDecay_local     = _instance->_inDecay;

  // ================= LÓGICA DE SELECCIÓN DE RESONADOR =================
  if (!inDecay_local) {
    // Ya no en decay → pero sigue activo, usar salida principal
    int32_t principal = 128 + ((centered_p * int32_t(level8_local)) >> 8);
    if (principal < 0) principal = 0;
    if (principal > 255) principal = 255;
    uint8_t output = uint8_t(principal);
    if (output != _instance->_lastDACValue) {
        dac_output_voltage(dacCh_local, output);
        _instance->_lastDACValue = output;
    }
    _instance->_lastDACValue = output;
    return;
  }

  // ---------- DECAY activo: ring down y mezcla (enteros, ISR friendly) ----------
  resPhaseAcc_local += resPhaseStep_local;
  uint32_t newResPhaseAcc = resPhaseAcc_local;
  _instance->_resPhaseAcc = newResPhaseAcc;

  const uint32_t rIdx   = (resPhaseAcc_local >> PHASE_FRAC) & (TABLE_SIZE - 1);
  const uint32_t rNext  = (rIdx + 1) & (TABLE_SIZE - 1);
  const uint32_t rFrac  = resPhaseAcc_local & ((1ULL << PHASE_FRAC) - 1);
  const uint8_t  r1     = _instance->_sineTable[rIdx];
  const uint8_t  r2     = _instance->_sineTable[rNext];
  const int32_t  rDelta = int32_t(r2) - int32_t(r1);
  const int32_t  rInterp = int32_t(r1) + int32_t((rDelta * rFrac) >> PHASE_FRAC);
  const int32_t  centered_r = rInterp - 128;


  uint16_t resAmp16_local = _instance->_resAmpInt16; // 0..65535
  uint32_t tmp = uint32_t(resAmp16_local) * uint32_t(env16_local >> 8);
  uint16_t resAmpEnvAdjusted16 = uint16_t(tmp >> 8);
  uint8_t resGain8 = uint8_t((uint32_t(resAmpEnvAdjusted16) * 255u) >> 16u);

  int32_t resonatorScaled = (int32_t(centered_r) * int32_t(resGain8)) >> 8;
  int32_t scaledLevel = (int32_t(level8_local) * int32_t(levelMul16_local)) >> 16;
  if (scaledLevel < 0) scaledLevel = 0;
  if (scaledLevel > 255) scaledLevel = 255;
  int32_t principalScaled = (int32_t(centered_p) * scaledLevel) >> 8;

  int32_t mixed_signed = principalScaled + resonatorScaled;
  int32_t out = mixed_signed + 128;
  out = (out < 0) ? 0 : (out > 255 ? 255 : out);

  uint8_t output = uint8_t(out);

  if (output != _instance->_lastDACValue) {
    dac_output_voltage(dacCh_local, output);
    _instance->_lastDACValue = output;
}

  _instance->_lastDACValue = output;
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

void AcousticInjector::testFloor() {
  Serial.println(F("[AI] Prueba piso DAC (1 LSB) iniciada..."));

  bool wasActive = _active;
  if (_timer) {
    timerAlarmDisable(_timer);
  }

  const float freq = 6000.0f;
  const float sampleRate = 64000.0f;
  const uint8_t bias = 128;
  const float oneLsbLevel = 1.0f / 127.0f;
  const float amplitude = 127.0f * oneLsbLevel;
  const float dPhase = 2.0f * PI * freq / sampleRate;

  float phase = 0.0f;
  const uint32_t durationMs = 2000;
  const uint32_t samples = uint32_t((durationMs / 1000.0f) * sampleRate);

  for (uint32_t i = 0; i < samples; ++i) {
    float value = bias + amplitude * sinf(phase);
    int v = constrain(int(value + 0.5f), 0, 255);
    dac_output_voltage(_dacChannel, uint8_t(v));
    phase += dPhase;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;
    delayMicroseconds(15);
  }

  dac_output_voltage(_dacChannel, bias);
  if (_timer && wasActive) {
    timerWrite(_timer, 0);
    timerAlarmEnable(_timer);
  }

  Serial.println(F("[AI] Prueba piso DAC finalizada."));
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

// mapLoadToWaveFrequency: convierte la potencia (0..1) en una frecuencia de barrido acoplada perceptualmente.
float AcousticInjector::mapLoadToWaveFrequency(float power) {
  power = constrain(power, SWEEP_LOW_START_LEVEL, 1.0f);

  const float levelFreqStart = SWEEP_LOW_START_LEVEL; // 0.005 -> 2 kHz
  const float levelFreqMin   = 0.015f;               // 0.015 -> freqMin (~5.5 kHz)

  const float freqStart = FORCE_SWEEP_START_HZ;
  const float freqMin   = max(_freqMin, FORCE_SWEEP_START_HZ);
  const float freqMax   = min(_freqMax, FORCE_FINAL_FREQ_MAX);

  if (power <= levelFreqStart) {
    return freqStart;
  }

  if (power <= levelFreqMin) {
    float t = (power - levelFreqStart) / (levelFreqMin - levelFreqStart);
    t = powf(constrain(t, 0.0f, 1.0f), SWEEP_FREQ_LOW_EXP);
    return freqStart + t * (freqMin - freqStart);
  }

  float t = (power - levelFreqMin) / (1.0f - levelFreqMin);
  t = powf(constrain(t, 0.0f, 1.0f), SWEEP_FREQ_HIGH_EXP);
  return freqMin + t * (freqMax - freqMin);
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
  _decayFinished = false;  // reset
  _decayStartMillis = now;
  _zeroCount = 0;
  interrupts();

  // escribir snapshot atómico en variables usadas por decay y precompute de amplitude inicial
  noInterrupts();
  _decayEnvInt16 = env_snapshot;
  _decayMixSnapshot16 = decayMixInt16_local;
  _decayLastLevelSnapshot = _level;
  _resPhaseAcc = phaseAcc_snapshot;   // conservar fase
  _resPhaseStep = phaseStep_snapshot; // congelar paso de freq al inicio
  // precompute resAmp initial = (mix * env) >> 16
  _resAmpInt16 = uint16_t((uint32_t(decayMixInt16_local) * uint32_t(env_snapshot)) >> 16u);
  // asegurar multiplicador de nivel por defecto
  _decayLevelMulInt = uint16_t(65535u);
  interrupts();
  // justo después de interrupts() final en startDecay
  /*
  
  Serial.printf("DBG:startDecay t=%lu env=%u mix=%u resAmp=%u phaseStep=%u phaseAcc=%u\n",
                now,
                env_snapshot,
                decayMixInt16_local,
                _resAmpInt16,
                phaseStep_snapshot,
                phaseAcc_snapshot);

  */
}

void AcousticInjector::setDecayParameters(uint32_t durationMs, float avgTPSLevel,
                                         float gFast, float gSustain, uint32_t tFastMs, uint32_t tSustainMs) {

  _decayDurationMs = max<uint32_t>(1u, durationMs);

  // mezcla base del ring down en float
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
/*
Serial.printf("DBG:setDecay dur=%u TPSf=%0.3f mix=%0.4f mix16=%u freq=%0.1f newStep=%u\n",
              _decayDurationMs,
              avgTPSLevel,
              _decayMix,
              mix16,
              _decayResFreq,
              newStep);

*/

}

void AcousticInjector::updateDecayState() {
  if (!_inDecay) return;
  // ------------------------------
  // Snapshot atómico (lecturas compartidas)
  // ------------------------------
  noInterrupts();
  uint16_t env16_copy    = _decayEnvInt16;
  uint16_t mix16_copy    = _decayMixInt16;
  uint16_t mixSnapshot16 = _decayMixSnapshot16;
  uint32_t curStepAtomic = _resPhaseStep;
  interrupts();
  // =========================================================
  // ===== FEATURE SWITCHES (todas desactivadas por defecto) ==
  // =========================================================
  const bool USE_SHOULDER      = true;//
  const bool USE_SOFT_BEND     = false;//
  const bool USE_REBOUND       = true;//
  const bool USE_TAIL_FREEZE   = false;//
  const bool USE_FADE_OUT      = true;//
  const bool USE_MIX_SNAPSHOT  = true;
  const bool USE_ENERGY_FLOOR  = true;//

  // ===========================================
  // ===== PARÁMETROS BASE / DEFAULTS ==========
  // ===========================================
  const float MIN_ENERGY_F_REAL  = USE_ENERGY_FLOOR ? DECAY_MIN_ENERGY_TIGHT : DECAY_MIN_ENERGY_BASE;

  float tSinceStart = float(millis() - _decayStartMillis);
  float envF = float(env16_copy) / 65535.0f;

  float lastLevelF = (_decayLastLevelSnapshot > 0.0f)
                       ? constrain(_decayLastLevelSnapshot, 0.0f, 1.0f)
                       : envF;
  if (lastLevelF < MIN_ENERGY_F_REAL) lastLevelF = MIN_ENERGY_F_REAL;

  float duration_sec_formula = DECAY_ZP_BASE
                     + 5.1f * powf(lastLevelF, DECAY_ZP_POWER_EXP)
                     + DECAY_ZP_LOG_SCALE * log2f(lastLevelF + 1.0f);
  uint32_t totalMsFormula = uint32_t(max<float>(0.001f, duration_sec_formula) * 1000.0f);
  uint32_t totalMs = max<uint32_t>(_decayDurationMs, totalMsFormula);

  // ===========================================
  // ===== PROGRESS & SHOULDER BLOCK ===========
  // ===========================================
  float progress = 0.0f;
  if (USE_SHOULDER) {
    uint32_t shoulderMs = uint32_t(float(totalMs) * DECAY_SHOULDER_FRAC);
    uint32_t activeMs   = (totalMs > shoulderMs) ? (totalMs - shoulderMs) : 1u;
    progress = (tSinceStart <= shoulderMs)
                 ? 0.0f
                 : constrain(float(tSinceStart - shoulderMs) / float(activeMs), 0.0f, 1.0f);
  } else {
    progress = constrain(tSinceStart / float(max<uint32_t>(1u, totalMs)), 0.0f, 1.0f);
  }

  progress = min(progress, 1.0f); // protección

  // ===========================================
  // ===== MAIN EXPONENTIAL DECAY BLOCK ========
  // ===========================================
  float lambda        = -logf(MIN_ENERGY_F_REAL / lastLevelF);
  float shaped        = powf(progress, DECAY_GAMMA);
  float expo          = expf(-lambda * shaped);
  float clipped_main  = lastLevelF * expo;
  if (clipped_main < MIN_ENERGY_F_REAL) clipped_main = MIN_ENERGY_F_REAL;

  // ===========================================
  // ===== SOFT BEND BLOCK =====================
  // ===========================================
  float softBendFactor = 1.0f;
  float bendProgress   = 0.0f;
  float b              = 0.0f;
  if (USE_SOFT_BEND) {
    bendProgress = constrain(tSinceStart / DECAY_SOFT_BEND_TIME_MS, 0.0f, 1.0f);
    if (bendProgress < 0.5f) {
      float x = bendProgress * 2.0f;
      b = 0.5f * (x * x * (3.0f - 2.0f * x));
    } else {
      float x = (bendProgress - 0.5f) * 2.0f;
      b = 0.5f + 0.5f * (1.0f - ((1.0f - x) * (1.0f - x) * (3.0f - 2.0f * (1.0f - x))));
    }
    softBendFactor = (bendProgress < 1.0f) ? (1.0f - DECAY_SOFT_BEND_DEPTH * b)
                                           : (1.0f - DECAY_SOFT_BEND_DEPTH);
  }

  // ===========================================
  // ===== REBOUND BLOCK =======================
  // ===========================================
  float rebound = 0.0f;
  if (USE_REBOUND) {
    if (tSinceStart < (totalMs * DECAY_REBOUND_WINDOW_FRAC)) {
      float tS    = tSinceStart / 1000.0f;
      float A     = DECAY_REBOUND_A * lastLevelF;
      float omega = 2.0f * 3.14159265f * DECAY_REBOUND_FREQ_HZ;
      float damp  = expf(-DECAY_REBOUND_DAMP * tS);
      rebound     = A * damp * cosf(omega * tS);
      if (fabsf(rebound) < 0.0005f) rebound = 0.0f;
    }
  }

  // ===========================================
  // ===== COMBINED LEVEL BLOCK ================
  // ===========================================
  float clipped_combined = clipped_main * softBendFactor + rebound;
  float smoothLevel = _level * 0.85f + clipped_combined * 0.15f;
  clipped_combined = constrain(smoothLevel, MIN_ENERGY_F_REAL, 1.0f);

  // ===========================================
  // ===== FREQUENCY BLOCK =====================
  // ===========================================
  float topFreqHz = (_decayResFreq > 0.0f) ? _decayResFreq : DECAY_MIN_FREQ_HZ;
  float energyFactor = powf(constrain(clipped_combined, 0.0f, 1.0f), DECAY_FREQ_COUPLE_EXP);
  float baseFreqHz = DECAY_MIN_FREQ_HZ + (topFreqHz - DECAY_MIN_FREQ_HZ) * energyFactor;
  float targetFreqHz = baseFreqHz;
  if (USE_SOFT_BEND) {
    float bendFreqDrop = DECAY_SOFT_BEND_DEPTH * 0.15f;
    float bendFactor = 1.0f - bendFreqDrop * b;
    bendFactor = constrain(bendFactor, 0.2f, 1.0f);
    targetFreqHz *= bendFactor;
  }

  if (USE_TAIL_FREEZE) {
      static bool tailLocked = false;
      static float frozenFreq = 0.0f;

      if (progress >= DECAY_TAIL_FREEZE_PROGRESS && !tailLocked) {
          tailLocked = true;
          frozenFreq = targetFreqHz;  // keep the current natural freq
      }
      if (tailLocked) {
          targetFreqHz = frozenFreq;  // keep it locked until decay ends
      }
      if (!_inDecay) {
          tailLocked = false;  // reset when finished
      }
  }


  const double sr = double(SAMPLE_RATE);
  const double stepToHz = sr / (double(TABLE_SIZE) * double(1ULL << PHASE_FRAC));
  double stepFloat = double(targetFreqHz) * double(TABLE_SIZE) * double(1ULL << PHASE_FRAC) / sr;
  uint32_t computedTarget = (stepFloat < 1.0) ? 1u : uint32_t(round(stepFloat));
  uint32_t minStep = uint32_t(round(double(DECAY_MIN_FREQ_HZ) * double(TABLE_SIZE) * double(1ULL << PHASE_FRAC) / sr));
  if (computedTarget < minStep) computedTarget = minStep;

  uint32_t targetStep = computedTarget;
  uint32_t nextStep   = curStepAtomic;

  if (curStepAtomic != targetStep) {
    uint32_t diff = (curStepAtomic > targetStep) ? (curStepAtomic - targetStep)
                                                 : (targetStep - curStepAtomic);
    float currentFreqHz = float(double(curStepAtomic) * stepToHz);
    float freqSpan = max(1.0f, topFreqHz - DECAY_MIN_FREQ_HZ);
    float normalized = (currentFreqHz - DECAY_MIN_FREQ_HZ) / freqSpan;
    normalized = constrain(normalized, 0.0f, 1.0f);
    float maxDeltaHz = DECAY_SLEW_HIGH_FREQ_DELTA_HZ
                     + (1.0f - normalized) * DECAY_SLEW_LOW_FREQ_EXTRA_HZ;
    double maxDeltaStepD = double(maxDeltaHz) / stepToHz;
    uint32_t maxDeltaStep = uint32_t(max(1.0, maxDeltaStepD));
    if (maxDeltaStep > diff) maxDeltaStep = diff;

    if (curStepAtomic > targetStep) {
      nextStep = curStepAtomic - maxDeltaStep;
    } else {
      nextStep = curStepAtomic + maxDeltaStep;
    }
  }

  float appliedFreqHz = float(double(nextStep) * stepToHz);

  if (USE_FADE_OUT) {
    if (progress >= DECAY_FADE_START_PROGRESS) {
      float remain = 1.0f - progress;
      float fade = remain <= 0.0f ? 0.0f : remain / (1.0f - DECAY_FADE_START_PROGRESS);
      fade = powf(fade, 1.3f);
      fade = constrain(fade, 0.0f, 1.0f);
      clipped_combined *= fade;
    }
  }

  uint16_t levelMulTarget16 = uint16_t(constrain((1.0f - clipped_combined) * 65535.0f, 0.0f, 65535.0f));
  uint8_t  levelInt8Target  = uint8_t(levelMulTarget16 >> 8u);
  if (clipped_combined <= 0.01f) levelInt8Target = 0;
  uint16_t target16 = uint16_t(constrain(clipped_combined * 65535.0f, 0.0f, 65535.0f));

  uint16_t mix_for_calc = USE_MIX_SNAPSHOT ? (mixSnapshot16 ? mixSnapshot16 : mix16_copy)
                                           : mix16_copy;
  uint16_t resAmpComputed = uint16_t((uint32_t(mix_for_calc) * uint32_t(target16)) >> 16u);

  noInterrupts();
  _resPhaseStep     = nextStep;
  _resTargetStep    = targetStep;
  _currentFrequency = appliedFreqHz;
  _decayLevelMulInt = levelMulTarget16;
  _levelInt         = levelInt8Target;
  _level            = clipped_combined;
  _decayEnvInt16    = target16;
  _resAmpInt16      = resAmpComputed;
  interrupts();

  // ===========================================
  // ===========================================
  // ===== EXIT CONDITIONS (DEPENDIENTES DE FEATURES) ========
  // ===========================================
  // ===== SALIDA CONDICIONAL POR FEATURES =====
  // ===========================================
  bool levelNearZero = (_decayEnvInt16 <= DECAY_ENVELOPE_EXIT_THRESHOLD);
  bool reboundDead   = (fabsf(rebound) < 0.001f);
  bool envelopeDone  = (progress >= 1.0f);
  bool freqAtMin     = (appliedFreqHz <= DECAY_MIN_FREQ_HZ + 1.0f);

  bool readyToEnd = envelopeDone && levelNearZero && freqAtMin;
  if (USE_REBOUND) readyToEnd = readyToEnd && reboundDead;

  // Acumulador de estabilidad
  if (readyToEnd) {
      _zeroCount++;
  } else {
      _zeroCount = 0;
  }

  // Salida directa del decay cuando se cumple la condición estable
  if (readyToEnd && _zeroCount >= 3) {
      noInterrupts();
      _decayFinished = true;   // avisar a FSM, pero NO tocar _inDecay
      _decayEnvInt16 = 0;
      _zeroCount = 0;
      interrupts();
      return;
  }

  // Failsafe: evita loops infinitos si alguna feature falla
  if (tSinceStart > totalMs + 200u) {
      noInterrupts();
      _decayFinished = true;   // avisar a FSM, pero NO tocar _inDecay
      _decayEnvInt16 = 0;
      _zeroCount = 0;
      interrupts();
      return;
  }


  // ================== DEBUG INMEDIATO CADA 20 ms ==================
  uint32_t nowMs = millis();
  if (nowMs - _lastDecayPrintMs >= 1) {
      _lastDecayPrintMs = nowMs;
      Serial.printf(
        "[DECAY_DBG] t=%lu prog=%.3f lvl=%.4f freq=%.1f env=%u zeroCnt=%u DecayFinished=%d\n",
        (unsigned long)nowMs,
        double(progress),
        double(_level),
        double(_currentFrequency),
        (unsigned int)_decayEnvInt16,
        (unsigned int)_zeroCount,
        (int)_decayFinished
      );
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










