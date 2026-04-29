// StateMachine.cpp

#include "StateMachine.h"
#include <Arduino.h>

float median(float a, float b, float c, float d, float e) {
    float v[5] = { a, b, c, d, e };
    for (int i = 1; i < 5; ++i) {
        float key = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > key) {
            v[j + 1] = v[j];
            --j;
        }
        v[j + 1] = key;
    }
    return v[2];
}

void StateMachine::begin(bool hasCalibration,
                         ActuatorManager* actuatorsPtr,
                         ThresholdManager* thresholdManagerPtr,
                         SensorManager* sensorsPtr,
                         CalibrationManager* calibMgrPtr) {
    current           = hasCalibration ? SystemState::OFF : SystemState::NO_CALIB;
    actuators         = actuatorsPtr;
    sensors           = sensorsPtr;
    thresholdManager  = thresholdManagerPtr;
    calibMgr          = calibMgrPtr;

    if (thresholdManager) {
        thresholds = thresholdManager->getThresholds();
    }

    vortexPending       = false;
    vortexStartMillis   = 0;
    decayStartMillis = 0;
    _fastAttackActive = false;
    _pressurePercent = 0.0f;
    _pressureDelta = 0.0f;
    _lastPressurePercent = 0.0f;
    _dMAFdtRaw = 0.0f;
    _dMAFdtEMA = 0.0f;
    flowAcousticBase = 0.0f;
    flowBoostBase = 0.0f;
    alignAcousticLevel = 0.0f;
    alignBoostLevel = 0.0f;
    flowOscillationKPa = 0.0f;
    flowRmsKPa = 0.0f;
    flowMadKPa = 0.0f;
    flowOutlierRatio = 0.0f;
    flowRmsSlope = 0.0f;
    flowStableNow = false;
    _flowStableStartMs = 0;
    _flowUnstableStartMs = 0;
    lastDeltaMAFLevelForBEAM = 0.0f;
    lastDeltaMAPLevelForBEAM = 0.0f;

    if (actuators) {
        actuators->stopAcoustic();
        actuators->stopVortex();
    }
    resetVortexTwoLayerState();

    lastState = current;
    Serial.print(">> StateMachine iniciado en estado: ");
    Serial.println(static_cast<int>(current));
}

float StateMachine::getLevel() const {
    return mafNormalized;
}

void StateMachine::update(float mapLoadPercent,
                          float mafLoadPercent,
                          bool serialCalibReq,
                          bool bleCalibReq,
                          bool calibLoaded,
                          const DebugManager &dbg) {
    if (current == SystemState::DEBUG) return;

    if (thresholdManager) {
        thresholds = thresholdManager->getThresholds();
    }

    // Mantener buffer de 5 muestras para MAF y MAP
    static float mafBuffer[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static float mapBuffer[3] = {0.0f, 0.0f, 0.0f};
    static float pressureBuffer[3] = {0.0f, 0.0f, 0.0f};

    // Desplazar las muestras anteriores
    mafBuffer[0] = mafBuffer[1];
    mafBuffer[1] = mafBuffer[2];
    mafBuffer[2] = mafBuffer[3];
    mafBuffer[3] = mafBuffer[4];
    mafBuffer[4] = mafLoadPercent;

    mapBuffer[0] = mapBuffer[1];
    mapBuffer[1] = mapBuffer[2];
    mapBuffer[2] = mapLoadPercent;

    pressureBuffer[0] = pressureBuffer[1];
    pressureBuffer[1] = pressureBuffer[2];
    pressureBuffer[2] = sensors ? sensors->getPressurePercentSigned() : 0.0f;

    // Aplicar mediana
    _mafLoadPercent  = median(mafBuffer[0], mafBuffer[1], mafBuffer[2], mafBuffer[3], mafBuffer[4]);
    _mapLoadPercent  = median(mapBuffer[0], mapBuffer[1], mapBuffer[2], mapBuffer[2], mapBuffer[2]);
    _pressurePercent = median(pressureBuffer[0], pressureBuffer[1], pressureBuffer[2], pressureBuffer[2], pressureBuffer[2]);
    _pressureDelta   = _pressurePercent - _lastPressurePercent;
    _lastPressurePercent = _pressurePercent;

    currentDeltaMAFLevelForBEAM = sensors->getRelativeMAFLevel(thresholds.BEAM_TPS_ON);
    currentDeltaMAPLevelForBEAM = sensors->getRelativeMAPLevel(thresholds.BEAM_MAP_ON);

    mapNormalized   = mapLoadPercent / 100.0f;
    mafNormalized   = mafLoadPercent / 100.0f;

    compute_dMAFdt_and_hold(mafBuffer[4], mafBuffer[0], currentDeltaMAFLevelForBEAM);

    uint32_t now = millis();
    bool vacuumOn = (_pressurePercent <= VACUUM_PCT_ON);
    bool vacuumOff = (_pressurePercent >= VACUUM_PCT_OFF);
    flowOscillationKPa = sensors ? sensors->computeOscillationAmplitude() : 0.0f;
    flowRmsKPa = sensors ? sensors->computeRMS() : 0.0f;
    flowMadKPa = sensors ? sensors->computeMAD() : 0.0f;
    flowOutlierRatio = sensors ? sensors->computeOutlierRatio() : 0.0f;
    flowRmsSlope = sensors ? sensors->computeRMSSlope() : 0.0f;
    flowStableNow = (flowOscillationKPa <= FLOW_OSCILLATION_KPA_MAX)
                 && (flowRmsKPa <= FLOW_RMS_KPA_MAX)
                 && (flowMadKPa <= FLOW_MAD_KPA_MAX)
                 && (flowOutlierRatio <= FLOW_OUTLIER_RATIO_MAX)
                 && (fabsf(flowRmsSlope) <= FLOW_RMS_SLOPE_KPA_S_MAX);

    if (flowStableNow) {
        if (_flowStableStartMs == 0) _flowStableStartMs = now;
        _flowUnstableStartMs = 0;
    } else {
        if (_flowUnstableStartMs == 0) _flowUnstableStartMs = now;
        _flowStableStartMs = 0;
    }

    bool flowStableHold = flowStableNow && ((now - _flowStableStartMs) >= FLOW_STABLE_HOLD_MS);
    bool flowUnstableHold = !flowStableNow && ((now - _flowUnstableStartMs) >= FLOW_UNSTABLE_HOLD_MS);

    switch (current) {
        static uint32_t offStartMillis = 0;

        case SystemState::OFF:
            if (offStartMillis == 0) offStartMillis = now;
            if (actuators) {
                actuators->stopAcoustic();
                actuators->stopVortex();
            }
            if ((now - offStartMillis) > 500
             && _mapLoadPercent > thresholds.MAP_WAKEUP_PERCENT) {
                current = SystemState::IDLE;
                offStartMillis = 0;
            }
            break;

        case SystemState::NO_CALIB:
            if (serialCalibReq || bleCalibReq) {
                current = SystemState::CALIBRATION;
            } else if (calibLoaded) {
                current = SystemState::OFF;
            }
            break;

        case SystemState::CALIBRATION:
            if (calibLoaded) {
                current = SystemState::OFF;
            }
            break;

        case SystemState::IDLE:
            if (vacuumOn && calibLoaded) {
                current = SystemState::ALIGN;
                if (sensors) {
                    mafInitialPercent = sensors->readMAFLoadPercent();
                    mapInitialPercent = sensors->readMAPLoadPercent();
                }
            }
            break;

        case SystemState::ALIGN:
            if (!calibLoaded || vacuumOff) {
                current = SystemState::DECAY;
            } else if (flowStableHold) {
                current = SystemState::FLOW;
            }
            break;

        case SystemState::FLOW:
            if (!calibLoaded || vacuumOff) {
                current = SystemState::DECAY;
            } else if (flowUnstableHold) {
                current = SystemState::ALIGN;
            }
            break;

        case SystemState::DECAY:
            if (actuators && actuators->decayFinished()) {
                current = (vacuumOn && calibLoaded) ? SystemState::ALIGN : SystemState::IDLE;
            }
            break;

        case SystemState::DEBUG:
            break;

        case SystemState::UNKNOWN:
            current = SystemState::OFF;
            break;
    }

    if (current != lastState) {
        if (current == SystemState::IDLE) {
            if (actuators) {
                actuators->stopAcoustic();
                actuators->stopVortex();
            }
            resetVortexTwoLayerState();
            _flowStableStartMs = 0;
            _flowUnstableStartMs = 0;
        } else if (current == SystemState::ALIGN) {
            if (actuators && (lastState == SystemState::IDLE || lastState == SystemState::DECAY)) {
                actuators->startVortex();
                actuators->startAcoustic(ALIGN_ACOUSTIC_SEED, _dMAFdtEMA);
                alignBoostLevel = ALIGN_BOOST_SEED;
                alignAcousticLevel = actuators->getAcousticLevel();
            } else if (actuators && lastState == SystemState::FLOW) {
                alignBoostLevel = actuators->getTurboLevel();
                alignAcousticLevel = actuators->getAcousticLevel();
            }
        } else if (current == SystemState::FLOW) {
            if (actuators && lastState == SystemState::ALIGN) {
                flowAcousticBase = actuators->getAcousticLevel();
                flowBoostBase = actuators->getTurboLevel();
            }
        } else if (current == SystemState::DECAY && actuators) {
            resetVortexTwoLayerState();
            resetBeamTracking();
            float deriv = _dMAFdtEMA;
            unsigned long holdMs = 0;
            if (_holdActive && _holdStartMillis != 0) {
                holdMs = now - _holdStartMillis;
            }

            float holdNorm = float(min<unsigned long>(holdMs, H_SCALE_MS)) / float(H_SCALE_MS);
            float x = (holdNorm - SIGMOID_H0) * SIGMOID_K;
            float w = 1.0f / (1.0f + expf(-x));

            float gFastRaw = 0.0f;
            if (deriv < 0.0f) {
                gFastRaw = constrain((-deriv) / DERIV_NORM, 0.0f, 1.0f);
            }
            float gSustainRaw = constrain(holdNorm, 0.0f, 1.0f);

            float gFast = (1.0f - w) * gFastRaw;
            float gSustain = w * gSustainRaw;

            float durationMsF = float(decayDurationMs) * (MIN_SCALE * (1.0f - w) + MAX_SCALE * w);
            uint32_t durationMs = uint32_t(constrain(durationMsF, 1.0f, 60000.0f));

            float tFast = _tFastMs;
            float tSustain = _tSustainMs;

            decayStartMillis = now;
            actuators->stopVortex();
            actuators->getAcousticInjector().setDecayParameters(durationMs, avgMAFLevel,
                                                                gFast, gSustain,
                                                                (uint32_t)tFast, (uint32_t)tSustain);
            actuators->getAcousticInjector().startDecay(now);
            _holdActive = false;
            _holdStartMillis = 0;
        if (current == SystemState::BOOST && sensors) {
            float osc = sensors->computeOscillationAmplitude();
            float rms = sensors->computeRMS();
            float median = sensors->computeMedianPressure();
            float mad = sensors->computeMAD();
            float outlierRatio = sensors->computeOutlierRatio();
            float rmsSlope = sensors->computeRMSSlope();
            bool oscOk = (osc <= FLOW_OSCILLATION_KPA_MAX);
            bool rmsOk = (rms <= FLOW_RMS_KPA_MAX);
            bool madOk = (mad <= FLOW_MAD_KPA_MAX);
            bool outlierOk = (outlierRatio <= FLOW_OUTLIER_RATIO_MAX);
            bool slopeOk = (fabsf(rmsSlope) <= FLOW_RMS_SLOPE_KPA_S_MAX);
            bool flowStable = oscOk && rmsOk && madOk && outlierOk && slopeOk;
            Serial.printf(
                ">> BOOST metrics | osc=%.3f kPa(%s) | rms=%.3f kPa(%s) | median=%.3f kPa | mad=%.3f kPa(%s) | outliers=%.2f(%s) | rms_slope=%.3f kPa/s(%s) | estado_sugerido=%s\n",
                osc,
                oscOk ? "OK" : "NO",
                rms,
                rmsOk ? "OK" : "NO",
                median,
                mad,
                madOk ? "OK" : "NO",
                outlierRatio,
                outlierOk ? "OK" : "NO",
                rmsSlope,
                slopeOk ? "OK" : "NO",
                flowStable ? "FLOW" : "ALIGN");
        }
        lastState = current;
    }

}

void StateMachine::handleActions() {
    if (!sensors || !actuators || !calibMgr) return;

    if (current == SystemState::ALIGN) {
        if (thresholdManager) {
            thresholds = thresholdManager->getThresholds();
        }

        mapSamples++;
        mafSamples++;

        avgMAPLevel += (currentDeltaMAPLevelForBEAM - avgMAPLevel) / float(mapSamples);
        avgMAFLevel += (currentDeltaMAFLevelForBEAM - avgMAFLevel) / float(mafSamples);

        float powerInput = currentDeltaMAFLevelForBEAM;
        float attackNorm = (_dMAFdtEMA - MAF_ATTACK_SLOW_DTPS)
                         / (MAF_ATTACK_FAST_DTPS - MAF_ATTACK_SLOW_DTPS);
        attackNorm = constrain(attackNorm, 0.0f, 1.0f);
        float attackGain = MAF_ATTACK_MIN_GAIN
                         + attackNorm * (MAF_ATTACK_MAX_GAIN - MAF_ATTACK_MIN_GAIN);

        float power = powerInput * attackGain;
        if (_dMAFdtEMA < 0.0f) {
            float releaseNorm = constrain(_dMAFdtEMA / MAF_RELEASE_REF_DTPS, 0.0f, 1.0f);
            power *= (1.0f - 0.5f * releaseNorm);
        }
        power = constrain(power, 0.0f, 1.0f);

        float oscNorm = flowOscillationKPa / FLOW_OSCILLATION_KPA_MAX;
        float rmsNorm = flowRmsKPa / FLOW_RMS_KPA_MAX;
        float madNorm = flowMadKPa / FLOW_MAD_KPA_MAX;
        float outlierNorm = flowOutlierRatio / FLOW_OUTLIER_RATIO_MAX;
        float slopeNorm = fabsf(flowRmsSlope) / FLOW_RMS_SLOPE_KPA_S_MAX;
        float turb = max(max(oscNorm, rmsNorm), max(max(madNorm, outlierNorm), slopeNorm));

        if (!flowStableNow) {
            if (turb > 1.0f) {
                alignAcousticLevel = constrain(alignAcousticLevel - ALIGN_ACOUSTIC_STEP_DOWN,
                                               0.0f, ALIGN_ACOUSTIC_MAX);
            } else {
                float target = max(power, alignAcousticLevel);
                float up = ALIGN_ACOUSTIC_STEP_UP * (1.0f - constrain(turb, 0.0f, 1.0f));
                alignAcousticLevel = constrain(target + up, 0.0f, ALIGN_ACOUSTIC_MAX);
            }
        } else {
            alignAcousticLevel = max(alignAcousticLevel, power);
        }
 
    }

        actuators->setAcousticParameters(alignAcousticLevel, alignAcousticLevel);
        lastDeltaMAFLevelForBEAM = currentDeltaMAFLevelForBEAM;
        lastDeltaMAPLevelForBEAM = currentDeltaMAPLevelForBEAM;

        float vortexCmd = computeVortexTwoLayerCmd(millis(), mafNormalized, _pressurePercent);
        actuators->updateVortexLevel(vortexCmd, vortexCmd);
        actuators->updateInjector();
    }

    if (current == SystemState::FLOW) {
        float scale = constrain(mafNormalized, 0.0f, 1.0f);
        float acousticLevel = constrain(flowAcousticBase * scale, 0.0f, 1.0f);
        float boostLevel = computeVortexTwoLayerCmd(millis(), scale, _pressurePercent);
        actuators->setAcousticParameters(acousticLevel, acousticLevel);
        actuators->updateVortexLevel(boostLevel, boostLevel);
        actuators->updateInjector();
    }

    if (current == SystemState::DECAY) {
        actuators->getAcousticInjector().updateDecayState();
    }
}

void StateMachine::resetBeamVortexRamp(float seedLevel) {
  float seeded = constrain(seedLevel, 0.0f, 1.0f);
  _beamVortexLevel = max(BEAM_VORTEX_ENTRY_LEVEL, seeded);
  _beamVortexLastUpdateMs = millis();
}

void StateMachine::resetBeamTracking() {
  _beamCondStartMs = 0;
  _beamStreamStartMs = 0;
  mapSamples = 0;
  mafSamples = 0;
  avgMAPLevel = 0.0f;
  avgMAFLevel = 0.0f;
  lastDeltaMAFLevelForBEAM = 0.0f;
  lastDeltaMAPLevelForBEAM = 0.0f;
  _holdActive = false;
  _holdStartMillis = 0;
  _beamVortexLevel = BEAM_VORTEX_ENTRY_LEVEL;
  _beamVortexLastUpdateMs = 0;
}

void StateMachine::resetVortexTwoLayerState() {
  _vortexFullActive = false;
  _vortexFullCondStartMs = 0;
  _vortexCmd = 0.0f;
}

float StateMachine::computeVortexTwoLayerCmd(uint32_t nowMs, float mafNorm, float pressurePctSigned) {
  constexpr float MAF_NEAR_MAX_TH = 0.80f;
  constexpr uint32_t HOLD_MS = 120u;
  constexpr float FULL_TARGET = 1.0f;
  constexpr float PASSIVE_MAX = 0.45f;
  constexpr float FULL_RAMP_UP_ALPHA = 0.22f;
  constexpr float FULL_RAMP_DOWN_ALPHA = 0.12f;

  bool vacuumDemandOn = (pressurePctSigned <= VACUUM_PCT_ON);
  bool vacuumDemandOff = (pressurePctSigned >= VACUUM_PCT_OFF);

  float vacuumNorm = 0.0f;
  if (pressurePctSigned < 0.0f) {
    vacuumNorm = constrain((-pressurePctSigned) / 20.0f, 0.0f, 1.0f);
  }
  float passiveCmd = vacuumNorm * PASSIVE_MAX;

  bool mafHigh = (mafNorm >= MAF_NEAR_MAX_TH);
  bool fullCond = vacuumDemandOn && mafHigh;

  if (fullCond) {
    if (_vortexFullCondStartMs == 0) _vortexFullCondStartMs = nowMs;
    if ((nowMs - _vortexFullCondStartMs) >= HOLD_MS) {
      _vortexFullActive = true;
    }
  } else {
    _vortexFullCondStartMs = 0;
  }

  if (_vortexFullActive && vacuumDemandOff) {
    _vortexFullActive = false;
  }

  float target = _vortexFullActive ? FULL_TARGET : passiveCmd;
  float alpha = _vortexFullActive ? FULL_RAMP_UP_ALPHA : FULL_RAMP_DOWN_ALPHA;
  _vortexCmd += (target - _vortexCmd) * alpha;
  _vortexCmd = constrain(_vortexCmd, 0.0f, 1.0f);
  return _vortexCmd;
}

float StateMachine::updateBeamVortexRamp(float mafPower) {
  mafPower = constrain(mafPower, 0.0f, 1.0f);
  uint32_t now = millis();
  if (_beamVortexLastUpdateMs == 0) {
    _beamVortexLastUpdateMs = now;
  }
  float dtMs = float(now - _beamVortexLastUpdateMs);
  _beamVortexLastUpdateMs = now;

  float tauFast = BEAM_VORTEX_RAMP_TAU_FAST_MS;
  float tauSlow = BEAM_VORTEX_RAMP_TAU_SLOW_MS;
  float tau = tauFast + (1.0f - mafPower) * (tauSlow - tauFast);
  tau = max(tau, 1.0f);

  float alpha = 1.0f - expf(-dtMs / tau);
  _beamVortexLevel += (1.0f - _beamVortexLevel) * alpha;
  if (_beamVortexLevel < BEAM_VORTEX_ENTRY_LEVEL) {
    _beamVortexLevel = BEAM_VORTEX_ENTRY_LEVEL;
  }

  return constrain(_beamVortexLevel, 0.0f, 1.0f);
}


void StateMachine::debugForceState(SystemState nuevoEstado) {
    if (current == SystemState::DEBUG) {
        current = nuevoEstado;
        Serial.print(">> Estado forzado a: ");
        Serial.println(static_cast<int>(nuevoEstado));
    }
}

String StateMachine::getStateName() const {
    switch (current) {
        case SystemState::OFF: return "OFF";
        case SystemState::NO_CALIB: return "NO_CALIB";
        case SystemState::CALIBRATION: return "CALIBRATION";
        case SystemState::IDLE: return "IDLE";
        case SystemState::ALIGN: return "ALIGN";
        case SystemState::FLOW: return "FLOW";
        case SystemState::DECAY: return "DECAY";
        case SystemState::DEBUG: return "DEBUG";
        case SystemState::UNKNOWN: return "UNKNOWN";
        default: return "UNKNOWN";
    }
}

// Llama a esto cada ciclo de control con los niveles 0..1
void StateMachine::compute_dMAFdt_and_hold(float newestMAFPercent,
                                           float oldestMAFPercent,
                                           float currentDeltaMAFLevel) {
  // derivada usando ventana completa (5 muestras => 4 intervalos)
  float window = 4.0f; // diferencia entre índice 4 y 0
  float raw = (newestMAFPercent - oldestMAFPercent) / window;

  _dMAFdtRaw = raw;

  // Para esta versión simple usamos el valor instantáneo como derivada filtrada
  _dMAFdtEMA = raw;

  // Ataque rápido activo solo si la pendiente supera el umbral positivo
  _fastAttackActive = (_dMAFdtEMA >= BEAM_MAF_ATTACK_MIN_DERIV);


  if (!_holdActive && currentDeltaMAFLevel > PRESS_EPS) {
    _holdActive = true;
    _holdStartMillis = millis();
  }
}
