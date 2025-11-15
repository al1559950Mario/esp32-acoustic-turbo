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
    current           = hasCalibration ? SystemState::IDLE : SystemState::NO_CALIB;
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
    lastDeltaTPSLevelForBOOST      = 0.0f;
    lastDeltaMAPLevelForBOOST      = 0.0f;

    if (actuators) {
        actuators->stopAcoustic();
        actuators->stopVortex();
    }

    lastState = current;
    Serial.print(">> StateMachine iniciado en estado: ");
    Serial.println(static_cast<int>(current));
}

float StateMachine::getLevel() const {
    return tpsNormalized;
}

bool StateMachine::readyForBOOST(float mapLoad, float tpsLoad) {
    /*return mapLoad >= thresholds.BOOST_MAP_ON
        && tpsLoad >= thresholds.BOOST_TPS_ON;
        */
    return tpsLoad >= thresholds.BOOST_TPS_ON;
}

bool StateMachine::readyForBEAM(float mapLoad, float tpsLoad) {
    /*return mapLoad >= thresholds.BEAM_MAP_ON
        && tpsLoad >= thresholds.BEAM_TPS_ON;
        */
    return tpsLoad >= thresholds.BEAM_TPS_ON;

}

void StateMachine::update(float mapLoadPercent,
                          float tpsLoadPercent,
                          bool serialCalibReq,
                          bool bleCalibReq,
                          bool calibLoaded,
                          const DebugManager &dbg) {
    if (current == SystemState::DEBUG) return;

    if (thresholdManager) {
        thresholds = thresholdManager->getThresholds();
    }

    // Mantener buffer de 5 muestras para TPS y MAP
    static float tpsBuffer[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static float mapBuffer[3] = {0.0f, 0.0f, 0.0f};

    // Desplazar las muestras anteriores
    tpsBuffer[0] = tpsBuffer[1];
    tpsBuffer[1] = tpsBuffer[2];
    tpsBuffer[2] = tpsBuffer[3];
    tpsBuffer[3] = tpsBuffer[4];
    tpsBuffer[4] = tpsLoadPercent;

    mapBuffer[0] = mapBuffer[1];
    mapBuffer[1] = mapBuffer[2];
    mapBuffer[2] = mapLoadPercent;

    // Aplicar mediana
    _tpsLoadPercent  = median(tpsBuffer[0], tpsBuffer[1], tpsBuffer[2], tpsBuffer[3], tpsBuffer[4]);
    _mapLoadPercent  = median(mapBuffer[0], mapBuffer[1], mapBuffer[2], mapBuffer[2], mapBuffer[2]);

    currentDeltaTPSLevelForBOOST = sensors->getRelativeTPSLevel(thresholds.BOOST_TPS_ON);
    currentDeltaMAPLevelForBOOST = sensors->getRelativeMAPLevel(thresholds.BOOST_MAP_ON);

    currentDeltaTPSLevelForBEAM = sensors->getRelativeTPSLevel(thresholds.BEAM_TPS_ON);
    currentDeltaMAPLevelForBEAM = sensors->getRelativeMAPLevel(thresholds.BEAM_MAP_ON);

    mapNormalized   = mapLoadPercent / 100.0f;
    tpsNormalized   = tpsLoadPercent / 100.0f;

    compute_dTPSdt_and_hold(currentDeltaTPSLevelForBEAM, lastDeltaTPSLevelForBOOST);


    switch (current) {
        static uint32_t offStartMillis = 0;

        case SystemState::OFF:
            if (offStartMillis == 0) offStartMillis = millis();
            if (actuators) {
                actuators->stopAcoustic();
                actuators->stopVortex();
            }
            if ((millis() - offStartMillis) > 500
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
            if (readyForBOOST(_mapLoadPercent, _tpsLoadPercent)) {
                current = SystemState::BOOST;
                if (sensors) {
                    tpsInitialPercent = sensors->readTPSLoadPercent();
                    mapInitialPercent = sensors->readMAPLoadPercent();
                }
                if (actuators) {
                    actuators->startVortex();
                    vortexPending     = true;
                    vortexStartMillis = millis();
                }
            }
            break;

        case SystemState::BOOST:
            {
            bool beamRaw = readyForBEAM(_mapLoadPercent, _tpsLoadPercent);
            unsigned long now = millis();
            if (beamRaw) {
                if (_beamCondStartMs == 0) _beamCondStartMs = now;
                if ((now - _beamCondStartMs) >= BEAM_COND_MIN_HOLD_MS) {
                    current = SystemState::BEAM;
                    _beamCondStartMs = 0; // reset tracker al entrar
                    actuators->stopAcoustic();
                    actuators->startAcoustic(0.005f, _dTPSdtEMA);
                    resetBeamVortexRamp(currentDeltaTPSLevelForBEAM);
                }
            } else {
                _beamCondStartMs = 0; // reset si la condición se rompe
            }

            if (_tpsLoadPercent <= thresholds.BOOST_TPS_OFF){
                current = SystemState::IDLE;
            }
            }
            break;

        case SystemState::BEAM: {
            // detectar caída usando derivada suavizada (unidades: nivel/sec)
            float deriv = _dTPSdtEMA; // ya calculada por compute_dTPSdt_and_hold
            bool dropDetected = (deriv <= -DERIV_DROP_THRESHOLD);
            belowThresholds = (_tpsLoadPercent <= thresholds.BEAM_TPS_OFF);

            if (belowThresholds) {
                unsigned long now = millis();

                // calcular hold_ms si estuvo activo
                unsigned long holdMs = 0;
                if (_holdActive && _holdStartMillis != 0) {
                    holdMs = now - _holdStartMillis;
                }

                // normalizar hold
                float holdNorm = float(min<unsigned long>(holdMs, H_SCALE_MS)) / float(H_SCALE_MS); // 0..1

                // sigmoide para mezcla entre toque y hold
                float x = (holdNorm - SIGMOID_H0) * SIGMOID_K;
                float w = 1.0f / (1.0f + expf(-x)); // peso_sustain 0..1

                // g_fast desde derivada negativa
                float gFastRaw = 0.0f;
                if (deriv < 0.0f) {
                    gFastRaw = constrain((-deriv) / DERIV_NORM, 0.0f, 1.0f);
                }

                // g_sustain desde holdNorm
                float gSustainRaw = constrain(holdNorm, 0.0f, 1.0f);

                // mezclar gains usando w: w=0 -> todo toque (gFast), w=1 -> todo hold (gSustain)
                float gFast = (1.0f - w) * gFastRaw;
                float gSustain = w * gSustainRaw;

                float durationMsF = float(decayDurationMs) * (MIN_SCALE * (1.0f - w) + MAX_SCALE * w);
                uint32_t durationMs = uint32_t(constrain(durationMsF, 1.0f, 60000.0f)); // limita rango seguro

                // tiempos explícitos que ya usas
                float tFast = _tFastMs;
                float tSustain = _tSustainMs;

                // disparar DECAY con parámetros ahora dinámicos
                decayStartMillis = now;
                current = SystemState::DECAY;
                vortexPending = false;
                actuators->getAcousticInjector().setDecayParameters(durationMs, avgTPSLevel,
                                                                    gFast, gSustain, (uint32_t)tFast, (uint32_t)tSustain);
                actuators->getAcousticInjector().startDecay(now);


                // reset hold tracker
                _holdActive = false;
                _holdStartMillis = 0;

            }

            break;
        }

        case SystemState::DECAY:
            // Interrumpir decay solo si la condición BEAM se mantiene
            {
            unsigned long now = millis();
            bool beamRaw = readyForBEAM(_mapLoadPercent, _tpsLoadPercent);
            if (beamRaw) {
                if (_beamCondStartMs == 0) _beamCondStartMs = now;
            } else {
                _beamCondStartMs = 0;
            }

            bool minDelayOk = (now - decayStartMillis >= MIN_BEAM_DELAY_MS);
            bool holdOk = (_beamCondStartMs != 0) && ((now - _beamCondStartMs) >= BEAM_COND_MIN_HOLD_MS);
            if (beamRaw && minDelayOk && holdOk) {
                current = SystemState::BEAM;  // o BEAM si así lo quieres
                _beamCondStartMs = 0;
                if (sensors) {
                    tpsInitialPercent = sensors->readTPSLoadPercent();
                    mapInitialPercent = sensors->readMAPLoadPercent();
                }
                if (actuators) {
                    actuators->stopAcoustic();
                    actuators->startAcoustic(0.005f, _dTPSdtEMA);
                    resetBeamVortexRamp(currentDeltaTPSLevelForBEAM);
                    vortexPending     = true;
                    vortexStartMillis = now;
                }
                break; // salimos del case para no seguir decay
            }
            }

            // Mantener lógica original de tiempo
            if (!actuators->decayFinished()) {
           
                current = SystemState::IDLE;
                if (actuators) {
                    actuators->stopAcoustic();
                    actuators->stopVortex();
                    vortexPending = false;
                }
            }
            break;
        case SystemState::DEBUG:
            break;

        case SystemState::UNKNOWN:
            current = SystemState::OFF;
            break;
    }

    if (current != lastState) {
        lastState = current;
    }
}

void StateMachine::handleActions() {
    if (!sensors || !actuators || !calibMgr) return;

    if (current == SystemState::BOOST) {
    // Solo turbo / vortex
    if (abs(currentDeltaTPSLevelForBOOST - lastDeltaTPSLevelForBOOST) > 0.01f
        || abs(currentDeltaMAPLevelForBOOST  - lastDeltaMAPLevelForBOOST ) > 0.01f) {
        //Usando solo TPS temporalmente
        actuators->updateVortexLevel(currentDeltaTPSLevelForBOOST, currentDeltaTPSLevelForBOOST);
        lastDeltaTPSLevelForBOOST = currentDeltaTPSLevelForBOOST;
        lastDeltaMAPLevelForBOOST = currentDeltaMAPLevelForBOOST;
        }
    }

    if (current == SystemState::BEAM) {
        if (thresholdManager) {
            thresholds = thresholdManager->getThresholds();
            }
        mapSamples++;
        tpsSamples++;

        avgMAPLevel += (currentDeltaMAPLevelForBEAM - avgMAPLevel) / float(mapSamples);
        avgTPSLevel += (currentDeltaTPSLevelForBEAM - avgTPSLevel) / float(tpsSamples);

        // Dinámica de MAF (antes TPS): ajustar potencia según rapidez
        float powerInput = currentDeltaTPSLevelForBEAM;
        float attackNorm = ( _dTPSdtEMA - MAF_ATTACK_SLOW_DTPS )
                         / (MAF_ATTACK_FAST_DTPS - MAF_ATTACK_SLOW_DTPS);
        attackNorm = constrain(attackNorm, 0.0f, 1.0f);
        float attackGain = MAF_ATTACK_MIN_GAIN
                         + attackNorm * (MAF_ATTACK_MAX_GAIN - MAF_ATTACK_MIN_GAIN);
        float power = powerInput * attackGain;
        if (_dTPSdtEMA < 0.0f) {
            float releaseNorm = constrain(_dTPSdtEMA / MAF_RELEASE_REF_DTPS, 0.0f, 1.0f);
            power *= (1.0f - 0.5f * releaseNorm); // reduce hasta 50 % en soltados rápidos
        }
        power = constrain(power, 0.0f, 1.0f);

        bool levelChanged = (abs(currentDeltaTPSLevelForBEAM - lastDeltaTPSLevelForBEAM) > 0.01f
                          || abs(currentDeltaMAPLevelForBEAM  - lastDeltaMAPLevelForBEAM ) > 0.01f);

        if (levelChanged) {
            actuators->setAcousticParameters(power, power);
            lastDeltaTPSLevelForBEAM = currentDeltaTPSLevelForBEAM;
            lastDeltaMAPLevelForBEAM = currentDeltaMAPLevelForBEAM;
        }

        float vortexLevel = updateBeamVortexRamp(power);
        actuators->updateVortexLevel(vortexLevel, vortexLevel);
        actuators->updateInjector();
    }

        // ──────── DECAY ───────────────────────────────────────────────────
    if (current == SystemState::DECAY) {

        actuators->getAcousticInjector().updateDecayState();

    }
}

void StateMachine::resetBeamVortexRamp(float seedLevel) {
  float seeded = constrain(seedLevel, 0.0f, 1.0f);
  _beamVortexLevel = max(BEAM_VORTEX_ENTRY_LEVEL, seeded);
  _beamVortexLastUpdateMs = millis();
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
        case SystemState::BOOST: return "BOOST";
        case SystemState::BEAM: return "BEAM";
        case SystemState::DECAY: return "DECAY";
        case SystemState::DEBUG: return "DEBUG";
        case SystemState::UNKNOWN: return "UNKNOWN";
        default: return "UNKNOWN";
    }
}

// Llama a esto cada ciclo de control con los niveles 0..1
void StateMachine::compute_dTPSdt_and_hold(float currentDeltaTPSLevel, float lastDeltaTPSLevel) {
  unsigned long now = millis();
  // primer llamado seguro: inicializar referencia temporal y salir
  if (_derivLastMillis == 0) {
    _derivLastMillis = now;
    _dTPSdtRaw = 0.0f;
    return;
  }

  unsigned long dt_ms = now - _derivLastMillis;
  if (dt_ms == 0) dt_ms = 1; // proteger

  // derivada en unidades por segundo (niveles 0..1)
  float raw = (currentDeltaTPSLevel - lastDeltaTPSLevel) / (float(dt_ms) / 1000.0f);

  // proteger contra outliers
  raw = constrain(raw, -10.0f, 10.0f);

  _dTPSdtRaw = raw;

  // EMA para suavizar la derivada
  _dTPSdtEMA = DERIV_EMA_ALPHA * raw + (1.0f - DERIV_EMA_ALPHA) * _dTPSdtEMA;

  if (!_holdActive && currentDeltaTPSLevel > PRESS_EPS) {
    _holdActive = true;
    _holdStartMillis = now;
  }

  _derivLastMillis = now;
}



