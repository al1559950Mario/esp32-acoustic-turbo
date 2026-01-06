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
    _fastAttackActive = false;
    _pressurePercent = 0.0f;
    _pressureDelta = 0.0f;
    _lastPressurePercent = 0.0f;
    lastDeltaMAFLevelForBOOST      = 0.0f;
    lastDeltaMAPLevelForBOOST      = 0.0f;
    _dMAFdtRaw = 0.0f;
    _dMAFdtEMA = 0.0f;

    if (actuators) {
        actuators->stopAcoustic();
        actuators->stopVortex();
    }

    lastState = current;
    Serial.print(">> StateMachine iniciado en estado: ");
    Serial.println(static_cast<int>(current));
}

float StateMachine::getLevel() const {
    return mafNormalized;
}

bool StateMachine::readyForBOOST(float mapLoad, float mafLoad) {
    /*return mapLoad >= thresholds.BOOST_MAP_ON
        && mafLoad >= thresholds.BOOST_TPS_ON;
        */
    return mafLoad >= thresholds.BOOST_TPS_ON;
}

bool StateMachine::readyForBEAM(float mapLoad, float mafLoad) {
    /*return mapLoad >= thresholds.BEAM_MAP_ON
        && mafLoad >= thresholds.BEAM_TPS_ON;
        */
    return mafLoad >= thresholds.BEAM_TPS_ON;

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
    pressureBuffer[2] = sensors ? sensors->getPressurePercent() : 0.0f;

    // Aplicar mediana
    _mafLoadPercent  = median(mafBuffer[0], mafBuffer[1], mafBuffer[2], mafBuffer[3], mafBuffer[4]);
    _mapLoadPercent  = median(mapBuffer[0], mapBuffer[1], mapBuffer[2], mapBuffer[2], mapBuffer[2]);
    _pressurePercent = median(pressureBuffer[0], pressureBuffer[1], pressureBuffer[2], pressureBuffer[2], pressureBuffer[2]);
    _pressureDelta   = _pressurePercent - _lastPressurePercent;
    _lastPressurePercent = _pressurePercent;

    currentDeltaMAFLevelForBOOST = sensors->getRelativeMAFLevel(thresholds.BOOST_TPS_ON);
    currentDeltaMAPLevelForBOOST = sensors->getRelativeMAPLevel(thresholds.BOOST_MAP_ON);

    currentDeltaMAFLevelForBEAM = sensors->getRelativeMAFLevel(thresholds.BEAM_TPS_ON);
    currentDeltaMAPLevelForBEAM = sensors->getRelativeMAPLevel(thresholds.BEAM_MAP_ON);

    mapNormalized   = mapLoadPercent / 100.0f;
    mafNormalized   = mafLoadPercent / 100.0f;

    compute_dMAFdt_and_hold(mafBuffer[4], mafBuffer[0], currentDeltaMAFLevelForBEAM);


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
            if (readyForBOOST(_mapLoadPercent, _mafLoadPercent)) {
                current = SystemState::BOOST;
                if (sensors) {
                    mafInitialPercent = sensors->readMAFLoadPercent();
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
            bool beamRaw = readyForBEAM(_mapLoadPercent, _mafLoadPercent);
            bool attackReady = _fastAttackActive;
            bool vacuum = (_pressurePercent <= BEAM_VACUUM_PCT_ON);
            bool pressureRiseReady = (_pressureDelta >= BEAM_VACUUM_DELTA_MIN);
            unsigned long now = millis();
            
            //if (beamRaw || attackReady || pressureReady || pressureRiseReady) {
            /*
            if ( pressureReady && pressureRiseReady) {

                Serial.printf("[BEAM][BOOST] raw=%d attack=%d pres=%.1f%% dPres=%.2f presReady=%d dReady=%d\n",
                              beamRaw ? 1 : 0,
                              attackReady ? 1 : 0,
                              _pressurePercent,
                              _pressureDelta,
                              pressureReady ? 1 : 0,
                              pressureRiseReady ? 1 : 0);
            }
            
            */
            
            //if (beamRaw && attackReady && pressureReady && pressureRiseReady) {
            if (vacuum) {
                if (_beamCondStartMs == 0) _beamCondStartMs = now;
                if ((now - _beamCondStartMs) >= BEAM_COND_MIN_HOLD_MS) {
                    resetBeamTracking();
                    current = SystemState::BEAM;
                    _beamCondStartMs = 0; // reset tracker al entrar
                    _beamStreamStartMs = now;
                    actuators->stopAcoustic();
                    actuators->startAcoustic(0.005f, _dMAFdtEMA);
                    resetBeamVortexRamp(currentDeltaMAFLevelForBEAM);
                }
            } else {
                _beamCondStartMs = 0; // reset si la condición se rompe
            }

            if (_mafLoadPercent <= thresholds.BOOST_TPS_OFF){
                current = SystemState::IDLE;
            }
            }
            break;

        case SystemState::BEAM: {
            // detectar caída usando derivada suavizada (unidades: nivel/sec)
            float deriv = _dMAFdtEMA; // ya calculada por compute_dMAFdt_and_hold
            bool dropDetected = (deriv <= -DERIV_DROP_THRESHOLD);
            belowThresholds = (_mafLoadPercent <= thresholds.BEAM_TPS_OFF);
            bool beamExit = (_pressurePercent >= BEAM_VACUUM_PCT_OFF);
            unsigned long now = millis();
            if (_beamStreamStartMs == 0) {
                _beamStreamStartMs = now;
            }
            bool beamMinTimeMet = (now - _beamStreamStartMs) >= BEAM_MIN_STREAM_MS;
            if (belowThresholds && !beamMinTimeMet) {
                belowThresholds = false;
            }

            if (beamExit) {

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
                _beamStreamStartMs = 0;
                vortexPending = false;
                actuators->getAcousticInjector().setDecayParameters(durationMs, avgMAFLevel,
                                                                    gFast, gSustain, (uint32_t)tFast, (uint32_t)tSustain);
                actuators->getAcousticInjector().startDecay(now);
                resetBeamTracking();


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
            bool beamRaw = readyForBEAM(_mapLoadPercent, _mafLoadPercent);
            bool attackReady = _fastAttackActive;
            bool pressureReady = (_pressurePercent <= BEAM_VACUUM_PCT_ON);
            bool pressureRiseReady = (_pressureDelta >= BEAM_VACUUM_DELTA_MIN);
            if (pressureReady && pressureRiseReady) {
                if (_beamCondStartMs == 0) _beamCondStartMs = now;
            } else {
                _beamCondStartMs = 0;
            }

            bool minDelayOk = (now - decayStartMillis >= MIN_BEAM_DELAY_MS);
            bool holdOk = (_beamCondStartMs != 0) && ((now - _beamCondStartMs) >= BEAM_COND_MIN_HOLD_MS);
            //if (beamRaw && attackReady && pressureReady && pressureRiseReady && minDelayOk && holdOk) {
            if (pressureRiseReady && minDelayOk && holdOk) {
                resetBeamTracking();
                current = SystemState::BEAM;  // o BEAM si así lo quieres
                _beamCondStartMs = 0;
                if (sensors) {
                    mafInitialPercent = sensors->readMAFLoadPercent();
                    mapInitialPercent = sensors->readMAPLoadPercent();
                }
                if (actuators) {
                    actuators->stopAcoustic();
                    actuators->startAcoustic(0.005f, _dMAFdtEMA);
                    resetBeamVortexRamp(currentDeltaMAFLevelForBEAM);
                    vortexPending     = true;
                    vortexStartMillis = now;
                }
                break; // salimos del case para no seguir decay
            }
            }

            // Mantener lógica original de tiempo
            if (actuators->decayFinished()) {
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
    if (abs(currentDeltaMAFLevelForBOOST - lastDeltaMAFLevelForBOOST) > 0.01f
        || abs(currentDeltaMAPLevelForBOOST  - lastDeltaMAPLevelForBOOST ) > 0.01f) {
        //Usando solo MAF temporalmente
        actuators->updateVortexLevel(currentDeltaMAFLevelForBOOST, currentDeltaMAFLevelForBOOST);
        lastDeltaMAFLevelForBOOST = currentDeltaMAFLevelForBOOST;
        lastDeltaMAPLevelForBOOST = currentDeltaMAPLevelForBOOST;
        }
    }

    if (current == SystemState::BEAM) {
        if (thresholdManager) {
            thresholds = thresholdManager->getThresholds();
            }
        mapSamples++;
        mafSamples++;

        avgMAPLevel += (currentDeltaMAPLevelForBEAM - avgMAPLevel) / float(mapSamples);
        avgMAFLevel += (currentDeltaMAFLevelForBEAM - avgMAFLevel) / float(mafSamples);

        // Dinámica de MAF (antes TPS): ajustar potencia según rapidez
        float powerInput = currentDeltaMAFLevelForBEAM;
        float attackNorm = ( _dMAFdtEMA - MAF_ATTACK_SLOW_DTPS )
                         / (MAF_ATTACK_FAST_DTPS - MAF_ATTACK_SLOW_DTPS);
        attackNorm = constrain(attackNorm, 0.0f, 1.0f);
        float attackGain = MAF_ATTACK_MIN_GAIN
                         + attackNorm * (MAF_ATTACK_MAX_GAIN - MAF_ATTACK_MIN_GAIN);

        float power = powerInput * attackGain;
        if (_dMAFdtEMA < 0.0f) {
            float releaseNorm = constrain(_dMAFdtEMA / MAF_RELEASE_REF_DTPS, 0.0f, 1.0f);
            power *= (1.0f - 0.5f * releaseNorm); // reduce hasta 50 % en soltados rápidos
        }
        power = constrain(power, 0.0f, 1.0f);

        bool levelChanged = (abs(currentDeltaMAFLevelForBEAM - lastDeltaMAFLevelForBEAM) > 0.01f
                          || abs(currentDeltaMAPLevelForBEAM  - lastDeltaMAPLevelForBEAM ) > 0.01f);

        if (levelChanged) {
            actuators->setAcousticParameters(power, power);
            lastDeltaMAFLevelForBEAM = currentDeltaMAFLevelForBEAM;
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

void StateMachine::resetBeamTracking() {
  _beamCondStartMs = 0;
  _beamStreamStartMs = 0;
  mapSamples = 0;
  mafSamples = 0;
  avgMAPLevel = 0.0f;
  avgMAFLevel = 0.0f;
  lastDeltaMAFLevelForBEAM = 0.0f;
  lastDeltaMAPLevelForBEAM = 0.0f;
  mafMinOnBeam = 100.0f;
  _holdActive = false;
  _holdStartMillis = 0;
  _beamVortexLevel = BEAM_VORTEX_ENTRY_LEVEL;
  _beamVortexLastUpdateMs = 0;
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




