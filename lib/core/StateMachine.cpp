// StateMachine.cpp

#include "StateMachine.h"
#include <Arduino.h>

float median3(float a, float b, float c) {
    if ((a >= b && a <= c) || (a >= c && a <= b)) return a;
    if ((b >= a && b <= c) || (b >= c && b <= a)) return b;
    return c;
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
    lastTPSLevel      = 0.0f;
    lastMAPLevel      = 0.0f;

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

bool StateMachine::readyForInjection(float mapLoad, float tpsLoad) {
    return mapLoad >= thresholds.INJ_MAP_ON
        && tpsLoad >= thresholds.INJ_TPS_ON;
}

bool StateMachine::readyForVortex(float mapLoad, float tpsLoad) {
    return mapLoad >= thresholds.VORTEX_MAP_ON
        && tpsLoad >= thresholds.VORTEX_TPS_ON;
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

        // Mantener buffer de 3 muestras para TPS y MAP
    static float tpsBuffer[3] = {0.0f, 0.0f, 0.0f};
    static float mapBuffer[3] = {0.0f, 0.0f, 0.0f};

    // Desplazar las muestras anteriores
    tpsBuffer[0] = tpsBuffer[1];
    tpsBuffer[1] = tpsBuffer[2];
    tpsBuffer[2] = tpsLoadPercent;

    mapBuffer[0] = mapBuffer[1];
    mapBuffer[1] = mapBuffer[2];
    mapBuffer[2] = mapLoadPercent;

    // Aplicar mediana
    _tpsLoadPercent  = median3(tpsBuffer[0], tpsBuffer[1], tpsBuffer[2]);
    _mapLoadPercent  = median3(mapBuffer[0], mapBuffer[1], mapBuffer[2]);
    
    mapNormalized   = mapLoadPercent / 100.0f;
    tpsNormalized   = tpsLoadPercent / 100.0f;

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
            if (_mapLoadPercent < thresholds.MAP_WAKEUP_PERCENT) {
                current = SystemState::OFF;
                if (actuators) {
                    actuators->stopAcoustic();
                    actuators->stopVortex();
                    vortexPending = false;
                }
                break;
            }
            if (readyForInjection(_mapLoadPercent, _tpsLoadPercent)) {
                current = SystemState::BEAM;
                if (sensors) {
                    tpsInitialPercent = sensors->readTPSLoadPercent();
                    mapInitialPercent = sensors->readMAPLoadPercent();
                }
                if (actuators) {
                    actuators->stopAcoustic();
                    actuators->startAcoustic(0.1f);
                    vortexPending     = true;
                    vortexStartMillis = millis();
                }
            }
            break;

        case SystemState::BEAM:
            mapDrop = (lastMAPLevel * 100.0f) - _mapLoadPercent;
            tpsDrop = (lastTPSLevel * 100.0f) - _tpsLoadPercent;
            dropDetected = (mapDrop >= MAP_DROP_THRESHOLD && tpsDrop >= TPS_DROP_THRESHOLD);
            belowThresholds = (_mapLoadPercent <= thresholds.INJ_MAP_OFF
                     || _tpsLoadPercent <= thresholds.INJ_TPS_OFF);
            if (readyForVortex(_mapLoadPercent, _tpsLoadPercent)) {
                current = SystemState::VORTEX;
            }
            else if (dropDetected || belowThresholds){
                // MODIFICADO: en vez de ir directo a IDLE, pasamos a COOLDOWN
                unsigned long now = millis();   

                current = SystemState::DECAY;
                decayStartMillis = now;
                actuators->getAcousticInjector().startDecay(now);
                actuators->getAcousticInjector().setDecayLevel(avgMAPLevel);
                actuators->getAcousticInjector().setDecayParameters(decayDurationMs, avgMAPLevel);
                //decayDurationMs = minDecay + (maxDecay - minDecay) * avgMAPLevel;

                vortexPending = false;
                // no apagamos Acoustic aquí, se apaga al terminar el cooldown
            }
            break;

        case SystemState::VORTEX:
            mapDrop = (lastMAPLevel * 100.0f) - _mapLoadPercent;
            tpsDrop = (lastTPSLevel * 100.0f) - _tpsLoadPercent;
            dropDetected = (mapDrop >= MAP_DROP_THRESHOLD && tpsDrop >= TPS_DROP_THRESHOLD);
            belowThresholds = (_mapLoadPercent <= thresholds.INJ_MAP_OFF
                     || _tpsLoadPercent <= thresholds.INJ_TPS_OFF);
            if (dropDetected || belowThresholds) {
                unsigned long now = millis();   
                current = SystemState::DECAY;
                vortexPending       = false;
                decayStartMillis = now;
                actuators->getAcousticInjector().startDecay(now);
                actuators->getAcousticInjector().setDecayLevel(avgMAPLevel);
                actuators->getAcousticInjector().setDecayParameters(decayDurationMs, avgMAPLevel);
                //decayDurationMs = minDecay + (maxDecay - minDecay) * avgMAPLevel;

            }
            break;

        case SystemState::DECAY:
            if ((millis() - decayStartMillis) < decayDurationMs) {
                break;
            }
            if (readyForVortex(_mapLoadPercent, _tpsLoadPercent)) {
                current = SystemState::VORTEX;
                vortexPending     = true;
                vortexStartMillis = millis();
            }
            else if (readyForInjection(_mapLoadPercent, _tpsLoadPercent)) {
                current = SystemState::BEAM;
                if (sensors) {
                    tpsInitialPercent = sensors->readTPSLoadPercent();
                    mapInitialPercent = sensors->readMAPLoadPercent();
                }
                if (actuators) {
                    actuators->stopAcoustic();
                    actuators->startAcoustic(0.1f);
                    vortexPending     = true;
                    vortexStartMillis = millis();
                }
            }
            else if (millis() - decayStartMillis >= decayDurationMs) {
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

    if (current == SystemState::BEAM
     || current == SystemState::VORTEX) {
        if (thresholdManager) {
            thresholds = thresholdManager->getThresholds();
            }

        
        float deltaTPSLevel = sensors->getRelativeTPSLevel(thresholds.INJ_TPS_ON);
        float deltaMAPLevel  = sensors->getRelativeMAPLevel(thresholds.INJ_MAP_ON);
        mapSamples++;
        avgMAPLevel += (deltaMAPLevel - avgMAPLevel) / float(mapSamples);


        if (abs(deltaTPSLevel - lastTPSLevel) > 0.03f
         || abs(deltaMAPLevel  - lastMAPLevel ) > 0.03f) {
            actuators->setAcousticParameters(deltaTPSLevel, deltaMAPLevel);
            actuators->setVortexLevel(deltaTPSLevel, deltaMAPLevel);
            lastTPSLevel = deltaTPSLevel;
            lastMAPLevel = deltaMAPLevel;
        }
        actuators->updateInjector();
    }

        // ──────── DECAY ───────────────────────────────────────────────────
    if (current == SystemState::DECAY) {
        actuators->getAcousticInjector().updateDecayState();
    }
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
        case SystemState::BEAM: return "BEAM";
        case SystemState::VORTEX: return "VORTEX";
        case SystemState::DECAY: return "DECAY";
        case SystemState::DEBUG: return "DEBUG";
        case SystemState::UNKNOWN: return "UNKNOWN";
        default: return "UNKNOWN";
    }
}

