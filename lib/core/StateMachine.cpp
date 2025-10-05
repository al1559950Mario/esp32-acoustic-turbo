// StateMachine.cpp

#include "StateMachine.h"
#include <Arduino.h>

float median3(float a, float b, float c) {    
    if ((a >= b && a <= c) || (a >= c && a <= b)) return a;     
    if ((b >= a && b <= c) || (b >= c && b <= a)) return b;   
    return c;}

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
    lastTPSPercent      = 0.0f;
    lastMAPPercent      = 0.0f;

    if (actuators) {
        actuators->stopAcoustic();
        actuators->stopVortex();
    }

    lastState = current;
    Serial.print(">> StateMachine iniciado en estado: ");
    Serial.println(static_cast<int>(current));
}

SystemState StateMachine::getState() const {
    return current;
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
            if (readyForVortex(_mapLoadPercent, _tpsLoadPercent)) {
                current = SystemState::VORTEX;
            }
            else if (_mapLoadPercent <= thresholds.INJ_MAP_OFF
                  || _tpsLoadPercent <= thresholds.INJ_TPS_OFF) {
                // MODIFICADO: en vez de ir directo a IDLE, pasamos a COOLDOWN
                unsigned long now = millis();   
                current = SystemState::DECAY;
                decayStartMillis = now;
                actuators->getAcousticInjector().startDecay(now);
                vortexPending = false;
            }
            break;

        case SystemState::VORTEX:
            if (_mapLoadPercent < thresholds.VORTEX_MAP_OFF
             || _tpsLoadPercent < thresholds.VORTEX_TPS_OFF) {
                unsigned long now = millis();   
                current = SystemState::DECAY;
                vortexPending       = false;
                decayStartMillis = now;
                actuators->getAcousticInjector().startDecay(now);
            }
            break;

        case SystemState::DECAY:
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
            else if (_mapLoadPercent <= thresholds.INJ_MAP_OFF
                  || _tpsLoadPercent <= thresholds.INJ_TPS_OFF) {
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
        
        float deltaTPSPercent = sensors->getRelativeTPSLoad(tpsInitialPercent);
        float deltaMAPercent  = sensors->getRelativeMAPLoad(mapInitialPercent);

        if (abs(deltaTPSPercent - lastTPSPercent) > 0.1f
         || abs(deltaMAPercent  - lastMAPPercent ) > 0.1f) {
            actuators->setAcousticParameters(deltaTPSPercent, deltaMAPercent);
            lastTPSPercent = deltaTPSPercent;
            lastMAPPercent = deltaMAPercent;
        }

        // Normalización TPS y MAP
        float tpsRel = (sensors->readTPSLoadPercent() - tpsInitialPercent) /
                       (thresholds.VORTEX_TPS_ON - tpsInitialPercent);
        float mapRel = (sensors->readMAPLoadPercent() - mapInitialPercent) /
                       (thresholds.VORTEX_MAP_ON - mapInitialPercent);

        tpsRel = constrain(tpsRel, 0.0f, 1.0f);
        mapRel = constrain(mapRel, 0.0f, 1.0f);

        // Cálculo de vortexLevel original
        float vortexLevel = tpsRel * mapRel;
        vortexLevel = (vortexLevel > 1.0f) ? 1.0f : vortexLevel;

        // 🔹 Aplicar curva exponencial SOLO al level
        float a = 4.0f; // controla la aceleración al final
        float curvedLevel = (exp(a * vortexLevel) - 1.0f) / (exp(a) - 1.0f);

        actuators->setVortexLevel(curvedLevel);

        // NOTA: no tocamos la frecuencia
        actuators->update(_tpsLoadPercent, _mapLoadPercent);
    }

        // ──────── DECAY ───────────────────────────────────────────────────
    if (current == SystemState::DECAY) {
        // 1) Mantén la misma amplitud (vortexLevel) usando los últimos porcentajes
        actuators->setAcousticParameters(lastTPSPercent, lastMAPPercent);

        // 2) Update() avanzará la rampa de frecuencia basada en
        //    mapLoadToWaveFrequency() que internamente detecta DECAY
        actuators->update(lastTPSPercent, lastMAPPercent);

        // 3) Al completar el tiempo de decay, apaga y vuelve a IDLE
        if (millis() - decayStartMillis >= decayDurationMs) {
            actuators->stopAcoustic();
            actuators->stopVortex();
            current = SystemState::IDLE;
        }
        return;
    }
}

void StateMachine::debugForceState(SystemState nuevoEstado) {
    if (current == SystemState::DEBUG) {
        current = nuevoEstado;
        Serial.print(">> Estado forzado a: ");
        Serial.println(static_cast<int>(nuevoEstado));
    }
}
