// StateMachine.cpp

#include "StateMachine.h"
#include <Arduino.h>

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
    cooldownStartMillis = 0;
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

    _mapLoadPercent = mapLoadPercent;
    _tpsLoadPercent = tpsLoadPercent;
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
            // OFF si MAP baja de wakeup, independientemente del TPS
            if (_mapLoadPercent < thresholds.MAP_WAKEUP_PERCENT) {
                current = SystemState::OFF;
                if (actuators) {
                    actuators->stopAcoustic();
                    actuators->stopVortex();
                    vortexPending = false;
                }
                break;
            }
            // BEAM si ambos cruzan sus ON thresholds
            if (readyForInjection(_mapLoadPercent, _tpsLoadPercent)) {
                current = SystemState::BEAM;
                if (sensors) {
                    tpsInitialPercent = sensors->readTPSLoadPercent();
                    mapInitialPercent = sensors->readMAPLoadPercent();
                }
                if (actuators) {
                    actuators->stopAcoustic();            // 1) detiene y resetea el injector
                    actuators->startAcoustic(0.1f);
                    vortexPending     = true;
                    vortexStartMillis = millis();
                }
            }
            break;

        case SystemState::BEAM:
            // VORTEX si ambos vuelven a ON
            if (readyForVortex(_mapLoadPercent, _tpsLoadPercent)) {
                current = SystemState::VORTEX;
            }
            // IDLE si baja cualquiera de los dos sensores
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

        case SystemState::VORTEX:
            // COOLDOWN si cae cualquiera por debajo de OFF
            if (_mapLoadPercent < thresholds.VORTEX_MAP_OFF
             || _tpsLoadPercent < thresholds.VORTEX_TPS_OFF) {
                current = SystemState::COOLDOWN;
                // no detener Vortex para rampa suave
                vortexPending       = false;
                cooldownStartMillis = millis();
            }
            break;

        case SystemState::COOLDOWN:
            // Mantener COOLDOWN el tiempo mínimo
            if ((millis() - cooldownStartMillis) < cooldownDurationMs) {
                break;
            }
            // Reingresar a VORTEX si vuelve a ON ambos
            if (readyForVortex(_mapLoadPercent, _tpsLoadPercent)) {
                current = SystemState::VORTEX;
                vortexPending     = true;
                vortexStartMillis = millis();
            }
            // Reingresar a BEAM si vuelve a ON inyección
            else if (readyForInjection(_mapLoadPercent, _tpsLoadPercent)) {
                current = SystemState::BEAM;
                if (sensors) {
                    tpsInitialPercent = sensors->readTPSLoadPercent();
                    mapInitialPercent = sensors->readMAPLoadPercent();
                }
                if (actuators) {
                    actuators->stopAcoustic();            // 1) detiene y resetea el injector
                    actuators->startAcoustic(0.1f);
                    vortexPending     = true;
                    vortexStartMillis = millis();
                }
            }
            // IDLE si baja cualquiera de los dos sensores
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
     || current == SystemState::VORTEX
     || current == SystemState::COOLDOWN) {
        float deltaTPSPercent = sensors->getRelativeTPSLoad(tpsInitialPercent);
        float deltaMAPercent  = sensors->getRelativeMAPLoad(mapInitialPercent);

        if (abs(deltaTPSPercent - lastTPSPercent) > 0.1f
         || abs(deltaMAPercent  - lastMAPPercent ) > 0.1f) {
            actuators->setAcousticParameters(deltaTPSPercent, deltaMAPercent);
            lastTPSPercent = deltaTPSPercent;
            lastMAPPercent = deltaMAPercent;
        }

        // ==== Turbo escalado relativo ====
        float tpsRel = (sensors->readTPSLoadPercent() - tpsInitialPercent) /
                       (thresholds.VORTEX_TPS_ON - tpsInitialPercent);
        float mapRel = (sensors->readMAPLoadPercent() - mapInitialPercent) /
                       (thresholds.VORTEX_MAP_ON - mapInitialPercent);

        tpsRel = constrain(tpsRel, 0.0f, 1.0f);
        mapRel = constrain(mapRel, 0.0f, 1.0f);

        float vortexLevel = tpsRel * mapRel;  // Escalado combinado TPS*MAP
        // Saturar a 100% si el cálculo excede 1.0
        vortexLevel = (vortexLevel > 1.0f) ? 1.0f : vortexLevel;
        actuators->setVortexLevel(vortexLevel);

        // Actualizar actuadores generales
        actuators->update(_tpsLoadPercent, _mapLoadPercent);
    }

    // Activar vortex tras delay inicial
    if (vortexPending && (millis() - vortexStartMillis >= vortexDelayMs)) {
        actuators->startVortex();
        vortexPending = false;
    }
}

void StateMachine::debugForceState(SystemState nuevoEstado) {
    if (current == SystemState::DEBUG) {
        current = nuevoEstado;
        Serial.print(">> Estado forzado a: ");
        Serial.println(static_cast<int>(nuevoEstado));
    }
}
