#include "StateMachine.h"
#include <Arduino.h>  // Para Serial

// ======= BEGIN =======

void StateMachine::begin(bool hasCalibration,
                         ActuatorManager* actuatorsPtr,
                         ThresholdManager* thresholdManagerPtr,
                         SensorManager* sensorsPtr,
                         CalibrationManager* calibMgrPtr) {
    current = hasCalibration ? SystemState::OFF : SystemState::NO_CALIB;
    this->sensors = sensorsPtr;
    this->actuators = actuatorsPtr;
    this->calibMgr = calibMgrPtr;
    thresholdManager = thresholdManagerPtr;

    if (thresholdManager) {
        thresholds = thresholdManager->getThresholds();
    }

    // Inicializa temporizador vortex
    vortexPending = false;
    vortexStartMillis = 0;

    lastState = current;  // Para Serial limitado
    Serial.print(">> StateMachine iniciado en estado: ");
    Serial.println(static_cast<int>(current));
}

SystemState StateMachine::getState() const {
    return current;
}

// ======= UPDATE =======
void StateMachine::update(float mapLoadPercent,
                          float tpsLoadPercent,
                          bool serialCalibReq,
                          bool bleCalibReq,
                          bool calibLoaded,
                          const DebugManager &dbg) {
  
  if (current == SystemState::DEBUG) {
    return;
  }

  // Actualizar umbrales dinámicamente
  if (thresholdManager) {
    thresholds = thresholdManager->getThresholds();
  }
  _mapLoadPercent = mapLoadPercent;
  _tpsLoadPercent = tpsLoadPercent;
  tpsNormalized = tpsLoadPercent / 100.0f;
  mapNormalized = mapLoadPercent / 100.0f;

    switch (current) {
        case SystemState::OFF:
            if (mapLoadPercent > thresholds.MAP_WAKEUP_PERCENT) {
                current = SystemState::IDLE;
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
            if (mapLoadPercent < 4.0f) {
                current = SystemState::OFF;
                if (actuators) {
                    actuators->stopAcoustic();
                    actuators->stopVortex();
                    vortexPending = false;
                }
                break;
            }
            if (readyForInjection(mapLoadPercent)) {
                current = SystemState::BEAM;
                if (actuators && !actuators->isAcousticOn()) {
                    actuators->startAcoustic(0.1f);
                    vortexStartMillis = millis();
                    vortexPending = true;

                    if (sensors) {
                        tpsInitialPercent = sensors->readTPSLoadPercent();
                        mapInitialPercent = sensors->readMAPLoadPercent();
                    }
                }
            }
            break;

        case SystemState::BEAM:
            if (tpsLoadPercent >= thresholds.VORTEX_TPS_ON && mapLoadPercent >= thresholds.VORTEX_MAP_ON) {
                current = SystemState::VORTEX;
            } else if (tpsLoadPercent <= thresholds.INJ_TPS_OFF && mapLoadPercent <= thresholds.INJ_MAP_OFF) {
                current = SystemState::IDLE;
                if (actuators) {
                    actuators->stopAcoustic();
                    actuators->stopVortex();
                    vortexPending = false;
                }
            }
            break;

        case SystemState::VORTEX:
            if (tpsLoadPercent < thresholds.VORTEX_TPS_OFF) {
                current = SystemState::COOLDOWN;
            }
            break;

        case SystemState::COOLDOWN:
            if (readyForInjection(mapLoadPercent)) {
                current = SystemState::BEAM;
                if (actuators && !actuators->isAcousticOn()) {
                    actuators->startAcoustic(0.1f);
                    vortexStartMillis = millis();
                    vortexPending = true;
                }
            } else if (tpsLoadPercent <= thresholds.INJ_TPS_OFF || mapLoadPercent <= thresholds.INJ_MAP_OFF) {
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

    // Serial solo si cambio de estado
    if (current != lastState) {
        Serial.print("→ Transición: ");
        Serial.print(static_cast<int>(lastState));
        Serial.print(" → ");
        Serial.println(static_cast<int>(current));
        lastState = current;
    }
}

// ======= HANDLE ACTIONS =======
void StateMachine::handleActions() {
    if (!sensors || !actuators || !calibMgr) return;

    if (current == SystemState::BEAM || current == SystemState::VORTEX || current == SystemState::COOLDOWN) {
        // ==== Acoustic Injector original ====
        float deltaTPSPercent = sensors->getRelativeTPSLoad(tpsInitialPercent);
        float deltaMAPercent = sensors->getRelativeMAPLoad(mapInitialPercent);

        // Solo actualizar Acoustic Injector si hay cambio significativo
        if (abs(deltaTPSPercent - lastTPSPercent) > 0.1f ||
            abs(deltaMAPercent - lastMAPPercent) > 0.1f) {
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
        Serial.println(">> Vortex activado tras timing inicial");
    }
}

// ======= DEBUG FORCE STATE =======
void StateMachine::debugForceState(SystemState nuevoEstado) {
    if (current == SystemState::DEBUG) {
        current = nuevoEstado;
        Serial.print(">> Estado forzado a: ");
        Serial.println(static_cast<int>(nuevoEstado));
    }
}

// ======= GET LEVEL =======
float StateMachine::getLevel() const {
    return tpsNormalized;
}

// ======= READY FOR INJECTION =======
bool StateMachine::readyForInjection(float mapLoad) {
    return mapLoad >= thresholds.INJ_MAP_ON;
}
