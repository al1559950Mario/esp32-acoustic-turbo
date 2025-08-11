#include "StateMachine.h"
#include <Arduino.h>  // Para Serial

void StateMachine::begin(bool hasCalibration, ActuatorManager* actuatorsPtr, ThresholdManager* thresholdManagerPtr, SensorManager* sensorsPtr, CalibrationManager* calibMgrPtr) {
  current = hasCalibration
              ? SystemState::OFF
              : SystemState::NO_CALIB;
  this->sensors = sensorsPtr;
  this->actuators = actuatorsPtr;
  this->calibMgr = calibMgrPtr;

  thresholdManager = thresholdManagerPtr;
  if (thresholdManager) {
    thresholds = thresholdManager->getThresholds();
  }

  Serial.print(">> StateMachine iniciado en estado: ");
  Serial.println(static_cast<int>(current));
}

SystemState StateMachine::getState() const {
  return current;
}

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

  tpsNormalized = tpsLoadPercent / 100.0f;
  mapNormalized = mapLoadPercent / 100.0f;

  switch (current) {

    case SystemState::OFF:
      if (mapLoadPercent > thresholds.MAP_WAKEUP_PERCENT) {

        current = SystemState::IDLE;
        Serial.println("→ Transición: OFF → IDLE");
      }
      break;

    case SystemState::NO_CALIB:
      if (serialCalibReq || bleCalibReq) {
        current = SystemState::CALIBRATION;
        Serial.println("→ Transición: SIN_CALIBRAR → CALIBRATION");
      } else if (calibLoaded) {
        current = SystemState::OFF;
        Serial.println("→ Transición: SIN_CALIBRAR → OFF (calibración detectada)");
      }
      break;

    case SystemState::CALIBRATION:
      //calibMgr->update();  // Avanza la calibración sin bloquear

      if (calibLoaded) {
        current = SystemState::OFF;
        Serial.println("→ Transición: CALIBRATION → OFF");
      }
      break;

    case SystemState::IDLE:
      if (mapLoadPercent < 4.0f) {
        current = SystemState::OFF;
        Serial.println("→ Transición: IDLE → OFF (MAP = 0%)");
        break;
      }
      if (readyForInjection( mapLoadPercent)) {
        current = SystemState::BEAM;
        if (!actuators->isAcousticOn()) {
          actuators->startAcoustic(0.1f);
          // Guardar bases para escalado
          if (!sensors) {
            Serial.println("Error: SensorManager no está inicializado");
            return;
          }
          tpsInitialPercent = sensors->readTPSLoadPercent();
          mapInitialPercent = sensors->readMAPLoadPercent();

        }
        Serial.println("→ Transición: IDLE → BEAM");
      }
      break;

    case SystemState::BEAM:
      if (tpsLoadPercent >= thresholds.VORTEX_TPS_ON && mapLoadPercent >= thresholds.VORTEX_MAP_ON) {
        current = SystemState::VORTEX;
        actuators->startVortex();
        Serial.println("→ Transición: BEAM → VORTEX");
      }
      else if (tpsLoadPercent <= thresholds.INJ_TPS_OFF && mapLoadPercent <= thresholds.INJ_MAP_OFF) {
        current = SystemState::IDLE;
        actuators->stopAcoustic();
        Serial.println("→ Transición: BEAM → IDLE");
      }
      break;

    case SystemState::VORTEX:
      if (tpsLoadPercent < thresholds.VORTEX_TPS_OFF) {
        current = SystemState::COOLDOWN;
        actuators->stopVortex();
        Serial.println("→ Transición: VORTEX → COOLDOWN");
      }
      break;

    case SystemState::COOLDOWN:
      if (readyForInjection(mapLoadPercent)) {
        current = SystemState::BEAM;
        if (!actuators->isAcousticOn()) {
          actuators->startAcoustic(0.1f);
          
        }
        Serial.println("→ Transición: COOLDOWN → BEAM");
      }
      else if (tpsLoadPercent <= thresholds.INJ_TPS_OFF || mapLoadPercent <= thresholds.INJ_MAP_OFF) {
        current = SystemState::IDLE;
        actuators->stopAcoustic();
        Serial.println("→ Transición: COOLDOWN → IDLE");
      }
      break;

    case SystemState::DEBUG:
      break;
    case SystemState::UNKNOWN:
      Serial.println(">> Estado UNKNOWN detectado, reseteando a OFF");
      current = SystemState::OFF;
      break;

  }
}

void StateMachine::handleActions() {
  if (current == SystemState::BEAM || current == SystemState::VORTEX || current == SystemState::COOLDOWN) {
    if (sensors == nullptr) {
      return;
    }
    if (actuators == nullptr) {
      return;
    }
    if (calibMgr == nullptr) {
      return;
    }

    float deltaTPSPercent = sensors->getRelativeTPSLoad(tpsInitialPercent);     // ←  // ← valor entre 0.0 y 100.0 (porcentaje)
    float deltaMAPercent = sensors->getRelativeMAPLoad(mapInitialPercent);     // ←  // ← valor entre 0.0 y 100.0 (porcentaje)
    actuators->setAcousticParameters(deltaTPSPercent, deltaMAPercent); 
    actuators->update();

  }
}





void StateMachine::debugForceState(SystemState nuevoEstado) {
  if (current == SystemState::DEBUG) {
    current = nuevoEstado;
    Serial.print(">> Estado forzado a: ");
    Serial.println(static_cast<int>(nuevoEstado));
  }
}

float StateMachine::getLevel() const {
  return tpsNormalized;
}

bool StateMachine::readyForInjection(float mapLoad) {
  return  mapLoad >= thresholds.INJ_MAP_ON;
}
