#include "StateMachine.h"
#include <Arduino.h>  // Para Serial

void StateMachine::begin(bool hasCalibration, ActuatorManager* actuatorsPtr, ThresholdManager* thresholdManagerPtr) {
  current = hasCalibration
              ? SystemState::OFF
              : SystemState::SIN_CALIBRAR;

  actuators = actuatorsPtr;
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
                          float tpsPct,
                          bool consoleCalibReq,
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

  currentLevel = 0.9f * currentLevel + 0.1f * (tpsPct / 100.0f);

  switch (current) {

    case SystemState::OFF:
      if (mapLoadPercent < thresholds.MAP_WAKEUP_PERCENT) {
        current = SystemState::IDLE;
        Serial.println("→ Transición: OFF → IDLE");
      }
      break;

    case SystemState::SIN_CALIBRAR:
      if (consoleCalibReq || bleCalibReq) {
        current = SystemState::CALIBRATION;
        Serial.println("→ Transición: SIN_CALIBRAR → CALIBRATION");
      } else if (calibLoaded) {
        current = SystemState::OFF;
        Serial.println("→ Transición: SIN_CALIBRAR → OFF (calibración detectada)");
      }
      break;

    case SystemState::CALIBRATION:
      if (calibLoaded) {
        current = SystemState::OFF;
        Serial.println("→ Transición: CALIBRATION → OFF");
      }
      break;

    case SystemState::IDLE:
      if (readyForInjection(tpsPct, mapLoadPercent)) {
        current = SystemState::INYECCION_ACUSTICA;
        if (!actuators->isAcousticOn()) {
          actuators->startAcoustic(currentLevel);
        }
        Serial.println("→ Transición: IDLE → INYECCION_ACUSTICA");
      }
      break;

    case SystemState::INYECCION_ACUSTICA:
      if (tpsPct >= thresholds.TURBO_TPS_ON && mapLoadPercent >= thresholds.TURBO_MAP_ON) {
        current = SystemState::TURBO;
        actuators->startTurbo();
        Serial.println("→ Transición: INYECCION_ACUSTICA → TURBO");
      }
      else if (tpsPct <= thresholds.INJ_TPS_OFF || mapLoadPercent <= thresholds.INJ_MAP_OFF) {
        current = SystemState::IDLE;
        actuators->stopAcoustic();
        Serial.println("→ Transición: INYECCION_ACUSTICA → IDLE");
      }
      break;

    case SystemState::TURBO:
      if (tpsPct < thresholds.TURBO_TPS_OFF) {
        current = SystemState::DESCAYENDO;
        actuators->stopAcoustic();
        actuators->stopTurbo();
        Serial.println("→ Transición: TURBO → DESCAYENDO");
      }
      break;

    case SystemState::DESCAYENDO:
      if (readyForInjection(tpsPct, mapLoadPercent)) {
        current = SystemState::INYECCION_ACUSTICA;
        if (!actuators->isAcousticOn()) {
          actuators->startAcoustic(currentLevel);
        }
        Serial.println("→ Transición: DESCAYENDO → INYECCION_ACUSTICA");
      }
      else if (tpsPct <= thresholds.INJ_TPS_OFF || mapLoadPercent <= thresholds.INJ_MAP_OFF) {
        current = SystemState::IDLE;
        actuators->stopAcoustic();
        Serial.println("→ Transición: DESCAYENDO → IDLE");
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
  if (current == SystemState::INYECCION_ACUSTICA) {
    actuators->setAcousticLevel(currentLevel);
    actuators->update();
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    //Serial.printf("TPS: %.1f%% → Level: %.2f\n", currentLevel * 100.0f, getLevel());
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
  return currentLevel;
}

bool StateMachine::readyForInjection(float tps, float mapLoad) {
  return tps >= thresholds.INJ_TPS_ON && mapLoad >= thresholds.INJ_MAP_ON;
}
