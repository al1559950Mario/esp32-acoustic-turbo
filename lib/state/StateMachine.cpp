#include "StateMachine.h"
#include <Arduino.h>  // Para Serial

void StateMachine::begin(bool hasCalibration,
                         TurboController* turboRef,
                         AcousticInjector* injectorRef) {
  // Arrancamos en SIN_CALIBRAR si no hay calibración,
  // o en OFF si ya la tenemos.
  current     = hasCalibration
                ? SystemState::OFF
                : SystemState::SIN_CALIBRAR;

  turboPtr    = turboRef;
  injectorPtr = injectorRef;

  Serial.print(">> StateMachine iniciado en estado: ");
  Serial.println(static_cast<int>(current));
}

SystemState StateMachine::getState() const {
  return current;
}

void StateMachine::update(float vacuumInHg,
                          float tpsPct,
                          bool consoleCalibReq,
                          bool bleCalibReq,
                          const DebugManager &dbg) {


  // Si estamos en DEBUG y el gestor forzó otro estado, respetar ese cambio
  if (current == SystemState::DEBUG) {
    return;
  }

  
  switch (current) {

    case SystemState::OFF:
      // Pasa a IDLE cuando hay al menos algo de vacío
      if (vacuumInHg < MAP_WAKEUP_INHG) { // Recuerda: inHg negativo = más vacío
        current = SystemState::IDLE;
        Serial.println("→ Transición: OFF → IDLE");
      }
      break;

    case SystemState::SIN_CALIBRAR:
      // Queda bloqueado hasta que el usuario pida calibrar
      if (consoleCalibReq || bleCalibReq) {
        current = SystemState::CALIBRATION;
        Serial.println("→ Transición: SIN_CALIBRAR → CALIBRATION");
      }
      break;

    case SystemState::CALIBRATION:
      // La lógica de calibración se ejecuta externamente
      // Tras completarla, se fuerza a OFF vía debugForceState(SystemState::OFF)
      break;

    case SystemState::IDLE:
      // Activar inyección acústica si hay suficiente vacío y carga
      if (readyForInjection(tpsPct, vacuumInHg)) {
        current = SystemState::INYECCION_ACUSTICA;
        if (!injectorPtr->isActive()) {
          injectorPtr->start(currentLevel);
        }

        Serial.println("→ Transición: IDLE → INYECCION_ACUSTICA");
      }
      break;

    case SystemState::INYECCION_ACUSTICA:
      // Escalar a TURBO si hay carga alta y casi sin vacío
      if (tpsPct >= TURBO_TPS_ON && vacuumInHg >= TURBO_VAC_ON) {
        current = SystemState::TURBO;
        turboPtr->start();
        Serial.println("→ Transición: INYECCION_ACUSTICA → TURBO");
      }
      // Volver a IDLE si se reduce carga o aumenta presión
      else if (tpsPct <= INJ_TPS_OFF || vacuumInHg > INJ_VAC_OFF) {
        current = SystemState::IDLE;
        injectorPtr->stop();
        Serial.println("→ Transición: INYECCION_ACUSTICA → IDLE");
      }
      break;

    case SystemState::TURBO:
      // Cuando el TPS baja, arrancamos fase de caída
      if (tpsPct < TURBO_TPS_OFF) {
        current = SystemState::DESCAYENDO;
        injectorPtr->stop();
        turboPtr->stop();
        Serial.println("→ Transición: TURBO → DESCAYENDO");
      }
      break;

    case SystemState::DESCAYENDO:
      // Retomar inyección acústica si vuelve a haber vacío y carga
      if (readyForInjection(tpsPct, vacuumInHg)) {
        current = SystemState::INYECCION_ACUSTICA;
        if (!injectorPtr->isActive()) {
          injectorPtr->start(currentLevel);
        }

        Serial.println("→ Transición: DESCAYENDO → INYECCION_ACUSTICA");
      }
      // Volver a IDLE si se pierde carga o vacío
      else if (tpsPct <= INJ_TPS_OFF || vacuumInHg > INJ_VAC_OFF) {
        current = SystemState::IDLE;
        injectorPtr->stop();
        Serial.println("→ Transición: DESCAYENDO → IDLE");
      }
      break;

    case SystemState::DEBUG:
      // No hacer nada, espera debugForceState()
      break;
  }
}

void StateMachine::handleActions() {
  // Solo la inyección acústica requiere ajuste continuo de nivel
  if (current == SystemState::INYECCION_ACUSTICA) {
    injectorPtr->setLevel(currentLevel);
    injectorPtr->update();
    injectorPtr->applyPendingDAC();
  }
  // Resto de estados no requieren acciones periódicas
}

void StateMachine::debugForceState(SystemState nuevoEstado) {
  if (current == SystemState::DEBUG) {
    current = nuevoEstado;
    Serial.print(">> Estado forzado a: ");
    Serial.println(static_cast<int>(nuevoEstado));
  }
}
