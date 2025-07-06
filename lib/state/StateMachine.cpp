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

void StateMachine::update(float mapKPa,
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
      // Pasa a IDLE cuando la presión supera un umbral
      if (mapKPa > MAP_WAKEUP_KPA) {
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
      // La lógica de calibración (runMAPCalibration/runTPS…) se ejecuta en main()
      // Tras completarla, main deberá invocar saveCalibration() y realizar:
           debugForceState(SystemState::OFF);
      break;

    case SystemState::IDLE:
      // Activar inyección acústica
      if (tpsPct >= INJ_TPS_ON && mapKPa >= INJ_VAC_ON) {
        current = SystemState::INYECCION_ACUSTICA;
        injectorPtr->start(0.0f);
        Serial.println("→ Transición: IDLE → INYECCION_ACUSTICA");
      }
      break;

    case SystemState::INYECCION_ACUSTICA:
      // Escalar a TURBO
      if (tpsPct >= TURBO_TPS_ON && mapKPa <= TURBO_VAC_ON) {
        current = SystemState::TURBO;
        turboPtr->start();
        Serial.println("→ Transición: INYECCION_ACUSTICA → TURBO");
      }
      // O desactivar inyección y volver a IDLE
      else if (tpsPct <= INJ_TPS_OFF || mapKPa < INJ_VAC_OFF) {
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
      // Si vuelve TPS alto y presión ideal, retomamos inyección acústica
      if (tpsPct >= INJ_TPS_ON && mapKPa >= INJ_VAC_ON) {
        current = SystemState::INYECCION_ACUSTICA;
        injectorPtr->start(0.0f);
        Serial.println("→ Transición: DESCAYENDO → INYECCION_ACUSTICA");
      }
      // Si ya no hay carga ni presión, volvemos a IDLE
      else if (tpsPct <= INJ_TPS_OFF || mapKPa < INJ_VAC_OFF) {
        current = SystemState::IDLE;
        injectorPtr->stop();
        Serial.println("→ Transición: DESCAYENDO → IDLE");
      }
      // En otro caso, permanecemos en DESCAYENDO
      break;

    case SystemState::DEBUG:
      // Queda a la espera de debugForceState()
      //debugForceState();
      break;
  }
}

void StateMachine::handleActions(float acousticLevel) {
  // Solo la inyección acústica requiere ajuste continuo de nivel
  if (current == SystemState::INYECCION_ACUSTICA) {
    injectorPtr->setLevel(acousticLevel);
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
