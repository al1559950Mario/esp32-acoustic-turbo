#include "StateMachine.h"

void StateMachine::begin(bool hasCalib) {
  // Define el estado inicial tras setup()
  current = hasCalib
            ? SystemState::OFF
            : SystemState::SIN_CALIBRAR;
}

SystemState StateMachine::getState() const {
  return current;
}

void StateMachine::update(float mapKPa,
                          float tpsPct,
                          bool consoleCalibReq,
                          bool bleCalibReq,
                          const DebugManager &dbg) {

  switch (current) {

    case SystemState::OFF:
      // Salta a IDLE cuando MAP supera umbral mínimo
      if (mapKPa > MAP_WAKEUP_KPA) {
        current = SystemState::IDLE;
      }
      break;

    case SystemState::SIN_CALIBRAR:
      // Solo avanza a CALIBRATION con petición del usuario
      if (consoleCalibReq || bleCalibReq) {
        current = SystemState::CALIBRATION;
      }
      break;

    case SystemState::CALIBRATION:
      // Tras rutina de calibración, vuelve a OFF
      current = SystemState::OFF;
      break;

    case SystemState::IDLE:
      // Vigila umbrales para activar inyector acústico
      if (tpsPct >= INJ_TPS_ON && mapKPa >= INJ_VAC_ON) {
        current = SystemState::INYECCION_ACUSTICA;
      }
      break;

    case SystemState::INYECCION_ACUSTICA:
      // Transición a TURBO si cumple umbral y vacío
      if (tpsPct >= TURBO_TPS_ON && mapKPa <= TURBO_VAC_ON) {
        current = SystemState::TURBO;
      }
      // Retrocede a IDLE si condiciones de inyección ya no se mantienen
      else if (tpsPct <= INJ_TPS_OFF || mapKPa < INJ_VAC_OFF) {
        current = SystemState::IDLE;
      }
      break;

    case SystemState::TURBO:
      // Si TPS cae por debajo, pasa a DECAY
      if (tpsPct < TURBO_TPS_OFF) {
        current = SystemState::DESCAYENDO;
      }
      break;

    case SystemState::DESCAYENDO:
      // 1) Mientras TPS < umbral decay → permanece
      if (tpsPct < TURBO_TPS_OFF) {
        break;
      }
      // 2) Si TPS recuperó y MAP en vacío, inyección acústica
      if (tpsPct >= INJ_TPS_ON && mapKPa >= INJ_VAC_ON) {
        current = SystemState::INYECCION_ACUSTICA;
      }
      // 3) En otro caso, regresa a IDLE
      else {
        current = SystemState::IDLE;
      }
      break;

    case SystemState::DEBUG:
      // Modo debug mantiene estado hasta comando externo
      break;
  }
}
