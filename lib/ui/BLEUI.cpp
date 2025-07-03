#include "BLEUI.h"

BLEUI::BLEUI() {}

void BLEUI::begin(Stream* serialRef) {
  bleSerial = serialRef;
}

void BLEUI::setFSM(StateMachine* fsmRef) {
  fsm = fsmRef;
}

bool BLEUI::getCalibRequest() {
  bool flag = calibRequested;
  calibRequested = false;
  return flag;
}

void BLEUI::update(SystemState fsmState) {
  if (!bleSerial || !bleSerial->available()) return;

  char cmd = bleSerial->read();

  switch (cmd) {
    case 's':
      imprimirEstado(fsmState);
      break;

    case 'c':
      calibRequested = true;
      bleSerial->println(">> Petición de calibración recibida.");
      break;

    case 'i':
    case 't':
    case 'x':
      if (fsm && fsm->getState() == SystemState::DEBUG) {
        switch (cmd) {
          case 'i':
            bleSerial->println(">> Forzando INYECCION_ACUSTICA (DEBUG)");
            fsm->debugForceState(SystemState::INYECCION_ACUSTICA);
            break;
          case 't':
            bleSerial->println(">> Forzando TURBO (DEBUG)");
            fsm->debugForceState(SystemState::TURBO);
            break;
          case 'x':
            bleSerial->println(">> Volviendo a IDLE (DEBUG)");
            fsm->debugForceState(SystemState::IDLE);
            break;
        }
      } else {
        bleSerial->println("⚠️  Comando solo válido en modo DEBUG.");
      }
      break;

    case '?':
      imprimirHelp();
      break;

    default:
      bleSerial->print("❓ Comando no reconocido: ");
      bleSerial->println(cmd);
      break;
  }
}

void BLEUI::imprimirEstado(SystemState s) {
  bleSerial->print("📡 Estado actual: ");
  switch (s) {
    case SystemState::OFF:                 bleSerial->println("OFF"); break;
    case SystemState::SIN_CALIBRAR:        bleSerial->println("SIN_CALIBRAR"); break;
    case SystemState::CALIBRATION:         bleSerial->println("CALIBRATION"); break;
    case SystemState::IDLE:                bleSerial->println("IDLE"); break;
    case SystemState::INYECCION_ACUSTICA:  bleSerial->println("INYECCION_ACUSTICA"); break;
    case SystemState::TURBO:               bleSerial->println("TURBO"); break;
    case SystemState::DESCAYENDO:          bleSerial->println("DESCAYENDO"); break;
    case SystemState::DEBUG:               bleSerial->println("DEBUG"); break;
    default:                               bleSerial->println("DESCONOCIDO"); break;
  }
}

void BLEUI::imprimirHelp() {
  bleSerial->println(F("\n📘 Comandos BLE disponibles:"));
  bleSerial->println(F("  c  → Solicitar calibración"));
  bleSerial->println(F("  s  → Mostrar estado del sistema"));
  bleSerial->println(F("  ?  → Mostrar esta ayuda"));

  if (fsm && fsm->getState() == SystemState::DEBUG) {
    bleSerial->println(F("  i  → Forzar inyección acústica (DEBUG)"));
    bleSerial->println(F("  t  → Forzar turbo (DEBUG)"));
    bleSerial->println(F("  x  → Paro manual del sistema (DEBUG)"));
  }
}
