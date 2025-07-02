#include "ConsoleUI.h"

void ConsoleUI::begin() {
  Serial.begin(115200);
  while (!Serial);  // Espera conexión en plataformas tipo USB nativa
  Serial.println("\n--- Sistema Turbo-Acústico - Consola Interactiva ---");
  imprimirHelp();
}

void ConsoleUI::setFSM(StateMachine* fsmRef) {
  fsm = fsmRef;
}

bool ConsoleUI::getCalibRequest() {
  if (consoleCalibRequested) {
    consoleCalibRequested = false;
    return true;
  }
  return false;
}

void ConsoleUI::update() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    interpretarComando(c);
  }
}

void ConsoleUI::interpretarComando(char c) {
  if (!fsm) return;

  switch (c) {
    case 'c':
      consoleCalibRequested = true;
      Serial.println(">> Calibración solicitada por consola.");
      break;

    case 's':
      imprimirEstado();
      break;

    case '?':
      imprimirHelp();
      break;

    case 'i':
    case 't':
    case 'x':
      if (fsm->getState() == SystemState::DEBUG) {
        switch (c) {
          case 'i':
            Serial.println(">> Inyector acústico: forzar INYECCION_ACUSTICA.");
            fsm->debugForceState(SystemState::INYECCION_ACUSTICA);
            break;
          case 't':
            Serial.println(">> Turbo: forzar TURBO.");
            fsm->debugForceState(SystemState::TURBO);
            break;
          case 'x':
            Serial.println(">> Paro manual: volver a IDLE.");
            fsm->debugForceState(SystemState::IDLE);
            break;
        }
      } else {
        Serial.println("⚠️  Comando solo válido en modo DEBUG.");
      }
      break;

    default:
      Serial.print("❓ Comando no reconocido: ");
      Serial.println(c);
      break;
  }
}

void ConsoleUI::imprimirEstado() {
  SystemState estado = fsm ? fsm->getState() : SystemState::OFF;

  Serial.print("📟 Estado actual: ");
  switch (estado) {
    case SystemState::OFF:                Serial.println("OFF"); break;
    case SystemState::SIN_CALIBRAR:      Serial.println("SIN_CALIBRAR"); break;
    case SystemState::CALIBRATION:       Serial.println("CALIBRATION"); break;
    case SystemState::IDLE:              Serial.println("IDLE"); break;
    case SystemState::INYECCION_ACUSTICA:Serial.println("INYECCION_ACUSTICA"); break;
    case SystemState::TURBO:             Serial.println("TURBO"); break;
    case SystemState::DESCAYENDO:        Serial.println("DESCAYENDO"); break;
    case SystemState::DEBUG:             Serial.println("DEBUG"); break;
    default:                             Serial.println("DESCONOCIDO"); break;
  }
}

void ConsoleUI::imprimirHelp() {
  Serial.println(F("\n📘 Comandos disponibles:"));
  Serial.println(F("  c  → Solicitar calibración"));
  Serial.println(F("  s  → Mostrar estado del sistema"));
  Serial.println(F("  ?  → Mostrar esta ayuda"));

  if (fsm && fsm->getState() == SystemState::DEBUG) {
    Serial.println(F("  i  → Forzar inyección acústica (modo DEBUG)"));
    Serial.println(F("  t  → Forzar turbo (modo DEBUG)"));
    Serial.println(F("  x  → Paro manual del sistema (modo DEBUG)"));
  }
}
