#include "ConsoleUI.h"

void ConsoleUI::begin() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n=== Consola Turbo-Acústica Iniciada ===");
  imprimirHelp();
}

void ConsoleUI::setFSM(StateMachine* ref) {
  fsm = ref;
  lastState = fsm->getState();
  lastTransitionMS = millis();
}

void ConsoleUI::attachSensors(MAPSensor* mapPtr, TPSSensor* tpsPtr) {
  mapSensor = mapPtr;
  tpsSensor = tpsPtr;
}

void ConsoleUI::attachActuators(TurboController* turboPtr, AcousticInjector* injectorPtr) {
  turbo = turboPtr;
  injector = injectorPtr;
}

bool ConsoleUI::getCalibRequest() {
  if (consoleCalibRequested) {
    consoleCalibRequested = false;
    return true;
  }
  return false;
}

void ConsoleUI::update() {
  if (!fsm) return;

  // Detecta cambio de estado
  if (fsm->getState() != lastState) {
    lastTransitionMS = millis();
    lastState = fsm->getState();
  }

  if (Serial.available()) {
    char c = Serial.read();
    interpretarComando(c);
  }
}

void ConsoleUI::interpretarComando(char c) {
  switch (c) {
    case '?':
      imprimirHelp();
      break;

    case 's':
      imprimirDashboard();
      break;

    case 'c':
      consoleCalibRequested = true;
      Serial.println(">> Solicitud de calibración registrada.");
      break;

    case 'd':
      developerMode = true;
      Serial.println(">> Modo desarrollador ACTIVADO.");
      break;

    case 'x':
      if (fsm) {
        fsm->debugForceState(SystemState::IDLE);
        Serial.println(">> Paro manual: regresando a IDLE.");
      }
      break;

    case 'i': case 't': case 'u': case 'v':
      if (!developerMode) {
        Serial.println("⚠️  Comando exclusivo del modo desarrollador.");
        return;
      }
      switch (c) {
        case 'i': fsm->debugForceState(SystemState::INYECCION_ACUSTICA); break;
        case 't': fsm->debugForceState(SystemState::TURBO); break;
        case 'u': Serial.println(">> [offsets internos] …"); break;
        case 'v': Serial.println(">> [visualización de curva] …"); break;
      }
      break;

    default:
      Serial.print("❓ Comando no reconocido: ");
      Serial.println(c);
  }
}

void ConsoleUI::imprimirDashboard() {
  if (!fsm || !mapSensor || !tpsSensor || !injector || !turbo) {
    Serial.println("⚠️  Dashboard incompleto: referencias no conectadas.");
    return;
  }

  SystemState estado = fsm->getState();
  const char* estadoStr = 
    (estado == SystemState::OFF)              ? "OFF" :
    (estado == SystemState::SIN_CALIBRAR)     ? "SIN_CALIB" :
    (estado == SystemState::CALIBRATION)      ? "CALIBRANDO" :
    (estado == SystemState::IDLE)             ? "IDLE" :
    (estado == SystemState::INYECCION_ACUSTICA)? "SONIC FLOW" :
    (estado == SystemState::TURBO)            ? "BOOST" :
    (estado == SystemState::DESCAYENDO)       ? "DESCAYENDO" :
    (estado == SystemState::DEBUG)            ? "DEBUG" : "???";

  float tpsV  = tpsSensor->readVolts();
  float mapV  = mapSensor->readVolts();
  uint8_t dac = injector->getCurrentDAC();  // método que debes exponer
  bool turboOn    = turbo->isOn();      // método que debes exponer
  bool injectorOn = injector->isActive();   // método que debes exponer

  unsigned long elapsed = (millis() - lastTransitionMS) / 1000;

  Serial.println("\n=== TURBO SYSTEM DASHBOARD ===");
  Serial.printf("Estado motor:      %s\n", estadoStr);
  Serial.printf("TPS:               %.2f V\n", tpsV);
  Serial.printf("MAP:               %.2f V\n", mapV);
  Serial.printf("DAC Output:        %u (PWM)\n", dac);
  Serial.printf("Turbo:             %s\n", turboOn ? "ON" : "OFF");
  Serial.printf("Inyector sónico:   %s\n", injectorOn ? "ON" : "OFF");
  Serial.printf("Último cambio:     %s → %s (hace %lu s)\n",
                 lastState == SystemState::OFF ? "OFF" : "…",
                 estadoStr, elapsed);
  Serial.println("==============================\n");
}

void ConsoleUI::imprimirHelp() {
  Serial.println(F("\n📘 Comandos disponibles:"));
  Serial.println(F("  s  → Mostrar dashboard del sistema"));
  Serial.println(F("  c  → Ejecutar rutina de calibración"));
  Serial.println(F("  x  → Paro manual, volver a IDLE"));
  Serial.println(F("  ?  → Mostrar esta ayuda"));
  Serial.println(F("  d  → Activar modo desarrollador"));

  if (developerMode) {
    Serial.println(F("\n🧪 Modo desarrollador activo:"));
    Serial.println(F("  i  → Forzar INYECCION_ACUSTICA"));
    Serial.println(F("  t  → Forzar TURBO"));
    Serial.println(F("  u  → Mostrar offsets internos"));
    Serial.println(F("  v  → Visualizar curva TPS-MAP"));
  }
}
