#include "ConsoleUI.h"
#include "CalibrationManager.h" 


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

  if (fsm->getState() != lastState) {
    lastTransitionMS = millis();
    lastState = fsm->getState();
  }

  if (Serial.available()) {
    char c = Serial.read();
    interpretarComando(c);
  }

  if (getCalibRequest()) {
    runConsoleCalibration();
  }
  // Nuevo: HUD en tiempo real
  if (dashboardEnabled) {
     static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 300) {  // Actualiza cada 300 ms
      imprimirDashboard();
      lastPrint = millis();
    }
  }

}

void ConsoleUI::interpretarComando(char c) {
  switch (c) {
    case '?':
      imprimirHelp();
      break;

    case 's':
      dashboardEnabled = !dashboardEnabled;
      Serial.printf(">> Dashboard en tiempo real %s.\n", dashboardEnabled ? "ACTIVADO" : "DESACTIVADO");
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
  // Lecturas en tiempo real
  float tpsV      = tpsSensor->readVolts();
  float mapV      = mapSensor->readVolts();
  uint8_t dac     = injector->getCurrentDAC();
  bool turboOn    = turbo->isOn();
  bool injOn      = injector->isActive();
  SystemState st  = fsm->getState();
  unsigned long elapsed = (millis() - lastTransitionMS) / 1000;

  // Umbrales de calibración
  auto& calib     = CalibrationManager::getInstance();
  uint16_t tpsMin = calib.getTPSMin();
  uint16_t tpsMax = calib.getTPSMax();
  uint16_t mapMin = calib.getMAPMin();
  uint16_t mapMax = calib.getMAPMax();

  // Estado en string corto
  static const char* stateNames[] = {
    "OFF", "SIN_CAL", "CALIB", "IDLE",
    "BEAM", "BOOST","DESCAY","DEBUG","??"
  };
  const char* stName = stateNames[int(st)];

  // HUD compacto con casi todo:
  // Antes de imprimir HUD, convierte los raw a voltios:
  float tpsMinV = tpsMin * 3.3f / 4095.0f;
  float tpsMaxV = tpsMax * 3.3f / 4095.0f;
  float mapMinV = mapMin * 3.3f / 4095.0f;
  float mapMaxV = mapMax * 3.3f / 4095.0f;
  Serial.printf(
    "\r[%s|%lus] TPS=%.2fV(%.2f–%.2fV) | MAP=%.2fV(%.2f–%.2fV) | DAC=%3u | T:%c | I:%c     ",
    stName,
    elapsed,
    tpsV, tpsMinV, tpsMaxV,
    mapV, mapMinV, mapMaxV,
    dac,
    turboOn ? '1' : '0',
    injOn   ? '1' : '0'
  );

  // Dashboard grande solo en transición de estado
  if (st != lastState) {
    lastState = st;
    Serial.println("\n\n=== TURBO SYSTEM DASHBOARD ===");
    Serial.printf("Estado motor:      %s\n", stName);
    Serial.printf("TPS Voltage:       %.3f V (raw %u–%u)\n",
                  tpsV, tpsMin, tpsMax);
    Serial.printf("MAP Voltage:       %.3f V (raw %u–%u)\n",
                  mapV, mapMin, mapMax);
    Serial.printf("DAC Output:        %u (PWM)\n", dac);
    Serial.printf("Turbo:             %s\n", turboOn ? "ON" : "OFF");
    Serial.printf("Inyector sónico:   %s\n", injOn ? "ON" : "OFF");
    Serial.printf("Último cambio:     hace %lu s\n", elapsed);
    Serial.println("==============================\n");
  }
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

void ConsoleUI::runConsoleCalibration() {
  Serial.println(">> Iniciando calibración…");
  auto& calib = CalibrationManager::getInstance();
  calib.clearCalibration();
  calib.runTPSCalibration(*tpsSensor);
  calib.runMAPCalibration(*mapSensor);
  calib.saveCalibration();
  Serial.println(">> Calibración completada.");
}