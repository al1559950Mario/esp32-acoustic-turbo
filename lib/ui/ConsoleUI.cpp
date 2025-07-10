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

void ConsoleUI::attachSensors(SensorManager* sensorManagerPtr) {
  sensors = sensorManagerPtr;
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

  // Paso 1: Verifica transición de estado
  // if (fsm->getState() != lastState) {
  //   lastTransitionMS = millis();
  //   lastState = fsm->getState();
  // }

  // Paso 2: Lectura serial
  // if (Serial.available()) {
  //   String linea = Serial.readStringUntil('\n');
  //   linea.trim();
  //   ...
  // }

  // Paso 3: Calibración
  // if (getCalibRequest()) {
  //   runConsoleCalibration();
  // }

  // Paso 4: Dashboard
  // if (dashboardEnabled) {
  //   static unsigned long lastPrint = 0;
  //   if (millis() - lastPrint > 300) {
  //     imprimirDashboard();
  //     lastPrint = millis();
  //   }
  // }

  Serial.println("ConsoleUI::update() activa, sin bloque interno.");
}

void ConsoleUI::interpretarComando(char c) {
  switch (c) {
    case 'm':
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

    case 'r':
      CalibrationManager::getInstance().clearCalibration();
      if (fsm) {
        fsm->debugForceState(SystemState::SIN_CALIBRAR);
        Serial.println(">> Se requiere recalibrar de nuevo para poder usar el sistema");
      } else {
        Serial.println("⚠️ No se puede cambiar estado: FSM no está disponible.");
      }
      break;

    case 'i': case 't': case 'u': case 'v':
      if (!developerMode) {
        Serial.println("⚠️  Comando exclusivo del modo desarrollador.");
        return;
      }
      switch (c) {
        case 'i': 
          if (injector) {
            bool estadoActual = injector->isRelayActive();
            injector->testRelay(!estadoActual);
            Serial.printf(">> Relé %s.\n", !estadoActual ? "activado" : "desactivado");
          } else {
            Serial.println("⚠️ Inyector no disponible.");
          }
          break;
        case 't': 
          if (turbo) {
            if (turbo->isActive()) {
              turbo->stop();
              Serial.println(">> Turbo desactivado.");
            } else {
              turbo->start();
              Serial.println(">> Turbo activado.");
            }
          } else {
            Serial.println("⚠️ Turbo no disponible.");
          }
          break;
        case 'v': Serial.println(">> [visualización de curva] …"); break;
      }
      break;

    case 'a':
      toggleSistema();
      break;

    case 'b':
      if (injector) injector->test();
      break;

    case 'n':
      if (injector) injector->stop();
      break;

    case 'z':
      if (!developerMode) {
        Serial.println("⚠️  Comando exclusivo del modo desarrollador.");
        return;
      }
      simulacionActiva = !simulacionActiva;
      Serial.printf(">> Modo simulación %s.\n", simulacionActiva ? "ACTIVADO" : "DESACTIVADO");
      break;

    default:
      Serial.print("❓ Comando no reconocido: ");
      Serial.println(c);
  }
}

void ConsoleUI::imprimirDashboard() {
  float tpsV = sensors->getTPS().readVolts();
  float mapV = sensors->getMAP().readVolts();
  uint8_t dac = injector->getCurrentDAC();
  bool turboOn = turbo->isOn();
  bool injOn = injector->isActive();
  SystemState st = fsm->getState();
  unsigned long elapsed = (millis() - lastTransitionMS) / 1000;

  auto& calib = CalibrationManager::getInstance();
  uint16_t tpsMin = calib.getTPSMin();
  uint16_t tpsMax = calib.getTPSMax();
  uint16_t mapMin = calib.getMAPMin();
  uint16_t mapMax = calib.getMAPMax();

  static const char* stateNames[] = {
    "OFF", "SIN_CAL", "CALIB", "IDLE",
    "BEAM", "BOOST", "DESCAY", "DEBUG", "??"
  };
  const char* stName = stateNames[int(st)];

  float tpsMinV = tpsMin * 3.3f / 4095.0f;
  float tpsMaxV = tpsMax * 3.3f / 4095.0f;
  float mapMinV = mapMin * 3.3f / 4095.0f;
  float mapMaxV = mapMax * 3.3f / 4095.0f;
  Serial.printf(
    "\r[%s|%lus] TPS=%.2fV(%.2f–%.2fV) | MAP=%.2fV(%.2f–%.2fV) | DAC=%3u | T:%c | I:%c     ",
    stName, elapsed,
    tpsV, tpsMinV, tpsMaxV,
    mapV, mapMinV, mapMaxV,
    dac,
    turboOn ? '1' : '0',
    injOn ? '1' : '0'
  );

  if (st != lastState) {
    lastState = st;
    Serial.println("\n\n=== TURBO SYSTEM DASHBOARD ===");
    Serial.printf("Estado motor:      %s\n", stName);
    Serial.printf("TPS Voltage:       %.3f V (raw %u–%u)\n", tpsV, tpsMin, tpsMax);
    Serial.printf("MAP Voltage:       %.3f V (raw %u–%u)\n", mapV, mapMin, mapMax);
    Serial.printf("DAC Output:        %u (PWM)\n", dac);
    Serial.printf("Turbo:             %s\n", turboOn ? "ON" : "OFF");
    Serial.printf("Inyector sónico:   %s\n", injOn ? "ON" : "OFF");
    Serial.printf("Último cambio:     hace %lu s\n", elapsed);
    Serial.println("==============================\n");
  }
}

void ConsoleUI::runConsoleCalibration() {
  Serial.println(">> Iniciando calibración…");
  auto& calib = CalibrationManager::getInstance();
  calib.clearCalibration();
  calib.runTPSCalibration(sensors->getTPS());
  calib.runMAPCalibration(sensors->getMAP());
  calib.saveCalibration();
  Serial.println(">> Calibración completada.");
}

void ConsoleUI::toggleSistema() {
  sistemaActivo = !sistemaActivo;
  Serial.printf(">> Sistema %s.\n", sistemaActivo ? "ACTIVADO" : "DESACTIVADO");
}

int ConsoleUI::parseValor(const String& linea, const String& clave) {
  int inicio = linea.indexOf(clave + ":");
  if (inicio == -1) return -1;

  inicio += clave.length() + 1;
  int fin = linea.indexOf(',', inicio);
  if (fin == -1) fin = linea.length();

  String valorStr = linea.substring(inicio, fin);
  return valorStr.toInt();
}

void ConsoleUI::imprimirHelp() {
  Serial.println(F("\n📘 Comandos disponibles:"));
  Serial.println(F("  a  → Activar/Desactivar sistema completo (seguridad/falla)"));
  Serial.println(F("  s  → Activar/Desactivar dashboard del sistema"));
  Serial.println(F("  c  → Ejecutar rutina de calibración de sensores"));
  Serial.println(F("  r  → Borrar calibración actual (solo clear)"));
  Serial.println(F("  x  → Paro manual, volver a IDLE"));
  Serial.println(F("  d  → Activar modo desarrollador"));
  Serial.println(F("  m  → Mostrar menu de comandos"));

  if (developerMode) {
    Serial.println(F("\n🧪 Modo desarrollador activo:"));
    Serial.println(F("  i  → Activar rele INYECCION_ACUSTICA"));
    Serial.println(F("  t  → Activar rele TURBO"));
    Serial.println(F("  v  → Visualizar curva TPS-MAP(Pendiente desarrollar)"));
    Serial.println(F("  b  → Probar sonido acústico"));
  }

}