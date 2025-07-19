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
  if (fsm) {  // opcional: verifica si está lista
    lastState = fsm->getState();
    lastTransitionMS = millis();
  } else {
    lastState = SystemState::UNKNOWN;
    lastTransitionMS = 0;
  }
}



void ConsoleUI::attachSensors(SensorManager* sensorManagerPtr) {
  sensors = sensorManagerPtr;
}

void ConsoleUI::attachActuators(ActuatorManager* actuatorManagerPtr) {
    actuators = actuatorManagerPtr;
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

  if (inputAvailable()) {
    String linea = readLine();
    linea.trim();

    if (simulacionActiva && linea.startsWith("tps_raw:")) {
      int idxTPS = linea.indexOf("tps_raw:");
      int idxMAP = linea.indexOf("map_raw:");

      if (idxTPS != -1 && idxMAP != -1) {
        uint16_t tpsRaw = linea.substring(idxTPS + 8, idxMAP - 1).toInt(); // -1 para excluir la coma
        uint16_t mapRaw = linea.substring(idxMAP + 8).toInt();

        sensors->getTPS().setSimulatedRaw(tpsRaw);
        sensors->getMAP().setSimulatedRaw(mapRaw);

        println("Recibido tps=" + String(tpsRaw) + " map=" + String(mapRaw));
      }
    } else if (linea.length() == 1) {
      interpretarComando(linea.charAt(0));
    } else if (linea.startsWith("[") || linea.startsWith("Gear:") ||
         linea.indexOf("RPM:") != -1 || linea.startsWith("ets ") ||
         linea.startsWith("rst:") || linea.startsWith("load:") ||
         linea.startsWith("clk_drv:") || linea.startsWith("entry ")) {
      // Es una línea de log del ESP o del simulador: ignorar
      } else {
        Serial.println("⚠️  Comando no reconocido.");
      }

  }

  if (getCalibRequest()) runConsoleCalibration();

  if (dashboardEnabled) {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 300) {
      imprimirDashboard();
      lastPrint = millis();
    }
  }
}


void ConsoleUI::interpretarComando(char c) {
  // Comandos solo en modo desarrollador
  auto devOnly = [&]() {
    if (!developerMode) {
      Serial.println("⚠️  Comando exclusivo del modo desarrollador.");
      return false;
    }
    return true;
  };

  switch (c) {
    case 'a':  // Toggle sistema ON/OFF
      toggleSistema();
      break;

    case 'b':  // Iniciar inyección acústica (100%)
      if (!developerMode) {
        Serial.println("⚠️ Comando exclusivo del modo desarrollador.");
        break;
      }
      actuators->startAcoustic(1.0f);
      if (actuators->isAcousticOn())
        actuators->getAcousticInjector().test();
      break;

    case 'c':  // Solicitar calibración por consola
      consoleCalibRequested = true;
      Serial.println(">> Solicitud de calibración registrada.");
      break;

    case 'd':  // Activar modo desarrollador
      developerMode = true;
      Serial.println(">> Modo desarrollador ACTIVADO.");
      break;

    case 'i':  // Toggle relé inyector acústico (dev mode)
      if (!devOnly()) break;
      if (actuators->getAcousticInjector().isActive()) {
        bool estadoActual = actuators->getAcousticInjector().isRelayActive();
        actuators->getAcousticInjector().testRelay(!estadoActual);
        Serial.printf(">> Relé %s.\n", !estadoActual ? "activado" : "desactivado");
      } else {
        Serial.println("⚠️ Inyector no disponible.");
      }
      break;

    case 'm':  // Mostrar ayuda
      imprimirHelp();
      break;

    case 'n':  // Detener inyección acústica
      if (actuators->isAcousticOn())
        actuators->stopAcoustic();
      break;

    case 'r':  // Borrar calibración y poner FSM en estado sin calibrar
      CalibrationManager::getInstance().clearCalibration();
      if (fsm) {
        fsm->debugForceState(SystemState::SIN_CALIBRAR);
        Serial.println(">> Se requiere recalibrar de nuevo para poder usar el sistema");
      } else {
        Serial.println("⚠️ No se puede cambiar estado: FSM no está disponible.");
      }
      break;

    case 's':  // Toggle dashboard en tiempo real
      dashboardEnabled = !dashboardEnabled;
      Serial.printf(">> Dashboard en tiempo real %s.\n", dashboardEnabled ? "ACTIVADO" : "DESACTIVADO");
      break;

    case 't':  // Toggle turbo (dev mode)
      if (!devOnly()) break;
      if (actuators->getTurboController().isActive()) {
        if (actuators->getTurboController().isActive()) {
          actuators->stopTurbo();
          Serial.println(">> Turbo desactivado.");
        } else {
          actuators->startTurbo();
          Serial.println(">> Turbo activado.");
        }
      } else {
        Serial.println("⚠️ Turbo no disponible.");
      }
      break;

    case 'u':  // Placeholder para otro comando dev
      if (!devOnly()) break;
      // Implementar acción para 'u' si aplica
      break;

    case 'v':  // Visualización curva (dev mode)
      if (!devOnly()) break;
      Serial.println(">> [visualización de curva] …");
      break;

    case 'x':  // Forzar estado IDLE en FSM
      if (fsm) {
        fsm->debugForceState(SystemState::IDLE);
        Serial.println(">> Paro manual: regresando a IDLE.");
      }
      break;

    case 'z':  // Toggle modo simulación (dev mode)
      if (!devOnly()) break;

      simulacionActiva = !simulacionActiva;

      sensors->getTPS().setSimulatedRaw(0);
      sensors->getMAP().setSimulatedRaw(0);

      if (simulacionActiva) {
        sensors->getTPS().enableSimulation();
        sensors->getMAP().enableSimulation();
      } else {
        sensors->getTPS().disableSimulation();
        sensors->getMAP().disableSimulation();
      }

      Serial.printf(">> Modo simulación %s.\n", simulacionActiva ? "ACTIVADO" : "DESACTIVADO");
      break;

    default:
      Serial.print("❓ Comando no reconocido: ");
      Serial.println(c);
  }
}

void ConsoleUI::imprimirDashboard() {
  if (!fsm || !sensors || !actuators) return;  // seguridad

  float tpsV = sensors->getTPS().readVolts();
  float mapV = sensors->getMAP().readVolts();
  uint8_t dac = actuators->getAcousticInjector().getCurrentDAC();
  bool turboOn = actuators->isTurboOn();
  bool injOn = actuators->isAcousticOn();

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

  // HUD en vivo: actualiza siempre en la misma línea
  Serial.printf(
    "\r[%s|%lus] TPS=%.2fV(%.2f–%.2fV) | MAP=%.2fV(%.2f–%.2fV) | DAC=%3u | T:%c | I:%c     ",
    stName, elapsed,
    tpsV, tpsMinV, tpsMaxV,
    mapV, mapMinV, mapMaxV,
    dac,
    turboOn ? '1' : '0',
    injOn ? '1' : '0'
  );

  // Solo cuando cambia el estado, imprimir detalles debajo
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
  calib.runTPSCalibration(sensors->getTPS(), simulacionActiva);
  calib.runMAPCalibration(sensors->getMAP(), simulacionActiva);
  calib.saveCalibration();
  calib.loadCalibration(); 
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
  Serial.println(F("  c  → Ejecutar rutina de calibración de sensores"));
  Serial.println(F("  m  → Mostrar menú de comandos"));
  Serial.println(F("  r  → Borrar calibración actual"));
  Serial.println(F("  s  → Activar/Desactivar dashboard del sistema"));
  Serial.println(F("  x  → Paro manual, volver a IDLE"));
  Serial.println(F("  d  → Activar modo desarrollador"));

  if (developerMode) {
    Serial.println(F("\n🧪 Modo desarrollador activo:"));
    Serial.println(F("  b  → Probar sonido acústico"));
    Serial.println(F("  i  → Activar relé INYECCIÓN_ACÚSTICA"));
    Serial.println(F("  t  → Activar relé TURBO"));
    Serial.println(F("  u  → (Comando dev pendiente)"));
    Serial.println(F("  v  → Visualizar curva TPS-MAP (pendiente desarrollo)"));
    Serial.println(F("  z  → Activar/Desactivar modo simulación"));
  }
}

bool ConsoleUI::isDeveloperMode() const {
  return developerMode;
}
