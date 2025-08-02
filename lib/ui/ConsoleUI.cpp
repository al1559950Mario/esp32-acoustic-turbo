#include "ConsoleUI.h"
#include "CalibrationManager.h" 

void ConsoleUI::begin() {
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
    if (simulationOnPython && linea.startsWith("tps_raw:")) {
      int idxTPS = linea.indexOf("tps_raw:");
      int idxMAP = linea.indexOf("map_raw:");

      if (idxTPS != -1 && idxMAP != -1) {
        uint16_t tpsRaw = linea.substring(idxTPS + 8, idxMAP - 1).toInt(); // -1 para excluir la coma
        uint16_t mapRaw = linea.substring(idxMAP + 8).toInt();

        sensors->getTPS().setSimulatedRaw(tpsRaw);
        sensors->getMAP().setSimulatedRaw(mapRaw);
      }
    } else if (linea.length() == 1) {
      interpretarComando(linea.charAt(0));
    } else if (linea.startsWith("[") || linea.startsWith("Gear:") ||
         linea.indexOf("RPM:") != -1 || linea.startsWith("ets ") ||
         linea.startsWith("rst:") || linea.startsWith("load:") ||
         linea.startsWith("clk_drv:") || linea.startsWith("entry ")) {
      // Es una línea de log del ESP o del simulador: ignorar
      } else {
        //
      }

  }
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
      this->println("⚠️  Comando exclusivo del modo desarrollador.");
      return false;
    }
    return true;
  };
  tiempoProximaImpresionHUD = millis() + 2500;

  switch (c) {
    case 'a':  // Toggle sistema ON/OFF
      toggleSistema();
      break;

    case 'b':  // Iniciar inyección acústica (100%)
      if (!devOnly()) break;
      actuators->startAcoustic(1.0f);
      if (actuators->isAcousticOn())
        actuators->getAcousticInjector().test();
      break;

    case 'c':  // Solicitar calibración por consola
      consoleCalibRequested = true;
      this->println(">> Solicitud de calibración registrada.");
      break;

    case 'd':  // Activar modo desarrollador
      developerMode = true;
      this->println(">> Modo desarrollador ACTIVADO.");
      imprimirHelp();
      break;

    case 'i':  // Toggle relé inyector acústico (dev mode)
      if (!devOnly()) break;
      if (actuators->getAcousticInjector().isActive()) {
        bool estadoActual = actuators->getAcousticInjector().isRelayActive();
        actuators->getAcousticInjector().testRelay(!estadoActual);
        this->printf(">> Relé %s.\n", !estadoActual ? "activado" : "desactivado");
      } else {
        this->println("⚠️ Inyector no disponible.");
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
      if (!devOnly()) break;
      CalibrationManager::getInstance().clearCalibration();
      if (fsm) {
        fsm->debugForceState(SystemState::SIN_CALIBRAR);
        this->println(">> Se requiere recalibrar de nuevo para poder usar el sistema");
      } else {
        this->println("⚠️ No se puede cambiar estado: FSM no está disponible.");
      }
      break;

    case 's':  // Toggle dashboard en tiempo real
      dashboardEnabled = !dashboardEnabled;
      this->printf(">> Dashboard en tiempo real %s.\n", dashboardEnabled ? "ACTIVADO" : "DESACTIVADO");
      break;

    case 't':  // Toggle turbo (dev mode)
      if (!devOnly()) break;
      if (actuators->getVortexController().isActive()) {
        if (actuators->getVortexController().isActive()) {
          actuators->stopVortex();
          this->println(">> Turbo desactivado.");
        } else {
          actuators->startVortex();
          this->println(">> Turbo activado.");
        }
      } else {
        this->println("⚠️ Turbo no disponible.");
      }
      break;

    case 'u':  // Placeholder para otro comando dev
      if (!devOnly()) break;
      // Implementar acción para 'u' si aplica
      break;

    case 'v':  // Visualización curva (dev mode)
      if (!devOnly()) break;
      this->println(">> [visualización de curva] …");
      break;

    case 'x':  // Forzar estado IDLE en FSM
      if (!devOnly()) break;
      if (fsm) {
        fsm->debugForceState(SystemState::IDLE);
        this->println(">> Paro manual: regresando a IDLE.");
      }
      break;

    case 'z':  // Toggle modo simulación (dev mode)
      if (!devOnly()) break;

      simulationOnPython = !simulationOnPython;

      if (simulationOnPython) {
        //sensors->getTPS().enableSimulation();
        //sensors->getMAP().enableSimulation();
      } else {
        sensors->getTPS().disableSimulation();
        sensors->getMAP().disableSimulation();
      }

      this->printf(">> Modo simulación %s.\n", simulationOnPython ? "ACTIVADO" : "DESACTIVADO");
      break;
    case 'k':  // Verificar valores de sensores
      if (!devOnly()) break;

      this->printf("== DEBUG Sensores ==\n");

      this->printf("TPS: raw=%d, volts=%.2f, %%=%.1f%%\n",
                  sensors->getTPS().readRaw(),
                  sensors->getTPS().readVolts(),
                  sensors->getTPS().readPorcent());

      this->printf("MAP: raw=%d, volts=%.2f\n",
                  sensors->getMAP().readRaw(),
                  sensors->getMAP().readVolts());
      break;
    default:
      if (!simulationOnPython) break;
      this->print("❓ Comando no reconocido: ");
      this->println(String(c));


  }
}

void ConsoleUI::imprimirDashboard() {
  if (!fsm || !sensors || !actuators) return;
  if (millis() < tiempoProximaImpresionHUD) return;

  float tpsV = sensors->getTPS().readVolts();
  float tpsPct = sensors->getTPS().readPorcent();
  float mapV = sensors->getMAP().readVolts();
  uint8_t dac = actuators->getAcousticInjector().getCurrentDAC();
  bool vortexOn = actuators->isTurboOn();
  bool injOn = actuators->isAcousticOn();

  // Obtener level y freq actuales
  float level = actuators->getAcousticInjector().getLevel();
  float freq = actuators->getAcousticInjector().getFrequency();

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

// HUD en vivo: actualización en línea
this->printf(
    "\r[%s|%lus] TPS=%.2fV(%.2f–%.2fV) %.0f%% | MAP=%.2fV(%.2f–%.2fV) | DAC=%3u | LVL=%.2f | FRQ=%.0fHz | Boost:%c | Beam:%c     ",
    stName, elapsed,
    tpsV, tpsMinV, tpsMaxV, tpsPct,
    mapV, mapMinV, mapMaxV,
    dac,
    level,
    freq,
    vortexOn ? '1' : '0',
    injOn ? '1' : '0'
  );


  // Detalle en nueva línea si cambió estado
  if (st != lastState) {
    lastState = st;
    this->println("\n\n=== VORTEX SYSTEM DASHBOARD ===");
    this->printf("Estado motor:      %s\n", stName);
    this->printf("TPS Voltage:       %.3f V (raw %u–%u)\n", tpsV, tpsMin, tpsMax);
    this->printf("MAP Voltage:       %.3f V (raw %u–%u)\n", mapV, mapMin, mapMax);
    this->printf("DAC Output:        %u (PWM)\n", dac);
    this->printf("Nivel acústico:    %.2f\n", level);
    this->printf("Frecuencia onda:   %.0f Hz\n", freq);
    this->printf("Vortex:             %s\n", vortexOn ? "ON" : "OFF");
    this->printf("Inyector sónico:   %s\n", injOn ? "ON" : "OFF");
    this->printf("Último cambio:     hace %lu s\n", elapsed);
    this->println("==============================\n");
  }
}


void ConsoleUI::toggleSistema() {
  sistemaActivo = !sistemaActivo;
  this->printf(">> Sistema %s.\n", sistemaActivo ? "ACTIVADO" : "DESACTIVADO");
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
  this->println(F("\n📘 Comandos disponibles:"));
  this->println(F("  a  → Activar/Desactivar sistema completo (seguridad/falla)"));
  this->println(F("  c  → Ejecutar rutina de calibración de sensores"));
  this->println(F("  m  → Mostrar menú de comandos"));
  this->println(F("  s  → Activar/Desactivar dashboard del sistema"));
  this->println(F("  d  → Activar modo desarrollador"));

  if (developerMode) {
    this->println(F("\n🧪 Modo desarrollador activo:"));
    this->println(F("  b  → Probar sonido acústico"));
    this->println(F("  i  → Activar relé INYECCIÓN_ACÚSTICA"));
    this->println(F("  t  → Activar relé TURBO"));
    this->println(F("  u  → (Comando dev pendiente)"));
    this->println(F("  x  → Paro manual, volver a IDLE"));
    this->println(F("  v  → Visualizar curva TPS-MAP (pendiente desarrollo)"));
    this->println(F("  r  → Borrar calibración actual"));
    this->println(F("  z  → Activar/Desactivar modo simulación"));
  }
}

bool ConsoleUI::isDeveloperMode() const {
  return developerMode;
}
