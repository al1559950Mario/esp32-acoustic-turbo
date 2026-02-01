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
    if (simulationOnPython && linea.startsWith("maf_raw:")) {
      int idxTPS = linea.indexOf("maf_raw:");
      int idxMAP = linea.indexOf("map_raw:");

      if (idxTPS != -1 && idxMAP != -1) {
        uint16_t tpsRaw = linea.substring(idxTPS + 8, idxMAP - 1).toInt(); // -1 para excluir la coma
        uint16_t mapRaw = linea.substring(idxMAP + 8).toInt();

        sensors->getMAF().setSimulatedRaw(tpsRaw);
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
  
  if (simulationOnPython)
      tiempoProximaImpresionHUD = millis() + 2500;

  switch (c) {
    case 'a':  // Toggle sistema ON/OFF
      toggleSistema();
      break;

    case 'b':  // Test acústico: piso dinámico del DAC
      if (!devOnly()) break;
      if (actuators) actuators->testFloorDynamic();
      break;

    case 'B':  // Test acústico: piso ultra (sub‑LSB) usando driver I2S
      if (!devOnly()) break;
      if (actuators) actuators->testFloorUltra();
      break;

    case 'N':  // Test acústico: piso nano (−70..−90 dBFS)
      if (!devOnly()) break;
      if (actuators) actuators->testFloorNano();
      break;

    case 'p':  // Iniciar seno continuo 1 kHz @ 60% (driver I2S)
      if (!devOnly()) break;
      this->println(F("[UI] Iniciando seno continuo 1 kHz (60%) por I2S"));
      if (actuators) actuators->startPureSine(1000, 0.6f);
      break;

    case 'q':  // Detener seno continuo
      if (!devOnly()) break;
      this->println(F("[UI] Deteniendo seno continuo I2S"));
      if (actuators) actuators->stopPureSine();
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

    case 'o':  // Seno por ISR (AcousticInjector) 1 kHz @ 30%
      if (!devOnly()) break;
      this->println(F("[UI] Iniciando seno ISR 4 kHz (30%)"));
      if (actuators) actuators->startISRSine(4000, 0.9f);
      break;

    case 'O':  // Detener seno ISR
      if (!devOnly()) break;
      this->println(F("[UI] Deteniendo seno ISR"));
      if (actuators) actuators->stopISRSine();
      break;

    case 'u':  // Mostrar estadísticas de audio (drops/underruns/watermarks)
      if (!devOnly()) break;
      if (actuators) {
        uint32_t drops=0, underruns=0; size_t minA=0, maxA=0;
        actuators->getAudioStats(drops, underruns, minA, maxA);
        this->printf("[AUDIO] dropsFromISR=%lu, underruns=%lu, minAvail=%u, maxAvail=%u\n",
                     (unsigned long)drops, (unsigned long)underruns,
                     (unsigned)minA, (unsigned)maxA);
      }
      break;

    case 'U':  // Reset estadísticas de audio
      if (!devOnly()) break;
      if (actuators) actuators->resetAudioStats();
      this->println(F("[AUDIO] Estadísticas reseteadas"));
      break;

    case 'f':  // Cambiar rango de frecuencia acústica
        if (!devOnly()) break;
        {
            auto& injector = actuators->getAcousticInjector();
            uint8_t nextOption = (static_cast<uint8_t>(injector.getFrequencyRangeOption()) + 1) % 4; // 4 opciones
            injector.setFrequencyRangeOption(static_cast<AcousticInjector::FrequencyRangeOption>(nextOption));

            // Obtener rango actual
            float fMin = injector.getFreqMin();
            float fMax = injector.getFreqMax();

            this->printf(">> Nuevo rango de frecuencia seleccionado: %d → %.0f Hz – %.0f Hz\n",
                        nextOption + 1, fMin, fMax);
        }
        break;

    case 'i': // Ajustar umbrales INJ ON desde consola (dev only)
      if (!devOnly()) break;
      this->println(">> Cambiar INJ ON: escribe dos valores min 60 max 90: <INJ_MAP_ON> <INJ_TPS_ON> (ej: 60 70)");
      {
        Serial.println("[DBG] case 'i' - waiting input");
        // Espera línea completa usando inputAvailable() y readLine() (timeout 10s)
        unsigned long start = millis();
        String line;
        while (millis() - start < 15000) { // espera hasta 10s
          if (inputAvailable()) {
            line = readLine();
            line.trim();
            if (line.length()) break;
          }
          delay(5);
        }
        if (line.length() == 0) {
          this->println("⚠️ Tiempo de entrada agotado. Operación cancelada.");
          break;
        }
        // Parsear dos floats
        float mapOn = NAN, tpsOn = NAN;
        int read = sscanf(line.c_str(), "%f %f", &mapOn, &tpsOn);
        if (read < 1) {
          this->println("⚠️ Entrada inválida. Formato: <INJ_MAP_ON> <INJ_TPS_ON>");
          break;
        }
        // Si solamente dio uno, pedimos el otro explícitamente
        if (read == 1) {
          this->printf(">> INJ_MAP_ON = %.1f. Ahora escribe INJ_TPS_ON:\n", mapOn);
          start = millis();
          String line2;
          while (millis() - start < 10000) {
            if (inputAvailable()) {
              line2 = readLine();
              line2.trim();
              if (line2.length()) break;
            }
            delay(5);
          }
          if (line2.length() == 0) {
            this->println("⚠️ Tiempo de entrada agotado. Operación cancelada.");
            break;
          }
          if (sscanf(line2.c_str(), "%f", &tpsOn) != 1) {
            this->println("⚠️ Valor INJ_TPS_ON inválido. Operación cancelada.");
            break;
          }
        } else {
        }

        // Validaciones simples de rango 0..100
        // Usamos comprobación NaN-portable
        auto isNumber = [](float v) { return v == v; }; // false para NaN
        if (!isNumber(mapOn) || !isNumber(tpsOn) || mapOn < 0.0f || mapOn > 100.0f || tpsOn < 0.0f || tpsOn > 100.0f) {
          this->println("⚠️ Valores fuera de rango (0..100). Operación cancelada.");
          break;
        }

        // Actualizar ThresholdManager y persistir
        bool okMap = ThresholdManager::getInstance().setThreshold("INJ_MAP_ON", mapOn);

        bool okTps = ThresholdManager::getInstance().setThreshold("INJ_TPS_ON", tpsOn);

        if (okMap && okTps) {
          bool saved = ThresholdManager::getInstance().save();
          if (saved) {
            this->printf(">> Umbrales guardados: INJ_MAP_ON=%.1f, INJ_TPS_ON=%.1f\n", mapOn, tpsOn);
          } else {
            this->println("⚠️ Error guardando en NVS.");
          }
        } else {
          this->println("⚠️ Error: claves INJ no encontradas en ThresholdManager.");
        }
      }
      break;

    case 'j':  // Activar logging con modo específico (1–4)
      if (!devOnly()) break;

      this->println(">> Activar logging con modo específico.");
      this->println("   Escribe un número del 1 al 4:");
      this->println("   1 = All ON");
      this->println("   2 = Acoustic ON, Turbo OFF");
      this->println("   3 = Acoustic OFF, Turbo ON");
      this->println("   4 = All OFF");

      {
        unsigned long start = millis();
        String line;
        while (millis() - start < 10000) {
          if (inputAvailable()) {
            line = readLine();
            line.trim();
            if (line.length()) break;
          }
          delay(5);
        }

        if (line.length() == 0) {
          this->println("⚠️ Tiempo de entrada agotado. Operación cancelada.");
          break;
        }

        int modo = atoi(line.c_str());
        if (modo < 1 || modo > 4) {
          this->println("⚠️ Modo inválido. Debe ser 1, 2, 3 o 4.");
          break;
        }

        logger->enable(modo);
        dashboardEnabled = false;  // desactivar HUD para evitar ruido visual

        this->printf(">> Logging ACTIVADO en modo %d.\n", modo);
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
        fsm->debugForceState(SystemState::NO_CALIB);
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
        //sensors->getMAF().enableSimulation();
        sensors->enableSimulacion();
        //sensors->getMAP().enableSimulation();
        sensors->enableSimulacion();
      } else {
        sensors->getMAF().disableSimulation();
        sensors->disableSimulacion();
        sensors->getMAP().disableSimulation();
        sensors->disableSimulacion();
      }

      this->printf(">> Modo simulación %s.\n", simulationOnPython ? "ACTIVADO" : "DESACTIVADO");
      break;
    case 'k':  // Verificar valores de sensores
      if (!devOnly()) break;

      this->printf("== DEBUG Sensores ==\n");

      this->printf("MAF: raw=%d, volts=%.2f, %%=%.1f%%\n",
                  sensors->getMAF().readRaw(),
                  sensors->getMAF().readVolts(),
                  sensors->getMAF().readPorcent());

      this->printf("MAP: raw=%d, volts=%.2f\n",
                  sensors->getMAP().readRaw(),
                  sensors->getMAP().readVolts());
      break;
    case 'l':  // Toggle logging CSV vía Bluetooth
      if (!devOnly()) break;
      if (logger) {
        bool nuevoEstado = !logger->isEnabled();
        logger->enable(nuevoEstado);

        if (nuevoEstado) {
          this->println(">> Logging ACTIVADO: se enviarán datos CSV por Bluetooth.");
          dashboardEnabled = false;  // detener HUD si es necesario
        } else {
          this->println(">> Logging DESACTIVADO: se reanuda consola normal.");
          dashboardEnabled = true;   // reactivar HUD
        }
      } else {
        this->println("⚠️ Logger no conectado.");
      }
      break;

    default:
      if (!simulationOnPython) break;
      this->print("❓ Comando no reconocido: ");
      this->println(String(c));


  }
}

void ConsoleUI::imprimirDashboard() {
    if (!fsm || !sensors || !actuators) return;
    //if (millis() < tiempoProximaImpresionHUD) return;

    auto& calib = CalibrationManager::getInstance();
    uint16_t tpsMin = calib.getTPSMin();
    uint16_t tpsMax = calib.getTPSMaxRaw();
    uint16_t mapMin = calib.getMAPMin();
    uint16_t mapMax = calib.getMAPMaxRaw();

    constexpr float LSB_MV = 0.1875f;  // mV por bit en GAIN_TWOTHIRDS
    float tpsMinV = (tpsMin * LSB_MV) / 1000.0f;
    float tpsMaxV = (tpsMax * LSB_MV) / 1000.0f;
    float mapMinV = (mapMin * LSB_MV) / 1000.0f;
    float mapMaxV = (mapMax * LSB_MV) / 1000.0f;


    float tpsV = sensors->readMAFVolts();
    float tpsPct = sensors->readMAFLoadPercent();
    float mapPct = sensors->readMAPLoadPercent();
    float mapV = sensors->readMAPVolts();
    uint8_t dac = actuators->getAcousticInjector().getCurrentDAC();
    bool vortexOn = actuators->isTurboOn();
    bool injOn = actuators->isAcousticOn();

    float level = actuators->getAcousticInjector().getLevel();
    float freq = actuators->getAcousticInjector().getFrequency();
    float oscillationAmplitude = sensors->computeOscillationAmplitude();
    float rms = sensors->computeRMS();
    float pressurePercent = sensors->getPressurePercentSigned();

    // Obtener potencia del turbo (0–100%)
    float boostLevel = actuators->getVortexController().getLastPWM() * 100.0f;
    float boostSense = actuators->getTurboSense();

    SystemState st = fsm->getState();
    unsigned long elapsed = (millis() - lastTransitionMS) / 1000;

    static const char* stateNames[] = {
        "OFF", "SIN_CAL", "CALIB", "IDLE",
        "ALIGN", "FLOW", "DECAY", "DEBUG", "??"
    };
    const char* stName = stateNames[int(st)];

    // HUD en vivo: actualización en línea
    this->printf(
        "\r[%s|%lus]MAF%.1f(%.1f–%.1f)%.0f%%MAP%.1f(%.1f–%.1f)%.0f%%|L%.3f|%.1fkhz|%.0f%% IS:%.1f%|Pk:%.1f|Rm%.1f|P:%.1f%",
        stName, elapsed,
        tpsV, tpsMinV, tpsMaxV, tpsPct,
        mapV, mapMinV, mapMaxV, mapPct,
        level,
        freq/1000.0f,
        boostLevel, 
        boostSense,
        oscillationAmplitude,
        rms,
        pressurePercent
      );

    // Detalle en nueva línea si cambió estado
    if (st != lastState) {
        lastState = st;
        this->println("\n\n=== VORTEX SYSTEM DASHBOARD ===");
        this->printf("Estado motor:      %s\n", stName);
        this->printf("MAF Voltage:       %.3f V\n", tpsV);
        this->printf("MAP Voltage:       %.3f V\n", mapV);
        this->printf("DAC Output:        %u\n", dac);
        this->printf("Nivel acústico:    %.2f\n", level);
        this->printf("Frecuencia onda:   %.0f Hz\n", freq);
        this->printf("Vortex:            %s\n", vortexOn ? "ON" : "OFF");
        this->printf("Boost Level:       %.0f%%\n", boostLevel);
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
  this->println(F("  d  → Activar modo desarrollador"));
  this->println(F("  m  → Mostrar menú de comandos"));
  this->println(F("  s  → Activar/Desactivar dashboard del sistema"));

  if (developerMode) {
    this->println(F("\n🧪 Modo desarrollador activo:"));
    this->println(F("  b  → Probar sonido acústico"));
    this->println(F("  f  → Alternar rango de frecuencia del BEAM"));
    this->println(F("  i  → Cambiar umbrales INJ ON"));
    this->println(F("  j  → Activar logging"));
    this->println(F("  r  → Borrar calibración actual"));
    this->println(F("  v  → Visualizar curva MAF-MAP (pendiente desarrollo)"));
    this->println(F("  x  → Paro manual, volver a IDLE"));
    this->println(F("  z  → Activar/Desactivar modo simulación"));
  }
}


bool ConsoleUI::isDeveloperMode() const {
  return developerMode;
}
