#include "BLEUI.h"
#include "CalibrationManager.h"

BLEUI::BLEUI() {
  // Constructor vacío — todo se configura en begin()
}

void BLEUI::begin(const String& nombreBT) {
  SerialBT.begin(nombreBT);
  SerialBT.println(">> Bluetooth iniciado como \"" + nombreBT + "\"");
  ConsoleUI::imprimirHelp();
}

void BLEUI::update() {
  if (!fsm) return;

  // 1. Transición de estado
  if (fsm->getState() != lastState) {
    lastTransitionMS = millis();
    lastState = fsm->getState();
  }

  // 2. Lectura y procesamiento por Bluetooth
  if (SerialBT.available()) {
    String linea = SerialBT.readStringUntil('\n');
    linea.trim();

    if (simulacionActiva && linea.startsWith("tps_raw:")) {
      int idxTPS = linea.indexOf("tps_raw:");
      int idxMAP = linea.indexOf("map_raw:");

      if (idxTPS != -1 && idxMAP != -1) {
        uint16_t tpsRaw = linea.substring(idxTPS + 8, linea.indexOf(",", idxTPS)).toInt();
        uint16_t mapRaw = linea.substring(idxMAP + 8).toInt();

        tpsSensor->setSimulatedRaw(tpsRaw);
        mapSensor->setSimulatedRaw(mapRaw);

        SerialBT.println("Recibido tps=" + String(tpsRaw) + " map=" + String(mapRaw));
      }
    } else if (linea.length() == 1) {
      interpretarComando(linea.charAt(0));  // hereda del padre
    } else {
      SerialBT.println("⚠️ Comando no reconocido o fuera de modo simulación.");
    }
  }

  // 3. Proceso de calibración
  if (getCalibRequest()) {
    SerialBT.println(">> Iniciando calibración...");
    auto& calib = CalibrationManager::getInstance();
    calib.clearCalibration();
    calib.runTPSCalibration(*tpsSensor);
    calib.runMAPCalibration(*mapSensor);
    calib.saveCalibration();
    SerialBT.println(">> Calibración completada.");
  }

  // 4. HUD en tiempo real
  if (dashboardEnabled) {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 300) {
      float tpsV = tpsSensor->readVolts();
      float mapV = mapSensor->readVolts();
      uint8_t dac = injector->getCurrentDAC();
      bool turboOn = turbo->isOn();
      bool injOn = injector->isActive();
      SystemState st = fsm->getState();
      unsigned long elapsed = (millis() - lastTransitionMS) / 1000;

      const char* stateNames[] = {
        "OFF", "SIN_CAL", "CALIB", "IDLE",
        "BEAM", "BOOST", "DESCAY", "DEBUG", "??"
      };
      const char* stName = stateNames[int(st)];

      SerialBT.printf(
        "\r[%s|%lus] TPS=%.2fV | MAP=%.2fV | DAC=%3u | T:%c | I:%c     \n",
        stName, elapsed,
        tpsV, mapV, dac,
        turboOn ? '1' : '0',
        injOn   ? '1' : '0'
      );
      lastPrint = millis();
    }
  }
}
