#include "BLEUI.h"
#include "CalibrationManager.h"

BLEUI::BLEUI() : ConsoleUI() {}

void BLEUI::begin(const String& nombreBT) {
  if (!SerialBT.begin(nombreBT)) {
    Serial.println("Error al iniciar Bluetooth");
    return;
  }
  SerialBT.setPin("0000");
  SerialBT.println(">> Bluetooth iniciado como \"" + nombreBT + "\"");
  imprimirHelp();
}

void BLEUI::update() {
  if (!fsm) return;
  if (!actuators) return;    // evita crash si nadie hizo attachSensors()
  // 1. Estado (mantenemos)
  if (fsm->getState() != lastState) {
    lastTransitionMS = millis();
    lastState = fsm->getState();
  }

  // 2. Lectura Bluetooth ⬅️ comentado para aislar el bug
  if (SerialBT.available()) {
    char c = SerialBT.read();
    interpretarComando(c);
  }

  // 3. Proceso calibración (podrías comentar también)
  if (getCalibRequest()) {
    // …
  }

  // 4. HUD en Bluetooth (cada 300 ms)
  // …
}

void BLEUI::interpretarComando(char c) {
  switch (c) {
    case 'm':
      imprimirHelp();
      break;

    case 's':
      SerialBT.println("Comando 's' recibido");
      break;

    case 'c':
      consoleCalibRequested = true;
      SerialBT.println(">> Solicitud de calibración registrada.");
      break;

    default:
      SerialBT.print("❓ Comando no reconocido: ");
      SerialBT.println(c);
      break;
  }
}

void BLEUI::attachSensors(SensorManager* manager) {
  sensors = manager;
}

void BLEUI::imprimirHelp() {
  SerialBT.println(F("\n📘 Comandos Bluetooth disponibles:"));
  SerialBT.println(F("  m  → Mostrar este menú"));
  SerialBT.println(F("  c  → Ejecutar calibración"));
  SerialBT.println(F("  s  → Comando personalizado"));
}