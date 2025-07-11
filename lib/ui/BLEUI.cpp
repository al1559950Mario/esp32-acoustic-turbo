// BLEConsoleUI.cpp
#include "BLEUI.h"

void BLEConsoleUI::begin() {
  if (!SerialBT.begin("TurboAcustico")) {
    Serial.println("❌ Error al iniciar Bluetooth.");
    return;
  }
  SerialBT.setPin("0000");
  print("=== Bluetooth listo ===");
  imprimirHelp();
}

String BLEConsoleUI::readLine() {
  if (SerialBT.available()) {
    String linea = SerialBT.readStringUntil('\n');
    linea.trim();
    return linea;
  }
  return "";
}

void BLEConsoleUI::print(const String& linea) {
  SerialBT.println(linea);
}