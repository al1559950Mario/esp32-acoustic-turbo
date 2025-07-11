// SerialUI.cpp
#include "SerialUI.h"

void SerialConsoleUI::begin() {
  Serial.begin(115200);
  while (!Serial);
  escribirLinea("=== Consola Serial iniciada ===");
  imprimirHelp();
}

String SerialConsoleUI::leerLinea() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();
    return linea;
  }
  return "";
}

void SerialConsoleUI::escribirLinea(const String& linea) {
  Serial.println(linea);
}