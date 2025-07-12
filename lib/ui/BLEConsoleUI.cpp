#include "BLEConsoleUI.h"
#include <stdarg.h>
#include <Arduino.h>

BLEConsoleUI::BLEConsoleUI() {}

BLEConsoleUI::~BLEConsoleUI() {
  SerialBT.end();
}

void BLEConsoleUI::begin() {
  if (!SerialBT.begin("TurboAcusticoV1", false)) {  // false = modo esclavo
    Serial.println("❌ Error al iniciar Bluetooth");
    return;
  }
  SerialBT.println("✅ Bluetooth iniciado como \"TurboAcusticoV1\"");
  imprimirHelp();  // si tienes ese método en ConsoleUI
}

void BLEConsoleUI::update() {
  // Detecta cambio de estado de conexión y muestra mensaje solo al cambiar
  bool clienteActual = SerialBT.hasClient();

  if (clienteActual && !clientePrevio) {
    Serial.println("Cliente Bluetooth conectado");
    SerialBT.println("Conexión establecida.");
  } else if (!clienteActual && clientePrevio) {
    Serial.println("Cliente Bluetooth desconectado");
  }
  clientePrevio = clienteActual;

  // Si hay cliente, lee y procesa datos
  if (clienteActual) {
    while (SerialBT.available()) {
      char c = SerialBT.read();
      Serial.write(c);   // muestra en monitor serie USB
      SerialBT.write(c); // eco para el cliente Bluetooth

      // Aquí puedes procesar el caracter 'c' para comandos o control
    }
  }

  ConsoleUI::update();  // Si quieres mantener lógica base
}

bool BLEConsoleUI::inputAvailable() {
  return SerialBT.available() > 0;
}

String BLEConsoleUI::readLine() {
  return SerialBT.readStringUntil('\n');
}

void BLEConsoleUI::print(const String& msg) {
  SerialBT.print(msg);
}

void BLEConsoleUI::println(const String& msg) {
  SerialBT.println(msg);
}

void BLEConsoleUI::printf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  SerialBT.print(buf);
}

bool BLEConsoleUI::isSistemaActivo() {
  return SerialBT.hasClient();
}
