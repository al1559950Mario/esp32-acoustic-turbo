#include "BluetoothSerialConsoleUI.h"
#include <stdarg.h>
#include <Arduino.h>

BluetoothSerialConsoleUI::BluetoothSerialConsoleUI() {}

BluetoothSerialConsoleUI::~BluetoothSerialConsoleUI() {
  SerialBT.end();
}

void BluetoothSerialConsoleUI::begin() {
  if (!SerialBT.begin("TurboAcusticoV1", false)) {  // false = modo esclavo
    Serial.println("❌ Error al iniciar Bluetooth");
    return;
  }
  SerialBT.println("✅ Bluetooth iniciado como \"TurboAcusticoV1\"");
  imprimirHelp();  // si tienes ese método en ConsoleUI
}

void BluetoothSerialConsoleUI::update() {
  // Detecta cambio de estado de conexión y muestra mensaje solo al cambiar
  bool clienteActual = SerialBT.hasClient();

  if (clienteActual && !clientePrevio) {
    SerialBT.println("Cliente Bluetooth conectado");
    SerialBT.println("Conexión establecida.");
  } else if (!clienteActual && clientePrevio) {
    SerialBT.println("Cliente Bluetooth desconectado");
  }
  clientePrevio = clienteActual;

  // Si hay cliente, lee y procesa datos
  if (clienteActual) {
    while (SerialBT.available()) {
      char c = SerialBT.read();
      //Serial.write(c);   // muestra en monitor serie USB
      SerialBT.write(c); // eco para el cliente Bluetooth

      // Aquí puedes procesar el caracter 'c' para comandos o control
    }
  }

  ConsoleUI::update();  // Si quieres mantener lógica base
}

bool BluetoothSerialConsoleUI::inputAvailable() {
  return SerialBT.available() > 0;
}

String BluetoothSerialConsoleUI::readLine() {
  return SerialBT.readStringUntil('\n');
}

void BluetoothSerialConsoleUI::print(const String& msg) {
  SerialBT.print(msg);
  if (mirror) mirror->print(msg);
}

void BluetoothSerialConsoleUI::println(const String& msg) {
  SerialBT.println(msg);
  if (mirror) mirror->println(msg);
}

void BluetoothSerialConsoleUI::printf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  SerialBT.print(buf);
  if (mirror) mirror->print(buf);
}

bool BluetoothSerialConsoleUI::isSistemaActivo() {
  return SerialBT.hasClient();
}
