#include "BluetoothSerialConsoleUI.h"
#include <stdarg.h>
#include <Arduino.h>

BluetoothSerialConsoleUI::BluetoothSerialConsoleUI() {}

BluetoothSerialConsoleUI::~BluetoothSerialConsoleUI() {
  SerialBT.end();
}

void BluetoothSerialConsoleUI::begin() {
  if (!SerialBT.begin("VortexAcusticoV1", false)) {  // false = modo esclavo
    Serial.println("❌ Error al iniciar Bluetooth");
    return;
  }
  SerialBT.println("✅ Bluetooth iniciado como \"VortexAcusticoV1\"");
  imprimirHelp();  // si tienes ese método en ConsoleUI
}

void BluetoothSerialConsoleUI::update() {
  // Detectar cambios de conexión para mostrar mensajes (puedes guardarlos en un flag para imprimir en update base)
  bool clienteActual = SerialBT.hasClient();

  if (clienteActual && !clientePrevio) {
    SerialBT.println("Cliente Bluetooth conectado");
    SerialBT.println("Conexión establecida.");
  } else if (!clienteActual && clientePrevio) {
    SerialBT.println("Cliente Bluetooth desconectado");
  }
  clientePrevio = clienteActual;

  // Solo llamar a la base para procesar comandos y línea completa
  ConsoleUI::update();
}


bool BluetoothSerialConsoleUI::inputAvailable() {
  return SerialBT.available() > 0;
}

String BluetoothSerialConsoleUI::readLine() {
  return SerialBT.readStringUntil('\n');
}

void BluetoothSerialConsoleUI::print(const String& msg) {
  SerialBT.print(msg);
    if (mirror && this == *ui) 
      mirror->print(msg);
}

void BluetoothSerialConsoleUI::println(const String& msg) {
  SerialBT.println(msg);
    if (mirror && this == *ui) 
      mirror->println(msg);
}

void BluetoothSerialConsoleUI::printf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  SerialBT.print(buf);
  if (mirror && this == *ui) 
    mirror->print(buf);
  if (mirror) mirror->print(buf);
}

bool BluetoothSerialConsoleUI::isSistemaActivo() {
  return SerialBT.hasClient();
}
