#include "BLEConsoleUI.h"
#include <stdarg.h>

void BLEConsoleUI::begin() {
  if (!SerialBT.begin("TurboAcustico")) {
    Serial.println("❌ Error al iniciar Bluetooth");
    return;
  }
  SerialBT.setPin("0000");
  SerialBT.println("✅ Bluetooth iniciado como \"TurboAcustico\"");
  imprimirHelp();
}

void BLEConsoleUI::update() {
  ConsoleUI::update();
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
