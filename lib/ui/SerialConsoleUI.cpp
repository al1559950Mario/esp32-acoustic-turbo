#include "SerialConsoleUI.h"
#include <stdarg.h>

void SerialConsoleUI::begin() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n=== Consola Turbo-Acústica Iniciada ===");
  imprimirHelp();
}

void SerialConsoleUI::update() {
  ConsoleUI::update();
}

bool SerialConsoleUI::inputAvailable() {
  return Serial.available() > 0;
}

String SerialConsoleUI::readLine() {
  return Serial.readStringUntil('\n');
}

void SerialConsoleUI::print(const String& msg) {
  Serial.print(msg);
}

void SerialConsoleUI::println(const String& msg) {
  Serial.println(msg);
}

void SerialConsoleUI::printf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
}
