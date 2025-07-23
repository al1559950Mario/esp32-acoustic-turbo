#include "USBSerialConsoleUI.h"
#include <stdarg.h>

void USBSerialConsoleUI::begin() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n=== Consola Turbo-Acústica Iniciada ===");
  imprimirHelp();
}

void USBSerialConsoleUI::update() {
  ConsoleUI::update();
}

bool USBSerialConsoleUI::inputAvailable() {
  return Serial.available() > 0;
}

String USBSerialConsoleUI::readLine() {
  return Serial.readStringUntil('\n');
}

void USBSerialConsoleUI::print(const String& msg) {
  Serial.print(msg);
  if (mirror) mirror->print(msg);
}

void USBSerialConsoleUI::println(const String& msg) {
  Serial.println(msg);
  if (mirror) mirror->println(msg);
}

void USBSerialConsoleUI::printf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  if (mirror) mirror->print(buf);
}
