#pragma once
#include "ConsoleUI.h"

class USBSerialConsoleUI : public ConsoleUI {
public:
  USBSerialConsoleUI(ConsoleUI** uiPtr): ui(uiPtr) {}//Puntero


  void begin() override;
  void update() override;
  bool inputAvailable() override;
  String readLine() override;
  void print(const String& msg) override;
  void println(const String& msg) override;
  void printf(const char* fmt, ...) override;
  ConsoleUI** ui;  // puntero al puntero global ui

};
