#pragma once
#include "ConsoleUI.h"

class SerialConsoleUI : public ConsoleUI {
public:
  void begin() override;
  virtual String leerLinea() override;
  virtual void escribirLinea(const String& linea) override;
};