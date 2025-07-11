#pragma once
#include "ConsoleUI.h"
#include <BluetoothSerial.h>

class BLEConsoleUI : public ConsoleUI {
public:
  void begin() override;
  virtual String readLine() override;
  virtual void print(const String& linea) override;

private:
  BluetoothSerial SerialBT;
};