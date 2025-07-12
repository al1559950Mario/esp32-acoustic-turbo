#pragma once
#include "ConsoleUI.h"
#include <BluetoothSerial.h>

class BLEConsoleUI : public ConsoleUI {
public:
  BLEConsoleUI();
  ~BLEConsoleUI();

  void begin() override;
  void update() override;
  bool inputAvailable() override;
  String readLine() override;
  void print(const String& msg) override;
  void println(const String& msg) override;
  void printf(const char* fmt, ...) override;

  bool isSistemaActivo();  // Retorna true si hay cliente Bluetooth conectado

private:
  BluetoothSerial SerialBT;
  bool clientePrevio = false;
};
