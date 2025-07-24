#pragma once
#include "ConsoleUI.h"
#include <BluetoothSerial.h>

class BluetoothSerialConsoleUI : public ConsoleUI {
public:
  BluetoothSerialConsoleUI(ConsoleUI** uiPtr) : ui(uiPtr) {}//

  BluetoothSerialConsoleUI();
  ~BluetoothSerialConsoleUI();

  void begin() override;
  void update() override;
  bool inputAvailable() override;
  String readLine() override;
  void print(const String& msg) override;
  void println(const String& msg) override;
  void printf(const char* fmt, ...) override;

  bool isSistemaActivo();  // Retorna true si hay cliente Bluetooth conectado

private:
  ConsoleUI** ui;  // puntero al puntero global ui
  BluetoothSerial SerialBT;
  bool clientePrevio = false;
};
