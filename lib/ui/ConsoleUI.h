#pragma once
#include <Arduino.h>

class ConsoleUI {
public:
  ConsoleUI() = default;
  void begin(unsigned long baud);
  bool calibrationRequested();
  void handle();

private:
};
