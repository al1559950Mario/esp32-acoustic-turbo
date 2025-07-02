#include "ConsoleUI.h"

void ConsoleUI::begin(unsigned long baud) {
  Serial.begin(baud);
}

bool ConsoleUI::calibrationRequested() {
  return false;  // stub
}

void ConsoleUI::handle() {
  // stub
}
