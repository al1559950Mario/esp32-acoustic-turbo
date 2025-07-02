#pragma once
#include <Arduino.h>

class DebugManager {
public:
  DebugManager() = default;
  void begin();
  void handle();
  int getLevel() const;

  static DebugManager& instance();

private:
  int _level = 0;
};
