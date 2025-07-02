#include "DebugManager.h"

void DebugManager::begin() {
  // stub
}

void DebugManager::handle() {
  // stub
}

int DebugManager::getLevel() const {
  return _level;  // stub
}

DebugManager& DebugManager::instance() {
  static DebugManager inst;
  return inst;
}
