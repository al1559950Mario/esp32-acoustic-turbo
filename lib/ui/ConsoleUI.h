#pragma once
#include <Arduino.h>
#include "StateMachine.h"
#include "MAPSensor.h"
#include "TPSSensor.h"
#include "AcousticInjector.h"
#include "TurboController.h"

/**
 * ConsoleUI
 * Interfaz por consola serial interactiva. Incluye:
 *  - Menu principal
 *  - Modo desarrollador extendido
 *  - Dashboard en tiempo real
 */
class ConsoleUI {
public:
  void begin();
  void update();
  void setFSM(StateMachine* fsmRef);

  void attachSensors(MAPSensor* mapPtr, TPSSensor* tpsPtr);
  void attachActuators(TurboController* turboPtr, AcousticInjector* injectorPtr);

  bool getCalibRequest();

private:
  StateMachine*      fsm         = nullptr;
  MAPSensor*         mapSensor   = nullptr;
  TPSSensor*         tpsSensor   = nullptr;
  TurboController*   turbo       = nullptr;
  AcousticInjector*  injector    = nullptr;

  bool consoleCalibRequested     = false;
  bool developerMode             = false;
  unsigned long lastTransitionMS = 0;
  SystemState lastState          = SystemState::OFF;

  void interpretarComando(char c);
  void imprimirDashboard();
  void imprimirHelp();
};
