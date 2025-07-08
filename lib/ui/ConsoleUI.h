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
  bool dashboardEnabled = true;
  
  void begin();

  virtual void update();
  void setFSM(StateMachine* fsmRef);

  void attachSensors(MAPSensor* mapPtr, TPSSensor* tpsPtr);
  void attachActuators(TurboController* turboPtr, AcousticInjector* injectorPtr);

  virtual void imprimirDashboard();
  bool getCalibRequest();
  void runConsoleCalibration();
  bool isSistemaActivo() const { return sistemaActivo; }
  void toggleSistema();  // Nuevo método para alternar
  int parseValor(const String& linea, const String& clave);

protected:
  bool sistemaActivo = true;
  StateMachine*      fsm = nullptr;
  float lastTPS = -1.0f;
  float lastMAP = -1.0f;
  uint8_t lastDAC = 0;

  bool simulacionActiva = false;

  MAPSensor*         mapSensor = nullptr;
  TPSSensor*         tpsSensor = nullptr;
  TurboController*   turbo = nullptr;
  AcousticInjector*  injector = nullptr;

  bool consoleCalibRequested = false;
  bool developerMode = false;
  unsigned long lastTransitionMS = 0;
  SystemState lastState = SystemState::OFF;

  virtual void interpretarComando(char c);
  virtual void imprimirHelp();

};
