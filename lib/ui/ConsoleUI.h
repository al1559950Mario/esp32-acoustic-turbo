#pragma once
#include <Arduino.h>
#include "StateMachine.h"
#include "SensorManager.h"
#include "AcousticInjector.h"
#include "TurboController.h"

class ConsoleUI {
public:
  bool dashboardEnabled = true;

  virtual void begin();
  virtual void update();
  virtual void setFSM(StateMachine* fsmRef);

  // Ahora solo un método para conectar el SensorManager
  virtual void attachSensors(SensorManager* sensorManagerPtr);

  virtual void attachActuators(TurboController* turboPtr, AcousticInjector* injectorPtr);
  virtual void imprimirDashboard();
  virtual bool getCalibRequest();
  virtual void runConsoleCalibration();
  bool isSistemaActivo() const { return sistemaActivo; }
  void toggleSistema();
  int parseValor(const String& linea, const String& clave);

protected:
  bool sistemaActivo = true;
  StateMachine*      fsm = nullptr;
  float lastTPS = -1.0f;
  float lastMAP = -1.0f;
  uint8_t lastDAC = 0;

  bool simulacionActiva = false;

  // Solo el SensorManager, no sensores separados
  SensorManager*     sensors = nullptr;
  TurboController*   turbo = nullptr;
  AcousticInjector*  injector = nullptr;

  bool consoleCalibRequested = false;
  bool developerMode = false;
  unsigned long lastTransitionMS = 0;
  SystemState lastState = SystemState::OFF;

  virtual void interpretarComando(char c);
  virtual void imprimirHelp();
};
