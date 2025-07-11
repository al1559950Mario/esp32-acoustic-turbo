#pragma once
#include <Arduino.h>
#include "StateMachine.h"
#include "SensorManager.h"
#include "TurboController.h"
#include "AcousticInjector.h"

class ConsoleUI {
public:
  virtual void begin() = 0;  // La base no implementa esto, lo hacen las hijas
  virtual void update();

  virtual void setFSM(StateMachine* fsmRef);
  virtual void attachSensors(SensorManager* sensorManagerPtr);
  virtual void attachActuators(TurboController* turboPtr, AcousticInjector* injectorPtr);

  virtual bool getCalibRequest();
  virtual void runConsoleCalibration();
  virtual void toggleSistema();
  virtual bool isSistemaActivo() const { return sistemaActivo; }
  virtual void imprimirDashboard();
  virtual int parseValor(const String& linea, const String& clave);

protected:
  bool sistemaActivo = true;
  StateMachine*      fsm = nullptr;
  SensorManager*     sensors = nullptr;
  TurboController*   turbo = nullptr;
  AcousticInjector*  injector = nullptr;

  bool dashboardEnabled = true;
  bool consoleCalibRequested = false;
  bool developerMode = false;
  bool simulacionActiva = false;

  unsigned long lastTransitionMS = 0;
  SystemState lastState = SystemState::OFF;

  // 🔧 Estas funciones deben ser implementadas por la clase hija (Serial, BLE, etc.)
  virtual bool inputAvailable() = 0;
  virtual String readLine() = 0;
  virtual void print(const String& msg) = 0;
  virtual void println(const String& msg) = 0;
  virtual void printf(const char* fmt, ...) = 0;

  virtual void interpretarComando(char c);
  virtual void imprimirHelp();
};
