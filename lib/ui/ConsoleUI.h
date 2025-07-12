#pragma once
#include <Arduino.h>
#include "StateMachine.h"
#include "SensorManager.h"
#include "TurboController.h"
#include "AcousticInjector.h"
#include "ActuatorManager.h"  // Necesario para el puntero actuators

class ConsoleUI {
public:
  virtual void begin() = 0;
  virtual void update();

  virtual void setFSM(StateMachine* fsmRef);
  virtual void attachSensors(SensorManager* sensorManagerPtr);
  virtual void attachActuators(ActuatorManager* actuatorManagerPtr);  // ✅ Corregido

  virtual bool getCalibRequest();
  virtual void runConsoleCalibration();
  virtual void toggleSistema();
  virtual bool isSistemaActivo() const { 
    return sistemaActivo; }
  virtual void imprimirDashboard();
  virtual int parseValor(const String& linea, const String& clave);

protected:
  bool sistemaActivo = true;
  StateMachine*      fsm = nullptr;
  SensorManager*     sensors = nullptr;
  ActuatorManager*   actuators = nullptr;   // ✅ Ahora sí está declarado correctamente

  bool dashboardEnabled = true;
  bool consoleCalibRequested = false;
  bool developerMode = false;
  bool simulacionActiva = false;

  unsigned long lastTransitionMS = 0;
  SystemState lastState = SystemState::OFF;

  // Métodos virtuales puros que deben implementarse en SerialUI o BLEUI
  virtual bool inputAvailable() = 0;
  virtual String readLine() = 0;
  virtual void print(const String& msg) = 0;
  virtual void println(const String& msg) = 0;
  virtual void printf(const char* fmt, ...) = 0;

  virtual void interpretarComando(char c);
  virtual void imprimirHelp();
};
