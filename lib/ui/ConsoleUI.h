#pragma once
#include <Arduino.h>
#include "StateMachine.h"
#include "SensorManager.h"
#include "ActuatorManager.h" 

class ConsoleUI {
public:
  virtual void begin() = 0;
  virtual void update();

  virtual void setFSM(StateMachine* fsmRef);
  virtual void attachSensors(SensorManager* sensorManagerPtr);
  virtual void attachActuators(ActuatorManager* actuatorManagerPtr);

  virtual bool getCalibRequest();
  virtual void runConsoleCalibration();
  virtual void toggleSistema();
  virtual bool isSistemaActivo() const { 
    return sistemaActivo; }
  virtual void imprimirDashboard();
  virtual int parseValor(const String& linea, const String& clave);
  void setMirror(ConsoleUI* mirrorUI) { this->mirror = mirrorUI; }
  virtual void print(const String& msg) = 0;
  virtual void println(const String& msg) = 0;
  virtual void printf(const char* fmt, ...) = 0;
  virtual bool isSimulation() const { return  simulationOnPython; };


  virtual bool isDeveloperMode() const;
protected:
  bool sistemaActivo = true;
  StateMachine*      fsm = nullptr;
  SensorManager*     sensors = nullptr;
  ActuatorManager*   actuators = nullptr;  

  bool dashboardEnabled = true;
  bool consoleCalibRequested = false;
  bool developerMode = false;
  bool simulationOnPython = false;

  unsigned long lastTransitionMS = 0;
  unsigned long tiempoProximaImpresionHUD = 0;

  SystemState lastState = SystemState::OFF;

  

  // Métodos virtuales puros que deben implementarse en SerialUI o BLEUI
  virtual bool inputAvailable() = 0;
  virtual String readLine() = 0;

  virtual void interpretarComando(char c);
  virtual void imprimirHelp();
  ConsoleUI* mirror = nullptr;  // UI secundaria para eco

};
