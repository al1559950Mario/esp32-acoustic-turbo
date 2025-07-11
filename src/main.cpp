// main.cpp
#include <Arduino.h>
#include <BluetoothSerial.h>
#include "StateMachine.h"
#include "ConsoleUI.h"
#include "BLEUI.h"
#include "CalibrationManager.h"
#include "DebugManager.h"
#include "ActuatorManager.h"
#include "SensorManager.h"

BluetoothSerial SerialBT;

// PIN-OUT
constexpr uint8_t PIN_MAP             = 35;
constexpr uint8_t PIN_TPS             = 34;
constexpr uint8_t PIN_RELAY_TURBO     =  2;
constexpr uint8_t PIN_RELAY_ACOUSTIC  =  4;
constexpr uint8_t PIN_DAC_ACOUSTIC    = 25;

// Objetos globales
StateMachine       fsm;
SensorManager          sensors;
ActuatorManager    actuators;
ConsoleUI          consoleUI;
BLEUI              bleUI;
CalibrationManager& calib = CalibrationManager::getInstance();
DebugManager       debugMgr;

void setup() {
  Serial.begin(115200); delay(100);
  Serial.println(">> Iniciando sistema turbo-acústico");

  sensors.begin(PIN_MAP, PIN_TPS);

  actuators.begin(PIN_RELAY_TURBO, PIN_DAC_ACOUSTIC, PIN_RELAY_ACOUSTIC);

  // Consola + FSM + BLE
  consoleUI.begin();
  consoleUI.setFSM(&fsm);
  consoleUI.attachSensors(&sensors);
  consoleUI.attachActuators(&actuators);
  consoleUI.imprimirDashboard(); // ← ya con referencias conectadas


  SerialBT.begin("TurboAcoustic");
  bleUI.begin();
  bleUI.setFSM(&fsm);

  calib.begin();
  actuators.stopTurbo();
  actuators.stopAcoustic();

  bool hasCalib = calib.loadCalibration();
  fsm.begin(hasCalib, &actuators);

  if (!hasCalib)
    Serial.println("  Estado inicial: SIN_CALIBRAR (necesita calibración)");
  else
    Serial.println("  Estado inicial: OFF (calibración cargada)");
}

void loop() {
  consoleUI.update();
  bleUI.update();
  debugMgr.updateFromSerial(Serial);

  if (consoleUI.isSistemaActivo()) {
    float mapVacuum   = sensors.readVacuum_inHg();
    float tpsPorcent   = sensors.readTPSPercent();

    fsm.update(
      mapVacuum,
      tpsPorcent,
      consoleUI.getCalibRequest(),
      bleUI.getCalibRequest(),
      debugMgr
    );
    fsm.handleActions();

  } else {
    // Opcional: Apagar subsistemas activos como seguridad
    actuators.stopAll();
  }

  delay(20);
}