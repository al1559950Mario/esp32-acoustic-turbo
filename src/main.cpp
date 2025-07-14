#include <Arduino.h>
#include "StateMachine.h"
#include "CalibrationManager.h"
#include "DebugManager.h"
#include "ActuatorManager.h"
#include "SensorManager.h"
#include "SerialConsoleUI.h"
#include "BLEConsoleUI.h"

// PIN-OUT
constexpr uint8_t PIN_MAP             = 35;
constexpr uint8_t PIN_TPS             = 34;
constexpr uint8_t PIN_RELAY_TURBO     =  2;
constexpr uint8_t PIN_RELAY_ACOUSTIC  =  4;
constexpr uint8_t PIN_DAC_ACOUSTIC    = 25;

// Objetos globales
StateMachine       fsm;
SensorManager      sensors;
ActuatorManager    actuators;
SerialConsoleUI    serialUI;
BLEConsoleUI       bleUI;
CalibrationManager& calib = CalibrationManager::getInstance();
DebugManager       debugMgr;

void setup() {
  Serial.begin(115200); delay(100);
  Serial.println(">> Iniciando sistema turbo-acústico");

  sensors.begin(PIN_MAP, PIN_TPS);
  actuators.begin(PIN_RELAY_TURBO, PIN_DAC_ACOUSTIC, PIN_RELAY_ACOUSTIC);

  // Iniciar UI Serial
  serialUI.begin();
  serialUI.setFSM(&fsm);
  serialUI.attachSensors(&sensors);
  serialUI.attachActuators(&actuators);
  serialUI.imprimirDashboard();

  // Iniciar UI Bluetooth
  bleUI.begin();
  bleUI.setFSM(&fsm);
  bleUI.attachSensors(&sensors);
  bleUI.attachActuators(&actuators);
  bleUI.imprimirDashboard();

  serialUI.setMirror(&bleUI);
  bleUI.setMirror(&serialUI);



  // Estado inicial
  calib.begin();
  bool hasCalib = calib.loadCalibration();
  fsm.begin(hasCalib, &actuators);

  actuators.stopAll();

  if (!hasCalib)
    Serial.println("  Estado inicial: SIN_CALIBRAR (necesita calibración)");
  else
    Serial.println("  Estado inicial: OFF (calibración cargada)");
}

void loop() {
  serialUI.update();
  bleUI.update();
  debugMgr.updateFromSerial(Serial);

  if (serialUI.isSistemaActivo() || bleUI.isSistemaActivo()) {
    float mapVacuum   = sensors.readVacuum_inHg();
    float tpsPorcent  = sensors.readTPSPercent();

    fsm.update(
      mapVacuum,
      tpsPorcent,
      serialUI.getCalibRequest(),
      bleUI.getCalibRequest(),
      debugMgr
    );
    fsm.handleActions();

  } else {
    actuators.stopAll();  // Seguridad
  }

  delay(20);
}
