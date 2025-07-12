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
  Serial.begin(115200); 
  delay(100);
  Serial.println(">> Iniciando sistema turbo-acústico");

  // Inicializar sensores y actuadores
  sensors.begin(PIN_MAP, PIN_TPS);
  actuators.begin(PIN_RELAY_TURBO, PIN_DAC_ACOUSTIC, PIN_RELAY_ACOUSTIC);

  // Iniciar UI Serial USB
  serialUI.begin();
  serialUI.setFSM(&fsm);
  serialUI.attachSensors(&sensors);
  serialUI.attachActuators(&actuators);
  serialUI.imprimirDashboard();

  // Iniciar UI Bluetooth Serial clásico
  bleUI.begin();
  bleUI.setFSM(&fsm);
  bleUI.attachSensors(&sensors);
  bleUI.attachActuators(&actuators);
  bleUI.imprimirDashboard();

  // Inicializar calibración y FSM
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

  bool sistemaActivo = serialUI.isSistemaActivo() || bleUI.isSistemaActivo();

  if (!serialUI.isDeveloperMode()) {
    calib.loadDebugCalibration();  // Fuerza valores debug cada ciclo (opcional, o solo la primera vez)
  } else {
    static bool calibLoaded = false;
    if (!calibLoaded) {
      calibLoaded = calib.loadCalibration();
      if (!calibLoaded) {
        Serial.println(">> ATENCIÓN: calibración no cargada, requiere calibrar.");
      }
    }
  }

  if (sistemaActivo) {
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
    actuators.stopAll();
  }

  delay(20);
}

