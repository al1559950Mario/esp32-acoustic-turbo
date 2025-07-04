// main.cpp
#include <Arduino.h>
#include <BluetoothSerial.h>
#include "StateMachine.h"
#include "MAPSensor.h"
#include "TPSSensor.h"
#include "TurboController.h"
#include "AcousticInjector.h"
#include "ConsoleUI.h"
#include "BLEUI.h"
#include "CalibrationManager.h"
#include "DebugManager.h"

BluetoothSerial SerialBT;

// PIN-OUT
constexpr uint8_t PIN_MAP             = 35;
constexpr uint8_t PIN_TPS             = 34;
constexpr uint8_t PIN_RELAY_TURBO     =  2;
constexpr uint8_t PIN_DAC_ACOUSTIC    = 25;
constexpr uint8_t PIN_RELAY_ACOUSTIC  = 26;

// Objetos globales
StateMachine       fsm;
MAPSensor          mapSensor;
TPSSensor          tpsSensor;
TurboController    turbo;
AcousticInjector   injector;
ConsoleUI          consoleUI;
BLEUI              bleUI;
CalibrationManager& calib = CalibrationManager::getInstance();
DebugManager       debugMgr;

void setup() {
  Serial.begin(115200); delay(100);
  Serial.println(">> Iniciando sistema turbo-acústico");

  mapSensor.begin(PIN_MAP);
  tpsSensor.begin(PIN_TPS);
  turbo.begin(PIN_RELAY_TURBO);
  injector.begin(PIN_DAC_ACOUSTIC, PIN_RELAY_ACOUSTIC);

  // Consola + FSM + BLE
  consoleUI.begin();
  consoleUI.setFSM(&fsm);
  consoleUI.attachSensors(&mapSensor, &tpsSensor);
  consoleUI.attachActuators(&turbo, &injector);
  consoleUI.imprimirDashboard(); // ← ya con referencias conectadas


  SerialBT.begin("TurboAcoustic");
  bleUI.begin(&SerialBT);
  bleUI.setFSM(&fsm);

  calib.begin();
  turbo.stop();
  injector.stop();

  bool hasCalib = calib.loadCalibration();
  fsm.begin(hasCalib, &turbo, &injector);

  if (!hasCalib)
    Serial.println("  Estado inicial: SIN_CALIBRAR (necesita calibración)");
  else
    Serial.println("  Estado inicial: OFF (calibración cargada)");
}

void loop() {
  consoleUI.update();
  bleUI.update(fsm.getState());
  debugMgr.updateFromSerial(Serial);

  float mapKPa   = mapSensor.readkPa();
  float tpsPct   = tpsSensor.readPct();
  float tpsNorm  = tpsSensor.readNormalized();
  float level    = constrain((tpsNorm - 0.30f) / 0.40f, 0.0f, 1.0f);

  fsm.update(
    mapKPa,
    tpsPct,
    consoleUI.getCalibRequest(),
    bleUI.getCalibRequest(),
    debugMgr
  );

  injector.setLevel(level);         // ← actualiza nivel deseado (ya lo haces)
  injector.update();                // ← aplica rampa si estás usando `update()` para suavizado
  injector.applyPendingDAC();      // ← DAC aplicado fuera del ISR (nuevo paso seguro)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 300) {  // Actualiza cada 100 ms
    consoleUI.imprimirDashboard();
    lastPrint = millis();
  }



  delay(20);
}
