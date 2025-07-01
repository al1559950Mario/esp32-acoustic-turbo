// src/main.cpp

#include <Arduino.h>
#include "StateMachine.h"
#include "sensors/MAPSensor.h"
#include "sensors/TPSSensor.h"
#include "controllers/TurboController.h"
#include "controllers/AcousticInjector.h"
#include "ui/ConsoleUI.h"
#include "ui/BLEUI.h"
#include "utils/CalibrationManager.h"
#include "utils/DebugManager.h"

// — PIN-OUT —
constexpr uint8_t PIN_MAP             = 34;
constexpr uint8_t PIN_TPS             = 35;
constexpr uint8_t PIN_RELAY_TURBO     = 2;
constexpr uint8_t PIN_DAC_ACOUSTIC    = 25;
constexpr uint8_t PIN_RELAY_ACOUSTIC  = 26;

// — UMBRALES —
constexpr float MAP_WAKEUP_KPA        = 0.5f;

constexpr float INJ_TPS_ON_PERCENT    = 30.0f;
constexpr float INJ_VACUUM_ON_KPA     = 50.5f;
constexpr float INJ_TPS_OFF_PERCENT   = 25.0f;
constexpr float INJ_VACUUM_OFF_KPA    = 44.0f;

constexpr float TURBO_TPS_ON_PERCENT  = 70.0f;
constexpr float TURBO_VACUUM_ON_KPA   = 16.9f;
constexpr float TURBO_TPS_OFF_PERCENT = 30.0f;

StateMachine       fsm;
MAPSensor          mapSensor;
TPSSensor          tpsSensor;
TurboController    turbo;
AcousticInjector   injector;
ConsoleUI          consoleUI;
BLEUI              bleUI;
CalibrationManager calib;
DebugManager       debugMgr;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(">> Iniciando sistema turbo-acústico");

  // Init hardware
  mapSensor.begin(PIN_MAP);
  tpsSensor.begin(PIN_TPS);
  turbo.begin(PIN_RELAY_TURBO);
  injector.begin(PIN_DAC_ACOUSTIC, PIN_RELAY_ACOUSTIC);
  consoleUI.begin(115200);
  bleUI.begin();
  debugMgr.begin();
  calib.begin();

  // Asegurar relés OFF
  turbo.stop();
  injector.stop();

  // Cargar calibración
  bool ok = calib.loadCalibration();
  if (!ok) {
    fsm.begin(SystemState::SIN_CALIBRAR);
    Serial.println("  Estado inicial: SIN_CALIBRAR");
  } else {
    fsm.begin(SystemState::OFF);
    Serial.println("  Estado inicial: OFF");
  }
}

void loop() {
  // 1) Lectura de sensores
  float mapKPa = mapSensor.readkPa();
  float tpsPct = tpsSensor.readPct();

  // 2) Actualizar FSM
  fsm.update(mapKPa, tpsPct);

  // 3) Interfaces
  consoleUI.poll();
  bleUI.poll();
  debugMgr.poll();

  // 4) Lógica por estado
  switch (fsm.getState()) {

    case SystemState::OFF:
      // Espera primera lectura válida de MAP
      if (mapKPa > MAP_WAKEUP_KPA) {
        fsm.setState(SystemState::IDLE);
        Serial.println("→ Transition OFF → IDLE");
      }
      break;

    case SystemState::SIN_CALIBRAR:
      // Solo sale a CALIBRATION si el usuario lo solicita
      if (consoleUI.calibrationRequested() || bleUI.calibrationRequested()) {
        fsm.setState(SystemState::CALIBRATION);
        Serial.println("→ Transition SIN_CALIBRAR → CALIBRATION");
      }
      break;

    case SystemState::CALIBRATION:
      // Corre rutina de calibración
      calib.runMAPCalibration(mapSensor);
      calib.runTPScalibration(tpsSensor);
      calib.saveCalibration();
      Serial.println(">> Calibración completada");
      fsm.setState(SystemState::OFF);
      Serial.println("→ Transition CALIBRATION → OFF");
      break;

    case SystemState::IDLE:
      // Checa umbrales para pasar a inyección acústica
      if (tpsPct >= INJ_TPS_ON_PERCENT && mapKPa >= INJ_VACUUM_ON_KPA) {
        fsm.setState(SystemState::INYECCION_ACUSTICA);
        Serial.println("→ Transition IDLE → INYECCION_ACUSTICA");
      }
      break;

    case SystemState::INYECCION_ACUSTICA:
      // Mantener inyector activo
      injector.start(debugMgr.getLevel());
      // Checa umbrales para turbo
      if (tpsPct >= TURBO_TPS_ON_PERCENT && mapKPa <= TURBO_VACUUM_ON_KPA) {
        fsm.setState(SystemState::TURBO);
        Serial.println("→ Transition INYECCION_ACUSTICA → TURBO");
        injector.stop();
      }
      // O volver a IDLE si condiciones fuera de rango
      else if (tpsPct <= INJ_TPS_OFF_PERCENT || mapKPa < INJ_VACUUM_OFF_KPA) {
        fsm.setState(SystemState::IDLE);
        Serial.println("→ Transition INYECCION_ACUSTICA → IDLE");
        injector.stop();
      }
      break;

    case SystemState::TURBO:
      // Mantener turbo activo
      turbo.update(tpsPct, mapKPa);
      // Si TPS cae por debajo del 30%, desacelera
      if (tpsPct < TURBO_TPS_OFF_PERCENT) {
        fsm.setState(SystemState::DESCAYENDO);
        Serial.println("→ Transition TURBO → DESCAYENDO");
      }
      break;

    case SystemState::DESCAYENDO:
      // 1) ¿Sigue decayendo? (TPS por debajo de 30 %)
      if (tpsPct < TURBO_TPS_OFF_PERCENT) {
        // Mantenemos DESCAYENDO hasta que TPS ≥ 30%
        // turbo e injector ya están apagados
        break;
      }

      // 2) Si TPS recuperó suficiente y MAP está en vacío para inyector
      if (tpsPct >= INJ_TPS_ON_PERCENT && mapKPa >= INJ_VACUUM_ON_KPA) {
        fsm.setState(SystemState::INYECCION_ACUSTICA);
        Serial.println("→ Transition DESCAYENDO → INYECCION_ACUSTICA");
      }
      // 3) Si terminó decay pero no cumple inyección, volvemos a IDLE
      else {
        fsm.setState(SystemState::IDLE);
        Serial.println("→ Transition DESCAYENDO → IDLE");
      }
      break;


    case SystemState::DEBUG:
      // Overrides manuales sin cambiar estados automáticos
      if (debugMgr.turboOverride()) {
        turbo.update(tpsPct, mapKPa);
      }
      if (debugMgr.acousticOverride()) {
        injector.start(debugMgr.getLevel());
      }
      break;
  }

  delay(20);  // loop ~50 Hz
}
