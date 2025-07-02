/**
 * main.cpp
 * Punto de entrada y orquestador de todo el sistema turbo-acústico.
 * 
 * - Inicializa hardware y módulos (sensores, relés, UI, calibración, debug)
 * - En cada ciclo:
 *    1. Lee MAP y TPS
 *    2. Actualiza la máquina de estados
 *    3. Atiende interfaces (consola, BLE, debug)
 *    4. Ejecuta acciones según el estado actual
 */

#include <Arduino.h>
#include "StateMachine.h"
#include "MAPSensor.h"
#include "TPSSensor.h"
#include "TurboController.h"
#include "AcousticInjector.h"
#include "ConsoleUI.h"
#include "BLEUI.h"
#include "CalibrationManager.h"
#include "DebugManager.h"

// — PIN-OUT —  
constexpr uint8_t PIN_MAP             = 34;  // Entrada analógica para sensor MAP  
constexpr uint8_t PIN_TPS             = 35;  // Entrada analógica para sensor TPS  
constexpr uint8_t PIN_RELAY_TURBO     = 2;   // Salida digital para relé del turbo  
constexpr uint8_t PIN_DAC_ACOUSTIC    = 25;  // PIN DAC para señal al tweeter  
constexpr uint8_t PIN_RELAY_ACOUSTIC  = 26;  // Salida digital para relé del inyector acústico  

// — Objetos globales —  
StateMachine       fsm;            // Máquina de estados central  
MAPSensor          mapSensor;      // Lecturas de presión MAP  
TPSSensor          tpsSensor;      // Lecturas de posición TPS  
TurboController    turbo;          // Control del relé del turbo  
AcousticInjector   injector;       // Generación de señal + relé acústico  
ConsoleUI          consoleUI;      // Interfaz serial de usuario  
BLEUI              bleUI;          // Interfaz Bluetooth LE de usuario  
CalibrationManager calib;          // Gestión y persistencia de calibración  
DebugManager       debugMgr;       // Modo debug y overrides  

/**
 * setup()
 * - Configura el puerto serie
 * - Inicializa todos los módulos y pines
 * - Asegura relés desactivados al inicio
 * - Carga datos de calibración desde NVS
 * - Define el estado inicial de la FSM (OFF o SIN_CALIBRAR)
 */
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(">> Iniciando sistema turbo-acústico");

  // Inicializar sensores y salidas
  mapSensor.begin(PIN_MAP);
  tpsSensor.begin(PIN_TPS);
  turbo.begin(PIN_RELAY_TURBO);
  injector.begin(PIN_DAC_ACOUSTIC, PIN_RELAY_ACOUSTIC);
  consoleUI.begin(115200);
  bleUI.begin();
  debugMgr.begin();
  calib.begin();

  // Asegurar relés inicializados en OFF
  turbo.stop();
  injector.stop();

  // Cargar calibración y determinar estado inicial
  bool hasCalib = calib.loadCalibration();
  fsm.begin(hasCalib);

  if (!hasCalib) {
    Serial.println("  Estado inicial: SIN_CALIBRAR (necesita calibración)");
  } else {
    Serial.println("  Estado inicial: OFF (calibración cargada)");
  }
}

/**
 * loop()
 * Bucle principal:
 * 1. Leer sensores
 * 2. Actualizar FSM
 * 3. Atender UI y debug
 * 4. Ejecutar salidas según estado
 * 5. Esperar 20 ms (≈50 Hz)
 */
void loop() {
  // 1) Lectura de sensores
  float mapKPa = mapSensor.readkPa();
  float tpsPct = tpsSensor.readPct();

  // 2) Actualizar máquina de estados
  fsm.update(
    mapKPa,
    tpsPct,
    consoleUI.calibrationRequested(),
    bleUI.calibrationRequested(),
    debugMgr
  );

  // 3) Atención de interfaces
  consoleUI.poll();
  bleUI.poll();
  debugMgr.poll();

  // 4) Acciones por estado
  switch (fsm.getState()) {

    case SystemState::OFF:
      // Nada que activar: espera lecturas
      break;

    case SystemState::SIN_CALIBRAR:
      // Pendiente de comando de calibración
      break;

    case SystemState::CALIBRATION:
      // Calibración ejecutada internamente en la FSM
      break;

    case SystemState::IDLE:
      // Todo inactivo, vigilando umbrales
      break;

    case SystemState::INYECCION_ACUSTICA:
      // Activa inyector acústico con nivel de debug
      injector.start(debugMgr.getLevel());
      break;

    case SystemState::TURBO:
      // Control dinámico del turbo según TPS y MAP
      turbo.update(tpsPct, mapKPa);
      break;

    case SystemState::DESCAYENDO:
      // Ambos actuadores ya detenidos en la transición
      break;

    case SystemState::DEBUG:
      // Overrides manuales para pruebas
      if (debugMgr.turboOverride())      
        turbo.update(tpsPct, mapKPa);
      if (debugMgr.acousticOverride())   
        injector.start(debugMgr.getLevel());
      break;
  }

  // 5) Frecuencia de ciclo
  delay(20);
}
