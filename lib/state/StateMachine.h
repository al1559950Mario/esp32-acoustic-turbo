#pragma once

#include "../utils/DebugManager.h"
#include "TurboController.h"
#include "AcousticInjector.h"


/**
 * SystemState
 * Enumeración de los posibles modos operativos del sistema.
 */
enum class SystemState {
  OFF,             // Sistema apagado, antes de la primera lectura válida
  SIN_CALIBRAR,    // No hay datos de calibración en NVS
  CALIBRATION,     // Ejecutando rutina de calibración
  IDLE,            // Sistema en reposo, vigilando umbrales
  INYECCION_ACUSTICA, // Inyector acústico activo
  TURBO,           // Turbo activo
  DESCAYENDO,      // Turbo e inyector detenidos, esperando fin de decay
  DEBUG            // Modo debug con overrides de actuadores
};

/**
 * StateMachine
 * Controla transiciones entre estados usando MAP, TPS y comandos de UI/debug.
 */
class StateMachine {
public:
  /**
   * begin()
   * @param hasCalibration true si existen datos en NVS → arranca en OFF  
   *                       false → arranca en SIN_CALIBRAR
   */
  void begin(bool hasCalibration,
            TurboController* turboRef,
            AcousticInjector* injectorRef);
            
  void debugForceState(SystemState nuevoEstado);

  /**
   * update()
   * Calcula la siguiente transición en base a:
   *  - Presión MAP
   *  - Porcentaje TPS
   *  - Solicitudes de calibración por consola/BLE
   *  - Estado de debug y overrides
   */
  void update(float mapKPa,
              float tpsPct,
              bool consoleCalibReq,
              bool bleCalibReq,
              const DebugManager &dbg);

  /**
   * getState()
   * @return Estado actual del sistema
   */
  SystemState getState() const;

private:
  SystemState current;  // Estado interno actual

  TurboController* turbo     = nullptr;
  AcousticInjector* injector = nullptr;


  // Umbrales privados para transiciones
  static constexpr float MAP_WAKEUP_KPA     = 0.5f;   // OFF → IDLE

  static constexpr float INJ_TPS_ON         = 30.0f;  // IDLE → INYECCION_ACUSTICA
  static constexpr float INJ_VAC_ON         = 50.5f;
  static constexpr float INJ_TPS_OFF        = 25.0f;  // INYECCION_ACUSTICA → IDLE
  static constexpr float INJ_VAC_OFF        = 44.0f;

  static constexpr float TURBO_TPS_ON       = 70.0f;  // INYECCION_ACUSTICA → TURBO
  static constexpr float TURBO_VAC_ON       = 16.9f;
  static constexpr float TURBO_TPS_OFF      = 30.0f;  // TURBO → DESCAYENDO
};
