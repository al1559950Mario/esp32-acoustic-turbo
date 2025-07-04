#pragma once

#include "TurboController.h"
#include "AcousticInjector.h"
#include "DebugManager.h"

/**
 * @enum SystemState
 * Define los distintos estados del sistema turbo-acústico.
 */
enum class SystemState {
  OFF,                   ///< Sistema apagado/standby
  SIN_CALIBRAR,          ///< No se ha realizado calibración
  CALIBRATION,           ///< Modo calibración activa
  IDLE,                  ///< Esperando subida de carga
  INYECCION_ACUSTICA,    ///< Inyección acústica activa
  TURBO,                 ///< Turbo encendido
  DESCAYENDO,            ///< Turbo descendiendo
  DEBUG                  ///< Estado de debug (solo con forzar)
};

/**
 * @class StateMachine
 * @brief Gestiona las transiciones y acciones de los estados del sistema.
 *
 * Usa lecturas de presión (MAP) y pedal (% TPS), además de peticiones de
 * calibración por consola o BLE, para decidir en qué estado operar.
 */
class StateMachine {
public:

  /**
   * Inicializa la máquina de estados.
   * @param hasCalibration true si ya hay datos de calibración válidos.
   * @param turboRef Puntero al controlador de turbo.
   * @param injectorRef Puntero al inyector acústico.
   */
  void begin(bool hasCalibration,
             TurboController* turboRef,
             AcousticInjector* injectorRef);

  /**
   * Obtiene el estado actual.
   * @return Estado activo de la FSM.
   */
  SystemState getState() const;

  /**
   * Realiza la lógica de transición de estados.
   * @param mapKPa Lectura de presión MAP en kPa.
   * @param tpsPct Lectura de TPS en porcentaje [0–100].
   * @param consoleCalibReq true si hubo petición de calibración por consola.
   * @param bleCalibReq true si hubo petición de calibración por BLE.
   * @param dbg Objeto DebugManager que puede forzar estado DEBUG.
   */
  void update(float mapKPa,
              float tpsPct,
              bool consoleCalibReq,
              bool bleCalibReq,
              const DebugManager& dbg);

  /**
   * Ejecuta las acciones de salida según el estado actual.
   * @param acousticLevel Nivel de inyección acústica normalizado [0–1].
   */
  void handleActions(float acousticLevel);

  /**
   * Si el estado actual es DEBUG, lo reemplaza por uno nuevo.
   * @param nuevoEstado Estado al que forzar la FSM.
   */
  void debugForceState(SystemState nuevoEstado);

private:
  SystemState        current{SystemState::OFF};   ///< Estado actual
  TurboController*   turboPtr{nullptr};           ///< Controlador de turbo
  AcousticInjector*  injectorPtr{nullptr};        ///< Manejador acústico

  // Umbrales de transición (ajusta según tu aplicación/calibración)
  static constexpr float MAP_WAKEUP_KPA   = 1.0f;   ///< kPa mínimos para pasar OFF→IDLE
  static constexpr float INJ_TPS_ON       = 10.0f;  ///< % TPS para iniciar inyección acústica
  static constexpr float INJ_VAC_ON       = 15.0f;  ///< kPa MAP para iniciar inyección acústica
  static constexpr float INJ_TPS_OFF      = 8.0f;   ///< % TPS para detener inyección acústica
  static constexpr float INJ_VAC_OFF      = 12.0f;  ///< kPa MAP para detener inyección acústica
  static constexpr float TURBO_TPS_ON     = 40.0f;  ///< % TPS para arrancar turbo
  static constexpr float TURBO_VAC_ON     = 10.0f;  ///< kPa MAP para arrancar turbo
  static constexpr float TURBO_TPS_OFF    = 30.0f;  ///< % TPS para apagar turbo
};
