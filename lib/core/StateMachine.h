#pragma once

#include "ActuatorManager.h"
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
  DEBUG,
  UNKNOWN                  ///< Estado de debug (solo con forzar)
};

/**
 * @class StateMachine
 * @brief Gestiona las transiciones y acciones de los estados del sistema.
 *
 * Usa lecturas de vacío MAP (inHg) y pedal (% TPS), además de peticiones de
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
  void begin(bool hasCalibration, ActuatorManager* actuators);

  /**
   * Obtiene el estado actual.
   * @return Estado activo de la FSM.
   */
  SystemState getState() const;

  /**
   * Realiza la lógica de transición de estados.
   * @param vacuumInHg Lectura de vacío MAP en inHg (valor negativo o 0).
   * @param tpsPct Lectura de TPS en porcentaje [0–100].
   * @param consoleCalibReq true si hubo petición de calibración por consola.
   * @param bleCalibReq true si hubo petición de calibración por BLE.
   * @param dbg Objeto DebugManager que puede forzar estado DEBUG.
   */
  void update(float vacuumInHg,
              float tpsPct,
              bool consoleCalibReq,
              bool bleCalibReq,
              const DebugManager& dbg);

  /**
   * Ejecuta las acciones de salida según el estado actual.
   * @param acousticLevel Nivel de inyección acústica normalizado [0–1].
   */
  void handleActions();

  /**
   * Si el estado actual es DEBUG, lo reemplaza por uno nuevo.
   * @param nuevoEstado Estado al que forzar la FSM.
   */
  void debugForceState(SystemState nuevoEstado);
  float getLevel() const;



private:
  SystemState        current{SystemState::OFF};   ///< Estado actual
  ActuatorManager* actuators = nullptr;

  
  float currentLevel{0.0f};  ///< Nivel actual de inyección acústica calculado internamente

  bool readyForInjection(float tps, float vac) {
    return tps >= INJ_TPS_ON && vac <= INJ_VAC_ON;
  }


  // Umbrales de transición en unidades reales (ajusta según aplicación/calibración)
  static constexpr float MAP_WAKEUP_INHG   = -1.0f;   ///< inHg mínimos para pasar OFF→IDLE (casi sin vacío)
  static constexpr float INJ_TPS_ON        = 10.0f;   ///< % TPS para iniciar inyección acústica
  static constexpr float INJ_VAC_ON        = -12.0f;  ///< inHg para iniciar inyección acústica (vacío moderado)
  static constexpr float INJ_TPS_OFF       = 8.0f;    ///< % TPS para detener inyección acústica
  static constexpr float INJ_VAC_OFF       = -9.0f;   ///< inHg para detener inyección acústica
  static constexpr float TURBO_TPS_ON      = 40.0f;   ///< % TPS para arrancar turbo
  static constexpr float TURBO_VAC_ON      = -4.0f;   ///< inHg para arrancar turbo (presión alta, casi sin vacío)
  static constexpr float TURBO_TPS_OFF     = 30.0f;   ///< % TPS para apagar turbo
};
