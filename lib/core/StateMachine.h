#pragma once

#include "ActuatorManager.h"
#include "DebugManager.h"
#include "ThresholdManager.h"


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
  void begin(bool hasCalibration, ActuatorManager* actuators, ThresholdManager* thresholdManagerPtr);

  /**
   * Obtiene el estado actual.
   * @return Estado activo de la FSM.
   */
  SystemState getState() const;

  /**
   * Realiza la lógica de transición de estados.
   * @param mapLoadPercent Porcentaje de carga MAP normalizado (0% = vacío máximo, 100% = presión atmosférica)
   * @param tpsPct Lectura de TPS en porcentaje [0–100].
   * @param consoleCalibReq true si hubo petición de calibración por consola.
   * @param bleCalibReq true si hubo petición de calibración por BLE.
   * @param dbg Objeto DebugManager que puede forzar estado DEBUG.
   */
  void update(float mapLoadPercent,
              float tpsPct,
              bool serialCalibReq,
              bool bleCalibReq,
              bool hasCalibration,
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
  bool readyForInjection(float, float);


private:
  Thresholds thresholds;                         ///< Copia local de los umbrales actuales
  ThresholdManager* thresholdManager = nullptr;  ///< Puntero al gestor de umbrales
  SystemState        current{SystemState::OFF};   ///< Estado actual
  ActuatorManager* actuators = nullptr;


  
  float currentLevel{0.0f};  ///< Nivel actual de inyección acústica calculado internamente


};
