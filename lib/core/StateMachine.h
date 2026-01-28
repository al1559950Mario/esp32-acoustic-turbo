#pragma once

#include "ActuatorManager.h"
#include "DebugManager.h"
#include "ThresholdManager.h"
#include "CalibrationManager.h"


/**
 * @enum SystemState
 * Define los estados del flujo acústico (FSM).
 */
enum class SystemState {
  IDLE,   ///< Todo en cero, arranque limpio
  ALIGN,  ///< Acople: BEAM dominante + BOOST mínimo
  FLOW,   ///< Flujo estable: escalar BEAM/BOOST con MAF
  DECAY   ///< Ring-down controlado
};

enum class FlowVerdict {
  LOST,
  INCONSISTENT,
  LAMINAR,
  UNKNOWN
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
   * @param calibLoaded true si la calibracion fue exitosa y hay valores validos para el sistema.
   * @param dbg Objeto DebugManager que puede forzar estado DEBUG.
   */
  void update(float mapLoadPercent,
              float tpsPct,
              bool serialCalibReq,
              bool bleCalibReq,
              bool calibLoaded,
              const DebugManager& dbg);

  /**
   * Ejecuta las acciones de salida según el estado actual.
   * @param acousticLevel Nivel de inyección acústica normalizado [0–1].
   */
  void handleActions();

  /**
   * Fuerza el estado de la FSM (uso de depuración).
   * @param nuevoEstado Estado al que forzar la FSM.
   */
  void debugForceState(SystemState nuevoEstado);
  void setCompatibilityMode(bool enabled);
  bool isCompatibilityMode() const;
  float getLevel() const;
  bool readyForInjection(float, float);

  CalibStep currentCalibStep = CalibStep::TPS_MIN;
  unsigned long lastStepTime = 0;



private:
  Thresholds thresholds;                         ///< Copia local de los umbrales actuales
  ThresholdManager* thresholdManager = nullptr;  ///< Puntero al gestor de umbrales
  SystemState        current{SystemState::IDLE};   ///< Estado actual
  ActuatorManager* actuators = nullptr;
  CalibrationManager* calibMgr= nullptr;
  float              lastMapLoadPercent = 0.0f; ///< Guardar el último mapLoadPercent
  float              lastTpsPercent = 0.0f;

  float currentLevel{0.0f};  ///< Nivel actual de inyección acústica calculado internamente
  unsigned long stateEntryMs = 0;
  unsigned long flowStableMs = 0;
  bool compatibilityMode = true;


};
