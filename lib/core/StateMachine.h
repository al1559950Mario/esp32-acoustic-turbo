#pragma once

#include "ActuatorManager.h"
#include "DebugManager.h"
#include "ThresholdManager.h"
#include "CalibrationManager.h"
#include "SensorManager.h"


/**
 * @enum SystemState
 * Define los distintos estados del sistema turbo-acústico.
 */
enum class SystemState {
  OFF,                   ///< Sistema apagado/standby
  NO_CALIB,          ///< No se ha realizado calibración
  CALIBRATION,           ///< Modo calibración activa
  IDLE,                  ///< Esperando subida de carga
  BOOST,    ///< Inyección acústica activa
  BEAM,                 ///< Turbo encendido
  DECAY,            ///< Turbo descendiendo
  DEBUG,
  UNKNOWN                  ///< Estado de debug (solo con forzar)
};

/**
 * @class StateMachine
 * @brief Gestiona las transiciones y acciones de los estados del sistema.
 *
 * Usa lecturas de vacío MAP (inHg) y flujo (% MAF), además de peticiones de
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
  void begin(bool hasCalibration, ActuatorManager* actuators, ThresholdManager* thresholdManagerPtr, SensorManager* sensorsPtr, CalibrationManager* calibMgrPtr);

  /**
   * Realiza la lógica de transición de estados.
   * @param mapLoadPercent Porcentaje de carga MAP normalizado (0% = vacío máximo, 100% = presión atmosférica)
   * @param mafPct Lectura del sensor MAF en porcentaje [0-100].
   * @param consoleCalibReq true si hubo petición de calibración por consola.
   * @param bleCalibReq true si hubo petición de calibración por BLE.
   * @param calibLoaded true si la calibracion fue exitosa y hay valores validos para el sistema.
   * @param dbg Objeto DebugManager que puede forzar estado DEBUG.
   */
  void update(float mapLoadPercent,
              float mafPct,
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
   * Si el estado actual es DEBUG, lo reemplaza por uno nuevo.
   * @param nuevoEstado Estado al que forzar la FSM.
   */
  void debugForceState(SystemState nuevoEstado);
  float getLevel() const;
  bool readyForBOOST( float, float);
  bool readyForBEAM( float, float);

  float getMAFInitialForInj(){return mafInitialPercent;};
  float getMAPInitialForInj(){return mapInitialPercent;};

  CalibStep currentCalibStep = CalibStep::TPS_MIN;
  unsigned long lastStepTime = 0;
  SystemState getState() const {return current;}
  String getStateName () const;
  void compute_dMAFdt_and_hold(float, float, float currentDeltaMAFLevel);

private:
  Thresholds thresholds;                         ///< Copia local de los umbrales actuales
  ThresholdManager* thresholdManager = nullptr;  ///< Puntero al gestor de umbrales
  SystemState        current{SystemState::OFF};   ///< Estado actual
  SystemState lastState;
  ActuatorManager* actuators = nullptr;
  CalibrationManager* calibMgr= nullptr;
  SensorManager* sensors = nullptr;
  
  float              lastMapLoadPercent = 0.0f; ///< Guardar el último mapLoadPercent
  float mafInitialPercent = 0.0f;
  float mapInitialPercent = 0.0f;
  float _mafLoadPercent = 0.0f;
  float _mapLoadPercent = 0.0f;
  float _pressurePercent = 0.0f;
  float _pressureDelta = 0.0f;
  unsigned long vortexStartMillis = 0;
  const unsigned long vortexDelayMs = 200;  // Tiempo en ms para esperar antes de activar vortex
  bool vortexPending = false;  
  float mafNormalized{0.0f}; 
  float mapNormalized{0.0f};
  float _lastPressurePercent = 0.0f;
  float _beamVortexLevel = 0.0f;
  uint32_t _beamVortexLastUpdateMs = 0;
  float lastDeltaMAFLevelForBOOST;   // <- último MAF%
  float lastDeltaMAPLevelForBOOST;   // <- último MAP%
  float lastDeltaMAFLevelForBEAM;   // <- último MAF%
  float lastDeltaMAPLevelForBEAM;   // <- último MAP%
  float currentDeltaMAFLevelForBOOST;
  float currentDeltaMAPLevelForBOOST;
  float currentDeltaMAFLevelForBEAM;
  float currentDeltaMAPLevelForBEAM;
  const float MAP_DROP_THRESHOLD = 0.2f;  
  const float MAF_DROP_THRESHOLD = 0.2f;
  float avgMAPLevel = 0.0f;
  uint32_t mapSamples = 0;
  float avgMAFLevel = 0.0f;
  uint32_t mafSamples = 0;
  float mapDrop = 0.0f;
  float mafDrop = 0.0f;
  bool dropDetected = false;
  bool belowThresholds = false;
  float mafMinOnBeam = 100.0f;
  uint32_t decayStartMillis     = 0;     // instante en que se disparó DECAY (ms)
  float decayDurationMs = 1500;          // **Base nominal** para la duración del DECAY en ms. Se escala con w/hold.
  

  // ---------- derivada dMAF/dt (medición y filtrado) ----------
  float _dMAFdtRaw = 0.0f;               // derivada instantánea (nivel por segundo, niveles 0..1)
  float _dMAFdtEMA = 0.0f;               // derivada filtrada (usar sin filtrar para respuesta rápida)
  bool _fastAttackActive = false;        // bandera si hay ataque positivo reciente

  // ---------- detección de hold ----------
  unsigned long _holdStartMillis = 0;    // cuando empezó la presión
  bool _holdActive = false;              // indica que hubo presión antes de soltar

  // ---------- mezcla y tiempos de envelope ----------
  float _gFast = 0.0f;                   // peso componente rápido del envelope (0..1)
  float _gSustain = 0.0f;                // peso componente sustain/cola (0..1)
  float _tFastMs = 150.0f;                // tiempo característico del componente rápido (ms). Valores: 10..200
  float _tSustainMs = 250.0f;            // tiempo característico del sustain/cola (ms). Valores: 200..5000

  // DERIV_NORM
// Qué controla: normalizador que mapea magnitud de derivada a gFast (0..1).
// Rango recomendado: 0.5 .. 5.0
// Ajuste: bajar para que pequeñas caídas activen gFast; subir si hay falsos positivos.
const float DERIV_NORM = 0.5f;

// DERIV_DROP_THRESHOLD
// Qué controla: umbral en unidades nivel/sec para considerar una caída rápida y disparar DECAY.
// Rango recomendado: 0.5 .. 5.0 (nivel/sec)
// Ajuste: subir para evitar triggers por jitter; bajar si falta sensibilidad a releases rápidos.
const float DERIV_DROP_THRESHOLD = 5.0f;

// H_SCALE_MS
// Qué controla: escala temporal para normalizar holdMs → holdNorm (0..1).
// Rango recomendado: 200 .. 2000 ms
// Ajuste: reducir para clasificar como hold más rápido; aumentar para requerir presiones más largas.
const unsigned long H_SCALE_MS = 200;

// SIGMOID_H0
// Qué controla: centro de la sigmoide aplicada a holdNorm que define transición tap→hold.
// Rango recomendado: 0.0 .. 0.7
// Ajuste: bajar para que hold se active con menos tiempo; subir para requerir más tiempo.
const float SIGMOID_H0 = 0.3f;

// SIGMOID_K
// Qué controla: pendiente de la sigmoide que suaviza o hace abrupta la transición tap→hold.
// Rango recomendado: 1.0 .. 12.0
// Ajuste: aumentar para cambio más brusco; disminuir para transición más gradual.
const float SIGMOID_K = 12.0f;

// MIN_SCALE
// Qué controla: factor mínimo aplicado sobre decayDurationMs para toques rápidos.
// Rango recomendado: 0.2 .. 1.0
// Ajuste: bajar para colas muy cortas en taps; no bajar demasiado o el sonido se cortará.
const float MIN_SCALE = 0.2f;

// MAX_SCALE
// Qué controla: factor máximo aplicado sobre decayDurationMs para holds largos.
// Rango recomendado: 1.0 .. 4.0
// Ajuste: aumentar para colas más largas en holds; limitar si las colas se vuelven irreales.
const float MAX_SCALE = 0.4f;

// SOFTCLIP_BETA
// Qué controla: agresividad del soft clip en la forma del envelope y control de picos.
// Rango recomendado: 0.1 .. 1.0
// Ajuste: bajar para clipping más suave; subir para limitar picos con más fuerza.
const float SOFTCLIP_BETA = 0.3f;

// POW_ALPHA
// Qué controla: exponente de la ley de potencia que modela la cola lenta (shape de sustain).
// Rango recomendado: 1.0 .. 2.5
// Ajuste: incrementar para caída más pronunciada; reducir para cola más larga y suave.
const float POW_ALPHA = 4.5f;

// RES_ZERO_THRESH16
// Qué controla: umbral 16-bit para considerar la envolvente efectivamente cero y terminar DECAY.
// Rango recomendado: 0 .. 1024 (0..65535 escala)
// Ajuste: aumentar si el ruido impide llegar a cero; bajar para terminar con niveles más bajos.
const uint16_t RES_ZERO_THRESH16 = 8;

// MIN_TAIL_MS
// Qué controla: tiempo mínimo de tail que siempre se permite antes de aceptar término del DECAY.
// Rango recomendado: 10 .. 200 ms
// Ajuste: aumentar si quieres asegurar un mínimo de decay audible; reducir para finales más cortos.
const float MIN_TAIL_MS = 10.0f;

// ZERO_COUNT_TO_END
// Qué controla: número de frames consecutivos por debajo del umbral requerido para confirmar fin.
// Rango recomendado: 1 .. 16
// Ajuste: aumentar para ser más conservador al terminar; disminuir para terminar más rápido.
  const uint8_t ZERO_COUNT_TO_END = 1;

  // trackear inicio de "hold" (cuando se detecta que hubo presión)
  const float PRESS_EPS = 0.01f;

  // --- Dinámica de MAF (antes TPS) para clasificar ataques ---
  static constexpr float MAF_ATTACK_SLOW_DTPS = 0.02f;   // nivel/sec considerado subida lenta
  static constexpr float MAF_ATTACK_FAST_DTPS = 0.15f;   // nivel/sec considerado ataque rápido
  static constexpr float MAF_ATTACK_MIN_GAIN  = 0.75f;   // factor aplicado al power cuando el ataque es muy lento
  static constexpr float MAF_ATTACK_MAX_GAIN  = 1.25f;   // factor cuando el ataque es muy rápido
  static constexpr float MAF_RELEASE_REF_DTPS = -0.12f;  // referencia (nivel/sec) para detectar soltado rápido
//Bájalo por debajo de 0.10f si quieres que BEAM se dispare con ataques más suaves
//Súbelo si deseas exigir pisadas más bruscas antes de considerar el ataque “válido”.
  static constexpr float BEAM_MAF_ATTACK_MIN_DERIV   = 0.04f;  // nivel/sec mínimo para considerar un ataque rápido
  static constexpr float BEAM_VACUUM_PCT_ON    = -3.0f;  // vacio mínima (en %) para permitir BEAM
  static constexpr float BEAM_VACUUM_PCT_OFF    = -1.0f;  // vacio max (en %) para romper BEAM
  static constexpr float BEAM_VACUUM_DELTA_MIN = 1.0f;   // delta mínimo de vacio (%)
static constexpr uint32_t MIN_BEAM_DELAY_MS = 50u; // m segundos minimos antes de permitir BEAM
static constexpr uint32_t BEAM_MIN_STREAM_MS = 50u; // tiempo minimo en BEAM antes de pasar a DECAY

  // BEAM_COND_MIN_HOLD_MS
  // Qué controla: tiempo mínimo que la condición de entrada a BEAM
  // (readyForBEAM) debe mantenerse verdadera de forma continua antes
  // de permitir la transición. Ayuda a filtrar falsos positivos.
  // Rango recomendado: 100 .. 500 ms
  // Ajuste: aumentar si aún hay falsos positivos; reducir para respuesta más rápida.
  static constexpr uint32_t BEAM_COND_MIN_HOLD_MS = 150u; // Tiempo necesario activando condicionales para entrar a BEAM
 
  static constexpr float BEAM_VORTEX_ENTRY_LEVEL      = 0.85f;
  static constexpr float BEAM_VORTEX_RAMP_TAU_FAST_MS = 60.0f;
  static constexpr float BEAM_VORTEX_RAMP_TAU_SLOW_MS = 260.0f;

  // Seguimiento de la condición BEAM sostenida
  uint32_t _beamCondStartMs = 0;   // instante en que se detecto la condicion por primera vez
  uint32_t _beamStreamStartMs = 0;


  void resetBeamVortexRamp(float seedLevel);
  float updateBeamVortexRamp(float mafPower);

};



