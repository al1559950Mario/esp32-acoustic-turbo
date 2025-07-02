#pragma once

#include <Arduino.h>

/**
 * AcousticInjector
 *
 * Genera una onda senoidal en f₀ ≈ 6370 Hz por el DAC del ESP32 y
 * controla un relé que alimenta el amplificador/acoplador acústico.
 * 
 * Incluye:
 *  - Arranque “mudo” (level empieza en 0 antes de cerrar el relé)
 *  - Rampado suave hasta el nivel deseado para evitar clics
 *  - Ajuste de nivel en tiempo real sin reiniciar la ISR ni el relé
 */
class AcousticInjector {
public:
  AcousticInjector() = default;

  /**
   * Inicializa DAC y relé, y prepara timer de alta precisión.
   * 
   * @param dacPin   Pin GPIO para salida DAC (25–26 en ESP32)
   * @param relayPin Pin digital que acciona el relé (HIGH = ON)
   */
  void begin(uint8_t dacPin, uint8_t relayPin);

  /**
   * Enciende el relé y arranca la señal:
   *  - Level inicia en 0
   *  - targetLevel se fija a level
   *  - ISR arranca pero genera salida muda hasta rampar
   *
   * @param level Nivel final deseado [0.0 … 1.0]
   */
  void start(float level);

  /**
   * Detiene y apaga la señal y el relé:
   *  - Deshabilita timer/ISR
   *  - Lleva DAC a valor medio (silencio)
   *  - Apaga relé
   */
  void stop();

  /**
   * Ajusta el nivel objetivo de amplitud sin detener la señal.
   * Se rampará progresivamente en update().
   *
   * @param level Nuevo nivel objetivo [0.0 … 1.0]
   */
  void setLevel(float level);

  /**
   * Debe llamarse periódicamente (p.ej. cada loop):
   *  - Realiza el rampado suave desde level → targetLevel
   */
  void update();

private:
  // Frecuencia y tabla de seno
  static constexpr uint16_t SAMPLE_RATE = 64 * 6370; 
  static constexpr uint8_t  TABLE_SIZE  = 64;        
  static const uint8_t      _sineTable[TABLE_SIZE];

  // Rampado
  static constexpr float RAMP_STEP = 0.02f; // ∆level por update()
  
  uint8_t    _dacPin      = 25;
  uint8_t    _relayPin    =  0;
  float      _level       =  0.0f; // amplitud actual [0..1]
  float      _targetLevel =  0.0f; // amplitud objetivo
  uint8_t    _index       =  0;    // índice sobre la tabla
  hw_timer_t* _timer      = nullptr;

  // ISR de generación de onda
  static void IRAM_ATTR onTimer();
};
