#pragma once

#include <Arduino.h>

/**
 * TurboController
 * Controla el relé del turbo, incluye lógica con histéresis y monitoreo de estado.
 */
class TurboController {
public:
  /**
   * begin()
   * Inicializa el pin del relé y apaga el turbo al arrancar.
   * @param pinRelay: número de pin conectado al relé
   */
  void begin(uint8_t pinRelay);

  /**
   * update()
   * Controla el encendido/apagado del turbo según TPS y MAP, aplicando histéresis.
   * @param tpsPct: Porcentaje de acelerador [0.0 – 100.0]
   * @param mapKPa: Presión absoluta del múltiple en kPa
   */
  /**
   * updatePowerLevel()
   * Preparado para controlar el turbo con nivel escalable [0.0 – 1.0]
   * @param level: Potencia relativa del turbo, normalizada
   */
  void updatePowerLevel(float level);

  /**
   * start()
   * Fuerza encendido del turbo (sin importar condiciones).
   */
  void start();

  /**
   * stop()
   * Apaga el turbo manualmente.
   */
  void stop();

  /**
   * isOn()
   * @return true si el turbo está activo.
   */
  bool isOn() const;

    /**
   * isActive()
   * Devuelve true si el relé está activado (nivel HIGH).
   */
  bool isActive() const;

private:
  uint8_t relayPin = 255;   // Pin asignado al relé
  uint8_t _relayPin = 0;
  bool active      = false; // Estado actual del turbo

  // Umbrales con histéresis
  static constexpr float TPS_ON      = 70.0f;
  static constexpr float TPS_OFF     = 65.0f;
  static constexpr float MAP_ON_KPA  = 16.9f;   // ≤ para encender (≈5 inHg)
  static constexpr float MAP_OFF_KPA = 20.3f;   // ≥ para apagar (≈6 inHg)
};
