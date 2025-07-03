#pragma once

#include <Arduino.h>

/**
 * Señales que pueden ser overrideadas para simulación o debugging.
 * Se utilizan como claves para acceder a cada canal de control en DebugManager.
 */
enum class DebugTarget {
  TPS,       // Sensor de acelerador (voltaje o porcentaje)
  MAP,       // Presión absoluta del múltiple
  TURBO,     // Activación manual del turbo
  INYECTOR   // Activación manual del inyector acústico con nivel variable
};

/**
 * Estructura interna para representar el estado de cada canal overrideado.
 */
struct OverrideData {
  bool  activo = false;   // Flag: override activo
  float valor  = 0.0f;    // Valor simulado (ej: voltaje o nivel)
};

/**
 * DebugManager
 * Módulo para pruebas, simulación y override controlado de señales.
 * Permite activar valores simulados de entrada/salida desde consola, BLE o Python vía Serial.
 */
class DebugManager {
public:
  // Activar override con valor simulado
  void enableOverride(DebugTarget target, float value);

  // Activar override sin valor (default = 0.0)
  void enableOverride(DebugTarget target);

  // Desactivar override individual
  void disableOverride(DebugTarget target);

  // Desactivar todos los canales
  void disableAll();

  // Consultar si un canal tiene override activo
  bool hasOverride(DebugTarget target) const;

  // Obtener valor simulado del canal (válido solo si override activo)
  float getValue(DebugTarget target) const;

  // Especial: verificar si override del turbo está activo
  bool turboOverride() const;

  // Especial: verificar si override del inyector acústico está activo
  bool acousticOverride() const;

  // Especial: obtener nivel overrideado del inyector acústico
  float getLevel() const;

  /**
   * updateFromSerial()
   * Lee una línea de texto enviada por Serial (ej: "tps:2.1,map:3.1,turbo:1,iny:0.75")
   * Extrae valores y activa/desactiva los overrides correspondientes.
   * @param serial Puerto Serial conectado al simulador Python
   */
  void updateFromSerial(Stream& serial);
  void updateFromLine(const String& line);

private:
  static constexpr int OVERRIDE_COUNT = 4;  // Total de canales soportados

  OverrideData overrides[OVERRIDE_COUNT];   // Arreglo por canal

  // Traduce enum DebugTarget a índice del arreglo
  int index(DebugTarget target) const;

  /**
   * setIfPresent()
   * Busca el prefijo en la línea, extrae valor y activa override si corresponde.
   * @param line   Línea completa de texto recibida
   * @param prefix Prefijo del campo (ej: "tps:")
   * @param target Canal que se está procesando
   */
  void setIfPresent(const String& line, const char* prefix, DebugTarget target);
};
