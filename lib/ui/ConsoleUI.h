#pragma once

#include <Arduino.h>
#include "StateMachine.h"

/**
 * ConsoleUI
 * Interfaz de comandos por consola serial para diagnóstico, pruebas y control manual.
 * No modifica el hardware directamente — solo activa flags o llama funciones públicas de FSM.
 */
class ConsoleUI {
public:
  /**
   * begin()
   * Inicializa el puerto serie e imprime mensaje de bienvenida.
   */
  void begin();

  /**
   * update()
   * Procesa comandos entrantes del usuario por consola serial.
   * Debe llamarse cada ciclo.
   */
  void update();

  /**
   * setFSM()
   * Asocia una referencia a la FSM para poder consultar estado o activar funciones.
   */
  void setFSM(StateMachine* fsmRef);

  /**
   * getCalibRequest()
   * @return true si el usuario ha solicitado calibración por consola.
   * Se limpia automáticamente tras ser consultado una vez.
   */
  bool getCalibRequest();

private:
  StateMachine* fsm = nullptr;

  bool consoleCalibRequested = false;  // Flag local de calibración

  /**
   * imprimirHelp()
   * Muestra el listado de comandos disponibles.
   */
  void imprimirHelp();

  /**
   * imprimirEstado()
   * Muestra el estado actual del sistema y FSM.
   */
  void imprimirEstado();

  /**
   * interpretarComando(char c)
   * Ejecuta la acción correspondiente al comando recibido.
   */
  void interpretarComando(char c);
};
