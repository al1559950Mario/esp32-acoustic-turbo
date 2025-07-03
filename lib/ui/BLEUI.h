#pragma once

#include <Arduino.h>
#include "StateMachine.h"

/**
 * BLEUI
 * Interfaz de usuario vía Bluetooth Serial para diagnóstico y control remoto.
 * Recibe comandos de un cliente BLE (como Serial Bluetooth Terminal) y comunica con FSM.
 */
class BLEUI {
public:
  BLEUI();

  /**
   * begin()
   * Inicializa la referencia al puerto BLE (ej. SerialBT).
   */
  void begin(Stream* serialRef);

  /**
   * setFSM()
   * Asocia una instancia de la FSM para poder consultar y modificar el estado.
   */
  void setFSM(StateMachine* fsmRef);

  /**
   * update()
   * Procesa comandos recibidos desde BLE y ejecuta acciones asociadas.
   */
  void update(SystemState fsmState);

  /**
   * getCalibRequest()
   * @return true si el usuario ha solicitado calibración por BLE.
   * Se limpia automáticamente después de leerlo.
   */
  bool getCalibRequest();

private:
  Stream* bleSerial = nullptr;
  StateMachine* fsm = nullptr;

  bool calibRequested = false;  // Flag local de calibración

  void imprimirEstado(SystemState s);
  void imprimirHelp();
};
