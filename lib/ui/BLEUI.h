#pragma once

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "ConsoleUI.h"

/**
 * BLEUI
 * Extiende ConsoleUI para operar sobre Bluetooth Serial (SPP)
 * Permite entrada remota por apps como Serial Bluetooth Terminal
 */
class BLEUI : public ConsoleUI {
public:
  BLEUI();

  /**
   * Inicializa Bluetooth con nombre personalizado
   */
  void begin(const String& nombreBT = "CalibradorESP32");

  /**
   * Redefinición de update() para procesar comandos por Bluetooth
   */
  void update();

private:
  BluetoothSerial SerialBT;
};
