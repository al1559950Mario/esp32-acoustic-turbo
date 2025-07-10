#pragma once

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "ConsoleUI.h"
#include "StateMachine.h"
#include "MAPSensor.h"
#include "TPSSensor.h"
#include "AcousticInjector.h"
#include "TurboController.h"

/**
 * BLEUI
 * Extiende ConsoleUI para comunicación por Bluetooth Serial.
 * Permite que el celular acceda al serial vía Bluetooth.
 * Usa PIN fijo "0000" para emparejamiento.
 */
class BLEUI : public ConsoleUI {
  SensorManager* sensors = nullptr;  // puntero a SensorManager para acceder a sensores

public:
  BLEUI();
  void attachSensors(SensorManager* sensorManagerPtr);

  /**
   * Inicializa Bluetooth con nombre y PIN 0000.
   */
  void begin(const String& nombreBT = "CalibradorESP32");

  /**
   * Override update() para leer comandos desde Bluetooth en lugar de USB Serial.
   */
  void update() override;

private:
  BluetoothSerial SerialBT;

  /**
   * Interpreta comandos recibidos por Bluetooth.
   */
  void interpretarComando(char c) override;

  /**
   * Imprime ayuda por Bluetooth.
   */
  void imprimirHelp() override;
};