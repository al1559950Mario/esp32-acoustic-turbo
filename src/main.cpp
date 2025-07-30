#include <Arduino.h>
#include "StateMachine.h"
#include "CalibrationManager.h"
#include "DebugManager.h"
#include "ActuatorManager.h"
#include "SensorManager.h"
#include "USBSerialConsoleUI.h"
#include "BluetoothSerialConsoleUI.h"
#include <BluetoothSerial.h>
#include "ThresholdManager.h"



// PIN-OUT
constexpr uint8_t PIN_MAP             = 35;
constexpr uint8_t PIN_TPS             = 34;
constexpr uint8_t PIN_RELAY_TURBO     =  2;
constexpr uint8_t PIN_RELAY_ACOUSTIC  =  4;
constexpr uint8_t PIN_DAC_ACOUSTIC    = 25;

// Objetos globales
StateMachine       fsm;
SensorManager      sensors;
ActuatorManager    actuators;
ConsoleUI* ui = nullptr;
USBSerialConsoleUI usbConsoleUI(&ui);
BluetoothSerialConsoleUI btConsoleUI(&ui);
BluetoothSerial SerialBT;
CalibrationManager& calib = CalibrationManager::getInstance();
DebugManager       debugMgr;
ThresholdManager* thresholdManagerPtr;
bool calibLoaded = false;

void TaskSensorUpdate(void* param) {
  SensorManager* sensorMgr = static_cast<SensorManager*>(param);
  for (;;) {
    sensorMgr->update();                    // Mantén este método como está
    vTaskDelay(pdMS_TO_TICKS(10));          // Ejecuta cada 10 ms
  }
}
void TaskConsoleUpdate(void* param) {
  for (;;) {
    static bool clientePrevio = false;
    bool clienteActual = SerialBT.hasClient();

    if (clienteActual && !clientePrevio) {
      Serial.println("→ Cliente Bluetooth conectado. Cambiando a BLE UI.");
      ui = &btConsoleUI;
    } else if (!clienteActual && clientePrevio) {
      Serial.println("→ Cliente Bluetooth desconectado. Volviendo a Serial UI.");
      ui = &usbConsoleUI;
    }
    clientePrevio = clienteActual;

    if (ui) ui->update();
    debugMgr.updateFromSerial(Serial);
    
    vTaskDelay(pdMS_TO_TICKS(20));  // ajusta según necesidad
  }
}



void setup() {

  // Inicializar sensores y actuadores
  sensors.begin(PIN_MAP, PIN_TPS);
  actuators.begin(PIN_RELAY_TURBO, PIN_DAC_ACOUSTIC, PIN_RELAY_ACOUSTIC);

  xTaskCreatePinnedToCore(
    TaskSensorUpdate,
    "SensorPolling",
    2048,
    &sensors,  // tu instancia global o singleton
    1,         // prioridad baja
    nullptr,
    1          // core 1 si se usa Wifi o BT
  );

  xTaskCreatePinnedToCore(
    TaskConsoleUpdate,
    "ConsoleUpdate",
    4096,     // más memoria si usas Bluetooth
    nullptr,
    1,        // misma prioridad que sensores
    nullptr,
    0         // Core 0, deja sensores en core 1
  );

  // Iniciar UI Serial USB
  usbConsoleUI.begin();
  usbConsoleUI.setFSM(&fsm);
  usbConsoleUI.attachSensors(&sensors);
  usbConsoleUI.attachActuators(&actuators);
  usbConsoleUI.imprimirDashboard();

  // Iniciar UI Bluetooth Serial clásico
  //SerialBT.setPin("0000");  // Opcional
  btConsoleUI.begin();
  btConsoleUI.setFSM(&fsm);
  btConsoleUI.attachSensors(&sensors);
  btConsoleUI.attachActuators(&actuators);
  btConsoleUI.imprimirDashboard();

  usbConsoleUI.setMirror(&btConsoleUI);
  btConsoleUI.setMirror(&usbConsoleUI);

  ui = &usbConsoleUI;

  esp_log_level_set("*", ESP_LOG_WARN);  // Silencia todos los módulos, solo muestra WARN o superior



  // Estado inicial
  calib.begin(&sensors);
  bool calibLoaded = calib.loadCalibration();
  thresholdManagerPtr = new ThresholdManager();
  if (!thresholdManagerPtr->begin()) {
    Serial.println("❌ Error al iniciar ThresholdManager");
  }

  fsm.begin(calibLoaded, &actuators, thresholdManagerPtr);

  actuators.stopAll();

  if (!calibLoaded)
    Serial.println("  Estado inicial: SIN_CALIBRAR (necesita calibración)");
  else
    Serial.println("  Estado inicial: OFF (calibración cargada)");
}

void loop() {

  bool sistemaActivo = usbConsoleUI.isSistemaActivo() || btConsoleUI.isSistemaActivo();
  static bool hasTriedLoad = false;
  static bool hasCalibration = false;

  

  if (!hasTriedLoad) {
    hasCalibration = calib.loadCalibration();
    hasTriedLoad = true;
  }
  if (ui->getCalibRequest()) {
      calib.clearCalibration();
  }
  calib.update(ui->isSimulation());


  if (sistemaActivo) {
    float mapLoad = sensors.readMAPLoadPercent();
    float tpsPorcent = sensors.readLoadTPSPercent();

    fsm.update(
      mapLoad,
      tpsPorcent,
      usbConsoleUI.getCalibRequest(),
      btConsoleUI.getCalibRequest(),
      hasCalibration, 
      debugMgr
    );
    
    fsm.handleActions();
  } else {
    actuators.stopAll();
  }

  delay(20);
}

