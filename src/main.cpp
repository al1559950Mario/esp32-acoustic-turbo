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
#include "Logger.h"

// FreeRTOS para mutex
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// PIN-OUT
constexpr uint8_t PIN_RELAY_TURBO     =  2;
constexpr uint8_t PIN_RELAY_ACOUSTIC  =  4;
constexpr uint8_t PIN_DAC_ACOUSTIC    = 25;
constexpr uint8_t PIN_PRESSURE_OUT    = 26; // HX710B OUT
constexpr uint8_t PIN_PRESSURE_SCK    = 27; // HX710B SCK
constexpr uint8_t PIN_I2C_SDA         = 18;
constexpr uint8_t PIN_I2C_SCL         = 19;

// Objetos globales
StateMachine       fsm;
SensorManager      sensors;
ActuatorManager    actuators;
ConsoleUI*         ui = nullptr;
USBSerialConsoleUI usbConsoleUI(&ui);
BluetoothSerialConsoleUI btConsoleUI(&ui);
BluetoothSerial    SerialBT;
CalibrationManager& calib = CalibrationManager::getInstance();
DebugManager       debugMgr;
ThresholdManager*  thresholdManagerPtr;
Logger             logger(SerialBT);

// Mutex para acceso seguro a I2C
SemaphoreHandle_t i2cMutex = nullptr;

// Task: UI / Consola
void TaskSensorConsole(void* param) {
  constexpr TickType_t sensorPeriod = pdMS_TO_TICKS(10); // 100 Hz
  uint32_t cycleCount = 0;
  static bool prevClient = false;

  for (;;) {
    // 1) Leer sensores
    sensors.update();

    // 2) Sólo cada 2 iteraciones (~20 ms) actualizo consola y debug
    if ((cycleCount++ & 0x01) == 0) {
      // Cambiar UI si hay conexión/desconexión BT
      bool clientNow = SerialBT.hasClient();
      if (clientNow && !prevClient) {
        Serial.println("→ Cliente Bluetooth conectado. Cambiando a BLE UI.");
        ui = &btConsoleUI;
      } else if (!clientNow && prevClient) {
        Serial.println("→ Cliente Bluetooth desconectado. Volviendo a Serial UI.");
        ui = &usbConsoleUI;
      }
      prevClient = clientNow;

      // Actualizar UI y debug
      if (ui)           ui->update();
      debugMgr.updateFromSerial(Serial);
    }

    // 3) Sin bloqueos: paso al siguiente ciclo
    vTaskDelay(sensorPeriod);
  }
}

void setup() {
  Serial.begin(115200);

  // Inicializar sensores y actuadores
  sensors.begin(PIN_PRESSURE_OUT, PIN_PRESSURE_SCK, PIN_I2C_SDA, PIN_I2C_SCL);
  actuators.begin(PIN_BTS_PWM, PWM_CHANNEL_BTS, PIN_DAC_ACOUSTIC);

  // Inicializar UIs (igual que antes)
  usbConsoleUI.begin();
  usbConsoleUI.setFSM(&fsm);
  usbConsoleUI.attachSensors(&sensors);
  usbConsoleUI.attachActuators(&actuators);
  usbConsoleUI.imprimirDashboard();

  btConsoleUI.begin();
  btConsoleUI.setFSM(&fsm);
  btConsoleUI.attachSensors(&sensors);
  btConsoleUI.attachActuators(&actuators);
  btConsoleUI.imprimirDashboard();
  btConsoleUI.attachLogger(&logger);

  usbConsoleUI.setMirror(&btConsoleUI);
  btConsoleUI.setMirror(&usbConsoleUI);
  ui = &usbConsoleUI;

  esp_log_level_set("*", ESP_LOG_WARN);

  // Calibración y FSM (igual)
  calib.begin(&sensors);
  bool calibLoaded = calib.loadCalibration();
  thresholdManagerPtr = new ThresholdManager();
  thresholdManagerPtr->begin();
  fsm.begin(calibLoaded, &actuators, thresholdManagerPtr, &sensors, &calib);
  actuators.stopAll();

  // Task unificada de sensores + consola
  xTaskCreatePinnedToCore(
    TaskSensorConsole,
    "SensorConsole",
    4096,
    nullptr,
    1,    // baja prioridad
    nullptr,
    1     // mismo core que I2C
  );
}

void loop() {
  static bool hasCalibrationLoaded = false;
  static bool firstLoop = true;

  if (firstLoop) {
    delay(50); // da tiempo a estabilizar I2C
    hasCalibrationLoaded = calib.loadCalibration();
    firstLoop = false;
  }

  // Recalibración solicitada por UI
  if (ui->getCalibRequest()) {
    Serial.println("[DEBUG] Solicitud de recalibración detectada");
    calib.clearCalibration();
  }

  calib.update(ui->isSimulation());

  float mapLoadPercent = 0.0f;
  float tpsLoadPercent = 0.0f;

  // Lectura sensores protegida con mutex
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    sensors.update();
    mapLoadPercent = sensors.readMAPLoadPercent();
    tpsLoadPercent = sensors.readTPSLoadPercent();
    xSemaphoreGive(i2cMutex);
  } else {
    Serial.println("[WARN] loop: timeout i2cMutex, saltando lectura sensores");
  }

  bool sistemaActivo = usbConsoleUI.isSistemaActivo() || btConsoleUI.isSistemaActivo();

  if (sistemaActivo) {
    if (tpsLoadPercent >= 100.0f || mapLoadPercent >= 100.0f) {
      Serial.println("[ERROR] Carga al 100% detectada. Saltando FSM.");
      return;
    }

    fsm.update(
      mapLoadPercent,
      tpsLoadPercent,
      usbConsoleUI.getCalibRequest(),
      btConsoleUI.getCalibRequest(),
      hasCalibrationLoaded,
      debugMgr
    );

    fsm.handleActions();

    if (logger.isEnabled()) {
      logger.log(tpsLoadPercent, mapLoadPercent);
    }
  } else {
    actuators.stopAll();
  }

  delay(10); // frecuencia loop ~100 Hz
}
