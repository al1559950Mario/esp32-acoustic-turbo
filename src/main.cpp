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
constexpr uint8_t PIN_DAC_ACOUSTIC    = 25;
constexpr uint8_t PIN_PRESSURE_OUT    = 26; // HX710B OUT
constexpr uint8_t PIN_PRESSURE_SCK    = 27; // HX710B SCK
constexpr uint8_t PIN_BTS_PWM = 18;   // pin conectado al PWM del BTS
constexpr uint8_t PWM_CHANNEL_BTS = 0; // canal de ESP32 (0-15)
constexpr uint8_t PIN_I2C_SDA         = 21;
constexpr uint8_t PIN_I2C_SCL         = 22;

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

void TaskSensorUpdate(void* param) {
  SensorManager* sensorMgr = static_cast<SensorManager*>(param);
  for (;;) {
    // Solo aquí protegemos la lectura de buses I2C
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      sensorMgr->update();      // lee TPS, MAP y ADS1115
      xSemaphoreGive(i2cMutex);
    } else {
      Serial.println("[WARN] SensorUpdate: timeout i2cMutex");
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // cada 10 ms
  }
}


// Task: UI / Consola
void TaskConsoleUpdate(void* param) {
  for (;;) {
    static bool prevClient = false;
    bool currClient = SerialBT.hasClient();

    if (currClient && !prevClient) {
      Serial.println("→ BLE cliente conectado");
      ui = &btConsoleUI;
    } else if (!currClient && prevClient) {
      Serial.println("→ BLE cliente desconectado");
      ui = &usbConsoleUI;
    }
    prevClient = currClient;

    // *Sin mutex* porque UI lee solo variables cacheadas (no I2C directo)
    if (ui) ui->update();

    debugMgr.updateFromSerial(Serial);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}


void setup() {
  Serial.begin(115200);
  // 1) Crear mutex I2C
  i2cMutex = xSemaphoreCreateMutex();
  if (!i2cMutex) {
    Serial.println("❌ No se pudo crear i2cMutex");
    while (1) delay(1000);
  }

  // 2) Init hardware
  sensors.begin(/*args I2C+GPIO*/);
  actuators.begin(/*args*/);

  // 3) Crear tasks
  xTaskCreatePinnedToCore(
    TaskSensorUpdate,    "SensorUpdate",
    2048, &sensors, 1, nullptr, 1
  );
  xTaskCreatePinnedToCore(
    TaskConsoleUpdate,   "ConsoleUpdate",
    4096, nullptr, 1, nullptr, 0
  );

  // 4) UI, FSM, calibration, loggers…
  usbConsoleUI.begin();   btConsoleUI.begin();
  usbConsoleUI.setFSM(&fsm);  btConsoleUI.setFSM(&fsm);
  usbConsoleUI.attachSensors(&sensors);
  btConsoleUI.attachSensors(&sensors);
  usbConsoleUI.attachActuators(&actuators);
  btConsoleUI.attachActuators(&actuators);
  usbConsoleUI.imprimirDashboard();
  btConsoleUI.imprimirDashboard();
  btConsoleUI.attachLogger(&logger);
  usbConsoleUI.setMirror(&btConsoleUI);
  btConsoleUI.setMirror(&usbConsoleUI);
  ui = &usbConsoleUI;

  esp_log_level_set("*", ESP_LOG_WARN);

  calib.begin(&sensors);
  bool cOK = false;
  // No cargamos calibración aquí: la cargamos en el primer loop() bajo mutex
  thresholdManagerPtr = new ThresholdManager();
  if (!thresholdManagerPtr->begin()) {
    Serial.println("❌ Error ThresholdManager");
  }
  fsm.begin(false, &actuators, thresholdManagerPtr, &sensors, &calib);
  actuators.stopAll();
  Serial.println("Setup completo");
}

void loop() {
  static bool calibLoaded = false;
  static bool first = true;

  // Primer ciclo, cargamos calibración bajo mutex
  if (first) {
    delay(50);
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
      calibLoaded = calib.loadCalibration();
      xSemaphoreGive(i2cMutex);
    }
    first = false;
  }

  // Si hay request de recalibración, borro y recalibro protegiendo I2C
  if (ui->getCalibRequest()) {
    calib.clearCalibration();
  }
  if (ui->getCalibRequest()) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
      calib.update(ui->isSimulation());
      xSemaphoreGive(i2cMutex);
    } else {
      Serial.println("[WARN] loop: timeout i2cMutex en calibración");
    }
  } else {
    // En run normal, calib.update() en modo simulación o usa variables internas
    calib.update(ui->isSimulation());
  }

  // Lecturas de sensores SIN mutex: toman valores cacheados por TaskSensorUpdate
  float mapPct = sensors.readMAPLoadPercent();
  float tpsPct = sensors.readTPSLoadPercent();

  bool active = usbConsoleUI.isSistemaActivo() || btConsoleUI.isSistemaActivo();
  if (active) {
    // FSM + actuadores usan valores ya actualizados
    fsm.update(
      mapPct,
      tpsPct,
      usbConsoleUI.getCalibRequest(),
      btConsoleUI.getCalibRequest(),
      calibLoaded,
      debugMgr
    );
    fsm.handleActions();
    actuators.update(tpsPct, mapPct);

    if (logger.isEnabled()) {
      logger.log(tpsPct, mapPct);
    }
  } else {
    actuators.stopAll();
  }

  delay(10);
}
