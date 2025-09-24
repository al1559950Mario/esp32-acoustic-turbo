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

#include <atomic>

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

static std::atomic<float> g_mapLoadPercent{0.0f};
static std::atomic<float> g_tpsLoadPercent{0.0f};

// Task: UI / Consola
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

    if (ui) {
      float mapVal = g_mapLoadPercent.load(std::memory_order_relaxed);
      float tpsVal = g_tpsLoadPercent.load(std::memory_order_relaxed);
      ui->updateValues(mapVal, tpsVal);
      ui->update();
    }

    debugMgr.updateFromSerial(Serial);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void TaskSensorUpdate(void* param) {
  auto* sensorMgr = static_cast<SensorManager*>(param);
  SensorManager& sm = *sensorMgr;
  for (;;) {
    sm.update();  // único lugar que toca I2C
    g_mapLoadPercent.store(sm.readMAPLoadPercent(), std::memory_order_relaxed);
    g_tpsLoadPercent.store(sm.readTPSLoadPercent(), std::memory_order_relaxed);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void setup() {
  Serial.begin(115200);

  // Crear mutex
  i2cMutex = xSemaphoreCreateMutex();
  if (!i2cMutex) {
    Serial.println("❌ Error creando i2cMutex");
    while (1) delay(1000);
  }

  // Inicializar sensores y actuadores
  sensors.begin(PIN_PRESSURE_OUT, PIN_PRESSURE_SCK, PIN_I2C_SDA, PIN_I2C_SCL);
  actuators.begin(PIN_RELAY_TURBO, PIN_DAC_ACOUSTIC, PIN_RELAY_ACOUSTIC);

    // Tarea dedicada a sensores (solo aquí se usa I2C)
  xTaskCreatePinnedToCore(
    TaskSensorUpdate,
    "SensorUpdate",
    4096,
    &sensors,  
    2,         // prioridad algo superior
    nullptr,
    1          // core 1 para no interferir con Wi-Fi/BT
  );

  // Crear task de consola
  if (xTaskCreatePinnedToCore(TaskConsoleUpdate, "ConsoleUpdate", 4096, nullptr, 1, nullptr, 0) != pdPASS) {
    Serial.println("❌ Error creando TaskConsoleUpdate");
  }

  // Inicializar UIs
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

  // Calibración
  calib.begin(&sensors);
  bool calibLoaded = calib.loadCalibration();

  thresholdManagerPtr = new ThresholdManager();
  if (!thresholdManagerPtr->begin()) {
    Serial.println("❌ Error al iniciar ThresholdManager");
  }

  fsm.begin(calibLoaded, &actuators, thresholdManagerPtr, &sensors, &calib);
  actuators.stopAll();

  if (!calibLoaded)
    Serial.println("  Estado inicial: SIN_CALIBRAR (necesita calibración)");
  else
    Serial.println("  Estado inicial: OFF (calibración cargada)");
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

  // Leer sensor “copiado”
  float mapLoadPercent = g_mapLoadPercent.load(std::memory_order_relaxed);
  float tpsLoadPercent = g_tpsLoadPercent.load(std::memory_order_relaxed);

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

  vTaskDelay(pdMS_TO_TICKS(10));  // mantiene ~100 Hz
}
