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

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      if (ui) ui->update();
      xSemaphoreGive(i2cMutex);
    } else {
      Serial.println("[WARN] TaskConsoleUpdate: timeout i2cMutex, saltando ui->update()");
    }

    debugMgr.updateFromSerial(Serial);
    vTaskDelay(pdMS_TO_TICKS(20));
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
  actuators.begin(PIN_BTS_PWM, PWM_CHANNEL_BTS, PIN_DAC_ACOUSTIC);

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
    float mapLoadPercent = sensors.readMAPLoadPercent();
    float tpsLoadPercent = sensors.readTPSLoadPercent();

    actuators.update(tpsLoadPercent, mapLoadPercent);

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
      logger.log(tpsLoadPercent, mapLoadPercent);  //agregar más valores en el futuro
    }
  } else {
    actuators.stopAll();
  }

  delay(10); // frecuencia loop ~100 Hz
}
