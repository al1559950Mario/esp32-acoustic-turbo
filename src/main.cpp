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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// PIN-OUT
constexpr uint8_t PIN_DAC_ACOUSTIC    = 25;
constexpr uint8_t PIN_PRESSURE_OUT    = 4; // HX710B OUT
constexpr uint8_t PIN_PRESSURE_SCK    = 23; // HX710B SCK
constexpr uint8_t PIN_BTS_SENSE = 35; // R_IS -> corriente del motor (ADC1_CH7)
constexpr uint8_t PIN_BTS_PWM = 18;   // pin conectado al PWM del BTS
constexpr uint8_t PIN_R_EN = 19;  //Pin para activar BTS
constexpr uint8_t PWM_CHANNEL_BTS = 0; // canal de ESP32 (0-15)
constexpr uint8_t PIN_I2C_SDA         = 21;
constexpr uint8_t PIN_I2C_SCL         = 22;

// Objetos globales
StateMachine         fsm;
SensorManager        sensors;
ActuatorManager      actuators;
ConsoleUI*           ui               = nullptr;
USBSerialConsoleUI   usbConsoleUI(&ui);
BluetoothSerialConsoleUI btConsoleUI(&ui);
BluetoothSerial      SerialBT;
CalibrationManager&  calib            = CalibrationManager::getInstance();
DebugManager         debugMgr;
ThresholdManager*    thresholdManagerPtr;
Logger               logger(SerialBT);

// Indicador de calibración cargada
bool calibLoaded = false;

// Tarea de sensores (sólo lee ADS1115 y cachea raws y porcentajes)- Falta agregar leer el sensor desde aqui con un timing independiente a otros sensores para mayor control de afinacion.
void TaskSensorUpdate(void* param) {
  auto* sm = static_cast<SensorManager*>(param);
  for (;;) {
    sm->updateADS1115();
    sm->updatePressure();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Tarea de consola/UI (core 0): nunca toca I²C
void TaskConsoleUpdate(void* param) {
  for (;;) {
    bool btClient = SerialBT.hasClient();
    if (btClient && ui != &btConsoleUI) {
      ui = &btConsoleUI;
    } else if (!btClient && ui != &usbConsoleUI) {
      ui = &usbConsoleUI;
    }

    if (ui) ui->update();
    debugMgr.updateFromSerial(Serial);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  // Inicialización de sensores y actuadores
  sensors.begin(PIN_PRESSURE_OUT, PIN_PRESSURE_SCK, PIN_I2C_SDA, PIN_I2C_SCL);
  actuators.begin(PIN_R_EN, PIN_BTS_PWM, PWM_CHANNEL_BTS, PIN_BTS_SENSE, PIN_DAC_ACOUSTIC);


  // Crear TaskSensorUpdate en Core 1
  xTaskCreatePinnedToCore(
    TaskSensorUpdate,
    "SensorUpdate",
    2048,
    &sensors,
    2,
    nullptr,
    1
  );

  // Crear TaskConsoleUpdate en Core 0
  xTaskCreatePinnedToCore(
    TaskConsoleUpdate,
    "ConsoleUpdate",
    4096,
    nullptr,
    1,
    nullptr,
    0
  );

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

  // Cargar calibración y configurar FSM
  calib.begin(&sensors);
  calibLoaded = calib.loadCalibration();

  // Inicializar ThresholdManager usando el singleton
  bool t_ok = ThresholdManager::getInstance().begin();
  if (!t_ok) {
    ui->println("❌ Error al iniciar ThresholdManager (singleton)");
  }
  fsm.begin(calibLoaded, &actuators, &ThresholdManager::getInstance(), &sensors, &calib);
  actuators.stopAll();

  ui->println(
    calibLoaded
      ? "  Estado inicial: OFF (calibración cargada)"
      : "  Estado inicial: SIN_CALIBRAR (necesita calibración)"
  );
}

void loop() {
  // Procesar solicitud de recalibración
  if (ui && ui->getCalibRequest()) {
    ui->println("[Loop] Recalibración solicitada");
    calib.clearCalibration();
    ui->println("[Loop] AFTER Recalibración solicitada");

    calibLoaded = false;
  }

  // Avanzar proceso de calibración si no está completa
  calib.update(ui ? ui->isSimulation() : false);


  bool calibrationRunning = usbConsoleUI.isCalibrationSessionActive() || btConsoleUI.isCalibrationSessionActive();
  bool sistemaActivo = usbConsoleUI.isSistemaActivo() || btConsoleUI.isSistemaActivo();
  if (calibrationRunning) {
    // Durante calibración de resonancia, el servicio controla actuadores de forma exclusiva.
    // No ejecutar FSM ni stopAll aquí para no cortar el streaming acústico de prueba.
  } else if (sistemaActivo) {
    float mapLoadPercent = sensors.readMAPLoadPercent();
    float mafLoadPercent = sensors.readMAFLoadPercent();

    if (mapLoadPercent >= 100.0f && mafLoadPercent > 100.0f) {
      ui->println("[ERROR] Carga 100%, saltando FSM");
    } else {
      fsm.update(
        mapLoadPercent,
        mafLoadPercent,
        usbConsoleUI.getCalibRequest(),
        btConsoleUI.getCalibRequest(),
        calibLoaded,
        debugMgr
      );


      fsm.handleActions();
      
      
      if (logger.isEnabled()) {
        float acousticFreq   = actuators.getCurrentFrequency();
        float acousticLevel  = actuators.getAcousticLevel();
        bool  acousticOn     = actuators.isAcousticOn();
        float turboLevel    = actuators.getTurboLevel();
        //float turboAmp  = actuators.getVortexController().getCurrent();  // si tienes medición de corriente
        float turboAmp = 1.0f;
        bool  turboOn = actuators.isTurboOn();
        float deltaP = sensors.computeOscillationAmplitude();
        float pressure_kPa = sensors.getPressure_kPa();
        float pressure_pct = sensors.getPressurePercentSigned();
        float pressure_psi = sensors.getPressurePSI();
        float eventRate = sensors.computeEventRate();
        float tau = sensors.computeTau();
        float rms = sensors.computeRMS();
        String state = fsm.getStateName();
        String event = "Main_loop";
        logger.logFull(mafLoadPercent, mapLoadPercent, pressure_kPa, pressure_pct, pressure_psi,
              deltaP,  tau,  eventRate,  rms,
              acousticFreq,  acousticLevel,  turboLevel,  turboAmp,
              acousticOn, turboOn, state, event);

      }
    }
  } else {
    actuators.stopAll();
  }

  vTaskDelay(pdMS_TO_TICKS(10));
}
