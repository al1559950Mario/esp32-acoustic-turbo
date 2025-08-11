#pragma once
#include "MAPSensor.h"
#include "TPSSensor.h"
#include <Arduino.h>
#include "CalibrationManager.h"
#include "PressureSensorHX710B.h"

class SensorManager {
public:
  SensorManager() = default;

  void begin(uint8_t pinMAP, uint8_t pinTPS, uint8_t pinPressureData, uint8_t pinPressureSCK);

  float readVacuum_inHg();
  float readTPSLoadPercent();
  uint16_t readMAPRaw();
  uint16_t readTPSRaw();
  float readMAPVolts();
  float readTPSVolts();
  bool isTPSValid();
  float readMAPLoadPercent();
  float representVoltsFromRaw(uint16_t raw) const;
  void enableSimulacion();
  void disableSimulacion();
  bool isSimulacionActiva() const { return simulacionActiva; }
  bool isSimulation();

  MAPSensor& getMAP();
  TPSSensor& getTPS();

  float getRelativeTPSLoad(uint16_t);
  float getRelativeMAPLoad(uint16_t);

  float readPressure_kPa();
  long readPressureRaw();


  void update(); // 👈 Opcional, si quieres usar una rutina periódica
private:
  MAPSensor mapSensor;
  TPSSensor tpsSensor;
  PressureSensorHX710B pressureSensor;

  float mapLoadPercent = 0.0f;  //
  bool simulacionActiva = false;
  float filteredRawTPS = 0;
  float filteredRawMAP = 0;
  const float alpha = 0.8;  // coeficiente del filtro


  float vacuum_inHg = 0;
  float tpsLoadPercent  = 0;
};
