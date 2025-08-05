#pragma once
#include "MAPSensor.h"
#include "TPSSensor.h"
#include <Arduino.h>
#include "CalibrationManager.h"

class SensorManager {
public:
  SensorManager() = default;

  void begin(uint8_t pinMAP, uint8_t pinTPS);

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

  float getRelativeTPSLoad(uint16_t, uint16_t );
  float getRelativeMAPLoad(uint16_t, uint16_t );


  void update(); // 👈 Opcional, si quieres usar una rutina periódica
private:
  MAPSensor mapSensor;
  TPSSensor tpsSensor;
  float mapLoadPercent = 0.0f;  //
  bool simulacionActiva = false;
  float filteredRawTPS = 0;
  float filteredRawMAP = 0;
  const float alpha = 0.6;  // coeficiente del filtro


  float vacuum_inHg = 0;
  float tpsLoadPercent  = 0;
};
