#pragma once

#include "MAPSensor.h"
#include "TPSSensor.h"

class SensorManager {
public:
  SensorManager() = default;

  void begin(uint8_t pinMAP, uint8_t pinTPS);

  float readVacuum_inHg();
  float readTPSPercent();
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


  MAPSensor& getMAP();
  TPSSensor& getTPS();

  void update(); // 👈 Opcional, si quieres usar una rutina periódica
private:
  MAPSensor mapSensor;
  TPSSensor tpsSensor;
  float mapLoadPercent = 0.0f;  //
  bool simulacionActiva = false;



  float vacuum_inHg = 0;
  float tpsPercent  = 0;
};
