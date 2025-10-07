#pragma once
#include "MAPSensor.h"
#include "TPSSensor.h"
#include <Arduino.h>
#include "CalibrationManager.h"
#include "PressureSensor.h"
#include <Adafruit_ADS1X15.h>

class SensorManager {
public:
  SensorManager() = default;

  void begin(uint8_t pinPressureData, uint8_t pinPressureSCK, uint8_t pinSDA, uint8_t pinSCL);

  float readVacuum_inHg();
  float readTPSLoadPercent();
  float readMAPLoadPercent();

  uint16_t readMAPRawCached();
  uint16_t readTPSRawCached();
  float readMAPVolts();
  float readTPSVolts();
  bool isTPSValid();
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
  bool adsReady = false;

  Adafruit_ADS1115 ads;

  float getPressureFromBuffer();
  void  updatePressure();

  float computeRMS();
  float computeEventRate(float threshold_kPa = 0.5f, float samplingPeriod_ms = 12.5f);
  float computeTau(float threshold_kPa = 0.5f, float samplingPeriod_ms = 12.5f);

  const float* getPressureBuffer() const { return pressureBuffer; }
  size_t getBufferSize() const { return PRESSURE_BUFFER_SIZE; }
  float getPressurePercent();
  float getPressurePSI();
  float computeOscillationAmplitude();  
  float readOscillationAmplitude(); 


  void update(); // 👈 Opcional, si quieres usar una rutina periódica
private:
  MAPSensor mapSensor;
  TPSSensor tpsSensor;
  PressureSensor pressureSensor;

  float mapLoadPercent = 0.0f;  //
  float tpsLoadPercent  = 0;

  bool simulacionActiva = false;
  float filteredRawTPS = 0;
  float filteredRawMAP = 0;
  const float alpha = 0.8;  // coeficiente del filtro
  float rawTPSCached = 0.0f;
  float rawMAPCached = 0.0f;


  float vacuum_inHg = 0;
  float amplitudeOscillation;  // ΔP: Pmax - Pmin

    // Buffer para métricas futuras
  static constexpr size_t PRESSURE_BUFFER_SIZE = 800; // 10 s a 80 Hz
  float pressureBuffer[PRESSURE_BUFFER_SIZE];
  size_t bufferIndex = 0;
};
