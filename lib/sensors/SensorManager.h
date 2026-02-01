#pragma once
#include "MAPSensor.h"
#include "MAFSensor.h"
#include <Arduino.h>
#include "CalibrationManager.h"
#include "PressureSensor.h"
#include <Adafruit_ADS1X15.h>

class SensorManager {
public:
  SensorManager() = default;

  void begin(uint8_t pinPressureData, uint8_t pinPressureSCK, uint8_t pinSDA, uint8_t pinSCL);

  float readVacuum_inHg();
  float readMAFLoadPercent();
  float readMAPLoadPercent();

  uint16_t readMAPRawCached();
  uint16_t readMAFRawCached();
  float readMAPVolts();
  float readMAFVolts();
  bool isMAFValid();
  float representVoltsFromRaw(uint16_t raw) const;
  void enableSimulacion();
  void disableSimulacion();
  bool isSimulacionActiva() const { return simulacionActiva; }
  bool isSimulation();

  MAPSensor& getMAP();
  MAFSensor& getMAF();

  float getRelativeMAFLevel(float);
  float getRelativeMAPLevel(float);

  float getPressure_kPa();
  bool adsReady = false;

  Adafruit_ADS1115 ads;

  float getPressureKPAFromBuffer();
  void  updatePressure();

  float computeRMS();
  float computeMedianPressure();
  float computeMAD();
  float computeOutlierRatio(float k = 3.0f);
  float computeRMSSlope();
  float computeEventRate(float threshold_kPa = 0.5f, float samplingPeriod_ms = 12.5f);
  float computeTau(float threshold_kPa = 0.5f, float samplingPeriod_ms = 12.5f);

  const float* getPressureBuffer() const { return pressureKPABuffer; }
  size_t getBufferSize() const { return PRESSURE_BUFFER_SIZE; }
  float getPressurePercentSigned();
  float getPressurePercentAbs();
  float getPressurePSI();
  float computeOscillationAmplitude();  
  float readOscillationAmplitude(); 
  float getPressureMedianKPa() const { return pressureMedianKPa; }
  float getPressureMADKPa() const { return pressureMadKPa; }
  float getPressureOutlierRatio() const { return pressureOutlierRatio; }
  float getPressureRMSSlope() const { return pressureRmsSlope; }


  void updateADS1115(); // 👈 Opcional, si quieres usar una rutina periódica
  // Compatibilidad con código antiguo (CalibrationManager, etc.)
  float readTPSLoadPercent() { return readMAFLoadPercent(); }
  uint16_t readTPSRawCached() { return readMAFRawCached(); }
  float readTPSVolts() { return readMAFVolts(); }
  bool isTPSValid() { return isMAFValid(); }
  float getRelativeTPSLevel(float ref) { return getRelativeMAFLevel(ref); }
  // Funciones de lectura para cada métrica
float readMedianPressure();
float readMAD();
float readOutlierRatio();
float readRMSSlope();
float readRMS();
float readTau();
float readEventRate();
void resetMetrics();

private:
  MAPSensor mapSensor;
  MAFSensor mafSensor;
  PressureSensor pressureSensor;

  float mapLoadPercent = 0.0f;  //
  float mafLoadPercent  = 0;

  bool simulacionActiva = false;
  float filteredRawMAF = 0;
  float filteredRawMAP = 0;
  const float alpha = 0.8;  // coeficiente del filtro
  float rawMAFCached = 0.0f;
  float rawMAPCached = 0.0f;

  size_t pressureCount = 0;


  float vacuum_inHg = 0;
  float amplitudeOscillation;  // ΔP: Pmax - Pmin

    // Buffer para métricas futuras
  static constexpr size_t PRESSURE_BUFFER_SIZE = 800; // 10 s a 80 Hz
  float pressureKPABuffer[PRESSURE_BUFFER_SIZE];
  size_t bufferIndex = 0;
  float pressureMedianKPa = 0.0f;
  float pressureMadKPa = 0.0f;
  float pressureOutlierRatio = 0.0f;
  float pressureRmsSlope = 0.0f;
  float lastPressureRms = 0.0f;
  uint32_t lastPressureRmsMs = 0;

  float pressureRMS = 0.0f;
  float pressureTau = 0.0f;
  float pressureEventRate = 0.0f;


};
