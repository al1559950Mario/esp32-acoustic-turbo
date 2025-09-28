#include "SensorManager.h"


void SensorManager::begin(uint8_t pinPressureData, uint8_t pinPressureSCK, uint8_t pinSDA, uint8_t pinSCL) {
  Wire.begin(pinSDA, pinSCL);
  if (!ads.begin()) {
    Serial.println("❌ ADS1115 no detectado");
  } else {
    ads.setGain(GAIN_TWOTHIRDS);
    adsReady = true;  
    Serial.println("✅ ADS1115 listo");
  }
  mapSensor.begin(1, &ads);  // A1 para MAP
  tpsSensor.begin(0, &ads);  // A0 para TPS
  pressureSensor.begin(pinPressureData, pinPressureSCK);
      // Inicializar buffer
  for (size_t i = 0; i < PRESSURE_BUFFER_SIZE; i++) pressureBuffer[i] = 0.0f;
  bufferIndex = 0;
}


float SensorManager::readVacuum_inHg() {
  return vacuum_inHg;
}

float SensorManager::readTPSLoadPercent() {
  return tpsLoadPercent;
}

uint16_t SensorManager::readMAPRawCached() {
  return rawMAPCached;
}

uint16_t SensorManager::readTPSRawCached() {
  return rawTPSCached;
}

float SensorManager::readMAPVolts() {
  return representVoltsFromRaw(rawMAPCached);
}

float SensorManager::readTPSVolts() {
  return representVoltsFromRaw(rawTPSCached);
}

bool SensorManager::isTPSValid() {
  return tpsSensor.isValidReading();
}

MAPSensor& SensorManager::getMAP() {
  return mapSensor;
}

TPSSensor& SensorManager::getTPS() {
  return tpsSensor;
}

float SensorManager::readMAPLoadPercent() {
  return mapLoadPercent;
}

float SensorManager::representVoltsFromRaw(uint16_t raw) const {
  // 6.144 V / 32768 pasos ≈ 0.1875 mV/bit
  constexpr float LSB = 6.144f / 32768.0f;  
  return raw * LSB;
}


void SensorManager::update() {
  if (simulacionActiva){
    rawTPSCached = (mapSensor.getSimulatedRaw() * 5.0f) / 32767.0f;
    rawMAPCached = (tpsSensor.getSimulatedRaw()* 5.0f) / 32767.0f;
  } else
  {
    rawMAPCached = ads.readADC_SingleEnded(1);
    rawTPSCached = ads.readADC_SingleEnded(0); 
  }
  //updatePressure();
  // Filtro IIR al raw directamente
  filteredRawMAP = alpha * rawMAPCached + (1 - alpha) * filteredRawMAP;
  filteredRawTPS = alpha * rawTPSCached + (1 - alpha) * filteredRawTPS;

  //Porcentaje absoluto
  mapLoadPercent = mapSensor.convertRawToPercent((uint16_t)filteredRawMAP);
  tpsLoadPercent = tpsSensor.convertRawToPercent((uint16_t)filteredRawTPS);
  
}

void SensorManager::updatePressure() {
    float pKPa = pressureSensor.readPressure_kPa();

    // Guardar en buffer
    pressureBuffer[bufferIndex++] = pKPa;
    if(bufferIndex >= PRESSURE_BUFFER_SIZE) bufferIndex = 0;

    // Porcentaje relativo a ±40 kPa
    float pPercent = (pKPa / 40.0f) * 100.0f;

    // Conversión a PSI
    float pPsi = pKPa * 0.145038f;

}




void SensorManager::enableSimulacion() {
  simulacionActiva = true;
}

void SensorManager::disableSimulacion() {
  simulacionActiva = false;
  mapSensor.disableSimulation();
  tpsSensor.disableSimulation();
}

bool SensorManager::isSimulation() {
  return simulacionActiva;
}


float SensorManager::getRelativeTPSLoad(uint16_t tpsInitial) {
  if (tpsInitial >= 100) return 0.0f;
  float norm = ((float)tpsLoadPercent - tpsInitial) / (100.0f - tpsInitial);
  return constrain(norm, 0.0f, 1.0f) * 100.0f;
}

float SensorManager::getRelativeMAPLoad(uint16_t mapInitialPercent) {
  if (mapInitialPercent >= 100) return 0.0f;
  float norm = ((float)mapLoadPercent - mapInitialPercent) / (100.0f - mapInitialPercent);
  return constrain(norm, 0.0f, 1.0f) * 100.0f;
}

float SensorManager::readPressure_kPa() {
    return pressureSensor.readPressure_kPa();
}

long SensorManager::readPressureRaw() {
    return pressureSensor.readRaw();
}

// Último valor del buffer (para UI o logging)
float SensorManager::getPressureFromBuffer() {
    size_t lastIndex = (bufferIndex == 0) ? PRESSURE_BUFFER_SIZE - 1 : bufferIndex - 1;
    return pressureBuffer[lastIndex];
}

 // Tau: tiempo de decaimiento de picos (>37% del valor máximo del pico)
float SensorManager::computeTau(float threshold_kPa, float samplingPeriod_ms) {
    float tauSum = 0.0f;
    size_t tauCount = 0;

    for (size_t i = 1; i < PRESSURE_BUFFER_SIZE; i++) {
      float diff = pressureBuffer[i] - pressureBuffer[i-1];
      if (diff > threshold_kPa) { // inicio de pico válido
        float peak = pressureBuffer[i];
        for (size_t j = i+1; j < PRESSURE_BUFFER_SIZE; j++) {
          if (pressureBuffer[j] <= 0.37f * peak) {
            tauSum += (j - i) * samplingPeriod_ms;
            tauCount++;
            break;
          }
        }
      }
    }

    return (tauCount > 0) ? (tauSum / tauCount) : 0.0f;
  }

  // EventRate: cantidad de cambios significativos por segundo
float SensorManager::computeEventRate(float threshold_kPa , float samplingPeriod_ms ) {
    size_t events = 0;

    for (size_t i = 1; i < PRESSURE_BUFFER_SIZE; i++) {
      float diff = pressureBuffer[i] - pressureBuffer[i-1];
      if (diff > threshold_kPa) events++;
    }

    // Convertir a eventos por segundo
    float totalTime_s = (PRESSURE_BUFFER_SIZE * samplingPeriod_ms) / 1000.0f;
    return (totalTime_s > 0.0f) ? (events / totalTime_s) : 0.0f;
  }

float SensorManager::computeRMS() {
    float sumSq = 0.0f;
    size_t count = 0;
    float offset = readPressure_kPa(); // usar último valor como referencia

    for (size_t i = 0; i < PRESSURE_BUFFER_SIZE; i++) {
      float val = pressureBuffer[i] - offset;
      if (val != 0.0f) { // ignorar ceros iniciales
        sumSq += val * val;
        count++;
      }
    }
    return (count > 0) ? sqrt(sumSq / count) : 0.0f;
  }


  // Devuelve la presión del buffer como porcentaje relativo a ±40 kPa
float SensorManager::getPressurePercent() {
    float pKPa = getPressureFromBuffer();
    return (pKPa / 40.0f) * 100.0f;
}

// Devuelve la presión del buffer en PSI
float SensorManager::getPressurePSI() {
    float pKPa = getPressureFromBuffer();
    return pKPa * 0.145038f;
}