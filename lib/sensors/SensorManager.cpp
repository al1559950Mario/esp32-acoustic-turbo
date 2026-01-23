#include "SensorManager.h"
#include <vector>
#include <algorithm>
#include <cmath>

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
  mafSensor.begin(2, &ads);  // A0 para MAF
  pressureSensor.begin(pinPressureData, pinPressureSCK);
      // Inicializar buffer
  for (size_t i = 0; i < PRESSURE_BUFFER_SIZE; i++) pressureKPABuffer[i] = 0.0f;
  bufferIndex = 0;
  pressureCount = 0;
}


float SensorManager::readVacuum_inHg() {
  return vacuum_inHg;
}

float SensorManager::readMAFLoadPercent() {
  return mafLoadPercent;
}

uint16_t SensorManager::readMAPRawCached() {
  return rawMAPCached;
}

uint16_t SensorManager::readMAFRawCached() {
  return rawMAFCached;
}

float SensorManager::readMAPVolts() {
  return representVoltsFromRaw(rawMAPCached);
}

float SensorManager::readMAFVolts() {
  return representVoltsFromRaw(rawMAFCached);
}

bool SensorManager::isMAFValid() {
  return mafSensor.isValidReading();
}

MAPSensor& SensorManager::getMAP() {
  return mapSensor;
}

MAFSensor& SensorManager::getMAF() {
  return mafSensor;
}

float SensorManager::readMAPLoadPercent() {
  return mapLoadPercent;
}

float SensorManager::representVoltsFromRaw(uint16_t raw) const {
  // 6.144 V / 32768 pasos ≈ 0.1875 mV/bit
  constexpr float LSB = 6.144f / 32768.0f;  
  return raw * LSB;
}

float SensorManager::getRelativeMAFLevel(float mafInitial) {
  if (mafInitial >= 100) return 0.0f;
  float norm = ((float)mafLoadPercent - mafInitial) / (100.0f - mafInitial);
  return constrain(norm, 0.0f, 1.0f);
}

float SensorManager::getRelativeMAPLevel(float mapInitialPercent) {
  if (mapInitialPercent >= 100) return 0.0f;
  float norm = ((float)mapLoadPercent - mapInitialPercent) / (100.0f - mapInitialPercent);
  return constrain(norm, 0.0f, 1.0f);
}

void SensorManager::enableSimulacion() {
  simulacionActiva = true;
}

void SensorManager::disableSimulacion() {
  simulacionActiva = false;
  mapSensor.disableSimulation();
  mafSensor.disableSimulation();
}

bool SensorManager::isSimulation() {
  return simulacionActiva;
}

void SensorManager::updateADS1115() {
  if (simulacionActiva){
    rawMAPCached = (mapSensor.getSimulatedRaw() * 5.0f) / 32767.0f;
    rawMAFCached = (mafSensor.getSimulatedRaw()* 5.0f) / 32767.0f;
  } else
  {
    rawMAPCached = ads.readADC_SingleEnded(0);
    rawMAFCached = ads.readADC_SingleEnded(3); 
  }
  // Filtro IIR al raw directamente
  filteredRawMAP = alpha * rawMAPCached + (1 - alpha) * filteredRawMAP;
  filteredRawMAF = alpha * rawMAFCached + (1 - alpha) * filteredRawMAF;

  //Porcentaje absoluto
  mapLoadPercent = mapSensor.convertRawToPercent((uint16_t)filteredRawMAP);
  mafLoadPercent = mafSensor.convertRawToPercent((uint16_t)filteredRawMAF);
}



void SensorManager::updatePressure() {
    if (!pressureSensor.updateRawCached()) {
      return;
    }
    float pKPa = getPressure_kPa();

    // Guardar en buffer
    pressureKPABuffer[bufferIndex++] = pKPa;
    if(bufferIndex >= PRESSURE_BUFFER_SIZE) bufferIndex = 0;
    if (pressureCount == 0){
      pressureCount++;
    }

    // Calcula la amplitud de oscilación en cada ciclo
    amplitudeOscillation = computeOscillationAmplitude();

}

float SensorManager::getPressure_kPa() {
    return pressureSensor.getPressure_kPa();
}

// Último valor del buffer (para UI o logging)
float SensorManager::getPressureKPAFromBuffer() {
    if(pressureCount == 0){
      return 0.0f;
    }
    size_t lastIndex = (bufferIndex == 0) ? PRESSURE_BUFFER_SIZE - 1 : bufferIndex - 1;
    return pressureKPABuffer[lastIndex];
}

  // Devuelve la presión del buffer como porcentaje relativo a ±40 kPa
float SensorManager::getPressurePercent() {
    float pKPa = getPressureKPAFromBuffer();
    return (pKPa / 40.0f) * 100.0f;
}

// Devuelve la presión del buffer en PSI
float SensorManager::getPressurePSI() {
    float pKPa = getPressureKPAFromBuffer();
    return pKPa * 0.145038f;
}

float SensorManager::computeOscillationAmplitude() {
    float low_pct = 0.05f;
    float high_pct = 0.95f;
    const size_t n = pressureCount;
    if (n == 0) return 0.0f;

    std::vector<float> tmp;
    tmp.reserve(n);
    for (size_t i = 0; i < n; ++i) tmp.push_back(pressureKPABuffer[i]);

    size_t idxLow  = (size_t)floorf(low_pct * (n - 1));
    size_t idxHigh = (size_t)floorf(high_pct * (n - 1));

    std::nth_element(tmp.begin(), tmp.begin() + idxLow, tmp.end());
    float lowVal = tmp[idxLow];

    std::nth_element(tmp.begin(), tmp.begin() + idxHigh, tmp.end());
    float highVal = tmp[idxHigh];

    return (highVal > lowVal) ? (highVal - lowVal) : 0.0f;
}

float SensorManager::readOscillationAmplitude() {
    return amplitudeOscillation;
}

 // Tau: tiempo de decaimiento de picos (>37% del valor máximo del pico)
float SensorManager::computeTau(float threshold_kPa, float samplingPeriod_ms) {
    float tauSum = 0.0f;
    size_t tauCount = 0;
    const size_t n = pressureCount;
    if (n == 0) {
      return 0.0f;
    }

    for (size_t i = 1; i < n; i++) {
      float diff = pressureKPABuffer[i] - pressureKPABuffer[i-1];
      if (diff > threshold_kPa) { // inicio de pico válido
        float peak = pressureKPABuffer[i];
        for (size_t j = i+1; j < n; j++) {
          if (pressureKPABuffer[j] <= 0.37f * peak) {
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
    const size_t n = pressureCount;
    if (n == 0) {
      return 0.0f;
    }
    for (size_t i = 1; i < n; i++) {
      float diff = pressureKPABuffer[i] - pressureKPABuffer[i-1];
      if (diff > threshold_kPa) events++;
    }

    // Convertir a eventos por segundo
    float totalTime_s = (n * samplingPeriod_ms) / 1000.0f;
    return (totalTime_s > 0.0f) ? (events / totalTime_s) : 0.0f;
  }

float SensorManager::computeRMS() {
    size_t count = pressureCount;
    if (count == 0) {
      return 0.0f;
    }    
    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
      sum += pressureKPABuffer[i];
    }
    float mean = sum / count;
    float sumSq = 0.0f;

    for (size_t i = 0; i < count; i++) {
      float val = pressureKPABuffer[i] - mean;
      sumSq += val * val;
    }
    return sqrt(sumSq / count);
  }
