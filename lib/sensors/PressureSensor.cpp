#include "PressureSensor.h"
#include <Arduino.h>

void PressureSensor::begin(uint8_t pinData, uint8_t pinSCK) {
    _pinData = pinData;
    _pinSCK  = pinSCK;

    pinMode(_pinData, INPUT);
    pinMode(_pinSCK, OUTPUT);
    digitalWrite(_pinSCK, LOW);

    delay(50); // estabilizar sensor

    const int N = 11; // número de muestras para la mediana (impar)
    long readings[N];

    for (int i = 0; i < N; i++) {
        long reading = 0;
        readRaw(reading);
        readings[i] = reading;
        delay(5);
    }

    // Bubble sort simple para mediana
    for (int i = 0; i < N - 1; i++) {
        for (int j = i + 1; j < N; j++) {
            if (readings[j] < readings[i]) {
                long temp = readings[i];
                readings[i] = readings[j];
                readings[j] = temp;
            }
        }
    }

    long rawZero = readings[N / 2]; // mediana

    // Rango físico del sensor
    const float RANGE_KPA = 40.0f; // ±40 kPa
    _scale  = RANGE_KPA / 8388607.0f;  
    _offset = -(rawZero * _scale);

    minReading = -RANGE_KPA;
    maxReading = RANGE_KPA;
}

bool PressureSensor::readRaw(long& value) {
    static long lastValid = 0;

    unsigned long startTime = micros();
    const unsigned long timeout_us = 200000;

    while (digitalRead(_pinData) == HIGH) {
        if (micros() - startTime > timeout_us) {
            value = lastValid;
            return false;
        }
        delayMicroseconds(5);
    }

    value = 0;
    for (uint8_t i = 0; i < 24; i++) {
        digitalWrite(_pinSCK, HIGH);
        delayMicroseconds(5);
        value = (value << 1) | digitalRead(_pinData);
        digitalWrite(_pinSCK, LOW);
        delayMicroseconds(5);
    }

    if (value & 0x800000) value |= ~0xFFFFFF;

    // Pulso extra para configuración
    digitalWrite(_pinSCK, HIGH);
    delayMicroseconds(5);
    digitalWrite(_pinSCK, LOW);

    lastValid = value;
    return true;
}

bool PressureSensor::updateRawCached() {
    long newVal = 0;
    bool hasNewSample = readRaw(newVal);
    if (hasNewSample) {
        _rawCached = newVal;
        _lastUpdate = millis();
    }
    return hasNewSample;
}


float PressureSensor::getPressure_kPa() {
    
    long rawCached = _rawCached;
    float pressure = (rawCached * _scale) + _offset;

    // Limitar al rango físico
    pressure = constrain(pressure, minReading, maxReading);

    return pressure;
}

void PressureSensor::setCalibration(float scale, float offset) {
    _scale  = scale;
    _offset = offset;
}

void PressureSensor::setMinMax(float minVal, float maxVal) {
    minReading = minVal;
    maxReading = maxVal;
}
