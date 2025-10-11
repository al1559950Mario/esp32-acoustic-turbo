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
        readings[i] = readRaw();
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

long PressureSensor::readRaw() {
    static long lastValid = 0;

    unsigned long startTime = micros();
    const unsigned long timeout_us = 200000;

    while (digitalRead(_pinData) == HIGH) {
        if (micros() - startTime > timeout_us) return lastValid;
        delayMicroseconds(5);
    }

    long value = 0;
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
    return value;
}

float PressureSensor::readPressure_kPa() {
    const int FILTER_N = 5;
    static float buffer[FILTER_N] = {0};
    static uint8_t index = 0;
    static bool filled = false;

    long raw = readRaw();
    float pressure = (raw * _scale) + _offset;

    // Limitar al rango físico
    pressure = constrain(pressure, minReading, maxReading);

    buffer[index++] = pressure;
    if (index >= FILTER_N) { index = 0; filled = true; }

    float sum = 0.0f;
    uint8_t count = filled ? FILTER_N : index;
    for (uint8_t i = 0; i < count; i++) sum += buffer[i];

    return sum / count;
}

void PressureSensor::tare() {
    long raw = readRaw();
    _offset = -(raw * _scale);
}

void PressureSensor::setCalibration(float scale, float offset) {
    _scale  = scale;
    _offset = offset;
}

float PressureSensor::readPressurePercent() {
    float p = readPressure_kPa();
    float pct = (p - minReading) / (maxReading - minReading) * 100.0f;
    return constrain(pct, 0.0f, 100.0f);
}

float PressureSensor::readPressure_psi() {
    float p = readPressure_kPa();
    float psi = p * 0.145038f;
    return constrain(psi, minReading * 0.145038f, maxReading * 0.145038f);
}

void PressureSensor::setMinMax(float minVal, float maxVal) {
    minReading = minVal;
    maxReading = maxVal;
}
