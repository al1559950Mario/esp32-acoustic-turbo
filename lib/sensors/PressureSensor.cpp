#include "PressureSensor.h"

void PressureSensor::begin(uint8_t pinData, uint8_t pinSCK) {
    _pinData = pinData;
    _pinSCK  = pinSCK;

    pinMode(_pinData, INPUT);
    pinMode(_pinSCK, OUTPUT);
    digitalWrite(_pinSCK, LOW);

    delay(50); // estabilizar sensor

    const int N = 11; // número de muestras para la mediana (impar)
    long readings[N];

    // Tomar N lecturas
    for (int i = 0; i < N; i++) {
        readings[i] = readRaw();
        delay(5); // pequeño delay entre lecturas
    }

    // Ordenar array para calcular mediana (bubble sort simple)
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

    // Calibración automática según ±40 kPa
    _scale  = 40.0f / 8388607.0f;  // o el rango que uses
    _offset = -(rawZero * _scale);  // ajusta para que rawZero -> 0 kPa

    // Para referencias de porcentaje
    minReading = 0.0f;
    maxReading = 40.0f;

    Serial.print("Sensor iniciado. Raw base (mediana): ");
    Serial.print(rawZero);
    Serial.print(", Scale: "); Serial.print(_scale);
    Serial.print(", Offset: "); Serial.println(_offset);
}




long PressureSensor::readRaw() {
    static long lastValid = 0;  // Guarda el último valor válido

    // Esperar a que DATA esté en LOW (dato listo) con timeout
    unsigned long startTime = micros();
    const unsigned long timeout_us = 200000; // 0.2 s, más realista que 1 s

    while (digitalRead(_pinData) == HIGH) {
        if (micros() - startTime > timeout_us) {
            // Timeout: devolver el último valor válido
            return lastValid;
        }
        delayMicroseconds(5); // dejar un pequeño espacio para no saturar
    }

    long value = 0;

    // Leer 24 bits
    for (uint8_t i = 0; i < 24; i++) {
        digitalWrite(_pinSCK, HIGH);
        delayMicroseconds(5);
        value = (value << 1) | digitalRead(_pinData);
        digitalWrite(_pinSCK, LOW);
        delayMicroseconds(5);
    }

    // Convertir de complemento a dos a número con signo
    if (value & 0x800000) {
        value |= ~0xFFFFFF;
    }

    // Pulso extra para configurar canal/ganancia
    digitalWrite(_pinSCK, HIGH);
    delayMicroseconds(5);
    digitalWrite(_pinSCK, LOW);

    // Guardar el valor válido y devolverlo
    lastValid = value;
    return value;
}

float PressureSensor::readPressure_kPa() {
    long raw = readRaw();
    return (raw * _scale) + _offset;
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
    return p * 0.145038f; // 1 kPa ≈ 0.145038 psi
}

void PressureSensor::setMinMax(float minVal, float maxVal) {
    minReading = minVal;
    maxReading = maxVal;
}