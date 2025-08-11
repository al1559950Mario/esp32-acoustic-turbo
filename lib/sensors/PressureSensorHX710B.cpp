#include "PressureSensorHX710B.h"

void PressureSensorHX710B::begin(uint8_t pinData, uint8_t pinSCK) {
    _pinData = pinData;
    _pinSCK  = pinSCK;

    pinMode(_pinData, INPUT);
    pinMode(_pinSCK, OUTPUT);
    digitalWrite(_pinSCK, LOW);
}

long PressureSensorHX710B::readRaw() {
    // Esperar a que DATA esté en LOW (dato listo)
    while (digitalRead(_pinData) == HIGH) {
        delayMicroseconds(1);
    }

    long value = 0;

    // Leer 24 bits
    for (uint8_t i = 0; i < 24; i++) {
        digitalWrite(_pinSCK, HIGH);
        delayMicroseconds(1);
        value = (value << 1) | digitalRead(_pinData);
        digitalWrite(_pinSCK, LOW);
        delayMicroseconds(1);
    }

    // Convertir de complemento a dos a número con signo
    if (value & 0x800000) {
        value |= ~0xFFFFFF;
    }

    // Un pulso extra para configurar el canal y ganancia (modo HX710B estándar)
    digitalWrite(_pinSCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(_pinSCK, LOW);

    return value;
}

float PressureSensorHX710B::readPressure_kPa() {
    long raw = readRaw();
    return (raw * _scale) + _offset;
}

void PressureSensorHX710B::tare() {
    long raw = readRaw();
    _offset = -(raw * _scale);
}

void PressureSensorHX710B::setCalibration(float scale, float offset) {
    _scale  = scale;
    _offset = offset;
}
