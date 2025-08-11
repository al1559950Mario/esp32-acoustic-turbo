#pragma once
#include <Arduino.h>

class PressureSensorHX710B {
public:
    PressureSensorHX710B() = default;

    void begin(uint8_t pinData, uint8_t pinSCK);
    long readRaw();
    float readPressure_kPa();
    void tare();
    void setCalibration(float scale, float offset);

private:
    uint8_t _pinData;
    uint8_t _pinSCK;
    float _scale = 1.0f;   // factor de calibración
    float _offset = 0.0f;  // offset en kPa
};
