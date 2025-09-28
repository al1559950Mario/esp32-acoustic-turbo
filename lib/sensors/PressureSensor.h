#pragma once
#include <Arduino.h>

class PressureSensor {
public:
    PressureSensor() = default;

    void begin(uint8_t pinData, uint8_t pinSCK);
    long readRaw();
    float readPressure_kPa();
    void tare();
    void setCalibration(float scale, float offset);

    // --- Nuevas funciones ---
    float readPressurePercent();  // porcentaje relativo al rango
    float readPressure_psi();     // conversión a psi
    void setMinMax(float minVal, float maxVal); // definir rango para %
    
private:
    uint8_t _pinData;
    uint8_t _pinSCK;
    float _scale = 1.0f;   // factor de calibración
    float _offset = 0.0f;  // offset en kPa

    // Rango esperado para calcular %
    float minReading = 0;
    float maxReading = 1000000; 
};
