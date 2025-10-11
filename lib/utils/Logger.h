#pragma once
#include "BluetoothSerial.h"
#include <vector>

class Logger {
public:
    Logger(BluetoothSerial& bt, bool header = true);

    void log(float tps, float map, const String& event = "");

    // Futuro: expansión con más sensores
    void log(float tps, float map, float pressureRel, const String& event = "");

    void logFull(float tpsPercent, float mapPercent, float pressure_kPa, float pressure_pct, float pressure_psi,
             float deltaP, float tau, float eventRate, float rms,
             float acousticFreq, float acousticLevel, float turboLevel, float turboCurr,
             bool acousticOn, bool turboOn, const String& state, const String& event);


    void logRaw(const String& msg);

    void enable(uint8_t mode);    
    bool isEnabled() const;

    void printHeader(); // Puede volver a enviar encabezado

private:
    BluetoothSerial& btSerial;
    bool active;
    uint8_t currentMode = 0;
};
