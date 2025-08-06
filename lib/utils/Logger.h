#pragma once
#include "BluetoothSerial.h"
#include <vector>

class Logger {
public:
    Logger(BluetoothSerial& bt, bool header = true);

    void log(float tps, float map, const String& event = "");

    // Futuro: expansión con más sensores
    void log(float tps, float map, float pressureRel, const String& event = "");

    void logRaw(const String& msg);
    void enable(bool state);
    bool isEnabled() const;

    void printHeader(); // Puede volver a enviar encabezado

private:
    BluetoothSerial& btSerial;
    bool active;
};
