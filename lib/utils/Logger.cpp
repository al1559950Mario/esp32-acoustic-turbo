#include "Logger.h"

Logger::Logger(BluetoothSerial& bt, bool header)
    : btSerial(bt), active(false) {
    if (header) printHeader();
}

void Logger::enable(uint8_t mode) {
    active = true;  // SIEMPRE activo para registrar comparativas

    switch (mode) {
        case 1:  // All on
            printHeader();
            logRaw("Modo 1: All ON");
            break;
        case 2:  // Acoustic ON, Turbo OFF
            printHeader();
            logRaw("Modo 2: Acoustic ON, Turbo OFF");
            break;
        case 3:  // Acoustic OFF, Turbo ON
            printHeader();
            logRaw("Modo 3: Acoustic OFF, Turbo ON");
            break;
        case 4:  // All OFF
            printHeader();
            logRaw("Modo 4: All OFF");
            break;
        default:
            logRaw("Modo desconocido");
            break;
    }

    // Guarda el modo como evento para cada logFull
    currentMode = mode;
}


bool Logger::isEnabled() const {
    return active;
}

void Logger::printHeader() {
    btSerial.println("timestamp_ms,maf_pct,map_pct,event");
}

void Logger::log(float maf, float map, const String& event) {
    if (!active) return;
    unsigned long ts = millis();
    btSerial.printf("%lu,%.2f,%.2f,%s\n", ts, maf, map, event.c_str());
}

// Versión extendida (futuro)
void Logger::log(float maf, float map, float pressureRel, const String& event) {
    if (!active) return;
    unsigned long ts = millis();
    btSerial.printf("%lu,%.2f,%.2f,%.2f,%s\n", ts, maf, map, pressureRel, event.c_str());
}

void Logger::logRaw(const String& msg) {
    if (active) {
        btSerial.println(msg);
    }
}

void Logger::logFull(float mafPercent, float mapPercent, float pressure_kPa, float pressure_pct, float pressure_psi,
                     float deltaP, float tau, float eventRate, float rms,
                     float acousticFreq, float acousticLevel, float turboLevel, float turboCurr,
                     bool acousticOn, bool turboOn, const String& state, const String& event) {
    if (!active) return;
    unsigned long ts = millis();
    btSerial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%s,%s\n",
                    ts, mafPercent, mapPercent, pressure_kPa, pressure_pct, pressure_psi,
                    deltaP, tau, eventRate, rms,
                    acousticFreq, acousticLevel, turboLevel, turboCurr,
                    acousticOn ? 1 : 0, turboOn ? 1 : 0,
                    state.c_str(), event.c_str());
}

