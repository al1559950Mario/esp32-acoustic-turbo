#include "Logger.h"

Logger::Logger(BluetoothSerial& bt, bool header)
    : btSerial(bt), active(false) {
    if (header) printHeader();
}

void Logger::enable(bool state) {
    active = state;
    if (active) {
        printHeader();  // timestamp,tps,map,...
    }
}

bool Logger::isEnabled() const {
    return active;
}

void Logger::printHeader() {
    btSerial.println("timestamp_ms,tps_pct,map_pct,event");
}

void Logger::log(float tps, float map, const String& event) {
    if (!active) return;
    unsigned long ts = millis();
    btSerial.printf("%lu,%.2f,%.2f,%s\n", ts, tps, map, event.c_str());
}

// Versión extendida (futuro)
void Logger::log(float tps, float map, float pressureRel, const String& event) {
    if (!active) return;
    unsigned long ts = millis();
    btSerial.printf("%lu,%.2f,%.2f,%.2f,%s\n", ts, tps, map, pressureRel, event.c_str());
}

void Logger::logRaw(const String& msg) {
    if (active) {
        btSerial.println(msg);
    }
}
