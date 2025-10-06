#include "ActuatorManager.h"
#include "AcousticInjector.h"

/// Inicializa actuadores: BTS7960 y Acoustic Injector
void ActuatorManager::begin(uint8_t turboPwmPin, uint8_t turboPwmChannel,
                            uint8_t acousticDacPin) {
    vortex.begin(turboPwmPin, turboPwmChannel);
    injector.begin(acousticDacPin);

    // Apagar ambos al inicio
    vortex.stop();
    injector.stop();
}

/// Actualiza actuadores. Turbo usa TPS*MAP y Acoustic Injector su lógica interna
/// @param tpsLoadPercent: porcentaje TPS [0-100]
/// @param mapLoadPercent: porcentaje MAP [0-100]
void ActuatorManager::update(float tpsLoadPercent, float mapLoadPercent) {
    // Asegurar rango válido
    tpsLoadPercent = constrain(tpsLoadPercent, 0.0f, 100.0f);
    mapLoadPercent = constrain(mapLoadPercent, 0.0f, 100.0f);

    // Actualiza Acoustic Injector
    injector.update();

}

/// Enciende el turbo al nivel actual
void ActuatorManager::startVortex() {
    vortex.start();
}

/// Detiene ambos actuadores
void ActuatorManager::stopAll() {
    vortex.stop();
    injector.stop();
}

/// Detiene solo el turbo
void ActuatorManager::stopVortex() {
    vortex.stop();
}

/// Modo manual: fuerza un nivel de PWM [0.0-1.0], ignorando TPS*MAP
/// @param level: 0.0 = apagado, 1.0 = máxima potencia
void ActuatorManager::setVortexLevel(float levelTPS, float levelMAP) {
    vortex.updatePowerLevel(levelTPS, levelMAP); // level ya está normalizado 0–1
}


/// Estado del turbo
bool ActuatorManager::isTurboOn() const {
    return vortex.isOn();
}

/// Enciende Acoustic Injector con nivel [0-1]
void ActuatorManager::startAcoustic(float level) {
    injector.start(level);
}

/// Apaga Acoustic Injector
void ActuatorManager::stopAcoustic() {
    injector.stop();
}

/// Configura parámetros del Acoustic Injector
/// @param level: potencia relativa [0-1]
/// @param mapLoadPercent: carga MAP para calcular frecuencia
void ActuatorManager::setAcousticParameters(float tpsLoadLevel, float mapLoadLevel) {
    float freq = injector.mapLoadToWaveFrequency(mapLoadLevel);
    injector.setTargetFrequency(freq);
    injector.setLevel(tpsLoadLevel);
}

/// Estado del Acoustic Injector
bool ActuatorManager::isAcousticOn() const {
    return injector.isActive();
}

/// Acceso directo al VortexController (BTS7960)
VortexController& ActuatorManager::getVortexController() {
    return vortex;
}

/// Acceso directo al AcousticInjector
AcousticInjector& ActuatorManager::getAcousticInjector() {
    return injector;
}
