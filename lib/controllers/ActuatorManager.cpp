#include "ActuatorManager.h"
#include "AcousticInjector.h"


void ActuatorManager::begin(uint8_t rEnPin, uint8_t turboPwmPin, uint8_t turboPwmChannel,
                            uint8_t turboSensePin,
                            uint8_t acousticDacPin) {
    // Inicializar VortexController con PWM + canal + pin de corriente
    vortex.begin(rEnPin, turboPwmPin, turboPwmChannel, turboSensePin);

    // Inicializar Acoustic Injector
    injector.begin(acousticDacPin);

    // Apagar ambos al inicio
    vortex.stop();
    injector.stop();
}


/// Actualiza actuadores. Turbo usa TPS*MAP y Acoustic Injector su lógica interna
void ActuatorManager::updateInjector() {
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
void ActuatorManager::updateVortexLevel(float levelTPS, float levelMAP) {
    vortex.updatePowerLevel(levelTPS, levelMAP); // level ya está normalizado 0–1
}


/// Estado del turbo
bool ActuatorManager::isTurboOn() const {
    return vortex.isOn();
}

/// Enciende Acoustic Injector con nivel [0-1]
void ActuatorManager::startAcoustic(float level, float dTPSdt) {
    injector.start(level, dTPSdt);
}

/// Apaga Acoustic Injector
void ActuatorManager::stopAcoustic() {
    injector.stop();
}

/// Configura parámetros del Acoustic Injector
/// @param level: potencia relativa [0-1]
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

float ActuatorManager::getCurrentFrequency() {
    return injector.getFrequency();
}

float ActuatorManager::getAcousticLevel() {
    return injector.getLevel();
}

float ActuatorManager::getTurboLevel() {
    //0.0-1.0
    return vortex.getLastPWM();
}
float ActuatorManager::getTurboSense() {
    return vortex.getCurrentSense();
}

bool ActuatorManager::inDecay() const{
    return injector.isInDecay();
}