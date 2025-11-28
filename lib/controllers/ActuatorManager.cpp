#include "ActuatorManager.h"
#include "AcousticInjector.h"
#include "../audio/PCM5102Driver.h"


void ActuatorManager::begin(uint8_t rEnPin, uint8_t turboPwmPin, uint8_t turboPwmChannel,
                            uint8_t turboSensePin,
                            uint8_t acousticDacPin) {
    // Inicializar VortexController con PWM + canal + pin de corriente
    vortex.begin(rEnPin, turboPwmPin, turboPwmChannel, turboSensePin);

    // Inicializar Acoustic Injector
    injector.begin(acousticDacPin);

    // Configurar PCM5102 I2S como backend de salida
    constexpr uint8_t PIN_I2S_BCK  = 26;
    constexpr uint8_t PIN_I2S_LRCK = 27;
    // Reutilizamos acousticDacPin como DATA (por defecto 25)
    pcm5102.attachPins(PIN_I2S_BCK, PIN_I2S_LRCK, acousticDacPin);
    pcm5102.begin(48000); // debe coincidir con SAMPLE_RATE del inyector
    // Activar modo pull: el driver llamará a AcousticInjector para obtener muestras
    injector.usePullMode(true);
    pcm5102.setSampleSource(&AcousticInjector::pullSampleThunk, &injector);

    // Apagar ambos al inicio
    vortex.stop();
    injector.stop();
}


/// Actualiza actuadores. Turbo usa MAF*MAP y Acoustic Injector su lógica interna
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

/// Modo manual: fuerza un nivel de PWM [0.0-1.0], ignorando MAF*MAP
/// @param level: 0.0 = apagado, 1.0 = máxima potencia
void ActuatorManager::updateVortexLevel(float levelMAF, float levelMAP) {
    vortex.updatePowerLevel(levelMAF, levelMAP); // level ya está normalizado 0–1
}


/// Estado del turbo
bool ActuatorManager::isTurboOn() const {
    return vortex.isOn();
}

/// Enciende Acoustic Injector con nivel [0-1]
void ActuatorManager::startAcoustic(float level, float dMAFdt) {
    injector.start(level, dMAFdt);
}

/// Apaga Acoustic Injector
void ActuatorManager::stopAcoustic() {
    injector.stop();
}

/// Configura parámetros del Acoustic Injector
/// @param level: potencia relativa [0-1]
void ActuatorManager::setAcousticParameters(float mafLoadLevel, float /*mapLoadLevel*/) {
    injector.setLevel(mafLoadLevel);

    // Debug compacto (rate limit ~200 ms)
    static uint32_t _amLastLogMs = 0;
    uint32_t _amNow = millis();
    if (_amNow - _amLastLogMs >= 200) {
        _amLastLogMs = _amNow;
        Serial.printf("[AM] maf=%.3f\n", double(mafLoadLevel));
    }
}

/// Estado del Acoustic Injector
bool ActuatorManager::isAcousticOn() const {
    return injector.isActive();
}

void ActuatorManager::testminimalPCMDAC() {
    // Ensure injector is not writing concurrently
    injector.stop();
    // Play a 1 kHz sine for 3 seconds at 60% amplitude
    pcm5102.testMinimal(1000, 3.0f, 0.6f);
}

void ActuatorManager::startPureSine(uint32_t freqHz, float amplitude) {
    injector.stop();
    pcm5102.startPureSine(freqHz, amplitude);
}

void ActuatorManager::stopPureSine() {
    pcm5102.stopPureSine();
}

void ActuatorManager::startISRSine(uint32_t freqHz, float level) {
    injector.startFixedSine(freqHz, level);
}

void ActuatorManager::stopISRSine() {
    injector.stopFixedSine();
}

void ActuatorManager::getAudioStats(uint32_t& dropsFromISR, uint32_t& underruns, size_t& minAvail, size_t& maxAvail) const {
    // const method: pcm5102 getters are const-safe
    pcm5102.getStats(dropsFromISR, underruns, minAvail, maxAvail);
}

void ActuatorManager::resetAudioStats() {
    pcm5102.resetStats();
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

bool ActuatorManager::decayFinished() const{
    return injector.decayFinished();
}
