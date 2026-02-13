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
    // Activar modo pull: el driver llamarÃ¡ a AcousticInjector para obtener muestras
    injector.usePullMode(true);
    pcm5102.setSampleSource16(&AcousticInjector::pullSample16Thunk, &injector);

    // Apagar ambos al inicio
    vortex.stop();
    injector.stop();
}


/// Actualiza actuadores. Turbo usa MAF*MAP y Acoustic Injector su lÃ³gica interna
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
/// @param level: 0.0 = apagado, 1.0 = mÃ¡xima potencia
void ActuatorManager::updateVortexLevel(float levelMAF, float levelMAP) {
    vortex.updatePowerLevel(levelMAF, levelMAP); // level ya estÃ¡ normalizado 0â€“1
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

/// Configura parÃ¡metros del Acoustic Injector
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

void ActuatorManager::setISRSineLevel(float level) {
    injector.setFixedSineLevel(level);
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

void ActuatorManager::testFloorDynamic() {
    // Detener generador normal para evitar mezcla
    injector.stop();

    // ParÃ¡metros del test
    const uint32_t freqHz = 6000;
    const uint32_t dwellMs = 1200;
    const uint8_t steps[] = { 1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64 };

    Serial.println(F("[AM] Test de piso dinÃ¡mico (driver) iniciado..."));
    for (size_t i = 0; i < sizeof(steps); ++i) {
        uint8_t lsb = steps[i];
        float amp = float(lsb) / 127.0f; // normaliza 8-bit LSB a 0..1
        if (amp > 1.0f) amp = 1.0f;
        Serial.printf("[AM][PISO] paso=%u LSB=%u amp=%.3f\n", (unsigned)i, (unsigned)lsb, double(amp));
        pcm5102.startPureSine(freqHz, amp);
        vTaskDelay(pdMS_TO_TICKS(dwellMs));
    }
    pcm5102.stopPureSine();
    Serial.println(F("[AM] Test de piso dinÃ¡mico finalizado."));
}

void ActuatorManager::testFloorUltra() {
    // Detener generador normal para no mezclar
    injector.stop();

    // Amplitudes subâ€‘LSB (referidas a fullâ€‘scale = 1.0)
    // Aprox. dBFS: -48, -50, -52, -54, -56, -58, -60, -62, -64, -66
    const float amps[] = { 0.0040f, 0.00316f, 0.00251f, 0.0020f, 0.00158f,
                           0.00126f, 0.00100f, 0.00079f, 0.00063f, 0.00050f };
    const uint32_t freqHz = 6000;
    const uint32_t dwellMs = 1400; // un poco mÃ¡s de tiempo por paso

    Serial.println(F("[AM] Test de piso ultra (subâ€‘LSB) iniciado..."));
    for (size_t i = 0; i < sizeof(amps)/sizeof(amps[0]); ++i) {
        float amp = amps[i];
        if (amp < 0.0f) amp = 0.0f; if (amp > 1.0f) amp = 1.0f;
        Serial.printf("[AM][PISO_ULTRA] paso=%u amp=%.5f\n", (unsigned)i, double(amp));
        pcm5102.startPureSine(freqHz, amp);
        vTaskDelay(pdMS_TO_TICKS(dwellMs));
    }
    pcm5102.stopPureSine();
    Serial.println(F("[AM] Test de piso ultra finalizado."));
}

void ActuatorManager::testFloorNano() {
    // Silencio y detenemos el inyector
    injector.stop();
    pcm5102.stopPureSine();

    // Pasos en dBFS muy bajos: -70 a -90 dBFS
    const float dB[] = { -70.0f, -75.0f, -80.0f, -85.0f, -90.0f };
    const uint32_t freqHz = 6000;
    const uint32_t dwellMs = 1800; // mÃ¡s tiempo por paso

    Serial.println(F("[AM] Test de piso nano (âˆ’70..âˆ’90 dBFS) iniciado..."));
    // Preâ€‘silencio para referencia
    Serial.println(F("[AM][PISO_NANO] silencio base"));
    pcm5102.startPureSine(freqHz, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(800));

    for (size_t i = 0; i < sizeof(dB)/sizeof(dB[0]); ++i) {
        // amp(lineal) = 10^(dB/20)
        float amp = powf(10.0f, dB[i] / 20.0f);
        if (amp < 0.0f) amp = 0.0f; if (amp > 1.0f) amp = 1.0f;
        Serial.printf("[AM][PISO_NANO] paso=%u dB=%.1f amp=%.6f\n", (unsigned)i, double(dB[i]), double(amp));
        pcm5102.startPureSine(freqHz, amp);
        vTaskDelay(pdMS_TO_TICKS(dwellMs));
        // pequeÃ±o silencio entre pasos para limpiar memoria auditiva
        pcm5102.startPureSine(freqHz, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    pcm5102.stopPureSine();
    Serial.println(F("[AM] Test de piso nano finalizado."));
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
