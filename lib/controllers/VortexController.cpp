#include "VortexController.h"

void VortexController::begin(uint8_t pwmPin_, uint8_t pwmChannel_, uint8_t sensePin_) {
    pwmPin = pwmPin_;
    pwmChannel = pwmChannel_;
    sensePin = sensePin_;

    pinMode(pwmPin, OUTPUT);
    ledcSetup(pwmChannel, 20000, 8);
    ledcAttachPin(pwmPin, pwmChannel);

    if (sensePin != 255) {
        analogSetPinAttenuation(sensePin, ADC_11db);
        analogReadResolution(12);
    }

    active = false;
    lastPWM = 0.0f;
    ledcWrite(pwmChannel, 0);
}


void VortexController::start() {
    if (!active) {
        ledcWrite(pwmChannel, (int)(lastPWM * 255));
        active = true;
    }
}

void VortexController::stop() {
    if (active) {
        ledcWrite(pwmChannel, 0);
        active = false;
    }
}

void VortexController::updatePowerLevel(float levelTPS, float levelMAP) {
    // 1) Normalizar entradas
    float tpsRel = constrain(levelTPS, 0.0f, 1.0f);
    float mapRel = constrain(levelMAP, 0.0f, 1.0f);

    // 2) Pesos ajustables para priorizar TPS en bajos
    const float tpsWeight = 0.85f;   // incrementar para más respuesta a pedal
    const float mapWeight = 0.15f;

    // 3) Nivel combinado lineal
    float level = tpsWeight * tpsRel + mapWeight * mapRel;
    level = constrain(level, 0.0f, 1.0f);

    // 4) Curva de refuerzo a bajos
    // Opción A (recomendada): gamma < 1 realza bajos sin saturar
    const float gammaBoost = 0.20f; // 0.4 - 0.8 prueba para más/menos refuerzo
    float boosted = pow(level, gammaBoost);

    // 5) Curva fina exponencial opcional para ajuste fino
    const float a = 0.4f; // reducir para menos pendiente si usas gammaBoost fuerte
    float curvedLevel = (exp(a * boosted) - 1.0f) / (exp(a) - 1.0f);
    curvedLevel = constrain(curvedLevel, 0.0f, 1.0f);

    // 6) Suavizado simple (low-pass) para evitar cambios bruscos
    const float smoothFactor = 0.08f; // 0 = sin suavizado, 1 = bloqueo total
    lastPWM = lastPWM + smoothFactor * (curvedLevel - lastPWM);

    lastPWM = constrain(lastPWM, 0.0f, 1.0f);

    // 7) Aplicar PWM
    if (active) {
        int pwmValue = int(lastPWM * 255.0f);
        ledcWrite(pwmChannel, pwmValue);
    }
}

bool VortexController::isOn() const {
    return active;
}

bool VortexController::isActive() const {
    return active;
}


void VortexController::updateCurrentSense() {
    if (sensePin == 255) return;

    int raw = analogRead(sensePin);
    float voltage = raw * (3.3f / 4095.0f);
    constexpr float sensitivity = 8.5f; // Ajustar según shunt real
    float current = voltage * sensitivity;

    // Filtro exponencial suave
    currentCached += (current - currentCached) * 0.1f;
}

float VortexController::getCurrentSense() const {
    return currentCached;
}
