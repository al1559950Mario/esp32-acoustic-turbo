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
    // 1) Normalizar niveles
    float tpsRel = constrain(levelTPS, 0.0f, 1.0f);
    float mapRel = constrain(levelMAP, 0.0f, 1.0f);

    // 2) Pesos (ajusta tpsWeight alto, mapWeight bajo)
    const float tpsWeight = 0.8f;  
    const float mapWeight = 0.2f;  

    // 3) Nivel combinado
    float level = tpsWeight * tpsRel + mapWeight * mapRel;

    // 4) Curva exponencial (tu easing existente)
    float a = 0.5f;
    float curvedLevel = (exp(a * level) - 1.0f) / (exp(a) - 1.0f);

    // 5) Aplicar PWM
    lastPWM = curvedLevel;
    if (active) {
        ledcWrite(pwmChannel, int(curvedLevel * 255));
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
