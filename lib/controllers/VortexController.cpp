#include "VortexController.h"

void VortexController::begin(uint8_t pwmPin_, uint8_t pwmChannel_) {
    pwmPin = pwmPin_;
    pwmChannel = pwmChannel_;

    pinMode(pwmPin, OUTPUT);

    // Configurar PWM: canal, frecuencia 20 kHz, resolución 8 bits
    ledcSetup(pwmChannel, 20000, 8);
    ledcAttachPin(pwmPin, pwmChannel);

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
    float tpsRel = constrain(levelTPS, 0.0f, 1.0f);
    float mapRel = constrain(levelMAP, 0.0f, 1.0f);
    // Asegurar rango 0–1
    float level = tpsRel * mapRel;

    // 🔹 Aplicar curva exponencial SOLO al level
    float a = 4.0f; // controla la aceleración al final
    float curvedLevel = (exp(a * level) - 1.0f) / (exp(a) - 1.0f);

    lastPWM = curvedLevel;

    if (active) {
        ledcWrite(pwmChannel, (int)(curvedLevel * 255));
    }
}


bool VortexController::isOn() const {
    return active;
}

bool VortexController::isActive() const {
    return active;
}
