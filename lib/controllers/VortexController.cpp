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

void VortexController::updatePowerLevel(float level) {
    // Asegurar rango 0–1
    level = constrain(level, 0.0f, 1.0f);

    lastPWM = level;

    if (active) {
        ledcWrite(pwmChannel, (int)(level * 255));
    }
}


bool VortexController::isOn() const {
    return active;
}

bool VortexController::isActive() const {
    return active;
}
