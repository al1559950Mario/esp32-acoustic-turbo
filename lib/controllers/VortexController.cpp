#include "VortexController.h"

void VortexController::begin(uint8_t rEnPin_, uint8_t pwmPin_, uint8_t pwmChannel_, uint8_t sensePin_) {
    pwmPin = pwmPin_;
    pwmChannel = pwmChannel_;
    sensePin = sensePin_;
    rEnPin= rEnPin_;

    pinMode(pwmPin, OUTPUT);
    pinMode(rEnPin, OUTPUT);
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
        digitalWrite(rEnPin, HIGH);
        ledcWrite(pwmChannel, (int)(lastPWM * 255));
        active = true;
    }
}

void VortexController::stop() {
    if (active) {
        digitalWrite(rEnPin, LOW);
        ledcWrite(pwmChannel, 0);
        active = false;
    }
}

void VortexController::updatePowerLevel(float levelTPS, float levelMAP) {
    // 1) Normalizar entradas
    float tpsRel = constrain(levelTPS, 0.0f, 1.0f);
    float mapRel = constrain(levelMAP, 0.0f, 1.0f);

    // 2) Pesos (mantén la suma ≈ 1)
    const float tpsWeight = 0.85f;
    const float mapWeight = 0.15f;
    float sumW = tpsWeight + mapWeight;
    float tpsW = (sumW > 0.0f) ? (tpsWeight / sumW) : 1.0f;
    float mapW = 1.0f - tpsW;

    // 3) Nivel combinado lineal (0..1)
    float level = constrain(tpsW * tpsRel + mapW * mapRel, 0.0f, 1.0f);

    // -------- Aqui modificamos el comportamiento arriba de 0.5 --------
    // constante ajustable: >1 comprime la mitad superior; 1.0 = sin compresión
    const float UPPER_HALF_COMPRESS_EXP = 1.0f; // 1.2..1.6 prueba para "un poquito" menos rápido

    float shaped;
    if (level <= 0.5f) {
        // mitad baja: dejamos igual (o aplicas tu boost habitual)
        shaped = level;
    } else {
        // mitad alta: remap a 0..1, comprimir con exponente >1, reescala a 0.5..1
        float above = (level - 0.5f) * 2.0f;        // 0..1 dentro de la mitad superior
        float compressed = powf(above, UPPER_HALF_COMPRESS_EXP); // comprime crecimiento
        shaped = 0.5f + 0.5f * compressed;         // vuelve a rango 0..1
    }
    // ------------------------------------------------------------------

    // 4) Opcional: mezcla perceptual leve (puedes eliminar si no la quieres)
    const float gammaBoost = 0.00f; // si quieres realce en bajos; dejar 0 para no afectar
    float boosted = (gammaBoost > 0.0f) ? powf(shaped, gammaBoost) : shaped;

    // 5) Suavizado simple (EMA) y limitación de paso en 0..255
    const float smoothFactor = 0.33f;
    const uint8_t MAX_STEP_PWM = 32;

    float alpha = constrain(smoothFactor, 0.0f, 1.0f);
    float newPWM = lastPWM + alpha * (boosted - lastPWM);

    int16_t lastInt = int16_t(constrain(int(lastPWM * 255.0f), 0, 255));
    int16_t candidateInt = int16_t(constrain(int(newPWM * 255.0f), 0, 255));
    int16_t diff = candidateInt - lastInt;
    if (diff > int16_t(MAX_STEP_PWM))  candidateInt = lastInt + MAX_STEP_PWM;
    if (diff < -int16_t(MAX_STEP_PWM)) candidateInt = lastInt - MAX_STEP_PWM;
    candidateInt = constrain(candidateInt, 0, 255);
    lastPWM = float(candidateInt) / 255.0f;

    // 6) Aplicar PWM si está activo
    if (active) {
        ledcWrite(pwmChannel, candidateInt);
    }
    updateCurrentSense();
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
    //Serial.printf("\nsense%d\n", raw);
    float voltage = raw * (3.3f / 4095.0f);
    constexpr float sensitivity = 8.5f; // Ajustar según shunt real
    float current = voltage * sensitivity;

    // Filtro exponencial suave
    currentCached += (current - currentCached) * 0.1f;
}

float VortexController::getCurrentSense() const {
    return currentCached;
}
