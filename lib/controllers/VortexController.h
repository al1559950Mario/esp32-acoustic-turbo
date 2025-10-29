#pragma once
#include <Arduino.h>

/**
 * VortexController
 * Controla el BTS7960 en una sola dirección con PWM
 * y potencia escalable combinando %TPS y %MAP.
 */
class VortexController {
public:
    /**
     * begin()
     * Inicializa el pin PWM del BTS7960 y lo deja apagado.
     * @param pwmPin: pin conectado al canal PWM del BTS
     * @param pwmChannel: canal de PWM de ESP32 (0-15)
     */
    void begin(uint8_t rEnPin, uint8_t pwmPin_, uint8_t pwmChannel_, uint8_t sensePin_ = 255);

    /**
     * start()
     * Activa el motor al último nivel configurado.
     */
    void start();

    /**
     * stop()
     * Detiene el motor (PWM = 0).
     */
    void stop();

    /**
     * updatePowerLevel()
     * Actualiza el PWM combinando TPS y MAP
     * @param tpsPct: porcentaje de acelerador (0-100)
     * @param mapLoadPercent: porcentaje de carga MAP (0-100)
     */
    void updatePowerLevel(float, float);

    bool isOn() const;
    bool isActive() const;
    float getLastPWM() const { return lastPWM; }
    void updateCurrentSense();   // lectura real + filtrado
    float getCurrentSense() const;    // lectura cacheada (rápida)




private:
    uint8_t pwmPin = 255;
    uint8_t rEnPin = 255;
    uint8_t pwmChannel = 0;   // canal ESP32 usado en ledcWrite
    bool active = false;
    float lastPWM = 0.0f;     // nivel actual (0.0 – 1.0)}
    uint8_t sensePin;
    float currentCached = 0.0f;
    unsigned long lastSenseUpdate = 0;


};
