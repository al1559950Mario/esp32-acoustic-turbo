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
    void begin(uint8_t pwmPin, uint8_t pwmChannel);

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
    void updatePowerLevel(float tpsLoadPercent, float mapLoadPercent);

    bool isOn() const;
    bool isActive() const;

private:
    uint8_t pwmPin = 255;
    uint8_t pwmChannel = 0;   // canal ESP32 usado en ledcWrite
    bool active = false;
    float lastPWM = 0.0f;     // nivel actual (0.0 – 1.0)
};
