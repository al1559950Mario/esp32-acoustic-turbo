#include "AcousticInjector.h"
#include "driver/dac.h"
#include "driver/timer.h"

/// Seno de 64 muestras (0…255) centrado en 128
const uint8_t AcousticInjector::_sineTable[AcousticInjector::TABLE_SIZE] = {
  128,140,152,164,175,186,196,205,
  214,222,229,235,240,244,246,248,
  249,248,246,244,240,235,229,222,
  214,205,196,186,175,164,152,140,
  128,115,103,91,80,69,59,50,
   41,33,26,20,15,11,9,7,
   6,7,9,11,15,20,26,33,
   41,50,59,69,80,91,103,115
};

/// Puntero único para que la ISR acceda a la instancia activa
static AcousticInjector* _instance = nullptr;

void AcousticInjector::begin(uint8_t dacPin, uint8_t relayPin) {
  _dacPin   = dacPin;
  _relayPin = relayPin;

  // Prepara relé (apagado)
  pinMode(_relayPin, OUTPUT);
  digitalWrite(_relayPin, LOW);

  // Habilita canal DAC (canal = dacPin-25)
  dac_output_enable((dac_channel_t)(dacPin - 25));

  // Configura timerHW: prescaler 80 → tick = 1µs
  _timer = timerBegin(0, 80, true);
  timerAttachInterrupt(_timer, onTimer, true);
  timerAlarmWrite(_timer, 1000000 / SAMPLE_RATE, true);
  timerAlarmDisable(_timer);

  _instance = this;
}

void AcousticInjector::start(float level) {
  // 1) Silencio inicial
  _level       = 0.0f;
  _targetLevel = constrain(level, 0.0f, 1.0f);

  // 2) Arranca relé con salida en 0V (silencio)
  digitalWrite(_relayPin, HIGH);
  delay(10);  // deja estabilizar alimentación

  // 3) Activa ISR de forma continua
  _index = 0;
  timerAlarmEnable(_timer);
}

void AcousticInjector::stop() {
  // Detiene ISR
  timerAlarmDisable(_timer);

  // Apaga relé
  digitalWrite(_relayPin, LOW);

  // Silencia DAC a nivel medio
  dac_output_voltage((dac_channel_t)(_dacPin - 25), 128);
}

void AcousticInjector::setLevel(float level) {
  // Actualiza nivel objetivo; se rampará en update()
  _targetLevel = constrain(level, 0.0f, 1.0f);
}

void AcousticInjector::update() {
  // Si el nivel actual es menor al objetivo, sube por pasos
  if (_level < _targetLevel) {
    _level = min(_level + RAMP_STEP, _targetLevel);
  }
  // Opcionalmente podrías también hacer un ramp-down:
  else if (_level > _targetLevel) {
    _level = max(_level - RAMP_STEP, _targetLevel);
  }
}

/**
 * ISR periódica a SAMPLE_RATE:
 *  - Toma muestra de _sineTable
 *  - Escala por _level (amplitud)
 *  - Escribe al DAC
 */
void IRAM_ATTR AcousticInjector::onTimer() {
  if (!_instance) return;

  // Muestra base [0..255], centra: delta ∈ [-128..127]
  uint8_t raw   = _instance->_sineTable[_instance->_index];
  int16_t delta = (int16_t)raw - 128;

  // Aplica factor de amplitud y re-centra
  int16_t out = 128 + (delta * _instance->_level);

  // Sirve al DAC
  dac_output_voltage(
    (dac_channel_t)(_instance->_dacPin - 25),
    constrain(out, 0, 255)
  );

  // Avanza índice cíclico
  if (++_instance->_index >= TABLE_SIZE) {
    _instance->_index = 0;
  }
}
