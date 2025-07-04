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

  // Habilita canal DAC (GPIO25 → CHANNEL_1, GPIO26 → CHANNEL_2)
  dac_channel_t channel;
  if (_dacPin == 25) {
    channel = DAC_CHANNEL_1;
  } else if (_dacPin == 26) {
    channel = DAC_CHANNEL_2;
  } else {
    return;
  }
  dac_output_enable(channel);

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
  dac_channel_t channel = (_dacPin == 25)
    ? DAC_CHANNEL_1
    : DAC_CHANNEL_2;
  dac_output_voltage(channel, 128);
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

uint8_t AcousticInjector::getCurrentDAC() const {
  return _lastDACValue;
}

bool AcousticInjector::isActive() const {
  return timerAlarmEnabled(_timer);  // true si la ISR está activa
}

void IRAM_ATTR AcousticInjector::onTimer() {
  if (!_instance) return;

  // Onda base [0–255]
  uint8_t raw   = _instance->_sineTable[_instance->_index];
  int16_t delta = (int16_t)raw - 128;

  // Aplica nivel de amplitud (_level ∈ [0.0 – 1.0]) y re-centra
  int16_t out = 128 + (delta * _instance->_level);

  // Guarda valor para consola y envía a DAC
  _instance->_lastDACValue = constrain(out, 0, 255);
  dac_output_voltage(_instance->_dacChannel, _instance->_lastDACValue);

  // Avanza índice cíclico
  if (++_instance->_index >= TABLE_SIZE) {
    _instance->_index = 0;
  }
}

