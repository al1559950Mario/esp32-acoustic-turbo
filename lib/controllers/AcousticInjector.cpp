#include "AcousticInjector.h"
#include "driver/dac.h"
#include "driver/timer.h"

volatile uint8_t dacPending = 128;
static AcousticInjector* _instance = nullptr;

/// Seno de 64 muestras (0–255)
const uint8_t AcousticInjector::_sineTable[TABLE_SIZE] = {
  128,140,152,164,175,186,196,205,214,222,229,235,240,244,246,248,
  249,248,246,244,240,235,229,222,214,205,196,186,175,164,152,140,
  128,115,103,91,80,69,59,50,41,33,26,20,15,11,9,7,
  6,7,9,11,15,20,26,33,41,50,59,69,80,91,103,115
};

void AcousticInjector::begin(uint8_t dacPin, uint8_t relayPin) {
  _dacPin = dacPin;
  _relayPin = relayPin;
  pinMode(_relayPin, OUTPUT);
  digitalWrite(_relayPin, LOW);

  _dacChannel = (_dacPin == 25) ? DAC_CHANNEL_1 : DAC_CHANNEL_2;
  dac_output_enable(_dacChannel);

  _timer = timerBegin(0, 80, true);
  timerAttachInterrupt(_timer, onTimer, true);
  timerAlarmWrite(_timer, 1000000 / SAMPLE_RATE, true);
  timerAlarmDisable(_timer);

  _instance = this;
}

void AcousticInjector::start(float level) {
  _level = 0.0f;
  _targetLevel = constrain(level, 0.0f, 1.0f);
  digitalWrite(_relayPin, HIGH);
  delay(10);
  _index = 0;
  timerAlarmEnable(_timer);
}

void AcousticInjector::stop() {
  timerAlarmDisable(_timer);
  digitalWrite(_relayPin, LOW);
  dac_output_voltage(_dacChannel, 128);
}

void AcousticInjector::setLevel(float level) {
  _targetLevel = constrain(level, 0.0f, 1.0f);
}

void AcousticInjector::update() {
  if (_level < _targetLevel)
    _level = min(_level + RAMP_STEP, _targetLevel);
  else if (_level > _targetLevel)
    _level = max(_level - RAMP_STEP, _targetLevel);
}

void AcousticInjector::applyPendingDAC() {
  int16_t delta = (int16_t)dacPending - 128;
  int16_t out = 128 + (delta * _level);  // ← fuera del ISR, sí puede usar float
  uint8_t final = constrain(out, 0, 255);
  dac_output_voltage(_dacChannel, final);
  _lastDACValue = final;
}

uint8_t AcousticInjector::getCurrentDAC() const {
  return _lastDACValue;
}

bool AcousticInjector::isActive() const {
  return timerAlarmEnabled(_timer);
}

void IRAM_ATTR AcousticInjector::onTimer() {
  if (!_instance) return;

  uint8_t raw = _instance->_sineTable[_instance->_index];
  dacPending = raw;  // ← sin escalamiento aquí
  _instance->_lastDACValue = raw;

  if (++_instance->_index >= TABLE_SIZE)
    _instance->_index = 0;
}

