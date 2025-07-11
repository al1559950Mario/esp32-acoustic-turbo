#include "AcousticInjector.h"
#include "driver/dac.h"
#include <math.h>

AcousticInjector* AcousticInjector::_instance = nullptr;

const uint8_t AcousticInjector::_sineTable[AcousticInjector::TABLE_SIZE] = {
  128, 176, 218, 245, 255, 245, 218, 176,
  128, 80, 38, 11, 1, 11, 38, 80
};

void AcousticInjector::begin(uint8_t dacPin, uint8_t relayPin) {
  _instance = this;

  _dacPin = dacPin;
  _relayPin = relayPin;
  pinMode(_relayPin, OUTPUT);
  digitalWrite(_relayPin, LOW);

  _dacChannel = (_dacPin == 25) ? DAC_CHANNEL_1 : DAC_CHANNEL_2;
  dac_output_enable(_dacChannel);

  _level = 0.0f;
  _targetLevel = 0.0f;
  _index = 0;
  _levelInt = 0;

  _timer = timerBegin(2, 80, true); // Timer 2, 1 MHz
  timerAttachInterrupt(_timer, &AcousticInjector::onTimer, true);
  timerAlarmWrite(_timer, 1000000 / SAMPLE_RATE, true); // 64kHz
  timerAlarmDisable(_timer);
}

void AcousticInjector::start(float level) {
  _targetLevel = constrain(level, 0.0f, 1.0f);
  _level = 0.0f;
  _levelInt = 0;
  _index = 0;

  digitalWrite(_relayPin, HIGH);
  delay(10);

  timerAlarmEnable(_timer);
}

void AcousticInjector::stop() {
  timerAlarmDisable(_timer);
  dac_output_voltage(_dacChannel, 128);
  digitalWrite(_relayPin, LOW);
  _level = 0.0f;
  _targetLevel = 0.0f;
  _levelInt = 0;
}

void AcousticInjector::setLevel(float level) {
  _targetLevel = constrain(level, 0.0f, 1.0f);
}

void AcousticInjector::update() {
  if (_level < _targetLevel)
    _level = min(_level + RAMP_STEP, _targetLevel);
  else if (_level > _targetLevel)
    _level = max(_level - RAMP_STEP, _targetLevel);

  _levelInt = (uint8_t)(_level * 255.0f);
}

void IRAM_ATTR AcousticInjector::onTimer() {
  if (!_instance) return;

  uint8_t raw = _instance->_sineTable[_instance->_index];
  int16_t delta = (int16_t)raw - 128;
  int16_t modulated = 128 + ((delta * _instance->_levelInt) >> 8);
  uint8_t output = constrain(modulated, 0, 255);

  dac_output_voltage(_instance->_dacChannel, output);
  _instance->_lastDACValue = output;

  _instance->_index = (_instance->_index + 1) % TABLE_SIZE;
}

void IRAM_ATTR AcousticInjector::applyPendingDAC() {
  uint8_t raw = _sineTable[_index];
  int16_t delta = (int16_t)raw - 128;
  int16_t modulated = 128 + ((delta * _levelInt) >> 8);
  uint8_t output = constrain(modulated, 0, 255);
  dac_output_voltage(_dacChannel, output);
  _lastDACValue = output;

  _index = (_index + 1) % TABLE_SIZE;
}

uint8_t AcousticInjector::getCurrentDAC() const {
  return _lastDACValue;
}

bool AcousticInjector::isActive() const {
  return digitalRead(_relayPin) == HIGH;
}

void AcousticInjector::testRelay(bool on) {
  digitalWrite(_relayPin, on ? HIGH : LOW);
  Serial.printf(">> Relé %s manualmente.\n", on ? "activado" : "desactivado");
}

bool AcousticInjector::isRelayActive() const {
  return digitalRead(_relayPin) == HIGH;
}

void AcousticInjector::test() {
  Serial.println(F("🔊 Prueba acústica iniciada..."));
  testRelay(true);
  start(1.0f);

  for (int i = 0; i < 250; i++) {
    update();
    delay(20);
    if (i % 50 == 0) {
      Serial.printf("Nivel actual: %.2f\n", _level);
    }
  }

  stop();
  testRelay(false);
  Serial.println(F("✅ Prueba finalizada."));
}

void AcousticInjector::emitResonant(float level) {
  Serial.println(F("🌼 Emitiendo señal resonante por fase acumulada (5s)..."));

  const float freq = 6370.0f;
  const float amplitude = 127.0f * constrain(level, 0.0f, 1.0f);
  const uint8_t bias = 128;
  const float sampleRate = 64000.0f;
  const float dPhase = 2.0f * PI * freq / sampleRate;

  const int sampleCount = (int)(5.0f * sampleRate);
  float phase = 0.0f;

  for (int i = 0; i < sampleCount; ++i) {
    float value = bias + amplitude * sinf(phase);
    dac_output_voltage(_dacChannel, constrain((int)value, 0, 255));
    phase += dPhase;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;
    delayMicroseconds(15);
  }

  dac_output_voltage(_dacChannel, bias);
  Serial.println(F("✅ Señal por fase acumulada finalizada."));
}

void AcousticInjector::testSimple() {
  Serial.println(F("🔊 Test simple iniciado"));
  testRelay(true);
  start(1.0f);

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    update();
    delay(20);
  }

  stop();
  testRelay(false);
  Serial.println(F("✅ Test simple finalizado"));
}
