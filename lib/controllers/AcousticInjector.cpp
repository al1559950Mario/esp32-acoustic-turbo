#include "AcousticInjector.h"
#include "driver/dac.h"
#include "driver/timer.h"
#include <math.h>

volatile uint8_t dacPending = 128;
AcousticInjector* AcousticInjector::_instance = nullptr;

// ISR redireccionada para timer
void IRAM_ATTR onTimerForwarder() {
  if (AcousticInjector::_instance)
    AcousticInjector::_instance->onTimer();
}

// Tabla seno 16 muestras para ISR rápido (0-255)
const uint8_t AcousticInjector::_sineTable[AcousticInjector::TABLE_SIZE] = {
  128,176,218,245,255,245,218,176,
  128,80,38,11,1,11,38,80
};

void AcousticInjector::begin(uint8_t dacPin, uint8_t relayPin) {
  _instance = this;
  _dacPin = dacPin;
  _relayPin = relayPin;
  pinMode(_relayPin, OUTPUT);
  digitalWrite(_relayPin, LOW);

  _dacChannel = (_dacPin == 25) ? DAC_CHANNEL_1 : DAC_CHANNEL_2;
  dac_output_enable(_dacChannel);

  _timer = timerBegin(0, 80, true); // Prescaler 80 = 1us tick @80 MHz APB
  timerAttachInterrupt(_timer, &onTimerForwarder, true);
  timerAlarmWrite(_timer, 1000000 / SAMPLE_RATE, true); // 64kHz
  timerAlarmDisable(_timer);

  _level = 0.0f;
  _targetLevel = 0.0f;
  _index = 0;
}

void AcousticInjector::start(float level) {
  _targetLevel = constrain(level, 0.0f, 1.0f);
  _level = 0.0f;
  _index = 0;
  digitalWrite(_relayPin, HIGH);
  delay(10);

  timerAlarmEnable(_timer);

  // Primera muestra directa
  dacPending = _sineTable[_index];
  applyPendingDAC();
}

void AcousticInjector::stop() {
  timerAlarmDisable(_timer);
  digitalWrite(_relayPin, LOW);
  dac_output_voltage(_dacChannel, 128);
  _level = 0.0f;
  _targetLevel = 0.0f;
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
  int16_t out = 128 + (int16_t)(delta * _level);
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

// ISR timer
void IRAM_ATTR AcousticInjector::onTimer() {
  if (!_instance) return;

  // Modo tabla senoidal (simple)
  uint8_t raw = _instance->_sineTable[_instance->_index];
  dac_output_voltage(_instance->_dacChannel, raw);
  _instance->_index = (_instance->_index + 1) % TABLE_SIZE;
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
  Serial.println(F("🎼 Emitiendo señal resonante por fase acumulada (5s)..."));

  const float freq = 6370.0f;
  const float amplitude = 127.0f * constrain(level, 0.0f, 1.0f);
  const uint8_t bias = 128;
  const float sampleRate = 64000.0f;  // Hz
  const float dPhase = 2.0f * PI * freq / sampleRate;

  const int sampleCount = (int)(5.0f * sampleRate);  // 5 segundos de señal
  float phase = 0.0f;

  for (int i = 0; i < sampleCount; ++i) {
    float value = bias + amplitude * sinf(phase);
    dac_output_voltage(_dacChannel, constrain((int)value, 0, 255));
    phase += dPhase;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;
    delayMicroseconds(15);  // Aproximadamente 64kHz
  }

  dac_output_voltage(_dacChannel, bias);
  Serial.println(F("✅ Señal por fase acumulada finalizada."));
}
