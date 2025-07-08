#include "AcousticInjector.h"
#include "driver/dac.h"
#include "driver/timer.h"

volatile uint8_t dacPending = 128;
AcousticInjector* AcousticInjector::_instance = nullptr;

// ISR redireccionada, fuera de la clase
void IRAM_ATTR onTimerForwarder() {
  if (AcousticInjector::_instance)
    AcousticInjector::_instance->onTimer();
}

/// Seno de 64 muestras (0–255)
const uint8_t AcousticInjector::_sineTable[TABLE_SIZE] = {
  128,140,152,164,175,186,196,205,214,222,229,235,240,244,246,248,
  249,248,246,244,240,235,229,222,214,205,196,186,175,164,152,140,
  128,115,103,91,80,69,59,50,41,33,26,20,15,11,9,7,
  6,7,9,11,15,20,26,33,41,50,59,69,80,91,103,115
};

void AcousticInjector::begin(uint8_t dacPin, uint8_t relayPin) {
  _instance = this;
  _dacPin = dacPin;
  _relayPin = relayPin;
  pinMode(_relayPin, OUTPUT);
  digitalWrite(_relayPin, LOW);

  _dacChannel = (_dacPin == 25) ? DAC_CHANNEL_1 : DAC_CHANNEL_2;
  dac_output_enable(_dacChannel);

  _timer = timerBegin(0, 80, true);
  timerAttachInterrupt(_timer, &onTimerForwarder, true);  // ✅ Correcto
  timerAlarmWrite(_timer, 1000000 / SAMPLE_RATE, true);
  timerAlarmDisable(_timer);

  _instance = this;
}

void AcousticInjector::start(float level) {
  _targetLevel = constrain(level, 0.0f, 1.0f);
  _level = 0.0f;
  _index = 0;
  digitalWrite(_relayPin, HIGH);
  delay(10);

  // Activamos el temporizador
  timerAlarmEnable(_timer);

  // Generamos manualmente la primera muestra
  dacPending = _sineTable[_index];
  applyPendingDAC();

  //Serial.printf("[DAC] start(level=%.2f), index=0, timer ENABLED\n", _targetLevel);
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
  int16_t out = 128 + (delta * _level);  // ← aquí sí puede usar float
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

volatile uint32_t isrTickCounter = 0;

void IRAM_ATTR AcousticInjector::onTimer() {
  if (!_instance) return;

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
  start(1.0f);  // Esto activa el temporizador y el ISR

  for (int i = 0; i < 250; i++) {
    update();
    delay(20);
    if (i % 50 == 0) {
        Serial.printf("Nivel actual: %.2f\n", _level);
      }
  }
  

  stop();            // Apaga temporizador, DAC y relé
  testRelay(false);

  Serial.println(F("✅ Prueba finalizada."));
  static uint32_t lastTick = 0;
  static uint32_t lastMillis = 0;

  if (millis() - lastMillis >= 1000) {
    Serial.printf("Ticks ISR/segundo: %lu\n", isrTickCounter - lastTick);
    lastTick = isrTickCounter;
    lastMillis = millis();
  }

}


void AcousticInjector::emitResonant(float level) {
  Serial.println(F("🎼 Emitiendo señal resonante por fase acumulada (5s)..."));

  const float freq = 6370.0f;
  const float amplitude = 127.0f * level;
  const uint8_t bias = 128;
  const float sampleRate = 64000.0f;         // Hz
  const float dPhase = 2.0f * PI * freq / sampleRate;

  const int sampleCount = (int)(5.0f * sampleRate);  // 5 segundos de señal
  float phase = 0.0f;

  for (int i = 0; i < sampleCount; ++i) {
    float value = bias + amplitude * sinf(phase);
    dac_output_voltage(_dacChannel, constrain((int)value, 0, 255));
    phase += dPhase;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;

    delayMicroseconds(15);  // Teóricamente ~64kHz
  }

  dac_output_voltage(_dacChannel, bias);
  Serial.println(F("✅ Señal por fase acumulada finalizada."));
}

