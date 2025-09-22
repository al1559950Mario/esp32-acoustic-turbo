#include "AcousticInjector.h"
#include "driver/dac.h"
#include <math.h>

AcousticInjector* AcousticInjector::_instance = nullptr;

// Nuevo:
uint8_t AcousticInjector::_sineTable[AcousticInjector::TABLE_SIZE] = {
  128, 140, 153, 165, 177, 188, 198, 207,
  215, 222, 227, 231, 234, 235, 235, 234,
  231, 227, 222, 215, 207, 198, 188, 177,
  165, 153, 140, 128, 115, 102,  90,  78,
   67,  57,  48,  40,  33,  28,  24,  21,
   20,  20,  21,  24,  28,  33,  40,  48,
   57,  67,  78,  90, 102, 115, 128, 140,
  153, 165, 177, 188, 198, 207, 215, 222
};

void AcousticInjector::begin(uint8_t dacPin) {
  _instance = this;

  _dacPin = dacPin;

  _dacChannel = (_dacPin == 25) ? DAC_CHANNEL_1 : DAC_CHANNEL_2;
  dac_output_enable(_dacChannel);

  _level = 0.0f;
  _targetLevel = 0.0f;
  _index = 0;
  _levelInt = 0;

  // construir tabla seno en RAM
  for (int i = 0; i < TABLE_SIZE; ++i) {
    float s = sinf(2.0f * PI * (float)i / (float)TABLE_SIZE);
    int v = (int)roundf(128.0f + s * 127.0f);
    v = constrain(v, 0, 255);
    _sineTable[i] = (uint8_t)v;
  }

  // configurar timer a sampleRate fijo (counts = us porque prescaler 80 -> 1MHz)
  _timer = timerBegin(2, 80, true); // Timer 2, 1 MHz
  timerAttachInterrupt(_timer, &AcousticInjector::onTimer, true);

  float sampleRate = DEFAULT_SAMPLE_RATE; // p.ej. 64kHz
  float periodPerSample = 1e6f / sampleRate; // μs
  timerAlarmWrite(_timer, static_cast<uint32_t>(periodPerSample), true);
  timerAlarmDisable(_timer);

  // init phase values
  _phaseAcc = 0;
  _phaseStep = 0;
  _levelInt = (uint8_t)(_level * 255.0f);

}


void AcousticInjector::start(float level) {
  _targetLevel = constrain(level, 0.0f, 1.0f);

  // Antes:
  // _level = 0.0f;
  // _levelInt = 0;
  // Ahora: arranca en el nivel objetivo solicitado (ej. 0.1f)
  _level = _targetLevel;
  _levelInt = (uint8_t)(_level * 255.0f);

  _index = 0;
  delay(10);
  timerAlarmEnable(_timer);
}



void AcousticInjector::stop() {
  timerAlarmDisable(_timer);
  dac_output_voltage(_dacChannel, 128);
  _level = 0.0f;
  _targetLevel = 0.0f;
  _levelInt = 0;
}

void AcousticInjector::setLevel(float level) {
  
  _targetLevel = constrain(level, 0.0f, 1.0f);
}

void AcousticInjector::update() {
    // Suavizado nivel 
    float diffLevel = _targetLevel - _level;
    if (fabs(diffLevel) < RAMP_STEP)
        _level = _targetLevel;
    else
        _level += (diffLevel > 0 ? RAMP_STEP : -RAMP_STEP);
    _levelInt = (uint8_t)(_level * 255.0f);

    // Suavizado frecuencia (lineal o logarítmico)
    if (fabs(_targetFrequency - _currentFrequency) > 0.5f) { // umbral para evitar "bailoteos"
        // Aquí puedes hacer un paso pequeño hacia la meta
        float stepFreq = 10.0f; // Hz por llamada, ajustar según sensibilidad

        if (_targetFrequency > _currentFrequency)
            _currentFrequency += stepFreq;
        else
            _currentFrequency -= stepFreq;

        // Evitar sobrepasar
        if ((_targetFrequency - _currentFrequency) * stepFreq < 0)
            _currentFrequency = _targetFrequency;

        updateWaveFrequency(_currentFrequency);
    }
}




void IRAM_ATTR AcousticInjector::onTimer() {
  if (!_instance) return;

  _instance->_phaseAcc += _instance->_phaseStep;

  // Índice entero y siguiente para interpolar
  uint32_t idx = (_instance->_phaseAcc >> PHASE_FRAC) & (TABLE_SIZE - 1);
  uint32_t nextIdx = (idx + 1) & (TABLE_SIZE - 1);

  // Fracción para interpolar
  uint32_t frac = _instance->_phaseAcc & ((1ULL << PHASE_FRAC) - 1);

  uint8_t sample1 = _instance->_sineTable[idx];
  uint8_t sample2 = _instance->_sineTable[nextIdx];

  int16_t delta = (int16_t)sample2 - (int16_t)sample1;
  uint16_t interp = (uint16_t)sample1 + ((delta * frac) >> PHASE_FRAC);

  // Modulación de nivel (0-255)
  int16_t centered = (int16_t)interp - 128;
  int16_t modulated = 128 + ((centered * _instance->_levelInt) >> 8);
  uint8_t output = (uint8_t)constrain(modulated, 0, 255);

  dac_output_voltage(_instance->_dacChannel, output);
  _instance->_lastDACValue = output;
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

}


void AcousticInjector::test() {
  Serial.println(F("🔊 Prueba acústica iniciada..."));
  start(1.0f);

  for (int i = 0; i < 250; i++) {
    update();
    delay(20);
    if (i % 50 == 0) {
      Serial.printf("Nivel actual: %.2f\n", _level);
    }
  }

  stop();
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
  start(1.0f);

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    update();
    delay(20);
  }

  stop();
  Serial.println(F("✅ Test simple finalizado"));
}

float AcousticInjector::mapLoadToWaveFrequency(float percent) {
    percent = constrain(percent, 0.0f, 100.0f);

    float logMin = logf(_freqMin);
    float logMax = logf(_freqMax);
    float logFreq = logMin + (percent / 100.0f) * (logMax - logMin);

    return expf(logFreq);
}


void AcousticInjector::updateWaveFrequency(float freqHz) {
    if (!_timer) return;

    // sampleRate fijo
    const float sampleRate = DEFAULT_SAMPLE_RATE;
    // suavizado perceptual (logarítmico)
    float targetFreq = freqHz;
    float smoothedFreq = powf(10.0f, 0.1f * log10f(_currentFrequency) + 0.9f * log10f(targetFreq));
    // calcular step en fixed-point: step = freqHz * TABLE_SIZE / sampleRate
    // representado en (1<<PHASE_FRAC) fraccional
    double step = (double)smoothedFreq * (double)TABLE_SIZE * (double)(1ULL << PHASE_FRAC) / (double)sampleRate;
    uint32_t newStep = (uint32_t)round(step);

    // garantizar que no sea cero (para frecuencias muy bajas)
    if (newStep == 0) newStep = 1;

    _currentFrequency = smoothedFreq;

    _phaseStep = newStep;

    float periodPerSample = 1e6f / sampleRate;
    timerAlarmWrite(_timer, static_cast<uint32_t>(periodPerSample), true);
}

void AcousticInjector::setFrequencyRangeOption(FrequencyRangeOption option) {
    _freqOption = option;

    switch(option) {
        case RANGE_1:
            _freqMin = 4400.0f;
            _freqMax = 5100.0f;
            break;
        case RANGE_2:
            _freqMin = 5100.0f;
            _freqMax = 5800.0f;
            break;
        case RANGE_3:
            _freqMin = 5800.0f;
            _freqMax = 6500.0f;
            break;
        case RANGE_4:
            _freqMin = 4400.0f;
            _freqMax = 6500.0f;
            break;            
        default:
            _freqMin = 5800.0f;
            _freqMax = 6500.0f;
            break;
    }
}
