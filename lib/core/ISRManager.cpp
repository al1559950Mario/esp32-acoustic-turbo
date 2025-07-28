#include "ISRManager.h"

ISRManager* ISRManager::_instance = nullptr;

ISRManager* ISRManager::getInstance() {
  return _instance;
}

void ISRManager::begin(SensorManager* sensors, AcousticInjector* injector) {
  _instance = this;
  _sensors = sensors;
  _injector = injector;

  _timer = timerBegin(1, 80, true); // timer 1, prescaler 80 → 1us por tick
  if (_timer == nullptr) {
    return;
  }

  timerAttachInterrupt(_timer, &ISRManager::onTimerISR, true);
  timerAlarmWrite(_timer, 1000, true); // alarma cada 1000 us (1ms)
  timerAlarmDisable(_timer);
}


void ISRManager::start() {
  timerAlarmEnable(_timer);
}

void ISRManager::stop() {
  timerAlarmDisable(_timer);
}

uint16_t ISRManager::getCachedMAPRaw() {
  return cachedMAPRaw;
}

uint16_t ISRManager::getCachedTPSRaw() {
  return cachedTPSRaw;
}


void IRAM_ATTR ISRManager::onTimerISR() {
  if (!_instance || !_instance->_sensors) return;


  auto& mapSensor = _instance->_sensors->getMAP();
  auto& tpsSensor = _instance->_sensors->getTPS();

  // Lectura rápida raw ADC en ISR, sin funciones pesadas
  _instance->cachedMAPRaw = mapSensor.readRawISR();
  _instance->cachedTPSRaw = tpsSensor.readRawISR();
}


