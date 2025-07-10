#include "ISRManager.h"

ISRManager* ISRManager::_instance = nullptr;

void ISRManager::begin(SensorManager* sensors, AcousticInjector* injector) {
  _instance = this;
  _sensors = sensors;
  _injector = injector;

  _timer = timerBegin(1, 80, true); // 1 µs por tick
  timerAttachInterrupt(_timer, &ISRManager::onTimerISR, true);
  timerAlarmWrite(_timer, 1000, true); // Cada 1 ms
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

  // Acceso directo al sensor, sin usar funciones complejas
  //MAPSensor& map = _instance->_sensors->getMAP();
  //TPSSensor& tps = _instance->_sensors->getTPS();

  //_instance->cachedMAPRaw = map.readRaw();  // Solo ADC
  //_instance->cachedTPSRaw = tps.readRaw();

  // DAC seguro si está en IRAM
  //if (_instance->_injector && _instance->_injector->isActive()) {
  //  _instance->_injector->applyPendingDAC();  // Solo si está marcada como IRAM_ATTR
  //}
}
