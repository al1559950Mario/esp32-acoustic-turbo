#include "StateMachine.h"
#include <Arduino.h>  // Para Serial
#include <math.h>

namespace {

constexpr float kAlignBeamMin = 0.20f;
constexpr float kAlignBeamMax = 0.35f;
constexpr float kAlignBoostMax = 0.10f;
constexpr float kFlowGamma = 1.2f;
constexpr float kStableMapDelta = 2.0f;
constexpr float kStableTpsDelta = 2.0f;
constexpr unsigned long kFlowHoldMs = 400;
constexpr unsigned long kAlignTimeoutMs = 2000;
constexpr float kDecayStopLevel = 0.02f;

float clamp01(float value) {
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

float lerp(float a, float b, float t) {
  return a + (b - a) * t;
}

}  // namespace

void StateMachine::begin(bool hasCalibration, ActuatorManager* actuatorsPtr, ThresholdManager* thresholdManagerPtr) {
  current = SystemState::IDLE;

  actuators = actuatorsPtr;
  thresholdManager = thresholdManagerPtr;
  if (thresholdManager) {
    thresholds = thresholdManager->getThresholds();
  }

  stateEntryMs = millis();
  flowStableMs = 0;

  Serial.print(">> StateMachine iniciado en estado: ");
  Serial.println(static_cast<int>(current));
}

SystemState StateMachine::getState() const {
  return current;
}

void StateMachine::update(float mapLoadPercent,
                          float tpsLoadPercent,
                          bool serialCalibReq,
                          bool bleCalibReq,
                          bool calibLoaded,
                          const DebugManager &dbg) {
  
  (void)serialCalibReq;
  (void)bleCalibReq;
  (void)calibLoaded;
  (void)dbg;
  const float previousMapLoad = lastMapLoadPercent;
  const float previousTps = lastTpsPercent;
  lastMapLoadPercent = mapLoadPercent;

  // Actualizar umbrales dinámicamente
  if (thresholdManager) {
    thresholds = thresholdManager->getThresholds();
  }

  currentLevel = tpsLoadPercent / 100.0f;
  const float mapLevel = clamp01(mapLoadPercent / 100.0f);
  const float stableMapDelta = fabsf(mapLoadPercent - previousMapLoad);
  const float stableTpsDelta = fabsf(tpsLoadPercent - previousTps);
  lastTpsPercent = tpsLoadPercent;

  if (compatibilityMode) {
    switch (current) {
      case SystemState::IDLE:
        if (readyForInjection(tpsLoadPercent, mapLoadPercent)) {
          current = SystemState::ALIGN;
          stateEntryMs = millis();
          flowStableMs = 0;
          if (!actuators->isAcousticOn()) {
            actuators->startAcoustic(currentLevel);
          }
          Serial.println("→ Transición: IDLE → ALIGN");
        }
        break;

      case SystemState::ALIGN: {
        const bool flowLost = (tpsLoadPercent <= thresholds.INJ_TPS_OFF) ||
                              (mapLoadPercent <= thresholds.INJ_MAP_OFF);
        const bool vortexRequest = (tpsLoadPercent >= thresholds.VORTEX_TPS_ON) &&
                                   (mapLoadPercent >= thresholds.VORTEX_MAP_ON);

        if (vortexRequest) {
          if (flowStableMs == 0) {
            flowStableMs = millis();
          }
          if (millis() - flowStableMs >= kFlowHoldMs) {
            current = SystemState::FLOW;
            actuators->startVortex();
            Serial.println("→ Transición: ALIGN → FLOW");
          }
        } else {
          flowStableMs = 0;
        }

        if (flowLost) {
          current = SystemState::DECAY;
          stateEntryMs = millis();
          actuators->stopVortex();
          Serial.println("→ Transición: ALIGN → DECAY");
        }
        break;
      }

      case SystemState::FLOW: {
        const bool flowLost = (tpsLoadPercent <= thresholds.VORTEX_TPS_OFF) ||
                              (mapLoadPercent <= thresholds.INJ_MAP_OFF);
        const bool inconsistent = (stableMapDelta > kStableMapDelta) ||
                                  (stableTpsDelta > kStableTpsDelta);

        if (flowLost) {
          current = SystemState::DECAY;
          stateEntryMs = millis();
          actuators->stopVortex();
          Serial.println("→ Transición: FLOW → DECAY");
        } else if (inconsistent) {
          current = SystemState::ALIGN;
          stateEntryMs = millis();
          flowStableMs = 0;
          Serial.println("→ Transición: FLOW → ALIGN");
        }
        break;
      }

      case SystemState::DECAY: {
        const float beamLevel = actuators->getAcousticInjector().getLevel();
        if (beamLevel <= kDecayStopLevel) {
          current = SystemState::IDLE;
          actuators->stopAcoustic();
          actuators->stopVortex();
          Serial.println("→ Transición: DECAY → IDLE");
        }
        break;
      }
    }
    return;
  }

  switch (current) {

    case SystemState::IDLE:
      if (readyForInjection(tpsLoadPercent, mapLoadPercent)) {
        current = SystemState::ALIGN;
        stateEntryMs = millis();
        flowStableMs = 0;
        if (!actuators->isAcousticOn()) {
          const float alignLevel = lerp(kAlignBeamMin, kAlignBeamMax, mapLevel);
          actuators->startAcoustic(alignLevel);
        }
        Serial.println("→ Transición: IDLE → ALIGN");
      }
      break;

    case SystemState::ALIGN: {
      const bool flowLost = (tpsLoadPercent <= thresholds.INJ_TPS_OFF) ||
                            (mapLoadPercent <= thresholds.INJ_MAP_OFF);
      const bool inconsistent = (stableMapDelta > kStableMapDelta) ||
                                (stableTpsDelta > kStableTpsDelta);
      const bool laminar = readyForInjection(tpsLoadPercent, mapLoadPercent) && !inconsistent;

      if (laminar) {
        if (flowStableMs == 0) {
          flowStableMs = millis();
        }
        if (millis() - flowStableMs >= kFlowHoldMs) {
          current = SystemState::FLOW;
          Serial.println("→ Transición: ALIGN → FLOW");
        }
      } else {
        flowStableMs = 0;
      }

      if (flowLost || (millis() - stateEntryMs > kAlignTimeoutMs)) {
        current = SystemState::DECAY;
        stateEntryMs = millis();
        Serial.println("→ Transición: ALIGN → DECAY");
      }
      break;
    }

    case SystemState::FLOW: {
      const bool flowLost = (tpsLoadPercent <= thresholds.INJ_TPS_OFF) ||
                            (mapLoadPercent <= thresholds.INJ_MAP_OFF);
      const bool inconsistent = (stableMapDelta > kStableMapDelta) ||
                                (stableTpsDelta > kStableTpsDelta);

      if (flowLost) {
        current = SystemState::DECAY;
        stateEntryMs = millis();
        Serial.println("→ Transición: FLOW → DECAY");
      } else if (inconsistent) {
        current = SystemState::ALIGN;
        stateEntryMs = millis();
        flowStableMs = 0;
        Serial.println("→ Transición: FLOW → ALIGN");
      }
      break;
    }

    case SystemState::DECAY: {
      const float beamLevel = actuators->getAcousticInjector().getLevel();
      if (beamLevel <= kDecayStopLevel) {
        current = SystemState::IDLE;
        actuators->stopAcoustic();
        actuators->stopVortex();
        Serial.println("→ Transición: DECAY → IDLE");
      }
      break;
    }

  }
}

void StateMachine::handleActions() {
  const float mapLevel = clamp01(lastMapLoadPercent / 100.0f);

  if (compatibilityMode) {
    switch (current) {
      case SystemState::ALIGN:
        actuators->setAcousticParameters(currentLevel, lastMapLoadPercent);
        actuators->setVortexLevel(0.0f);
        actuators->update();
        break;
      case SystemState::FLOW:
        if (!actuators->isTurboOn()) {
          actuators->startVortex();
        }
        actuators->setAcousticParameters(currentLevel, lastMapLoadPercent);
        actuators->update();
        break;
      case SystemState::DECAY:
        actuators->setAcousticParameters(0.0f, lastMapLoadPercent);
        actuators->setVortexLevel(0.0f);
        actuators->update();
        break;
      case SystemState::IDLE:
        actuators->stopAll();
        break;
    }
    return;
  }

  switch (current) {
    case SystemState::ALIGN: {
      const float alignLevel = lerp(kAlignBeamMin, kAlignBeamMax, mapLevel);
      const float boostLevel = lerp(0.0f, kAlignBoostMax, mapLevel);
      actuators->setAcousticParameters(alignLevel, lastMapLoadPercent);
      actuators->setVortexLevel(boostLevel);
      actuators->update();
      break;
    }
    case SystemState::FLOW: {
      const float scaled = powf(mapLevel, kFlowGamma);
      const float beamLevel = clamp01(scaled);
      const float boostLevel = clamp01(scaled);
      actuators->setAcousticParameters(beamLevel, lastMapLoadPercent);
      actuators->setVortexLevel(boostLevel);
      actuators->update();
      break;
    }
    case SystemState::DECAY: {
      actuators->setAcousticParameters(0.0f, lastMapLoadPercent);
      actuators->setVortexLevel(0.0f);
      actuators->update();
      break;
    }
    case SystemState::IDLE:
      actuators->stopAll();
      break;
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    //Serial.printf("TPS: %.1f%% → Level: %.2f\n", currentLevel * 100.0f, getLevel());
  }
}

void StateMachine::debugForceState(SystemState nuevoEstado) {
  current = nuevoEstado;
  stateEntryMs = millis();
  flowStableMs = 0;
  Serial.print(">> Estado forzado a: ");
  Serial.println(static_cast<int>(nuevoEstado));
}

float StateMachine::getLevel() const {
  return currentLevel;
}

bool StateMachine::readyForInjection(float tps, float mapLoad) {
  return tps >= thresholds.INJ_TPS_ON && mapLoad >= thresholds.INJ_MAP_ON;
}

void StateMachine::setCompatibilityMode(bool enabled) {
  compatibilityMode = enabled;
}

bool StateMachine::isCompatibilityMode() const {
  return compatibilityMode;
}
