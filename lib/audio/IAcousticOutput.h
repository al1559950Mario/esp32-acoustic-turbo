#pragma once

#include <Arduino.h>

// Minimal audio output interface used by AcousticInjector.
// Accepts 8-bit centered samples: 0..255, where 128 = 0.
class IAcousticOutput {
public:
  virtual ~IAcousticOutput() = default;

  // Optional initialization hook (useful for backends like I2S).
  virtual void begin(uint32_t /*sampleRateHz*/) {}

  // ISR-safe write
  virtual void writeFromISR(uint8_t sample8) = 0;

  // Task-context write
  virtual void write(uint8_t sample8) = 0;
};

