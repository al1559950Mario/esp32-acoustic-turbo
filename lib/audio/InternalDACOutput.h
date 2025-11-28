#pragma once

#include <Arduino.h>
#include "driver/dac.h"
#include "IAcousticOutput.h"

// Default backend that writes to ESP32 internal DAC (for compatibility during migration).
class InternalDACOutput : public IAcousticOutput {
public:
  void attachPin(uint8_t pin) {
    _pin = pin;
    _ch = (_pin == 25) ? DAC_CHANNEL_1 : DAC_CHANNEL_2;
  }

  void begin(uint32_t /*sampleRateHz*/) override {
    // Internal DAC needs only enable; sample rate is driven by caller's timer.
    dac_output_enable(_ch);
  }

  void writeFromISR(uint8_t sample8) override {
    dac_output_voltage(_ch, sample8);
  }

  void write(uint8_t sample8) override {
    dac_output_voltage(_ch, sample8);
  }

private:
  uint8_t _pin = 25;
  dac_channel_t _ch = DAC_CHANNEL_1;
};
