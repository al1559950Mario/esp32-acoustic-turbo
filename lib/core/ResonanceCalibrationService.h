#pragma once

#include <Arduino.h>
#include "SensorManager.h"
#include "ActuatorManager.h"

class ResonanceCalibrationReporter {
public:
  virtual ~ResonanceCalibrationReporter() = default;
  virtual void println(const String& msg) = 0;
};

class ResonanceCalibrationService {
public:
  enum class Grade : uint8_t {
    NONE = 0,
    LIGHT = 1,
    STRONG = 2
  };

  struct BinResult {
    Grade grade = Grade::NONE;
    float amplitude = 0.0f;
    float improvement = 0.0f;
    bool measured = false;
  };

  struct FrequencyResult {
    float freqHz = 0.0f;
    BinResult bins[10];
  };

  static constexpr uint8_t kFreqCount = 3;

  bool run(SensorManager& sensors,
           ActuatorManager& actuators,
           ResonanceCalibrationReporter& reporter);

  bool hasResults() const { return resultsValid; }
  const FrequencyResult* getResults() const { return results; }

private:
  static constexpr float kFreqListHz[kFreqCount] = {4000.0f, 5250.0f, 6500.0f};
  static constexpr float kAmpLevels[3] = {0.3f, 0.5f, 0.7f};

  static constexpr uint8_t kBinCount = 10;
  static constexpr float kBinSizePct = 100.0f / kBinCount;
  static constexpr uint32_t kRampDurationMs = 5000;
  static constexpr uint32_t kBaselineSampleMs = 250;
  static constexpr uint32_t kBinSampleMs = 120;
  static constexpr float kStrongImproveRatio = 0.15f;
  static constexpr float kLightImproveRatio = 0.05f;
  static constexpr float kMinBaseline = 0.01f;

  bool resultsValid = false;
  FrequencyResult results[kFreqCount];

  float sampleMetric(SensorManager& sensors, uint32_t durationMs) const;
  static const char* gradeText(Grade grade);
  String formatBinLine(uint8_t binIndex, float amp, Grade grade) const;
};

