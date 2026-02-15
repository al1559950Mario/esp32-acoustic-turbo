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
    BinResult bins[6];
  };

  static constexpr uint8_t kFreqCount = 3;

  bool run(SensorManager& sensors,
           ActuatorManager& actuators,
           ResonanceCalibrationReporter& reporter);

  bool hasResults() const { return resultsValid; }
  const FrequencyResult* getResults() const { return results; }

private:
  static constexpr float kFreqListHz[kFreqCount] = {5000.0f, 5250.0f, 5500.0f};
  static constexpr uint8_t kBinCount = 6;
  static constexpr float kMafMapMinPct = 20.0f;
  static constexpr float kMafMapMaxPct = 40.0f;
  static constexpr float kBinSizePct = (kMafMapMaxPct - kMafMapMinPct) / kBinCount;
  static constexpr uint8_t kAmpCount = 4;
  static constexpr float kAmpLevels[kAmpCount] = {0.3f, 0.5f, 0.7f, 1.0f};
  static constexpr uint32_t kFreqWarmupMs = 80;
  static constexpr uint32_t kLevelSettleMs = 120;
  static constexpr uint32_t kBaselineSampleMs = 420;
  static constexpr uint32_t kBinSampleMs = 420;
  static constexpr uint32_t kFreqCooldownMs = 20;
  static constexpr uint32_t kBinStabilityHoldMs = 260;
  static constexpr float kBinStabilityTolPct = 0.9f;
  static constexpr uint8_t kRepeatCount = 2;
  static constexpr float kRepeatSpreadPenalty = 0.20f;
  static constexpr uint8_t kConfirmRepeatCount = 1;
  static constexpr bool kEnableConfirmPass = false;
  static constexpr uint32_t kStableWaitMaxMs = 1500;
  static constexpr float kStrongImproveRatio = 0.15f;
  static constexpr float kLightImproveRatio = 0.05f;
  static constexpr float kMinBaseline = 0.01f;
  static constexpr float kMafRiseEpsPct = 0.25f;
  static constexpr float kMafPostBaselineRisePct = 0.8f;
  static constexpr uint32_t kMafPostBaselineTimeoutMs = 8000;
  static constexpr uint32_t kLivePrintPeriodMs = 120;
  static constexpr uint8_t kMetricMinSamplesHard = 6;
  static constexpr uint8_t kMetricMinSamplesTarget = 12;
  static constexpr uint32_t kMetricPrimeTimeoutMs = 450;
  static constexpr float kMetricMaxValidOsc = 12.0f;
  static constexpr float kMetricSpikeJumpFactor = 2.0f;
  static constexpr float kMetricSpikeJumpAbs = 1.5f;
  static constexpr uint8_t kMetricMinAcceptedPoints = 4;

  bool resultsValid = false;
  FrequencyResult results[kFreqCount];

  struct RepeatStats {
    float median = 0.0f;
    float min = 0.0f;
    float max = 0.0f;
    uint8_t validCount = 0;
    bool valid = false;
  };

  float sampleMetric(SensorManager& sensors,
                     ResonanceCalibrationReporter& reporter,
                     float freqHz,
                     float amplitude,
                     uint32_t durationMs,
                     const char* phase) const;
  float measureWithStreaming(SensorManager& sensors,
                             ActuatorManager& actuators,
                             ResonanceCalibrationReporter& reporter,
                             float freqHz,
                             float amplitude,
                             uint32_t durationMs,
                             const char* phase) const;
  uint32_t estimateBinTestTimeMs() const;
  bool waitForBinEntry(SensorManager& sensors,
                       ResonanceCalibrationReporter& reporter,
                       uint8_t binIndex,
                       float& lastAcceptedMaf) const;
  bool ensureStableInBin(SensorManager& sensors,
                         ResonanceCalibrationReporter& reporter,
                         uint8_t binIndex) const;
  bool waitForMafRiseAfterBaseline(SensorManager& sensors,
                                   ResonanceCalibrationReporter& reporter,
                                   uint8_t binIndex,
                                   float baselineMafPct) const;
  RepeatStats measureMedianWithRepeats(SensorManager& sensors,
                                       ActuatorManager& actuators,
                                       ResonanceCalibrationReporter& reporter,
                                       float freqHz,
                                       float amplitude,
                                       uint32_t durationMs,
                                       const char* phase,
                                       uint8_t binIndex,
                                       uint8_t repeats) const;
  bool confirmBestCandidateInBin(SensorManager& sensors,
                                 ActuatorManager& actuators,
                                 ResonanceCalibrationReporter& reporter,
                                 uint8_t binIndex,
                                 uint8_t bestFreqIdx) const;
  static const char* gradeText(Grade grade);
  String formatBinLine(uint8_t binIndex, float freq, float amp, Grade grade, float ratio) const;
  uint8_t findBestFreqForBin(uint8_t binIndex) const;
};
