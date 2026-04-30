#include "ResonanceCalibrationService.h"
#include <math.h>

constexpr float ResonanceCalibrationService::kFreqListHz[ResonanceCalibrationService::kFreqCount];
constexpr float ResonanceCalibrationService::kAmpLevels[4];

namespace {
void sortSmall(float* values, uint8_t count) {
  for (uint8_t i = 0; i + 1 < count; ++i) {
    uint8_t minIdx = i;
    for (uint8_t j = i + 1; j < count; ++j) {
      if (values[j] < values[minIdx]) {
        minIdx = j;
      }
    }
    if (minIdx != i) {
      float tmp = values[i];
      values[i] = values[minIdx];
      values[minIdx] = tmp;
    }
  }
}
}  // namespace

float ResonanceCalibrationService::sampleMetric(SensorManager& sensors,
                                              ResonanceCalibrationReporter& reporter,
                                              float freqHz,
                                              float amplitude,
                                              uint32_t durationMs,
                                              const char* phase) const {
  // Aísla cada medición (baseline/test) para evitar arrastrar historial
  // de bins o corridas anteriores.
  sensors.resetPressureMetrics();

  // Espera breve para que se llene un mínimo de muestras de presión antes
  // de evaluar oscilación; evita métricas casi cero por ventana vacía.
  const uint32_t primeStart = millis();
  while (sensors.getPressureSampleCount() < kMetricMinSamplesHard &&
         (millis() - primeStart) < kMetricPrimeTimeoutMs) {
    delay(5);
  }

  const size_t primedSamples = sensors.getPressureSampleCount();
  if (primedSamples < kMetricMinSamplesHard) {
    reporter.println("[LIVE]" + String(phase) + " | MUESTRAS INSUFICIENTES n=" +
                     String((int)primedSamples) +
                     " (min-hard=" + String((int)kMetricMinSamplesHard) + ")");
    return NAN;
  }

  if (primedSamples < kMetricMinSamplesTarget) {
    reporter.println("[LIVE]" + String(phase) + " | AVISO n=" + String((int)primedSamples) +
                     " (target=" + String((int)kMetricMinSamplesTarget) + ")");
  }

  const uint32_t start = millis();
  uint32_t lastPrintMs = 0;
  float lastAcceptedMetric = -1.0f;
  float metrics[48] = {0.0f};
  uint8_t metricCount = 0;

  while (millis() - start < durationMs) {
    const float metric = sensors.computeOscillationAmplitudeWindow(durationMs, 10.0f);

    bool accepted = false;
    if (std::isfinite(metric) && metric >= 0.0f && metric <= kMetricMaxValidOsc) {
      if (lastAcceptedMetric < 0.0f) {
        accepted = true;
      } else {
        const float delta = fabsf(metric - lastAcceptedMetric);
        const float jumpLimit = kMetricSpikeJumpFactor * lastAcceptedMetric;
        accepted = (delta <= jumpLimit) || (delta <= kMetricSpikeJumpAbs);
      }
    }

    if (accepted) {
      if (metricCount < 48) {
        metrics[metricCount++] = metric;
      }
      lastAcceptedMetric = metric;
    }

    const uint32_t now = millis();
    if ((now - lastPrintMs) >= kLivePrintPeriodMs) {
      lastPrintMs = now;
      float mafPct = sensors.readMAFLoadPercent();
      if (mafPct < 0.0f) mafPct = 0.0f;
      if (mafPct > 100.0f) mafPct = 100.0f;

      const float shownMetric = accepted ? metric : lastAcceptedMetric;
      reporter.println(String("[LIVE]") + phase +
                       " | MAF=" + String(mafPct, 1) + "%" +
                       " | freq=" + String(freqHz, 0) + " Hz" +
                       " | level=" + String(amplitude * 100.0f, 0) + "%" +
                       " | osc=" + String(shownMetric >= 0.0f ? shownMetric : metric, 3) +
                       " | n=" + String((int)sensors.getPressureSampleCount()));
    }

    delay(10);
  }

  if (metricCount < kMetricMinAcceptedPoints) {
    reporter.println("[LIVE]" + String(phase) +
                     " | MUESTRA OSC INESTABLE (pts=" + String((int)metricCount) + ")");
    return NAN;
  }

  sortSmall(metrics, metricCount);
  if ((metricCount % 2) == 0) {
    const uint8_t hi = metricCount / 2;
    const uint8_t lo = hi - 1;
    return (metrics[lo] + metrics[hi]) * 0.5f;
  }
  return metrics[metricCount / 2];
}

float ResonanceCalibrationService::measureWithStreaming(SensorManager& sensors,
                                                        ActuatorManager& actuators,
                                                        ResonanceCalibrationReporter& reporter,
                                                        float freqHz,
                                                        float amplitude,
                                                        uint32_t durationMs,
                                                        const char* phase) const {
  actuators.setISRSineLevel(amplitude);
  delay(kLevelSettleMs);
  return sampleMetric(sensors, reporter, freqHz, amplitude, durationMs, phase);
}

uint32_t ResonanceCalibrationService::estimateBinTestTimeMs() const {
  const uint32_t perFreqMs = kFreqWarmupMs +
                             (kRepeatCount * (kLevelSettleMs + kBaselineSampleMs)) +
                             (kAmpCount * kRepeatCount * (kLevelSettleMs + kBinSampleMs)) +
                             kFreqCooldownMs;

  uint32_t confirmMs = 0;
  if (kEnableConfirmPass) {
    confirmMs = kFreqWarmupMs +
                (kConfirmRepeatCount * (kLevelSettleMs + kBaselineSampleMs)) +
                (kConfirmRepeatCount * (kLevelSettleMs + kBinSampleMs)) +
                kFreqCooldownMs;
  }

  return (perFreqMs * kFreqCount) + confirmMs;
}

bool ResonanceCalibrationService::ensureStableInBin(SensorManager& sensors,
                                                    ResonanceCalibrationReporter& reporter,
                                                    uint8_t binIndex) const {
  const float binStart = kMafMapMinPct + (float(binIndex) * kBinSizePct);
  const float binEnd = kMafMapMinPct + (float(binIndex + 1) * kBinSizePct);
  const float stableMin = binStart + kBinStabilityTolPct;
  const float stableMax = binEnd - kBinStabilityTolPct;

  if (stableMax <= stableMin) {
    return true;
  }

  const uint32_t start = millis();
  uint32_t stableSince = 0;
  uint32_t lastMsgMs = 0;

  while (millis() - start < kStableWaitMaxMs) {
    float mafPct = sensors.readMAFLoadPercent();
    if (mafPct < 0.0f) mafPct = 0.0f;
    if (mafPct > 100.0f) mafPct = 100.0f;

    const bool inStableBand = (mafPct >= stableMin && mafPct <= stableMax);
    if (inStableBand) {
      if (stableSince == 0) stableSince = millis();
      if ((millis() - stableSince) >= kBinStabilityHoldMs) {
        return true;
      }
    } else {
      stableSince = 0;
      const uint32_t now = millis();
      if (now - lastMsgMs > 1000) {
        lastMsgMs = now;
        reporter.println("[MAPEO] Ajusta pedal para estabilizar bin " + String(binIndex + 1) +
                         " (" + String((int)binStart) + "-" + String((int)binEnd) +
                         "%). MAF=" + String(mafPct, 1) + "%");
      }
    }
    delay(25);
  }

  reporter.println("[MAPEO] Aviso: no se estabilizó totalmente el bin " + String(binIndex + 1) +
                   ", se continúa con la medición");
  return false;
}

bool ResonanceCalibrationService::waitForMafRiseAfterBaseline(SensorManager& sensors,
                                                              ResonanceCalibrationReporter& reporter,
                                                              uint8_t binIndex,
                                                              float baselineMafPct) const {
  const float binStart = kMafMapMinPct + (float(binIndex) * kBinSizePct);
  const float binEnd = kMafMapMinPct + (float(binIndex + 1) * kBinSizePct);
  float baselineInBin = baselineMafPct;
  if (baselineInBin < binStart) baselineInBin = binStart;
  if (baselineInBin > binEnd) baselineInBin = binEnd;

  float targetMaf = baselineInBin + kMafPostBaselineRisePct;
  if (targetMaf > binEnd) {
    targetMaf = binEnd;
  }

  const uint32_t start = millis();
  uint32_t lastMsgMs = 0;
  while ((millis() - start) < kMafPostBaselineTimeoutMs) {
    float mafPct = sensors.readMAFLoadPercent();
    if (mafPct < 0.0f) mafPct = 0.0f;
    if (mafPct > 100.0f) mafPct = 100.0f;

    const bool insideOrSlightlyAboveBin = (mafPct >= binStart && mafPct <= (binEnd + kMafRiseEpsPct));
    if (insideOrSlightlyAboveBin && mafPct >= targetMaf) {
      return true;
    }

    const uint32_t now = millis();
    if (now - lastMsgMs > 1000) {
      lastMsgMs = now;
      reporter.println("[MAPEO] Esperando subida de MAF tras baseline en bin " + String(binIndex + 1) +
                       ": baseline=" + String(baselineMafPct, 1) + "% -> objetivo=" +
                       String(targetMaf, 1) + "% | actual=" + String(mafPct, 1) + "%");
      if (mafPct < targetMaf) {
        reporter.println("Acelera suavemente para continuar pruebas");
      } else if (mafPct > (binEnd + kMafRiseEpsPct)) {
        reporter.println("Suelta un poco el pedal: te saliste del bin");
      } else {
        reporter.println("Mantén pedal estable para continuar pruebas");
      }
    }
    delay(40);
  }

  reporter.println("[MAPEO] Aviso: timeout esperando subida post-baseline, se continúa para evitar bloqueo");
  return false;
}

ResonanceCalibrationService::RepeatStats ResonanceCalibrationService::measureMedianWithRepeats(
    SensorManager& sensors,
    ActuatorManager& actuators,
    ResonanceCalibrationReporter& reporter,
    float freqHz,
    float amplitude,
    uint32_t durationMs,
    const char* phase,
    uint8_t binIndex,
    uint8_t repeats) const {
  if (repeats == 0) {
    return {};
  }

  float values[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  const uint8_t cappedRepeats = repeats > 5 ? 5 : repeats;
  uint8_t validCount = 0;

  ensureStableInBin(sensors, reporter, binIndex);
  for (uint8_t r = 0; r < cappedRepeats; ++r) {
    const float m = measureWithStreaming(sensors, actuators, reporter, freqHz, amplitude, durationMs, phase);
    if (!isnan(m)) {
      values[validCount++] = m;
    }
  }

  RepeatStats stats;
  stats.validCount = validCount;
  if (validCount == 0) {
    reporter.println("[MAPEO] Sin muestras válidas para " + String(phase) +
                     " | freq=" + String(freqHz, 0) + " Hz | level=" +
                     String(amplitude * 100.0f, 0) + "%");
    return stats;
  }

  sortSmall(values, validCount);

  stats.min = values[0];
  stats.max = values[validCount - 1];
  if ((validCount % 2) == 0) {
    const uint8_t hi = validCount / 2;
    const uint8_t lo = hi - 1;
    stats.median = (values[lo] + values[hi]) * 0.5f;
  } else {
    stats.median = values[validCount / 2];
  }
  stats.valid = true;
  return stats;
}

bool ResonanceCalibrationService::confirmBestCandidateInBin(SensorManager& sensors,
                                                            ActuatorManager& actuators,
                                                            ResonanceCalibrationReporter& reporter,
                                                            uint8_t binIndex,
                                                            uint8_t bestFreqIdx) const {
  const float freqHz = results[bestFreqIdx].freqHz;
  const BinResult& bestBin = results[bestFreqIdx].bins[binIndex];

  actuators.startISRSine(static_cast<uint32_t>(freqHz), 0.0f);
  delay(kFreqWarmupMs);

  const RepeatStats baselineStats = measureMedianWithRepeats(
      sensors, actuators, reporter, freqHz, 0.0f, kBaselineSampleMs, " confirm-baseline", binIndex,
      kConfirmRepeatCount);
  if (!baselineStats.valid) {
    actuators.stopISRSine();
    delay(kFreqCooldownMs);
    return false;
  }

  float baselineMafPct = sensors.readMAFLoadPercent();
  if (baselineMafPct < 0.0f) baselineMafPct = 0.0f;
  if (baselineMafPct > 100.0f) baselineMafPct = 100.0f;
  waitForMafRiseAfterBaseline(sensors, reporter, binIndex, baselineMafPct);

  const RepeatStats testStats = measureMedianWithRepeats(
      sensors, actuators, reporter, freqHz, bestBin.amplitude, kBinSampleMs, " confirm-test", binIndex,
      kConfirmRepeatCount);
  if (!testStats.valid) {
    actuators.stopISRSine();
    delay(kFreqCooldownMs);
    return false;
  }

  actuators.stopISRSine();
  delay(kFreqCooldownMs);

  float denom = fabsf(baselineStats.median);
  if (denom < kMinBaseline) denom = kMinBaseline;
  const float confirmRatio = (testStats.median - baselineStats.median) / denom;

  reporter.println("[MAPEO] Confirmación bin " + String(binIndex + 1) +
                   " | ratio=" + String(confirmRatio * 100.0f, 1) + "%");
  return confirmRatio >= kLightImproveRatio;
}

bool ResonanceCalibrationService::waitForBinEntry(SensorManager& sensors,
                                                  ResonanceCalibrationReporter& reporter,
                                                  uint8_t binIndex,
                                                  float& lastAcceptedMaf) const {
  const float binStart = kMafMapMinPct + (float(binIndex) * kBinSizePct);
  const float binEnd = kMafMapMinPct + (float(binIndex + 1) * kBinSizePct);
  uint32_t lastMsgMs = 0;
  while (true) {
    float mafPct = sensors.readMAFLoadPercent();
    if (mafPct < 0.0f) mafPct = 0.0f;
    if (mafPct > 100.0f) mafPct = 100.0f;

    const bool mafRising = (mafPct > (lastAcceptedMaf + kMafRiseEpsPct));
    if (mafPct >= binStart && mafRising) {
      lastAcceptedMaf = mafPct;
      return true;
    }

    const uint32_t now = millis();
    if (now - lastMsgMs > 1200) {
      lastMsgMs = now;
      reporter.println("[MAPEO] Esperando subida de MAF para entrar a bin " + String(binIndex + 1) +
                       " (" + String((int)binStart) + "-" + String((int)binEnd) +
                       "%). MAF actual=" + String(mafPct, 1) + "%");
      reporter.println("Acelera MUY LENTO (sin saltos)");
    }
    delay(40);
  }
}

const char* ResonanceCalibrationService::gradeText(Grade grade) {
  switch (grade) {
    case Grade::STRONG: return "VERDE";
    case Grade::LIGHT: return "AMARILLO";
    default: return "ROJO";
  }
}

String ResonanceCalibrationService::formatBinLine(uint8_t binIndex,
                                                  float freq,
                                                  float amp,
                                                  Grade grade,
                                                  float ratio) const {
  const uint8_t binStart = static_cast<uint8_t>(kMafMapMinPct + (binIndex * kBinSizePct));
  const uint8_t binEnd = static_cast<uint8_t>(kMafMapMinPct + ((binIndex + 1) * kBinSizePct));
  String line = "Bin ";
  line += String(binIndex + 1);
  line += " (";
  line += String(binStart);
  line += "-";
  line += String(binEnd);
  line += "%) | ";
  line += String(freq, 0);
  line += " Hz | amp ";
  line += String(amp * 100.0f, 0);
  line += "% | imp ";
  line += String(ratio * 100.0f, 1);
  line += "% -> ";
  line += gradeText(grade);
  return line;
}

uint8_t ResonanceCalibrationService::findBestFreqForBin(uint8_t binIndex) const {
  uint8_t bestFreq = 0;
  Grade bestGrade = Grade::NONE;
  float bestImprovement = -9999.0f;
  for (uint8_t freqIndex = 0; freqIndex < kFreqCount; ++freqIndex) {
    const BinResult& b = results[freqIndex].bins[binIndex];
    if (!b.measured) continue;
    const bool betterGrade = static_cast<uint8_t>(b.grade) > static_cast<uint8_t>(bestGrade);
    const bool sameGradeBetterRatio =
        (static_cast<uint8_t>(b.grade) == static_cast<uint8_t>(bestGrade) &&
         b.improvement > bestImprovement);
    if (betterGrade || sameGradeBetterRatio) {
      bestGrade = b.grade;
      bestImprovement = b.improvement;
      bestFreq = freqIndex;
    }
  }
  return bestFreq;
}

bool ResonanceCalibrationService::run(SensorManager& sensors,
                                      ActuatorManager& actuators,
                                      ResonanceCalibrationReporter& reporter) {
  resultsValid = false;

  for (uint8_t freqIndex = 0; freqIndex < kFreqCount; ++freqIndex) {
    results[freqIndex].freqHz = kFreqListHz[freqIndex];
    for (uint8_t bin = 0; bin < kBinCount; ++bin) results[freqIndex].bins[bin] = {};
  }

  reporter.println("\n=== Calibración de resonancia (mapeo por bin MAF) ===");
  reporter.println("Se prueban 3 frecuencias dentro de CADA bin de MAF.");
  reporter.println("Mapeo de bins limitado a " + String(kMafMapMinPct, 0) + "-" +
                   String(kMafMapMaxPct, 0) + "% de MAF.");
  reporter.println("Amplitud acústica de prueba: 30-100%");
  reporter.println("Avance post-baseline: requiere +" + String(kMafPostBaselineRisePct, 1) + "% MAF");
  reporter.println("Repeticiones por punto: " + String(kRepeatCount) +
                   " | confirmación final: " + String(kEnableConfirmPass ? "ON" : "OFF"));
  reporter.println("Acelera MUY LENTO");
  reporter.println("Mantén la rampa suave");
  reporter.println("No subas el MAF de golpe");
  reporter.println("Tiempo de prueba por bin (sin pedal): ~" +
                   String(estimateBinTestTimeMs() / 1000.0f, 1) + " s");

  float lastAcceptedMaf = -1.0f;

  for (uint8_t binIndex = 0; binIndex < kBinCount; ++binIndex) {
    if (!waitForBinEntry(sensors, reporter, binIndex, lastAcceptedMaf)) {
      return false;
    }

    const uint8_t binStart = static_cast<uint8_t>(kMafMapMinPct + (binIndex * kBinSizePct));
    const uint8_t binEnd = static_cast<uint8_t>(kMafMapMinPct + ((binIndex + 1) * kBinSizePct));
    reporter.println("\n[MAPEO] Bin activo " + String(binIndex + 1) + " (" + String((int)binStart) +
                     "-" + String((int)binEnd) + "%) | MAF=" + String(lastAcceptedMaf, 1) + "%");

    for (uint8_t freqIndex = 0; freqIndex < kFreqCount; ++freqIndex) {
      const float freqHz = kFreqListHz[freqIndex];
      actuators.startISRSine(static_cast<uint32_t>(freqHz), 0.0f);
      delay(kFreqWarmupMs);

      const RepeatStats baselineStats = measureMedianWithRepeats(
          sensors, actuators, reporter, freqHz, 0.0f, kBaselineSampleMs, " baseline", binIndex,
          kRepeatCount);
      if (!baselineStats.valid) {
        reporter.println("[MAPEO] Baseline inválido; se marca frecuencia como ROJO en este bin");
        BinResult& bin = results[freqIndex].bins[binIndex];
        bin.grade = Grade::NONE;
        bin.amplitude = kAmpLevels[0];
        bin.improvement = -9999.0f;
        bin.measured = true;
        actuators.stopISRSine();
        delay(kFreqCooldownMs);
        continue;
      }
      const float baseline = baselineStats.median;
      float baselineMafPct = sensors.readMAFLoadPercent();
      if (baselineMafPct < 0.0f) baselineMafPct = 0.0f;
      if (baselineMafPct > 100.0f) baselineMafPct = 100.0f;
      reporter.println("[MAPEO] Esperando avance post-baseline (sin acústico: nivel 0%)");
      const bool postBaselineAdvanced =
          waitForMafRiseAfterBaseline(sensors, reporter, binIndex, baselineMafPct);
      if (!postBaselineAdvanced) {
        reporter.println("[MAPEO] Continuando sin bloqueo por timeout post-baseline");
      }

      Grade bestGrade = Grade::NONE;
      float bestAmp = kAmpLevels[0];
      float bestRatio = -9999.0f;

      for (uint8_t ampIndex = 0; ampIndex < kAmpCount; ++ampIndex) {
        const float amp = kAmpLevels[ampIndex];
        reporter.println("[INJ] freq=" + String(freqHz, 0) + " Hz | level=" + String(amp * 100.0f, 0) + "%");

        const RepeatStats testStats = measureMedianWithRepeats(
            sensors, actuators, reporter, freqHz, amp, kBinSampleMs, " test", binIndex, kRepeatCount);
        if (!testStats.valid) {
          reporter.println("[INJ] medición inválida por falta de muestras; se omite este nivel");
          continue;
        }
        const float measured = testStats.median;
        float denom = fabsf(baseline);
        if (denom < kMinBaseline) denom = kMinBaseline;
        const float rawRatio = (measured - baseline) / denom;

        float spreadDenom = fabsf(measured);
        if (spreadDenom < kMinBaseline) spreadDenom = kMinBaseline;
        const float spreadRatio = (testStats.max - testStats.min) / spreadDenom;
        const float ratio = rawRatio - (kRepeatSpreadPenalty * spreadRatio);

        reporter.println("[INJ] rep-med=" + String(measured, 3) +
                         " | spread=" + String((testStats.max - testStats.min), 3) +
                         " | raw=" + String(rawRatio * 100.0f, 1) + "% | adj=" +
                         String(ratio * 100.0f, 1) + "%");

        Grade grade = Grade::NONE;
        if (ratio >= kStrongImproveRatio) {
          grade = Grade::STRONG;
        } else if (ratio >= kLightImproveRatio) {
          grade = Grade::LIGHT;
        }

        if (ratio > bestRatio) {
          bestRatio = ratio;
          bestGrade = grade;
          bestAmp = amp;
        }
      }

      BinResult& bin = results[freqIndex].bins[binIndex];
      bin.grade = bestGrade;
      bin.amplitude = bestAmp;
      bin.improvement = bestRatio;
      bin.measured = true;

      reporter.println(formatBinLine(binIndex, freqHz, bestAmp, bestGrade, bestRatio));
      actuators.stopISRSine();
      delay(kFreqCooldownMs);
    }

    const uint8_t bestFreqIdx = findBestFreqForBin(binIndex);
    BinResult& bestBin = results[bestFreqIdx].bins[binIndex];

    if (kEnableConfirmPass) {
      const bool confirmed = confirmBestCandidateInBin(sensors, actuators, reporter, binIndex, bestFreqIdx);
      if (!confirmed && bestBin.grade != Grade::NONE) {
        reporter.println("[MAPEO] Confirmación fallida en bin " + String(binIndex + 1) +
                         ": se degrada a ROJO");
        bestBin.grade = Grade::NONE;
        bestBin.improvement = 0.0f;
      }
    }

    reporter.println("[MAPEO] Mejor para bin " + String(binIndex + 1) + ": " +
                     String(results[bestFreqIdx].freqHz, 0) + " Hz | amp " +
                     String(bestBin.amplitude * 100.0f, 0) + "% | " + gradeText(bestBin.grade));
  }

  resultsValid = true;

  reporter.println("\n=== Resumen por frecuencia ===");
  for (uint8_t freqIndex = 0; freqIndex < kFreqCount; ++freqIndex) {
    const FrequencyResult& freqResult = results[freqIndex];
    reporter.println("\nFrecuencia " + String(freqResult.freqHz, 0) + " Hz:");

    bool anyImprovement = false;
    for (uint8_t binIndex = 0; binIndex < kBinCount; ++binIndex) {
      const BinResult& bin = freqResult.bins[binIndex];
      if (!bin.measured || bin.grade == Grade::NONE) continue;
      const uint8_t binStart = static_cast<uint8_t>(kMafMapMinPct + (binIndex * kBinSizePct));
      const uint8_t binEnd = static_cast<uint8_t>(kMafMapMinPct + ((binIndex + 1) * kBinSizePct));
      reporter.println("  " + String((int)binStart) + "-" + String((int)binEnd) + "% | amp " +
                       String(bin.amplitude * 100.0f, 0) + "% | " + gradeText(bin.grade));
      anyImprovement = true;
    }
    if (!anyImprovement) reporter.println("  (sin mejora detectada)");
  }

  reporter.println("\n=== Mapeo final por MAF (mejor frecuencia por bin) ===");
  for (uint8_t binIndex = 0; binIndex < kBinCount; ++binIndex) {
    const uint8_t bestFreqIdx = findBestFreqForBin(binIndex);
    const BinResult& bestBin = results[bestFreqIdx].bins[binIndex];
    const uint8_t binStart = static_cast<uint8_t>(kMafMapMinPct + (binIndex * kBinSizePct));
    const uint8_t binEnd = static_cast<uint8_t>(kMafMapMinPct + ((binIndex + 1) * kBinSizePct));

    String line = String((int)binStart) + "-" + String((int)binEnd) + "% -> ";
    if (!bestBin.measured) {
      line += "sin datos";
    } else {
      line += String(results[bestFreqIdx].freqHz, 0) + " Hz | amp ";
      line += String(bestBin.amplitude * 100.0f, 0) + "% | ";
      line += gradeText(bestBin.grade);
    }
    reporter.println(line);
  }

  reporter.println("Rampa completada");
  return true;
}
