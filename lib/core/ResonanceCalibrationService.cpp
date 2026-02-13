#include "ResonanceCalibrationService.h"
#include <math.h>

float ResonanceCalibrationService::sampleMetric(SensorManager& sensors,
                                              ResonanceCalibrationReporter& reporter,
                                              float freqHz,
                                              float amplitude,
                                              uint32_t durationMs,
                                              const char* phase) const {
  const uint32_t start = millis();
  uint32_t lastPrintMs = 0;
  float sum = 0.0f;
  uint16_t count = 0;

  while (millis() - start < durationMs) {
    const float metric = sensors.computeOscillationAmplitude();
    sum += metric;
    ++count;

    const uint32_t now = millis();
    if ((now - lastPrintMs) >= kLivePrintPeriodMs) {
      lastPrintMs = now;
      float mafPct = sensors.readMAFLoadPercent();
      if (mafPct < 0.0f) mafPct = 0.0f;
      if (mafPct > 100.0f) mafPct = 100.0f;

      reporter.println(String("[LIVE]") + phase +
                       " | MAF=" + String(mafPct, 1) + "%" +
                       " | freq=" + String(freqHz, 0) + " Hz" +
                       " | level=" + String(amplitude * 100.0f, 0) + "%" +
                       " | osc=" + String(metric, 3));
    }

    delay(10);
  }
  return count > 0 ? (sum / count) : 0.0f;
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
                             kLevelSettleMs + kBaselineSampleMs +
                             (3 * (kLevelSettleMs + kBinSampleMs)) +
                             kFreqCooldownMs;
  return perFreqMs * kFreqCount;
}

bool ResonanceCalibrationService::waitForBinEntry(SensorManager& sensors,
                                                  ResonanceCalibrationReporter& reporter,
                                                  uint8_t binIndex,
                                                  float& lastAcceptedMaf) const {
  const float binStart = float(binIndex) * kBinSizePct;
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
                       " (" + String((int)binStart) + "-" + String((int)(binStart + kBinSizePct)) +
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
  const uint8_t binStart = static_cast<uint8_t>(binIndex * kBinSizePct);
  const uint8_t binEnd = static_cast<uint8_t>((binIndex + 1) * kBinSizePct);
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
  float bestImprovement = -9999.0f;
  for (uint8_t freqIndex = 0; freqIndex < kFreqCount; ++freqIndex) {
    const BinResult& b = results[freqIndex].bins[binIndex];
    if (!b.measured) continue;
    if (b.improvement > bestImprovement) {
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

    const uint8_t binStart = static_cast<uint8_t>(binIndex * kBinSizePct);
    const uint8_t binEnd = static_cast<uint8_t>((binIndex + 1) * kBinSizePct);
    reporter.println("\n[MAPEO] Bin activo " + String(binIndex + 1) + " (" + String(binStart) +
                     "-" + String(binEnd) + "%) | MAF=" + String(lastAcceptedMaf, 1) + "%");

    for (uint8_t freqIndex = 0; freqIndex < kFreqCount; ++freqIndex) {
      const float freqHz = kFreqListHz[freqIndex];
      actuators.startISRSine(static_cast<uint32_t>(freqHz), 0.0f);
      delay(kFreqWarmupMs);

      const float baseline = measureWithStreaming(sensors, actuators, reporter, freqHz, 0.0f, kBaselineSampleMs, " baseline");

      Grade bestGrade = Grade::NONE;
      float bestAmp = kAmpLevels[0];
      float bestRatio = -9999.0f;

      for (uint8_t ampIndex = 0; ampIndex < 3; ++ampIndex) {
        const float amp = kAmpLevels[ampIndex];
        reporter.println("[INJ] freq=" + String(freqHz, 0) + " Hz | level=" + String(amp * 100.0f, 0) + "%");

        const float measured = measureWithStreaming(sensors, actuators, reporter, freqHz, amp, kBinSampleMs, " test");
        float denom = fabsf(baseline);
        if (denom < kMinBaseline) denom = kMinBaseline;
        const float ratio = (measured - baseline) / denom;

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
    const BinResult& bestBin = results[bestFreqIdx].bins[binIndex];
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
      const uint8_t binStart = static_cast<uint8_t>(binIndex * kBinSizePct);
      const uint8_t binEnd = static_cast<uint8_t>((binIndex + 1) * kBinSizePct);
      reporter.println("  " + String(binStart) + "-" + String(binEnd) + "% | amp " +
                       String(bin.amplitude * 100.0f, 0) + "% | " + gradeText(bin.grade));
      anyImprovement = true;
    }
    if (!anyImprovement) reporter.println("  (sin mejora detectada)");
  }

  reporter.println("\n=== Mapeo final por MAF (mejor frecuencia por bin) ===");
  for (uint8_t binIndex = 0; binIndex < kBinCount; ++binIndex) {
    const uint8_t bestFreqIdx = findBestFreqForBin(binIndex);
    const BinResult& bestBin = results[bestFreqIdx].bins[binIndex];
    const uint8_t binStart = static_cast<uint8_t>(binIndex * kBinSizePct);
    const uint8_t binEnd = static_cast<uint8_t>((binIndex + 1) * kBinSizePct);

    String line = String(binStart) + "-" + String(binEnd) + "% -> ";
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
