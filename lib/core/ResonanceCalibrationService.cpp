#include "ResonanceCalibrationService.h"
#include <math.h>

float ResonanceCalibrationService::sampleMetric(SensorManager& sensors, uint32_t durationMs) const {
  uint32_t start = millis();
  float sum = 0.0f;
  uint16_t count = 0;
  while (millis() - start < durationMs) {
    sum += sensors.computeOscillationAmplitude();
    count++;
    delay(10);
  }
  return count > 0 ? (sum / count) : 0.0f;
}

const char* ResonanceCalibrationService::gradeText(Grade grade) {
  switch (grade) {
    case Grade::STRONG: return "VERDE";
    case Grade::LIGHT: return "AMARILLO";
    default: return "ROJO";
  }
}

String ResonanceCalibrationService::formatBinLine(uint8_t binIndex, float amp, Grade grade) const {
  uint8_t binStart = static_cast<uint8_t>(binIndex * kBinSizePct);
  uint8_t binEnd = static_cast<uint8_t>((binIndex + 1) * kBinSizePct);
  String line = "Bin ";
  line += String(binIndex + 1);
  line += " (";
  line += String(binStart);
  line += "-";
  line += String(binEnd);
  line += "%) stream amp ";
  line += String(amp * 100.0f, 0);
  line += "% -> ";
  line += gradeText(grade);
  return line;
}

bool ResonanceCalibrationService::run(SensorManager& sensors,
                                      ActuatorManager& actuators,
                                      ResonanceCalibrationReporter& reporter) {
  resultsValid = false;

  reporter.println("\n=== Calibración rápida de resonancia (streaming) ===");
  reporter.println("Acelera MUY LENTO");
  reporter.println("Mantén la rampa suave");
  reporter.println("No subas el MAF de golpe");
  reporter.println("(Se probarán 3 frecuencias: 4000, 5250 y 6500 Hz)");

  for (uint8_t freqIndex = 0; freqIndex < kFreqCount; ++freqIndex) {
    const float freqHz = kFreqListHz[freqIndex];
    results[freqIndex].freqHz = freqHz;
    for (uint8_t bin = 0; bin < kBinCount; ++bin) {
      results[freqIndex].bins[bin] = {};
    }

    reporter.println("\n>> Frecuencia " + String(freqHz, 0) + " Hz | Rampa ~" + String(kRampDurationMs) + " ms");
    reporter.println("Acelera MUY LENTO");

    float baselineRef = sampleMetric(sensors, kBaselineSampleMs);
    float currentAmp = kAmpLevels[0];
    actuators.startISRSine(static_cast<uint32_t>(freqHz), currentAmp);

    unsigned long rampStart = millis();
    while (millis() - rampStart < kRampDurationMs) {
      float mafPct = sensors.readMAFLoadPercent();
      if (mafPct < 0.0f) mafPct = 0.0f;
      if (mafPct > 100.0f) mafPct = 100.0f;

      uint8_t binIndex = static_cast<uint8_t>(mafPct / kBinSizePct);
      if (binIndex >= kBinCount) binIndex = kBinCount - 1;

      BinResult& bin = results[freqIndex].bins[binIndex];
      if (!bin.measured) {
        float amplitude = kAmpLevels[binIndex % 3];
        if (fabsf(amplitude - currentAmp) > 0.001f) {
          actuators.getAcousticInjector().setLevel(amplitude);
          currentAmp = amplitude;
          delay(20);
        }

        float measuredStreaming = sampleMetric(sensors, kBinSampleMs);
        float denom = fabsf(baselineRef);
        if (denom < kMinBaseline) denom = kMinBaseline;
        float ratio = (measuredStreaming - baselineRef) / denom;

        Grade grade = Grade::NONE;
        if (ratio >= kStrongImproveRatio) {
          grade = Grade::STRONG;
        } else if (ratio >= kLightImproveRatio) {
          grade = Grade::LIGHT;
        }

        bin.grade = grade;
        bin.amplitude = amplitude;
        bin.improvement = ratio;
        bin.measured = true;

        reporter.println(formatBinLine(binIndex, amplitude, grade));
      }
      delay(20);
    }

    actuators.stopISRSine();
    reporter.println("Rampa completada");
  }

  resultsValid = true;
  reporter.println("\n=== Resumen de resonancia ===");
  for (uint8_t freqIndex = 0; freqIndex < kFreqCount; ++freqIndex) {
    const FrequencyResult& freqResult = results[freqIndex];
    reporter.println("\nFrecuencia " + String(freqResult.freqHz, 0) + " Hz:");
    reporter.println("  Mejores rangos (MAF):");

    bool anyImprovement = false;
    for (uint8_t binIndex = 0; binIndex < kBinCount; ++binIndex) {
      const BinResult& bin = freqResult.bins[binIndex];
      if (!bin.measured || bin.grade == Grade::NONE) continue;

      uint8_t binStart = static_cast<uint8_t>(binIndex * kBinSizePct);
      uint8_t binEnd = static_cast<uint8_t>((binIndex + 1) * kBinSizePct);
      String line = "    ";
      line += String(binStart);
      line += "-";
      line += String(binEnd);
      line += "% | amp ";
      line += String(bin.amplitude * 100.0f, 0);
      line += "% | ";
      line += (bin.grade == Grade::STRONG) ? "VERDE" : "AMARILLO";
      reporter.println(line);
      anyImprovement = true;
    }
    if (!anyImprovement) reporter.println("    (sin mejora detectada)");

    reporter.println("  Zonas donde no ayuda:");
    bool anyNoHelp = false;
    for (uint8_t binIndex = 0; binIndex < kBinCount; ++binIndex) {
      const BinResult& bin = freqResult.bins[binIndex];
      if (!bin.measured || bin.grade != Grade::NONE) continue;
      uint8_t binStart = static_cast<uint8_t>(binIndex * kBinSizePct);
      uint8_t binEnd = static_cast<uint8_t>((binIndex + 1) * kBinSizePct);
      reporter.println("    " + String(binStart) + "-" + String(binEnd) + "%");
      anyNoHelp = true;
    }
    if (!anyNoHelp) reporter.println("    (sin datos)");
  }

  return true;
}

