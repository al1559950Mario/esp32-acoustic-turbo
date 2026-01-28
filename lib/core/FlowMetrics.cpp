#include "FlowMetrics.h"

#include <algorithm>
#include <math.h>

namespace {

float median_of_buffer(float* data, size_t n) {
  if (n == 0) {
    return 0.0f;
  }
  std::sort(data, data + n);
  if (n % 2 == 0) {
    size_t upper = n / 2;
    size_t lower = upper - 1;
    return 0.5f * (data[lower] + data[upper]);
  }
  return data[n / 2];
}

}  // namespace

bool compute_metrics(const float* p,
                     size_t n,
                     float dt_s,
                     float prev_rms,
                     const FlowMetricsConfig& cfg,
                     float* scratch,
                     size_t scratch_len,
                     FlowMetrics& out) {
  if (n == 0 || scratch_len < (2 * n)) {
    return false;
  }

  float* temp = scratch;
  float* temp_abs = scratch + n;

  for (size_t i = 0; i < n; ++i) {
    temp[i] = p[i];
  }

  const float p0 = median_of_buffer(temp, n);
  out.median = p0;

  for (size_t i = 0; i < n; ++i) {
    temp[i] = p[i] - p0;
  }

  const float median_x = median_of_buffer(temp, n);
  out.median_x = median_x;

  float sum_sq = 0.0f;
  size_t outliers = 0;

  for (size_t i = 0; i < n; ++i) {
    const float centered = temp[i];
    sum_sq += centered * centered;
    temp_abs[i] = fabsf(centered - median_x);
  }

  const float mad = median_of_buffer(temp_abs, n);
  out.mad = mad;

  const float outlier_threshold = mad * cfg.outlier_k;
  if (outlier_threshold > 0.0f) {
    for (size_t i = 0; i < n; ++i) {
      if (fabsf(temp[i]) > outlier_threshold) {
        ++outliers;
      }
    }
  }

  out.rms = sqrtf(sum_sq / static_cast<float>(n));
  out.ratio_outliers = (n > 0) ? (static_cast<float>(outliers) / static_cast<float>(n)) : 0.0f;
  out.rms_slope = (dt_s > 0.0f) ? ((out.rms - prev_rms) / dt_s) : 0.0f;

  return true;
}

bool is_flow_lost(const FlowMetrics& metrics, const FlowMetricsConfig& cfg) {
  return (metrics.rms < cfg.th_rms_lost) && (metrics.ratio_outliers < cfg.th_outliers_low);
}

bool is_inconsistent(const FlowMetrics& metrics, const FlowMetricsConfig& cfg) {
  if (metrics.ratio_outliers >= cfg.th_outliers_high) {
    return true;
  }
  return (metrics.rms > cfg.th_rms_high) && (metrics.ratio_outliers > cfg.th_outliers_med);
}

bool is_laminar_constant(const FlowMetrics& metrics,
                         const FlowMetricsConfig& cfg,
                         FlowHoldState& hold,
                         float dt_s) {
  const bool ok_now = (metrics.rms >= cfg.th_rms_min) &&
                      (metrics.ratio_outliers <= cfg.th_outliers_ok) &&
                      (fabsf(metrics.rms_slope) <= cfg.th_slope_ok);

  if (ok_now) {
    hold.ok_time_s += dt_s;
  } else {
    hold.ok_time_s = 0.0f;
  }

  return hold.ok_time_s >= cfg.hold_s;
}
