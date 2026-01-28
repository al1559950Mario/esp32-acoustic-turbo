#pragma once

#include <stddef.h>

struct FlowMetricsConfig {
  float th_rms_lost = 0.0f;
  float th_rms_min = 0.0f;
  float th_rms_high = 0.0f;
  float th_outliers_low = 0.0f;
  float th_outliers_ok = 0.0f;
  float th_outliers_med = 0.0f;
  float th_outliers_high = 0.0f;
  float th_slope_ok = 0.0f;
  float outlier_k = 6.0f;
  float hold_s = 0.3f;
};

struct FlowMetrics {
  float rms = 0.0f;
  float mad = 0.0f;
  float ratio_outliers = 0.0f;
  float rms_slope = 0.0f;
  float median = 0.0f;
  float median_x = 0.0f;
};

struct FlowHoldState {
  float ok_time_s = 0.0f;
};

/**
 * @brief Calcula métricas de flujo para una ventana de muestras de presión.
 *
 * @param p Señal cruda de presión (kPa/Pa).
 * @param n Número de muestras.
 * @param dt_s Duración de la ventana en segundos.
 * @param prev_rms RMS de la ventana anterior (para pendiente).
 * @param scratch Buffer temporal con longitud >= 2*n.
 * @param scratch_len Longitud del buffer temporal.
 * @param out Estructura de salida con métricas.
 * @return true si se pudo calcular, false si el buffer es insuficiente o n=0.
 */
bool compute_metrics(const float* p,
                     size_t n,
                     float dt_s,
                     float prev_rms,
                     const FlowMetricsConfig& cfg,
                     float* scratch,
                     size_t scratch_len,
                     FlowMetrics& out);

bool is_flow_lost(const FlowMetrics& metrics, const FlowMetricsConfig& cfg);

bool is_inconsistent(const FlowMetrics& metrics, const FlowMetricsConfig& cfg);

bool is_laminar_constant(const FlowMetrics& metrics,
                         const FlowMetricsConfig& cfg,
                         FlowHoldState& hold,
                         float dt_s);
