# FSM + métricas de flujo (HX710B)

## Estados válidos

- `IDLE`
- `ALIGN`
- `FLOW`
- `DECAY`

## Transiciones permitidas

- `IDLE → ALIGN` (arranque para acoplar)
- `ALIGN → FLOW` (flujo constante, laminar, sin turbulencias durante ventana de confirmación)
- `ALIGN → DECAY` (si no acopla o las condiciones caen)
- `FLOW → ALIGN` (si se detecta inconsistencia)
- `FLOW → DECAY` (si se pierde el flujo o no supera umbral mínimo)
- `DECAY → IDLE` (ring-down completado)

> Nota: **No se permite** `ALIGN → IDLE`.

## Intención por estado

- **IDLE**: todo en cero; sólo sirve para arranque limpio.
- **ALIGN**: fase de acople; **BEAM dominante** y **BOOST mínimo**; ajustes suaves (amplitud/frecuencia) hasta lograr flujo laminar y constante.
- **FLOW**: operación nominal; si el flujo es estable → escalar BEAM y BOOST con MAF; si aparece inconsistencia → volver a **ALIGN**; si el flujo se pierde o cae por debajo de umbral → **DECAY**.
- **DECAY**: ring-down controlado (bajar B y T con rampas/slew) y salir a **IDLE**.

## Detección de calidad de flujo (HX710B)

- Señal: `p[n]` (kPa/Pa), `p0 = median(window)`, `x[n] = p[n] - p0`.
- Ventana de evaluación: `T_eval = 200–400 ms` (ajustar a `fs` real).
- Histeresis temporal de veredictos: `H_keep = 300–600 ms`.
- Métricas por ventana:
  - `rms = sqrt(mean(x^2))`
  - `mad = median(|x - median(x)|)`
  - Outlier si `|x| > K * mad` (K = 6–9)
  - `ratio_outliers = (#outliers)/N`
  - `rms_slope = (rms_t - rms_{t-1}) / Δt`
- Veredictos:
  - **Flujo perdido**: `rms < TH_RMS_LOST` **y** `ratio_outliers < TH_OUTLIERS_LOW`
  - **Flujo inconsistente**: `ratio_outliers ≥ TH_OUTLIERS_HIGH` **o** (`rms > TH_RMS_HIGH` **y** `ratio_outliers > TH_OUTLIERS_MED`)
  - **Flujo laminar y constante**: `rms ≥ TH_RMS_MIN` **y** `ratio_outliers ≤ TH_OUTLIERS_OK` **y** `|rms_slope| ≤ TH_SLOPE_OK` sostenido por `H_keep`

## Reglas de control (síntesis)

- Variables: `B`=BEAM%, `T`=BOOST%, `M`=MAF%.

### ALIGN (acople)

- Subir `B` hacia `B_align(M)` (p.ej. 20–35%).
- Limitar `T ≤ T_align_max` (p.ej. 10–15%) o 0.
- Si no acopla tras `N_try` ventanas, barrer frecuencia BEAM ±3–5% alrededor de `f0`.
- `ALIGN → FLOW` cuando **laminar + constante** por `H_keep`.
- `ALIGN → DECAY` si falla acople o condiciones caen.

### FLOW (operación)

- **Si inconsistente** → **`FLOW → ALIGN`** y aplicar correctivo **↑BEAM, ↓BOOST** (p.ej. `B += kB_inc`, `T -= kT_dec`).
- **Si flujo perdido o debajo de umbral** → **`FLOW → DECAY`** (rampa ordenada a 0).
- **Si laminar + constante** → permanecer en `FLOW` y **escalar** `B` y `T` con `M` (función `f(M)` lineal/convexa), respetando límites y slew rates.

### DECAY

- Ring-down de `B` y `T`; al terminar, **`DECAY → IDLE`**.

## Modo de compatibilidad gradual

Para reducir cambios bruscos durante la migración, la FSM soporta un modo de compatibilidad
que mantiene la lógica histórica basada en los umbrales `VORTEX_*` e `INJ_*` antes de adoptar
por completo las métricas de flujo. Este modo permite transicionar de forma progresiva sin
romper el comportamiento previo y se puede desactivar una vez calibrado el sistema.

## Parámetros iniciales (semillas)

- `TH_RMS_LOST = 0.01–0.02 * RMS_res_típico`
- `TH_RMS_MIN = 0.05–0.10 * RMS_res_típico`
- `TH_RMS_HIGH = 0.5–0.7 * RMS_res_max_seguro`
- `TH_OUTLIERS_LOW = 0.01–0.02`
- `TH_OUTLIERS_OK = 0.02–0.03`
- `TH_OUTLIERS_MED = 0.02–0.04`
- `TH_OUTLIERS_HIGH = 0.04–0.08`
- `TH_SLOPE_OK = 0.02–0.05 * RMS_res / s`
- `H_keep = 0.3–0.6 s`
- Correctivo inconsistente: `kB_inc = 6–10`, `kT_dec = 6–10`
- Correctivo suave (opcional): `kB_small = 2–4`, `kT_small = 2–4`
- Escalado estable: `kB_scale = 1–3`, `kT_scale = 1–3`, `f(M) = (M/100)^γ`, `γ ∈ [1.0, 1.4]`
- Límites:
  - `Bmax = 100` (o limitado por temperatura)
  - `Tmax = map(M, 20..100 → 30..100)`
  - Limitar `ΔB/Δt` y `ΔT/Δt` (slew cada 20–50 ms)
  - Histeresis de 1–2 ventanas para evitar parpadeo de veredictos

## Logging mínimo recomendado (por ventana)

`timestamp, state, MAF%, B, T, f, rms, ratio_outliers, rms_slope, verdict`
