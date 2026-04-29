# Diseño v2 (simplificado): Vortex en 2 capas + gate AFR futuro

> Este documento reemplaza la versión anterior.
> Objetivo: definir una lógica simple, precisa y aplicable para el vortex, manteniendo la semántica actual de la FSM (`ALIGN`, `FLOW`, `DECAY`) y sin dependencia de BEAM.

---

## 1) Objetivo principal

Sostener rendimiento de admisión en motor atmosférico con dos capas de actuación del vortex:

1. **Capa pasiva** de asistencia de flujo (bajas/medias RPM) gobernada por demanda de vacío.
2. **Capa full progresiva** al detectar demanda sostenida de vacío y MAF en zona alta (`>= 0.80`) durante un tiempo mínimo.

No se evalúa caída de MAF en esta versión.

---

## 2) Principios de diseño (reglas duras)

1. **Sin nuevos estados en FSM**: `ALIGN/FLOW/DECAY` mantienen su significado.
2. **Sin dependencia de BEAM** para activar vortex.
3. **Base por vacío**: la intención de potencia la determina el vacío.
4. **Disparo por umbral MAF alto + tiempo** (no por caída de MAF).
5. **Salida de full solo por liberación de vacío** (el conductor dejó de pedir potencia).
6. **AFR se agrega como gate en fase posterior**.

---

## 3) Arquitectura funcional de 2 capas

### 3.1 Capa 1 — Passive Assist
- Activa cuando hay demanda de vacío.
- Entrega comando suave de vortex para favorecer corriente/laminaridad.
- No busca 100% PWM.

### 3.2 Capa 2 — Full Progressive
- Entra cuando:
  - vacío en demanda sostenida,
  - `maf_norm >= MAF_NEAR_MAX_TH`,
  - condición sostenida por `HOLD_MS`.
- Una vez activa, sube el comando a `100%` con rampa progresiva.
- Sale cuando el vacío indique fin de demanda.

---

## 4) Señales de entrada (conceptuales)

- `pressure_pct_signed` (vacío firmado, negativo en vacío)
- `maf_norm` (MAF normalizado 0..1)
- `base_boost_cmd` (comando base actual, 0..1)
- Futuro:
  - `afr_value`
  - `afr_valid`

---

## 5) Lógica simplificada

## 5.1 Detección de demanda por vacío

Con histéresis:
- `vacuum_demand_on` si `pressure_pct_signed <= VACUUM_PCT_ON`
- `vacuum_demand_off` si `pressure_pct_signed >= VACUUM_PCT_OFF`

(Usar ventana breve para estabilidad de señal, evitando rebotes instantáneos.)

### 5.2 Umbral alto de MAF

- `maf_high = (maf_norm >= MAF_NEAR_MAX_TH)`
- Valor inicial acordado: `MAF_NEAR_MAX_TH = 0.80`

### 5.3 Activación Full

Activar full progresivo si:

```text
vacuum_demand_on && maf_high sostenidos por HOLD_MS
```

### 5.4 Sostén y salida

- Mientras `vacuum_demand_on` permanezca activo, mantener full.
- Salir de full cuando se cumpla `vacuum_demand_off`.

No hay condición por caída de MAF en esta versión.

---

## 6) Pseudoflujo de control

```text
if !vacuum_demand_on:
    mode = IDLE/PASSIVE_OFF
else:
    mode = PASSIVE_ASSIST

if vacuum_demand_on && maf_norm >= 0.80 durante HOLD_MS:
    mode = FULL_PROGRESSIVE

if mode == FULL_PROGRESSIVE && vacuum_demand_off:
    salir de FULL (rampa de retorno)
```

---

## 7) Parámetros iniciales (calibrables)

- `VACUUM_PCT_ON = -2.0`
- `VACUUM_PCT_OFF = -0.5`
- `MAF_NEAR_MAX_TH = 0.80`
- `HOLD_MS = 120`
- `FULL_TARGET = 1.00` (100%)
- `FULL_RAMP_UP_MS` (definir en pruebas)
- `FULL_RAMP_DOWN_MS` (definir en pruebas)

---

## 8) AFR gate (fase posterior)

Se aplicará solo cuando exista sensor AFR válido.

Política acordada:
- **Habilitación de sistema** cuando `AFR < 13.5`.
- **Corte/retiro de full** cuando `AFR < 12.0` (mezcla rica de protección).

Nota: mientras no exista señal AFR confiable, la lógica opera solo con vacío + MAF.

---

## 9) Integración sin romper arquitectura

1. Mantener la FSM actual y su semántica.
2. Ejecutar esta lógica como capa de arbitraje de comando de vortex en `ALIGN/FLOW`.
3. En `DECAY/IDLE/OFF`, reset de banderas internas de esta capa.
4. Mantener responsabilidades:
   - FSM decide contexto de estado.
   - capa vortex decide nivel objetivo (pasivo/full).
   - `VortexController` aplica PWM con suavizado/rampa.

---

## 10) Telemetría mínima recomendada

- `vacuum_demand_on/off`
- `maf_norm`
- `maf_high`
- `full_armed`
- `full_active`
- `hold_elapsed_ms`
- `boost_cmd_out`
- Futuro: `afr_valid`, `afr_value`, `afr_gate_state`

---

## 11) Criterios de aceptación

1. En demanda baja/media: funciona capa pasiva sin entrar a full.
2. Con vacío sostenido + `maf_norm >= 0.80` por `120 ms`: entra full progresivo.
3. Full se mantiene mientras exista demanda de vacío.
4. Full se desactiva al liberar demanda (vacío off).
5. No se agregan estados FSM nuevos.
6. Cuando se integre AFR:
   - habilita bajo `<13.5`
   - corta bajo `<12.0`

---

## 12) Frase de intención

Extender el vortex para sostener rendimiento de admisión con una lógica simple y precisa (vacío + umbral MAF + tiempo), preservando la semántica de `ALIGN/FLOW` y dejando AFR como gate de seguridad en fase posterior.
