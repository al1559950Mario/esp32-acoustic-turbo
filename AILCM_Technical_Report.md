# AILCM – Acoustic-Injection Laminar Conditioning Module
Documentación Técnica Completa

## 1. Visión General
El AILCM (Acoustic-Injection Laminar Conditioning Module) es un sistema híbrido que combina emisión acústica de alta frecuencia con preacondicionamiento aerodinámico, para mejorar el flujo de admisión en turbomáquinas. Emplea un tweeter direccional, una cavidad parcialmente resonante y un rotor pasivo, todo controlado por un microcontrolador ESP32.

**Objetivos clave**
- Inducir flujo laminar en la admisión
- Pre-acumular energía cinética en el aire antes del turbo
- Reducir turbo lag y turbulencias
- Integrarse sin reprogramar ECU (lectura MAF/vacío)

## 2. Fundamentos Físicos

### 2.1 Resonancia Helmholtz (modo forzado)
Frecuencia natural:
\(f_H =
rac{c}{2\pi} \sqrt{
rac{A}{V\,L}}\)
Con mediciones reales → \(f_H pprox 484 	ext{ Hz}\).

Modo de operación: excitación forzada a 6.4 kHz para guía de onda y streaming acústico.

### 2.2 Acoustic Streaming
Flujo medio estacionario inducido por absorción de onda sonora.

Intensidad acústica:
\(I = p_0^2/(\rho c),\quad p_0 \propto \text{level}\)

Potencia transferida:
\(P = I\;A_{	ext{sección}} \quad\Rightarrow\; P\propto 	ext{level}^2\)

## 3. Geometría y Dimensiones
| Elemento               | Dimensión        |
| ---------------------- | ---------------- |
| Tweeter bala Ø         | 38.1 mm          |
| Filtro cónico Ø int.   | 88.9 mm          |
| Filtro cónico longitud | 101.6 mm         |
| Distancia a rotor      | 101.6 mm         |
| Rotor Ø disco          | 90 mm            |
| Aspas (l × a)          | 23 mm × 25 mm    |
| Distancia al turbo     | 44.5 mm          |
| Conducto final Ø       | 90 mm (continuo) |

Notas geométricas
- Conducto continuo de 90 mm elimina escalones y mantiene impedancia constante.
- Zona de rotor: cavidad semiencerrada de V ≈ 1.46·10⁻⁴ m³.
- Transiciones redondeadas (<1 mm) para evitar reflexiones.

---

## 4. Especificaciones de Componentes
_(Pendiente de completar)_
## 5. Sistema de Control
_(Pendiente de completar)_
## 6. Procesamiento de Señal
_(Pendiente de completar)_
## 7. Integración Mecánica
_(Pendiente de completar)_
## 8. Protocolo de Validación

### 8.1 Calibración rápida de resonancia (mapeo por bin MAF)
**Objetivo:** ubicar rápidamente zonas dulces de resonancia por carga (MAF), validando varias frecuencias en cada rango de pedal.
**Comando de consola:** `C` (una sola ejecución).
**Cobertura de frecuencia:** 3 puntos enfocados alrededor del mejor resultado observado (5.0, 5.25 y 5.5 kHz).
**Bins de MAF:** 3 bins en la ventana útil (20-26, 26-33 y 33-40%).
#### Flujo actual (bin-first)
1. El sistema espera subida real de MAF para entrar al bin (no avanza si el pedal no sube).
2. Dentro de cada bin, prueba las 3 frecuencias.
3. Para cada frecuencia, mide baseline con stream en 0%.
4. Luego prueba amplitudes 30%, 50% y 70% con streaming continuo.
5. Clasifica mejora por ratio (VERDE/AMARILLO/ROJO) y guarda la mejor combinación por frecuencia-bin.
6. Al cerrar el bin, reporta la mejor frecuencia del bin.
#### Ventanas de tiempo de medición
- Estabilización por cambio de nivel: 120 ms.
- Baseline por frecuencia: 280 ms.
- Medición por amplitud: 260 ms.
- Warmup/cooldown por frecuencia: 80/20 ms.
Con esta configuración, la prueba interna de un bin (sin considerar tiempo del pedal) es ~4.0 s. El tiempo total real depende de qué tan suave suba el MAF entre bins.
Durante el mapeo, la consola imprime telemetría en tiempo real con formato `[LIVE] ... | MAF=...% | freq=... Hz | level=...% | osc=...` para visualizar al instante la condición de prueba activa.

#### Instrucciones simples al usuario
- “Acelera MUY LENTO”
- “Mantén la rampa suave”
- “No subas el MAF de golpe”
- “Rampa completada”
## 9. Simulación y Modelado
_(Pendiente de completar)_

## 10. Seguridad y Mantenimiento
_(Pendiente de completar)_

## 11. Futuras Extensiones
_(Pendiente de completar)_

## 7. Integración Mecánica  
_(Pendiente de completar)_  

## 8. Protocolo de Validación  
_(Pendiente de completar)_  

## 9. Simulación y Modelado  
_(Pendiente de completar)_  

## 10. Seguridad y Mantenimiento  
_(Pendiente de completar)_  

## 11. Futuras Extensiones  
_(Pendiente de completar)_  
