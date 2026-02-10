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
\(f_H = rac{c}{2\pi} \sqrt{rac{A}{V\,L}}\)  
Con mediciones reales → \(f_H pprox 484 	ext{ Hz}\).  

Modo de operación: excitación forzada a 6.4 kHz para guía de onda y streaming acústico.  

### 2.2 Acoustic Streaming  
Flujo medio estacionario inducido por absorción de onda sonora.  

Intensidad acústica:  
\(I = rac{p_0^2}{ho\,c},\quad p_0 \propto 	ext{level}\)  

Potencia transferida:  
\(P = I\;A_{	ext{sección}} \quad\Rightarrow\; P\propto 	ext{level}^2\)  

## 3. Geometría y Dimensiones  
| Elemento               | Dimensión        |  
| ---------------------- | ---------------- |  

### 8.1 Calibración rápida de resonancia (≈ 1 minuto)
**Objetivo:** identificar de forma rápida los rangos de MAF donde la resonancia ofrece mejora, para luego refinar con un mapeo más fino.  

**Comando de consola:** `C` (una sola ejecución inicia todo el barrido).  

**Rango de prueba:** 4.0 kHz a 6.5 kHz.  
**Cobertura rápida (1 min):** 3 frecuencias distribuidas en el rango completo (4.0, 5.25 y 6.5 kHz).  
**Rampa por frecuencia:** 5 s aprox. (usuario acelera muy lentamente).  
**Tiempo total estimado:** ~15 s de rampa + overhead mínimo de consola.  

#### Flujo por frecuencia (secuencial)
1. El sistema fija una frecuencia.  
2. El usuario realiza **una única rampa lenta** de MAF.  
3. Durante la rampa, el sistema divide el MAF en **bins porcentuales**.  
4. Durante toda la rampa se mantiene **streaming acústico continuo**; por bin se rota amplitud: **30% → 50% → 70%** (nunca > 70%).  
5. Se miden métricas internas y se clasifica el bin:  
   - **Verde:** mejora fuerte  
   - **Amarillo:** mejora ligera  
   - **Rojo:** sin mejora  
6. Al final de la rampa, se guardan resultados en RAM y se pasa a la siguiente frecuencia.  

#### Resumen final por consola
Al finalizar todas las frecuencias, se imprime:  
- Rangos de MAF donde cada frecuencia funcionó mejor.  
- Amplitud más efectiva por rango.  
- Zonas donde la resonancia no ayuda.  

#### Instrucciones simples al usuario (texto exacto)
- “Acelera MUY LENTO”  
- “Mantén la rampa suave”  
- “No subas el MAF de golpe”  
- “Rampa completada”  

#### Restricciones
- No activar FLOW ni lógica externa.  
- Proceso rápido y con mínimo esfuerzo del usuario.  
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
_(Pendiente de completar)_  

## 9. Simulación y Modelado  
_(Pendiente de completar)_  

## 10. Seguridad y Mantenimiento  
_(Pendiente de completar)_  

## 11. Futuras Extensiones  
_(Pendiente de completar)_  
