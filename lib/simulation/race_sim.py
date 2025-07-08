import time
import serial
import msvcrt

# --- Parámetros de simulación ---
IDLE_RPM       = 800
MAX_RPM        = 7000
SHIFT_RPM      = 6200
DT             = 0.1
THROTTLE_STEP  = 0.09      # incremento de throttle por paso
THROTTLE_DROP  = 0.2       # caída rápida tras cambio
MAP_RISE_COEF  = 0.05
MAP_REBOTE_COEF = 0.2      # más agresivo al recuperar presión
RPM_RISE_COEF  = 0.1

# Voltajes TPS: suelto = 2.25 V; fondo = 0.5 V
TPS_V_OPEN     = 2.25
TPS_V_CLOSED   = 0.5

# Voltajes MAP: vacío (~-15 inHg) = 3.05 V; atmosférico = 3.26 V
# Al soltar pedal, se simula vacío más fuerte (~-18 inHg) = 2.95 V
MAP_V_IDLE     = 3.05
MAP_V_MAX      = 3.26
MAP_V_REBOTE   = 2.95

# Relaciones de caja (simplificado)
GEAR_RATIOS    = [3.8, 2.2, 1.5, 1.0, 0.8]

# Puerto serial (ajusta si usas modo real)
SERIAL_PORT    = "COM6"
BAUDRATE       = 115200

# --- Modo Debug: evita abrir puerto y muestra datos por consola ---
DEBUG_MODE     = True

# --- Inicialización ---
throttle = 0.0
rpm      = IDLE_RPM
map_v    = MAP_V_IDLE
gear     = 0
idle_phase    = True
throttle_rebote = False
idle_start = time.time()

def volt_to_adc(volts):
    return int((volts / 3.3) * 4095)

if DEBUG_MODE:
    ser = None
    print(">>> MODO DEBUG ACTIVADO: solo impresión en consola")
else:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    time.sleep(1)

def key_pressed():
    if msvcrt.kbhit():
        ch = msvcrt.getch()
        if ch == b'\xe0':      # tecla especial
            ch2 = msvcrt.getch()
            return ch2
    return None

def describir_tps(volts):
    if volts < 0.8:
        return "Fondo"
    elif volts > 2.0:
        return "Suelto"
    else:
        return "Parcial"

def describir_map(volts):
    if volts < 3.0:
        return "SobreVacío"
    elif volts < 3.1:
        return "Vacío"
    else:
        return "Atmósfera"

print(">>> Controles: [↑] Acelera | [↓] Frena | [→] Sube marcha | [←] Baja marcha")
print("Iniciando simulación de carrera...")

try:
    while gear < len(GEAR_RATIOS):
        elapsed = time.time() - idle_start

        # 🟢 1) IDLE al encender
        if idle_phase and elapsed < 5.0:
            rpm = IDLE_RPM
            throttle = 0.0
            map_v = MAP_V_IDLE
        else:
            idle_phase = False

            # 🔼 2) Aceleración
            target_rpm = IDLE_RPM + throttle * (MAX_RPM - IDLE_RPM)
            rpm += (target_rpm - rpm) * RPM_RISE_COEF

            if throttle_rebote:
                # MAP responde a caída brusca en TPS
                target_map = MAP_V_REBOTE
                map_v += (target_map - map_v) * MAP_REBOTE_COEF
                if abs(map_v - target_map) < 0.005:
                    throttle_rebote = False
            else:
                target_map = MAP_V_IDLE + throttle * (MAP_V_MAX - MAP_V_IDLE)
                map_v += (target_map - map_v) * MAP_RISE_COEF

        # 🎮 3) Leer teclado
        key = key_pressed()
        if key == b'H':  # Flecha ↑ (acelera)
            throttle = min(1.0, throttle + THROTTLE_STEP)
        elif key == b'P':  # Flecha ↓ (frena)
            throttle = max(0.0, throttle - THROTTLE_STEP)
        elif key == b'M':  # Flecha →
            if gear < len(GEAR_RATIOS) - 1:
                prev_ratio = GEAR_RATIOS[gear]
                gear += 1
                next_ratio = GEAR_RATIOS[gear]
                rpm *= (next_ratio / prev_ratio)
                throttle *= THROTTLE_DROP
                throttle_rebote = True
                print(f"\n>>> CAMBIO A MARCHA {gear+1}, RPM ajustado a {rpm:.0f}")
        elif key == b'K':  # Flecha ←
            if gear > 0:
                prev_ratio = GEAR_RATIOS[gear]
                gear -= 1
                next_ratio = GEAR_RATIOS[gear]
                rpm *= (next_ratio / prev_ratio)
                throttle *= THROTTLE_DROP
                throttle_rebote = True
                print(f"\n<<< RETROCEDE A MARCHA {gear+1}, RPM ajustado a {rpm:.0f}")

        # 🧮 4) Calcular valores simulados
        tps_v = TPS_V_OPEN + (TPS_V_CLOSED - TPS_V_OPEN) * throttle
        tps_adc = volt_to_adc(tps_v)
        map_adc = volt_to_adc(map_v)

        # 📨 5) Construir y enviar payload por serial
        payload = f"tps_raw:{tps_adc},map_raw:{map_adc}\n"

        # 5) Enviar por serial SI está habilitado
        if not DEBUG_MODE and ser:
            ser.write(payload.encode('utf-8'))

        if not DEBUG_MODE and ser:
            linea_respuesta = ser.readline().decode().strip()
            if linea_respuesta:
                print(f"\n<<< ESP32 responde: {linea_respuesta}")


        # 🧠 6) Interpretación semántica
        tps_estado = describir_tps(tps_v)
        map_estado = describir_map(map_v)

        # 📺 7) HUD en consola sin parpadeo
        hud = (
            f"\r[{elapsed:5.2f}s] Gear:{gear+1} | RPM:{rpm:5.0f} | Throttle:{throttle:.2f} | "
            f"TPS:{tps_v:5.3f}V ({tps_estado}) | MAP:{map_v:5.3f}V ({map_estado})   "
        )
        print(hud, end='', flush=True)

        time.sleep(DT)

    print("\n>>> Simulación finalizada.")
finally:
    if ser:
        ser.close()
