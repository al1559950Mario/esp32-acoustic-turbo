import time
import serial
import msvcrt

# --- Parámetros de simulación ---
IDLE_RPM       = 800
MAX_RPM        = 7000
SHIFT_RPM      = 6200
DT             = 0.3
THROTTLE_STEP  = 0.09
THROTTLE_DROP  = 0.2
MAP_RISE_COEF  = 0.05
MAP_REBOTE_COEF = 0.2
RPM_RISE_COEF  = 0.1

TPS_V_OPEN     = 2.25
TPS_V_CLOSED   = 0.5

MAP_V_IDLE     = 3.05
MAP_V_MAX      = 3.26
MAP_V_REBOTE   = 2.95

GEAR_RATIOS    = [3.8, 2.2, 1.5, 1.0, 0.8]

SERIAL_PORT    = "COM7"
BAUDRATE       = 115200

DEBUG_MODE     = False

throttle = 0.0
rpm      = IDLE_RPM
map_v    = MAP_V_IDLE
gear     = 0
idle_phase = True
throttle_rebote = False
idle_start = time.time()

modo_envio_constante = 0  # 0=normal, 1=tps_min, 2=tps_max, 3=map_min, 4=map_max

def volt_to_adc(volts):
    return int((volts / 3.3) * 4095)

def enviar_comando(ser, cmd):
    ser.write((cmd + '\n').encode('utf-8'))
    time.sleep(0.1)
    while ser.in_waiting:
        respuesta = ser.readline().decode('utf-8', errors='ignore').strip()
        if respuesta:
            print(f"\n<<< ESP32 responde: {respuesta}")

def key_pressed():
    if msvcrt.kbhit():
        ch = msvcrt.getch()
        if ch == b'\xe0':  # tecla especial (flechas, etc)
            ch2 = msvcrt.getch()
            return ch + ch2  # Retornamos bytes doble para flechas
        else:
            # Retornamos letra minúscula si es alfabeto o números para este caso
            if ch.isalpha() or ch.isdigit():
                return ch.decode('utf-8').lower()
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
print("Botones 1-4: Envío constante de umbrales (tps_min, tps_max, map_min, map_max)")
print("Botón 5: Regresar a modo normal")
print("Presiona letra para enviar comando al ESP32 (ej. 'z' para simulación)")
print("Iniciando simulación de carrera...")

if DEBUG_MODE:
    ser = None
    print(">>> MODO DEBUG ACTIVADO: solo impresión en consola")
else:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    ser.setDTR(False)
    ser.setRTS(False)
    time.sleep(1)

try:
    while gear < len(GEAR_RATIOS):
        elapsed = time.time() - idle_start

        if idle_phase and elapsed < 5.0:
            rpm = IDLE_RPM
            throttle = 0.0
            map_v = MAP_V_IDLE
        else:
            idle_phase = False

            target_rpm = IDLE_RPM + throttle * (MAX_RPM - IDLE_RPM)
            rpm += (target_rpm - rpm) * RPM_RISE_COEF

            if throttle_rebote:
                target_map = MAP_V_REBOTE
                map_v += (target_map - map_v) * MAP_REBOTE_COEF
                if abs(map_v - target_map) < 0.005:
                    throttle_rebote = False
            else:
                target_map = MAP_V_IDLE + throttle * (MAP_V_MAX - MAP_V_IDLE)
                map_v += (target_map - map_v) * MAP_RISE_COEF

        key = key_pressed()
        if key:
            if isinstance(key, bytes):
                # Detectamos flechas con código de bytes
                if key == b'\xe0H':  # Flecha arriba
                    throttle = min(1.0, throttle + THROTTLE_STEP)
                elif key == b'\xe0P':  # Flecha abajo
                    throttle = max(0.0, throttle - THROTTLE_STEP)
                elif key == b'\xe0M':  # Flecha derecha
                    if gear < len(GEAR_RATIOS) - 1:
                        prev_ratio = GEAR_RATIOS[gear]
                        gear += 1
                        next_ratio = GEAR_RATIOS[gear]
                        rpm *= (next_ratio / prev_ratio)
                        throttle *= THROTTLE_DROP
                        throttle_rebote = True
                        print(f"\n>>> CAMBIO A MARCHA {gear+1}, RPM ajustado a {rpm:.0f}")
                elif key == b'\xe0K':  # Flecha izquierda
                    if gear > 0:
                        prev_ratio = GEAR_RATIOS[gear]
                        gear -= 1
                        next_ratio = GEAR_RATIOS[gear]
                        rpm *= (next_ratio / prev_ratio)
                        throttle *= THROTTLE_DROP
                        throttle_rebote = True
                        print(f"\n<<< RETROCEDE A MARCHA {gear+1}, RPM ajustado a {rpm:.0f}")
            else:
                # Botones 1-5 para enviar valores constantes
                if key == '1':
                    modo_envio_constante = 1
                    print("\n>>> Enviando TPS_MIN constante")
                elif key == '2':
                    modo_envio_constante = 2
                    print("\n>>> Enviando TPS_MAX constante")
                elif key == '3':
                    modo_envio_constante = 3
                    print("\n>>> Enviando MAP_MIN constante")
                elif key == '4':
                    modo_envio_constante = 4
                    print("\n>>> Enviando MAP_MAX constante")
                elif key == '5':
                    modo_envio_constante = 0
                    print("\n>>> Regresando a modo normal")
                else:
                    # Si es letra, enviamos comando al ESP32
                    print(f"\n>>> Enviando comando: '{key}'")
                    if not DEBUG_MODE and ser:
                        enviar_comando(ser, key)

        # Valores simulados para envío normal (modo_envio_constante == 0)
        tps_v = TPS_V_OPEN + (TPS_V_CLOSED - TPS_V_OPEN) * throttle
        map_adc_valor = map_v  # lo uso para el envío

        # Defino los valores fijos para los umbrales (volts)
        TPS_MIN_V = TPS_V_CLOSED
        TPS_MAX_V = TPS_V_OPEN
        MAP_MIN_V = MAP_V_IDLE
        MAP_MAX_V = MAP_V_MAX

        # Decide qué valores enviar según el modo de envío constante
        if modo_envio_constante == 1:  # TPS_MIN
            tps_v = TPS_MIN_V
        elif modo_envio_constante == 2:  # TPS_MAX
            tps_v = TPS_MAX_V
        if modo_envio_constante == 3:  # MAP_MIN
            map_adc_valor = MAP_MIN_V
        elif modo_envio_constante == 4:  # MAP_MAX
            map_adc_valor = MAP_MAX_V

        tps_adc = volt_to_adc(tps_v)
        map_adc = volt_to_adc(map_adc_valor)

        if not DEBUG_MODE and ser:
            payload = f"tps_raw:{tps_adc},map_raw:{map_adc}\n"
            ser.write(payload.encode('utf-8'))

            if ser.in_waiting:
                linea_respuesta = ser.readline().decode('utf-8', errors='ignore').strip()
                if linea_respuesta:
                    print(f"\n<<< ESP32 responde: {linea_respuesta}")

        hud = (
            f"\r[{elapsed:5.2f}s] Gear:{gear+1} | RPM:{rpm:5.0f} | Throttle:{throttle:.2f} | "
            f"TPS:{tps_v:5.3f}V ({describir_tps(tps_v)}) | MAP:{map_adc_valor:5.3f}V ({describir_map(map_adc_valor)})   "
        )
        print(hud, end='', flush=True)

        time.sleep(DT)

    print("\n>>> Simulación finalizada.")
finally:
    if ser:
        ser.close()
