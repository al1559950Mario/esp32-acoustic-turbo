import time
import serial
import msvcrt
import threading

# --- Parámetros de simulación ---
IDLE_RPM       = 800
MAX_RPM        = 7000
SHIFT_RPM      = 6200
DT             = 0.1
THROTTLE_STEP  = 0.09   # Incremento deseado para objetivo
THROTTLE_RATE  = 0.009   # Velocidad con la que throttle real se acerca al objetivo
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

# Variables globales simulación
throttle = 0.0          # Valor real del throttle que enviamos (suavizado)
throttle_target = 0.0   # Valor objetivo al que queremos llegar (incremento en pasos)
rpm      = IDLE_RPM
map_v    = MAP_V_IDLE
gear     = 0
idle_phase = True
throttle_rebote = False
idle_start = time.time()

modo_envio_constante = 0  # 0=normal, 1=tps_min, 2=tps_max, 3=map_min, 4=map_max

running = True

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
            return ch + ch2  # Retornamos bytes dobles para flechas
        else:
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

def thread_envio(ser):
    global throttle, map_v, modo_envio_constante, running
    while running:
        # Calcula el voltaje de TPS y MAP según modo
        tps_v = TPS_V_CLOSED + (TPS_V_OPEN - TPS_V_CLOSED) * throttle
        map_adc_valor = map_v

        TPS_MIN_V = TPS_V_CLOSED
        TPS_MAX_V = TPS_V_OPEN
        MAP_MIN_V = MAP_V_IDLE
        MAP_MAX_V = MAP_V_MAX

        if modo_envio_constante == 1:
            tps_v = TPS_MIN_V
        elif modo_envio_constante == 2:
            tps_v = TPS_MAX_V
        if modo_envio_constante == 3:
            map_adc_valor = MAP_MIN_V
        elif modo_envio_constante == 4:
            map_adc_valor = MAP_MAX_V

        tps_adc = volt_to_adc(tps_v)
        map_adc = volt_to_adc(map_adc_valor)

        if ser:
            payload = f"tps_raw:{tps_adc},map_raw:{map_adc}\n"
            try:
                ser.write(payload.encode('utf-8'))
            except Exception as e:
                print(f"Error enviando datos: {e}")
        time.sleep(DT)

def thread_lectura(ser):
    global running
    while running:
        if ser and ser.in_waiting:
            try:
                linea_respuesta = ser.readline().decode('utf-8', errors='ignore').strip()
                if linea_respuesta:
                    print(f"\n<<< ESP32 responde: {linea_respuesta}")
            except Exception as e:
                print(f"Error leyendo datos: {e}")
        time.sleep(0.01)

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

# Inicia hilos para envío y lectura asincrónica
thread_send = threading.Thread(target=thread_envio, args=(ser,))
thread_read = threading.Thread(target=thread_lectura, args=(ser,))
thread_send.start()
thread_read.start()

try:
    while gear < len(GEAR_RATIOS):
        elapsed = time.time() - idle_start

        # Suavizado del throttle para acercarse al target lentamente
        if abs(throttle - throttle_target) > 0.001:
            if throttle < throttle_target:
                throttle += THROTTLE_RATE
                if throttle > throttle_target:
                    throttle = throttle_target
            else:
                throttle -= THROTTLE_RATE
                if throttle < throttle_target:
                    throttle = throttle_target

        if idle_phase and elapsed < 5.0:
            rpm = IDLE_RPM
            throttle = 0.0
            throttle_target = 0.0
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
                if key == b'\xe0H':  # Flecha arriba (acelerar)
                    throttle_target = min(1.0, throttle_target + THROTTLE_STEP)
                elif key == b'\xe0P':  # Flecha abajo (frenar)
                    throttle_target = max(0.0, throttle_target - THROTTLE_STEP)
                elif key == b'\xe0M':  # Flecha derecha (subir marcha)
                    if gear < len(GEAR_RATIOS) - 1:
                        prev_ratio = GEAR_RATIOS[gear]
                        gear += 1
                        next_ratio = GEAR_RATIOS[gear]
                        rpm *= (next_ratio / prev_ratio)
                        throttle_target *= THROTTLE_DROP
                        throttle_rebote = True
                        print(f"\n>>> CAMBIO A MARCHA {gear+1}, RPM ajustado a {rpm:.0f}")
                elif key == b'\xe0K':  # Flecha izquierda (bajar marcha)
                    if gear > 0:
                        prev_ratio = GEAR_RATIOS[gear]
                        gear -= 1
                        next_ratio = GEAR_RATIOS[gear]
                        rpm *= (next_ratio / prev_ratio)
                        throttle_target *= THROTTLE_DROP
                        throttle_rebote = True
                        print(f"\n<<< RETROCEDE A MARCHA {gear+1}, RPM ajustado a {rpm:.0f}")
            else:
                if key == '1':
                    modo_envio_constante = 1
                    print("\n>>>\n TPS_MIN\n")
                elif key == '2':
                    modo_envio_constante = 2
                    print("\n>>>\n TPS_MAX \n")
                elif key == '3':
                    modo_envio_constante = 3
                    print("\n>>>\n  MAP_MIN \n")
                elif key == '4':
                    modo_envio_constante = 4
                    print("\n>>>\n  MAP_MAX \n")
                elif key == '5':
                    modo_envio_constante = 0
                    print("\n>>> Regresando a modo normal")
                else:
                    print(f"\n>>> Enviando comando: '{key}'")
                    if not DEBUG_MODE and ser:
                        enviar_comando(ser, key)

        time.sleep(DT)

    print("\n>>> Simulación finalizada.")
finally:
    running = False
    thread_send.join()
    thread_read.join()
    if ser:
        ser.close()
