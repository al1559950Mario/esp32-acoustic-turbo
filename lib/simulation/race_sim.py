import time
import serial

# --- Parámetros de simulación ---
IDLE_RPM       = 800
MAX_RPM        = 7000
RPM_RISE_COEF  = 0.1      # suaviza aceleración de RPM
MAP_RISE_COEF  = 0.05     # suaviza transición de voltaje MAP
DT             = 0.1      # paso de tiempo en segundos
THROTTLE_STEP  = 0.01     # incremento de throttle por paso

# Voltajes TPS: suelto = 2.25 V; pisado a fondo = 0.5 V
TPS_V_OPEN     = 2.25
TPS_V_CLOSED   = 0.5

# Voltajes MAP: vacío (~-15inHg) = 3.05 V; atmosférica = 3.26 V
MAP_V_IDLE     = 3.05
MAP_V_MAX      = 3.26

# Relaciones de caja (simplificado)
GEAR_RATIOS    = [3.8, 2.2, 1.5, 1.0, 0.8]
SHIFT_RPM      = 6000

# Puerto serial (ajusta a tu sistema)
SERIAL_PORT    = "COM1"
BAUDRATE       = 115200

# --- Inicialización ---
throttle = 0.0        # 0.0 = suelto, 1.0 = fondo
rpm      = IDLE_RPM
map_v    = MAP_V_IDLE
gear     = 0

# Abre puerto serial
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
time.sleep(1)

print("Iniciando simulación de carrera...")
start_time = time.time()

try:
    while gear < len(GEAR_RATIOS):
        # 1) Aumentar throttle sin saltos
        throttle = min(1.0, throttle + THROTTLE_STEP)

        # 2) Actualizar RPM hacia target según throttle
        target_rpm = IDLE_RPM + throttle*(MAX_RPM - IDLE_RPM)
        rpm += (target_rpm - rpm) * RPM_RISE_COEF

        # 3) Actualizar voltaje MAP suavemente
        target_map_v = MAP_V_IDLE + throttle*(MAP_V_MAX - MAP_V_IDLE)
        map_v += (target_map_v - map_v) * MAP_RISE_COEF

        # 4) Calcular voltaje TPS invertido
        tps_v = TPS_V_OPEN + (TPS_V_CLOSED - TPS_V_OPEN)*throttle

        # 5) Detectar cambio de marcha
        if rpm >= SHIFT_RPM and gear < len(GEAR_RATIOS)-1:
            prev_ratio = GEAR_RATIOS[gear]
            gear += 1
            next_ratio = GEAR_RATIOS[gear]
            rpm = rpm * (next_ratio/prev_ratio)
            print(f">>> Cambio a marcha {gear+1}, rpm ajustado a {rpm:.0f}")

        # 6) Imprimir valores para debug
        elapsed = time.time() - start_time
        log = (f"[{elapsed:5.2f}s] Gear:{gear+1} | "
               f"RPM:{rpm:0.0f} | Throttle:{throttle:0.2f} | "
               f"TPS:{tps_v:0.3f}V | MAP:{map_v:0.3f}V")
        print(log)

        # 7) Convertir voltajes simulados a valores crudos ADC
        def volt_to_adc(volts):
            return int((volts / 3.3) * 4095)

        tps_adc = volt_to_adc(tps_v)
        map_adc = volt_to_adc(map_v)

        # 8) Enviar valores simulados como ADC crudo por Serial
        payload = f"tps_raw:{tps_adc},map_raw:{map_adc}\n"
        ser.write(payload.encode('utf-8'))


        log = (
            f"[{elapsed:5.2f}s] Gear:{gear+1} | "
            f"Throttle:{throttle:.2f} | RPM:{rpm:.0f} | "
            f"TPS:{tps_v:.3f}V | MAP:{map_v:.3f}V"
        )
        print(log)


        time.sleep(DT)

    print("Simulación finalizada.")
finally:
    ser.close()
