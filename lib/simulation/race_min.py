import serial
import time

# Ajusta el puerto si es diferente
ser = serial.Serial('COM7', 115200, timeout=0.1)
print("[PYTHON] Esperando al arranque del ESP32...")
time.sleep(2.5)  # ⏳ Esperar a que el ESP32 arranque completamente

# Paso 1: Activar modo desarrollador
ser.write(b'd\n')
print("[PYTHON] ENVIADO: d (modo desarrollador)")
time.sleep(0.5)

# Paso 2: Activar simulación
ser.write(b'z\n')
print("[PYTHON] ENVIADO: z (modo simulación)")
time.sleep(0.5)

# Paso 3: Iniciar envío de datos simulados
tps = 1000
map_val = 2000

print("[PYTHON] Iniciando envío de datos simulados...\n")

try:
    while True:
        mensaje = f"tps_raw:{tps},map_raw:{map_val}\n"
        ser.write(mensaje.encode('utf-8'))
        print(f"[PYTHON] ENVIADO: {mensaje.strip()}")

        while ser.in_waiting > 0:
            respuesta = ser.readline().decode('utf-8', errors='ignore').strip()
            if respuesta:
                print(f"[ESP32] RESPUESTA: {respuesta}")

        tps += 1
        map_val += 1
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\n[PYTHON] Terminando...")

finally:
    ser.close()
