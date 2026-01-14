import serial
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from tkinter import Tk, Scale, HORIZONTAL, Label, StringVar
import threading
import time

# Parámetros de conexión
puerto_serial = 'COM4'  # Cambia esto según tu puerto
baud_rate = 115200
archivo_csv = 'datos_pid.csv'

# Conexión serial
ser = serial.Serial(puerto_serial, baud_rate, timeout=1)
time.sleep(2)

# Variables globales
datos = {'Tiempo': [], 'Setpoint': [], 'Temperatura': [], 'PWM': [], 'Error': []}
start_time = time.time()
Kp = 2.5
Ki = 0.4
Kd = 3.5

# Variables de datos actuales
valores_actuales = {
    "setpoint": 0.0,
    "temperatura": 0.0,
    "pwm": 0,
    "error": 0.0
}

# Función para enviar PID a la ESP32
def enviar_pid():
    comando = f"K {Kp:.2f} {Ki:.2f} {Kd:.2f}\n"
    ser.write(comando.encode())

# Crear ventana con sliders y display en tiempo real
def crear_interface():
    global Kp, Ki, Kd

    def actualizar(val=None):
        nonlocal slider_kp, slider_ki, slider_kd
        global Kp, Ki, Kd
        Kp = slider_kp.get()
        Ki = slider_ki.get()
        Kd = slider_kd.get()
        enviar_pid()

    root = Tk()
    root.title("Ajuste PID y Lectura en Tiempo Real")

    # Sliders
    Label(root, text="Kp").pack()
    slider_kp = Scale(root, from_=0, to=10, resolution=0.1, orient=HORIZONTAL, command=actualizar)
    slider_kp.set(Kp)
    slider_kp.pack()

    Label(root, text="Ki").pack()
    slider_ki = Scale(root, from_=0, to=5, resolution=0.05, orient=HORIZONTAL, command=actualizar)
    slider_ki.set(Ki)
    slider_ki.pack()

    Label(root, text="Kd").pack()
    slider_kd = Scale(root, from_=0, to=10, resolution=0.1, orient=HORIZONTAL, command=actualizar)
    slider_kd.set(Kd)
    slider_kd.pack()

    # Etiquetas de datos actuales
    estado_var = StringVar()
    estado_label = Label(root, textvariable=estado_var, font=('Consolas', 12), justify='left')
    estado_label.pack(pady=10)

    def actualizar_valores():
        while True:
            texto = (
                f"Setpoint:    {valores_actuales['setpoint']:.1f} °C\n"
                f"Temperatura: {valores_actuales['temperatura']:.1f} °C\n"
                f"PWM:         {valores_actuales['pwm']:2d}\n"
                f"Error:       {valores_actuales['error']:.2f} %"
            )
            estado_var.set(texto)
            time.sleep(0.2)

    threading.Thread(target=actualizar_valores, daemon=True).start()
    root.mainloop()

# Iniciar GUI en hilo aparte
threading.Thread(target=crear_interface, daemon=True).start()

# Gráfica
fig, ax = plt.subplots()
line_temp, = ax.plot([], [], label='Temperatura (°C)')
line_set, = ax.plot([], [], label='Setpoint (°C)', linestyle='--')
ax.set_ylim(20, 90)
ax.set_xlim(0, 60)
ax.set_xlabel('Tiempo (s)')
ax.set_ylabel('Temperatura (°C)')
ax.legend()
plt.title('Control de temperatura con PID')

def animar(i):
    global datos
    while ser.in_waiting:
        linea = ser.readline().decode().strip()
        if linea.startswith("Setpoint:"):
            try:
                partes = linea.replace("Setpoint:", "").replace("Temp:", "").replace("PWM:", "").replace("Error:", "").replace("°C", "").replace("%", "").split()
                if len(partes) >= 4:
                    tiempo_actual = time.time() - start_time
                    setpoint = float(partes[0])
                    temperatura = float(partes[1])
                    pwm = int(partes[2])
                    error = float(partes[3])

                    # Guardar valores actuales
                    valores_actuales['setpoint'] = setpoint
                    valores_actuales['temperatura'] = temperatura
                    valores_actuales['pwm'] = pwm
                    valores_actuales['error'] = error

                    # Guardar datos
                    datos['Tiempo'].append(tiempo_actual)
                    datos['Setpoint'].append(setpoint)
                    datos['Temperatura'].append(temperatura)
                    datos['PWM'].append(pwm)
                    datos['Error'].append(error)

                    pd.DataFrame(datos).to_csv(archivo_csv, index=False)

                    if tiempo_actual > ax.get_xlim()[1]:
                        ax.set_xlim(0, tiempo_actual + 10)

                    line_temp.set_data(datos['Tiempo'], datos['Temperatura'])
                    line_set.set_data(datos['Tiempo'], datos['Setpoint'])

            except Exception as e:
                print("Error:", e)

    return line_temp, line_set

ani = FuncAnimation(fig, animar, interval=200)
plt.tight_layout()
plt.show()
