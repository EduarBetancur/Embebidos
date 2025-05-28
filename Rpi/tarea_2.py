import os
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"

import threading
import queue
import random
import cv2
import numpy as np
from picamera2 import Picamera2
import paho.mqtt.client as mqtt
import json
from rich import print
from rich.console import Console
from rich.table import Table
from datetime import datetime

# Crear consola global
console = Console()

# --- Configuración MQTT ---
broker_ip = "192.168.22.115"
mqtt_port = 1883
topic_iluminacion = "camara/localizacion"
topic_objetivo = "camara/objetivo"
topic_estado = "robot/estado"

client = mqtt.Client()
client.connect(broker_ip, mqtt_port)
client.loop_start()

# --- Inicializar cámara ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (320, 440)})
picam2.configure(config)
picam2.start()

# --- Rango HSV para detectar azul ---
lower_color = np.array([100, 150, 150])
upper_color = np.array([130, 255, 255])
calibrating = False
selecting_obj = False

# --- Parámetros de la matriz NxM ---
rows, cols = 8, 6
margin = 0

# --- Selección aleatoria inicial del objetivo ---
while True:
    objetivo_i = random.randint(0, rows - 1)
    objetivo_j = random.randint(0, cols - 1)
    if (objetivo_i, objetivo_j) != (0, 0):
        break

print(f"[bold green]Celda objetivo seleccionada aleatoriamente: ({objetivo_i+1},{objetivo_j+1})[/bold green]")

ultima_coordenada = None
shared_queue = queue.Queue()
frame_queue = queue.Queue(maxsize=1)

# --- Función de clic para calibrar color o elegir objetivo ---
def mouse_callback(event, x, y, flags, param):
    global lower_color, upper_color, calibrating, selecting_obj, objetivo_i, objetivo_j
    if event == cv2.EVENT_LBUTTONDOWN:
        if calibrating:
            pixel = hsv[y, x]
            lower_color = np.clip(np.array([pixel[0] - 10, pixel[1] - 50, pixel[2] - 50]), 0, 255)
            upper_color = np.clip(np.array([pixel[0] + 10, pixel[1] + 50, pixel[2] + 50]), 0, 255)
            lower_color[0] = max(0, lower_color[0])
            upper_color[0] = min(179, upper_color[0])
            print(f"[yellow]Nuevo rango HSV calibrado: {lower_color} - {upper_color}[/yellow]")
            calibrating = False
        elif selecting_obj:
            col = x // (frame.shape[1] // cols)
            row = y // (frame.shape[0] // rows)
            objetivo_i, objetivo_j = row, col
            print(f"[cyan]Nuevo objetivo seleccionado: ({objetivo_i+1},{objetivo_j+1})[/cyan]")
            selecting_obj = False

# --- Procesamiento de cámara ---
def camera_processing():
    global hsv, ultima_coordenada, frame
    print("[blue]Hilo de cámara iniciado[/blue]")
    while True:
        frame = picam2.capture_array()
        height, width, _ = frame.shape
        cell_width = (width - 2 * margin) // cols
        cell_height = (height - 2 * margin) // rows
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        illumination_matrix = np.zeros((rows, cols), dtype=int)

        for i in range(rows):
            for j in range(cols):
                x1, y1 = j * cell_width, i * cell_height
                x2, y2 = x1 + cell_width, y1 + cell_height
                cell_mask = mask[y1:y2, x1:x2]
                illumination_matrix[i, j] = cv2.countNonZero(cell_mask)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 1)

        # Dibuja objetivo
        x1 = objetivo_j * cell_width
        y1 = objetivo_i * cell_height
        x2 = x1 + cell_width
        y2 = y1 + cell_height
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
        cv2.putText(frame, "Objetivo", (x1+5, y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        max_val = np.max(illumination_matrix)
        if max_val > 0:
            i, j = np.unravel_index(np.argmax(illumination_matrix), illumination_matrix.shape)
            x1 = j * cell_width
            y1 = i * cell_height
            x2 = x1 + cell_width
            y2 = y1 + cell_height
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 3)
            cv2.putText(frame, f"Detectado ({i+1},{j+1})", (x1+5, y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            ultima_coordenada = (i, j)
            if (i, j) == (objetivo_i, objetivo_j):
                print("[bold green]¡Objetivo iluminado![/bold green]")

        try:
            frame_queue.put_nowait((frame.copy(), mask.copy()))
        except queue.Full:
            pass

# --- Callback MQTT ---
def on_message(client, userdata, msg):
    if msg.topic == topic_estado:
        try:
            data = json.loads(msg.payload.decode())
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            table = Table(title=f"[bold blue]Estado del Robot - {timestamp}[/bold blue]")
            table.add_column("Campo", style="cyan", no_wrap=True)
            table.add_column("Valor", style="magenta")

            for key, value in data.items():
                table.add_row(str(key), str(value))

            console.print(table)

            with open("registro_estado.txt", "a") as f:
                f.write(f"{timestamp} - {json.dumps(data)}\n")

        except Exception as e:
            print(f"[red][ERROR][/red] Al procesar mensaje MQTT: {e}")

client.subscribe(topic_estado)
client.on_message = on_message

# --- Hilo MQTT ---
def mqtt_thread():
    print("[blue]Hilo MQTT iniciado[/blue]")
    while True:
        msg = shared_queue.get()
        if msg == "salir":
            break
        elif msg == "listo":
            if ultima_coordenada:
                client.publish(topic_iluminacion, f"{ultima_coordenada[0]+1},{ultima_coordenada[1]+1}")
            client.publish(topic_objetivo, f"{objetivo_i+1},{objetivo_j+1}")
            print("[green]Coordenadas enviadas por mensaje 'listo'.[/green]")

# --- Lanzar hilos ---
t_cam = threading.Thread(target=camera_processing)
t_mqtt = threading.Thread(target=mqtt_thread)
t_cam.start()
t_mqtt.start()

# --- Interfaz visual y control ---
cv2.namedWindow("Tracking")
cv2.setMouseCallback("Tracking", mouse_callback)

print("[bold cyan]Teclas disponibles:[/bold cyan]")
print("[cyan]'q'[/cyan]: salir | [cyan]'c'[/cyan]: calibrar color | [cyan]'p'[/cyan]: publicar coordenadas | [cyan]'o'[/cyan]: seleccionar nuevo objetivo")

while True:
    if not frame_queue.empty():
        frame, mask = frame_queue.get()
        cv2.imshow("Tracking", frame)
        cv2.imshow("Mascara", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        shared_queue.put("salir")
        break
    elif key == ord('c'):
        calibrating = True
    elif key == ord('p'):
        if ultima_coordenada:
            client.publish(topic_iluminacion, f"{ultima_coordenada[0]+1},{ultima_coordenada[1]+1}")
            print(f"Mensaje enviado a '{topic_iluminacion}': {ultima_coordenada[0]+1},{ultima_coordenada[1]+1}")
        client.publish(topic_objetivo, f"{objetivo_i+1},{objetivo_j+1}")
        print(f"Mensaje enviado a '{topic_objetivo}': {objetivo_i+1},{objetivo_j+1}")
    elif key == ord('o'):
        selecting_obj = True

# --- Cierre del sistema ---
cv2.destroyAllWindows()
picam2.stop()
client.loop_stop()
client.disconnect()
t_cam.join()
t_mqtt.join()
print("[bold green]Sistema finalizado correctamente.[/bold green]")