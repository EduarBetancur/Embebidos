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
from multiprocessing import Process, Queue
from estado_robot import estado_robot
from scipy.optimize import linear_sum_assignment

console = Console()

broker_ip = "192.168.35.115"
mqtt_port = 1883
topic_iluminacion = "camara/localizacion"
topic_objetivo = "camara/objetivo"

client = mqtt.Client()
client.connect(broker_ip, mqtt_port)
client.loop_start()

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (320, 440)})
picam2.configure(config)
picam2.start()

lower_color = np.array([-10, 205, 205])
upper_color = np.array([10, 305, 305])
calibrating = False
selecting_obj = False

rows, cols = 8, 6
margin = 0

while True:
	objetivo_i = random.randint(0, rows - 1)
	objetivo_j = random.randint(0, cols - 1)
	if (objetivo_i, objetivo_j) != (0, 0):
		break

print(f"Celda objetivo seleccionada: ({objetivo_i+1},{objetivo_j+1})")

ultima_coordenada = None
shared_queue = queue.Queue()
frame_queue = queue.Queue(maxsize=1)
robots_actuales = []
figura_objetivo = []
nombre_figura = ""

estado_queue = Queue()
proceso_estado = Process(target=estado_robot, args=(estado_queue,))
proceso_estado.start()

def mouse_callback(event, x, y, flags, param):
	global lower_color, upper_color, calibrating, selecting_obj, objetivo_i, objetivo_j
	if event == cv2.EVENT_LBUTTONDOWN:
		if calibrating:
			pixel = hsv[y, x]
			lower_color = np.array([pixel[0] - 10, pixel[1] - 50, pixel[2] - 50])
			upper_color = np.array([pixel[0] + 10, pixel[1] + 50, pixel[2] + 50])
			print(f"Nuevo rango HSV: {lower_color} - {upper_color}")
			calibrating = False
		elif selecting_obj:
			col = x // (frame.shape[1] // cols)
			row = y // (frame.shape[0] // rows)
			objetivo_i, objetivo_j = row, col
			print(f"Nuevo objetivo seleccionado: ({objetivo_i+1},{objetivo_j+1})")
			selecting_obj = False

def generar_formacion_linea_vertical():
	col = cols // 2
	return [(fila, col) for fila in range(2, 7)]

def generar_formacion_linea_horizontal():
	row = rows // 2
	return [(row, col) for col in range(1, 6)]

def generar_formacion_cruz():
	return [(4, 3), (3, 3), (5, 3), (4, 2), (4, 4)]

def generar_formacion_cuadrado():
	return [(3,2), (3,3), (4,2), (4,3), (4,4)]

def seleccionar_figura():
	global figura_objetivo, nombre_figura
	print("\n[FIGURA] Elige la figura a formar:")
	print("1. LÃ­nea vertical")
	print("2. LÃ­nea horizontal")
	print("3. Cruz")
	print("4. Cuadrado")
	print("5. Cancelar")
	op = input("OpciÃ³n: ").strip()
	if op == "1":
		figura_objetivo = generar_formacion_linea_vertical()
		nombre_figura = "LÃ­nea vertical"
	elif op == "2":
		figura_objetivo = generar_formacion_linea_horizontal()
		nombre_figura = "LÃ­nea horizontal"
	elif op == "3":
		figura_objetivo = generar_formacion_cruz()
		nombre_figura = "Cruz"
	elif op == "4":
		figura_objetivo = generar_formacion_cuadrado()
		nombre_figura = "Cuadrado"
	else:
		figura_objetivo = []
		nombre_figura = ""
		print("[FIGURA] Cancelado.")
def asignar_robots_a_objetivos(actuales, objetivos):
    print("Asignar robots:")
    print("  actuales:", actuales)
    print("  objetivos:", objetivos)
    costo = np.zeros((len(actuales), len(objetivos)))
    for i, (xa, ya) in enumerate(actuales):
        for j, (xo, yo) in enumerate(objetivos):
            costo[i, j] = abs(xa - xo) + abs(ya - yo)
    print("  matriz costo:")
    print(costo)
    filas, columnas = linear_sum_assignment(costo)
    print("  filas:", filas, "columnas:", columnas)
    try:
        asignaciones = list(zip([actuales[i] for i in filas], [objetivos[j] for j in columnas]))
        print("  asignaciones:", asignaciones)
    except:
        pass
        #asignaciones = []
    return asignaciones


def camera_processing():
	global hsv, ultima_coordenada
	while True:
		frame = picam2.capture_array()
		height, width, _ = frame.shape
		cell_width = (width - 2 * margin) // cols
		cell_height = (height - 2 * margin) // rows
		hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
		mask = cv2.inRange(hsv, lower_color, upper_color)
		illumination_matrix = np.zeros((rows, cols), dtype=int)
		robots_actuales.clear()

		for i in range(rows):
			for j in range(cols):
				x1, y1 = j * cell_width, i * cell_height
				x2, y2 = x1 + cell_width, y1 + cell_height
				cell_mask = mask[y1:y2, x1:x2]
				illumination_matrix[i, j] = cv2.countNonZero(cell_mask)
				if illumination_matrix[i, j] > 150:
					robots_actuales.append((i+1, j+1))
				cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 1)

		for i, j in figura_objetivo:
			x1 = (j-1) * cell_width
			y1 = (i-1) * cell_height
			x2 = x1 + cell_width
			y2 = y1 + cell_height
			cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 2)

		x1 = objetivo_j * cell_width
		y1 = objetivo_i * cell_height
		x2 = x1 + cell_width
		y2 = y1 + cell_height
		cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
		cv2.putText(frame, "Objetivo", (x1+5, y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

		ultima_coordenada = None
		for i in range(rows):
			for j in range(cols):
				if illumination_matrix[i, j] > 150:
					x1 = j * cell_width
					y1 = i * cell_height
					x2 = x1 + cell_width
					y2 = y1 + cell_height
					cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 3)
					cv2.putText(frame, f"Detectado ({i+1},{j+1})", (x1+5, y1+20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
					ultima_coordenada = (i, j)  
			if (i, j) == (objetivo_i, objetivo_j):
				print("Â¡Objetivo iluminado!")

		try:
			frame_queue.put_nowait((frame.copy(), mask.copy()))
		except queue.Full:
			pass

def estado_listener():
	while True:
		estado = estado_queue.get()
		if isinstance(estado, dict):
			timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
			table = Table(title=f"[bold blue]Estado del Robot - {timestamp}[/bold blue]")
			table.add_column("Campo", style="cyan", no_wrap=True)
			table.add_column("Valor", style="magenta")

			for key, value in estado.items():
				table.add_row(str(key), str(value))

			console.print(table)

			with open("registro_estado.txt", "a") as f:
				f.write(f"{timestamp} - {json.dumps(estado)}\n")

			if estado.get("status") == "completado":
				print("[INFO] El robot ha completado su movimiento.")
def mqtt_thread():
	while True:
		msg = shared_queue.get()
		if msg == "salir":
			break
		elif msg == "listo":
			if ultima_coordenada:
				client.publish(topic_iluminacion, f"{ultima_coordenada[0]+1},{ultima_coordenada[1]+1}")
			client.publish(topic_objetivo, f"{objetivo_i+1},{objetivo_j+1}")
			print("Coordenadas enviadas por mensaje 'listo'.")

t_cam = threading.Thread(target=camera_processing)
t_mqtt = threading.Thread(target=mqtt_thread)
t_estado = threading.Thread(target=estado_listener)
t_cam.start()
t_mqtt.start()
t_estado.start()

cv2.namedWindow("Tracking")
cv2.setMouseCallback("Tracking", mouse_callback)

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
	elif key == ord('s'):
		seleccionar_figura()
	elif key == ord('f'):
		if not figura_objetivo:
			print("[ERROR] No has seleccionado figura  (presiona 'm').")
		elif len(robots_actuales) == 0:
			print("[ERROR] No se detectaron robots.")
		else:
			objetivos_usados = figura_objetivo  # <--- USAR TODOS LOS OBJETIVOS
			asignaciones = asignar_robots_a_objetivos(robots_actuales, objetivos_usados)
			for i, (actual, destino) in enumerate(asignaciones):
				client.publish(topic_iluminacion, f"{actual[0]},{actual[1]}")
				client.publish(f"robot/{i}/objetivo", f"{destino[0]},{destino[1]}")
				print(f"[ENViO] Robot {i}: actual {actual} -> objetivo {destino}")


cv2.destroyAllWindows()
picam2.stop()
client.loop_stop()
client.disconnect()
proceso_estado.terminate()
proceso_estado.join()