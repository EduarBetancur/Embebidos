import cv2
import numpy as np
from picamera2 import Picamera2
import paho.mqtt.client as mqtt

# ---------------- CONFIGURACIÓN MQTT ---------------- #
MQTT_BROKER = "192.168.183.115"  # Cambia por la IP de tu broker o ESP32
MQTT_PORT = 1883
TOPIC_APAGAR_MATRIZ    = "robot/matriz/off"
TOPIC_COORDENADA_DEST  = "robot/destino"
TOPIC_COORD_ACTUAL     = "robot/coordenada_actual"
TOPIC_LLEGADA          = "robot/llegada"

# Bandera de llegada
robot_ha_llegado = False

# Callback cuando llega un mensaje al tópico robot/llegada
def on_message(client, userdata, msg):
    global robot_ha_llegado
    if msg.topic == TOPIC_LLEGADA:
        print("[MQTT] Robot ha llegado al destino:", msg.payload.decode())
        robot_ha_llegado = True

# Inicializar cliente MQTT y conectar
client = mqtt.Client()
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT)
client.loop_start()
client.subscribe(TOPIC_LLEGADA)

# Publicar coordenada destino (una vez al inicio)
destino = (5, 1)  # nueva coordenada válida para 6x8
mensaje_destino = f"{destino[0]},{destino[1]}"
client.publish(TOPIC_COORDENADA_DEST, mensaje_destino)
print(f"[MQTT] Coordenada destino publicada: {mensaje_destino}")

# ---------------- CONFIGURACIÓN CÁMARA Y CUADRÍCULA ---------------- #
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (480, 640), "format": "RGB888"})
picam2.configure(config)
picam2.start()

COLS, ROWS   = 6, 8
CELL_SIZE    = 80   # 480/6 = 80 → cuadrado
WIDTH, HEIGHT = 480, 640
GRID_HEIGHT  = ROWS * CELL_SIZE
GRID_WIDTH   = COLS * CELL_SIZE
MARGIN_TOP   = 0
MARGIN_LEFT  = 0

# Rango HSV para cian claro
lower_cyan = np.array([80, 50, 100])
upper_cyan = np.array([100, 255, 255])

cv2.namedWindow("Vista de Camara")
cv2.namedWindow("Mascara limpia")

# Bandera para publicar apagado solo una vez
matriz_detectada = False

while True:
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (WIDTH, HEIGHT))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Detección color cian claro (matriz LED)
    mask = cv2.inRange(hsv, lower_cyan, upper_cyan)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    display = frame.copy()

    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cell_highlight = None

    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            if 0 <= cx < WIDTH and 0 <= cy < HEIGHT:
                col = cx // CELL_SIZE
                row = cy // CELL_SIZE
                if 0 <= col < COLS and 0 <= row < ROWS:
                    cell_highlight = (col, row)

                    if not matriz_detectada:
                        mensaje_actual = f"{col},{row}"
                        # Publica coordenada actual
                        client.publish(TOPIC_COORD_ACTUAL, mensaje_actual)
                        print(f"[MQTT] Coordenada actual publicada: {mensaje_actual}")
                        # Vuelve a publicar la coordenada de destino
                        client.publish(TOPIC_COORDENADA_DEST, mensaje_destino)
                        print(f"[MQTT] Coordenada destino (reenvío): {mensaje_destino}")
                        # Apagar la matriz detectada
                        client.publish(TOPIC_APAGAR_MATRIZ, mensaje_actual)
                        print(f"[MQTT] Matriz detectada en ({col},{row}) → enviado apagado")
                        matriz_detectada = True

            cv2.circle(display, (cx, cy), 5, (255, 0, 255), -1)
            cv2.putText(display, f"({cx},{cy})", (cx + 5, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

    # Dibujar cuadrícula
    for i in range(COLS + 1):
        x = MARGIN_LEFT + i * CELL_SIZE
        cv2.line(display, (x, MARGIN_TOP), (x, MARGIN_TOP + GRID_HEIGHT), (0, 255, 255), 1)
    for j in range(ROWS + 1):
        y = MARGIN_TOP + j * CELL_SIZE
        cv2.line(display, (MARGIN_LEFT, y), (MARGIN_LEFT + GRID_WIDTH, y), (0, 255, 255), 1)

    # Resaltar celda donde se detectó la matriz LED
    if cell_highlight:
        i, j = cell_highlight
        x1, y1 = i * CELL_SIZE, j * CELL_SIZE
        x2, y2 = x1 + CELL_SIZE, y1 + CELL_SIZE
        cv2.rectangle(display, (x1, y1), (x2, y2), (255, 0, 0), 2)

    # Mostrar ventanas
    cv2.imshow("Vista de Camara", display)
    cv2.imshow("Mascara limpia", cv2.resize(cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR), (WIDTH, HEIGHT)))

    # Salida por tecla ESC o si el robot llega
    key = cv2.waitKey(1)
    if key == 27 or robot_ha_llegado:
        break

cv2.destroyAllWindows()
picam2.stop()
