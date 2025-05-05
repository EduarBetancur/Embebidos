import network
import time
import json
from umqtt.robust import MQTTClient
from machine import Pin, I2C, PWM
import neopixel
# import VL53L0X  # Descomenta si usas el sensor VL53L0X

# Configuración de red y broker
SSID = "EBM"
PWD = "Rv186502"
BROKER_IP = "192.168.105.115"  # IP de la Raspberry Pi (broker)

# Configuración de la matriz RGB (Neopixel WS2812)
MATRIX_PIN = 23   # Cambia al pin que uses
NUM_LEDS = 64     # Número de LEDs en la matriz
matrix = neopixel.NeoPixel(Pin(MATRIX_PIN), NUM_LEDS)

def set_matrix_color(r, g, b, intensity=0.5):
    intensity =max(0.0, min(intensity,1.0))
    r=int(r*intensity)
    g=int(g*intensity)
    b=int(b*intensity)
    for i in range(NUM_LEDS):
        matrix[i] = (r, g, b)
    matrix.write()

def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Conectando a WiFi...")
        wlan.connect(SSID, PWD)
        retries = 0
        while not wlan.isconnected():
            time.sleep(1)
            retries += 1
            if retries > 20:
                print("No se pudo conectar a WiFi. Reiniciando...")
                import machine
                machine.reset()
    print("Conectado a WiFi:", wlan.ifconfig())

# Definición de la clase Motores para controlar el robot
class Motores:
    def __init__(self, enA, pA1, pA2, enB, pB1, pB2):
        self.enA = PWM(Pin(enA), freq=1000, duty=0)
        self.enB = PWM(Pin(enB), freq=1000, duty=0)
        self.pinA1 = Pin(pA1, Pin.OUT)
        self.pinA2 = Pin(pA2, Pin.OUT)
        self.pinB1 = Pin(pB1, Pin.OUT)
        self.pinB2 = Pin(pB2, Pin.OUT)

    def adelante(self, velocidad):
        duty = int(velocidad * 1023 / 100)
        self.enA.duty(duty)
        self.enB.duty(duty)
        self.pinA1.value(1)
        self.pinA2.value(0)
        self.pinB1.value(1)
        self.pinB2.value(0)
    
    def atras(self, velocidad):
        duty = int(velocidad * 1023 / 100)
        self.enA.duty(duty)
        self.enB.duty(duty)
        self.pinA1.value(0)
        self.pinA2.value(1)
        self.pinB1.value(0)
        self.pinB2.value(1)
    
    def detener(self):
        self.enA.duty(0)
        self.enB.duty(0)
        self.pinA1.value(0)
        self.pinA2.value(0)
        self.pinB1.value(0)
        self.pinB2.value(0)

# Instancia de Motores (actualiza los pines según tu hardware)
motores = Motores(enA=5, pA1=18, pA2=19, enB=17, pB1=16, pB2=4)

# Función para procesar mensajes MQTT
def message(topic, msg):
    topic = topic.decode().strip()
    command = msg.decode().strip()

    # Motor control
    if topic == "control/motores":
        if command.lower() == "adelante":
            motores.adelante(80)
            print("motores se mueven adelante")
        elif command.lower() == "atras":
            motores.atras(80)
            print("motores se mueven atrás")
        elif command.lower() == "detener":
            motores.detener()
            print("motores se detienen")
        elif command.lower() == "girar_derecha":
            motores.girar_derecha(80)
            print("motores giran a la derecha")
        elif command.lower() == "girar_izquierda":
            motores.girar_izquierda(80)
            print("motores giran a la izquierda")
        else:
            print(f"motores: comando desconocido '{command}'")

    # Matrix control
    elif topic == "control/matriz":
        if command.lower() == "encender":
            matriz.encender()
            print("matriz encendida")
        elif command.lower() == "apagar":
            matriz.apagar()
            print("matriz apagada")
        elif command.lower() == "ubicacion":
            matriz.mostrar_ubicacion()
            print("matriz muestra la ubicación")
        else:
            print(f"matriz: comando desconocido '{command}'")

    # Sensores frontales
    elif topic == "control/sensores/distancia_frontal":
        if command.lower() == "iniciar":
            sensores.distancia_frontal.iniciar()
            print("sensor de distancia frontal iniciado")
        else:
            print(f"sensor frontal: comando desconocido '{command}'")

    # Sensores laterales
    elif topic == "control/sensores/distancia_lateral":
        if command.lower() == "iniciar":
            sensores.distancia_lateral.iniciar()
            print("sensor de distancia lateral iniciado")
        else:
            print(f"sensor lateral: comando desconocido '{command}'")

    # Giroscopio
    elif topic == "control/sensores/giroscopio":
        if command.lower() == "iniciar":
            sensores.giroscopio.iniciar()
            print("giroscopio iniciado")
        elif command.lower() == "giro_90":
            sensores.giroscopio.giro_90()
            print("giroscopio realizó giro de 90°")
        else:
            print(f"giroscopio: comando desconocido '{command}'")

    else:
        print(f"Topic no gestionado: '{topic}'")


# Inicializa el cliente MQTT
client = MQTTClient("ESP_robot", BROKER_IP, keepalive=60)
client.set_callback(message)

def connect_mqtt():
    while True:
        try:
            client.connect()
            print("Conectado al broker MQTT")
           
            break  # Salir del loop si la conexión es exitosa
        except Exception as e:
            print("Error al conectar con el broker MQTT:", e)
            time.sleep(5)  # Espera antes de reintentar

# Conexión WiFi y MQTT
wifi_connect()
connect_mqtt()
topics = [
        b"control/matriz",
        b"control/motores",
        b"control/sensores/distancia_frontal",
        b"control/sensores/distancia_lateral",
        b"control/sensores/giroscopio"
    ]
            
for t in topics:
        client.subscribe(t)
        print("Suscrito a", t.decode())

# Configuración del bus I2C para el sensor VL53L0X (descomenta si lo usas)
# i2c = I2C(0, scl=Pin(22), sda=Pin(21))
# sensor = VL53L0X.VL53L0X(i2c)

while True:
    try:
        # client.publish(b"comando", "test")
        client.wait_msg()
    except Exception as e:
        print("Error en espera de mensajes MQTT:", e)
        try:
            client.disconnect()
        except:
            pass
        print("Intentando reconectar MQTT...")
        connect_mqtt()
    time.sleep(1)
