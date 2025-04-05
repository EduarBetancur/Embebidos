import network
from time import sleep
import json
from umqtt.robust import MQTTClient
from machine import Pin, I2C, PWM
import VL53L0X
import neopixel

SSID = "EBM"
PWD = "Rv186502"

def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PWD)
    while not wlan.isconnected():
        sleep(1)
    print("Connected to WiFi")

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

# Configuración de la matriz RGB (Neopixel WS2812)
MATRIX_PIN = 23  # Cambia al pin que usas
NUM_LEDS = 64  # Número de LEDs en la matriz
matrix = neopixel.NeoPixel(Pin(MATRIX_PIN), NUM_LEDS)

def set_matrix_color(r, g, b):
    for i in range(NUM_LEDS):
        matrix[i] = (r, g, b)
    matrix.write()

def message(topic, msg):
    print(msg.decode())
    mensaje = msg.decode()
    if mensaje == "ADELANTE":
        motores.adelante(80)
    elif mensaje == "ATRAS":
        motores.atras(80)
    elif mensaje == "DETENER":
        motores.detener()
    elif mensaje == "RGB_ROJO":
        set_matrix_color(55, 0, 0)
    elif mensaje == "RGB_VERDE":
        set_matrix_color(255, 255, 255)
    elif mensaje == "RGB_AZUL":
        set_matrix_color(145, 55, 55)
    elif mensaje == "RGB_APAGAR":
        set_matrix_color(0, 0, 0)

client = MQTTClient("ESP_robot", "192.168.28.9")
client.set_callback(message)

wifi_connect()
client.connect()
print("Client connected")
client.subscribe(b"comandos")

# Configuración del bus I2C para el sensor VL53L0X
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
#sensor = VL53L0X.VL53L0X(i2c)

while True:
    client.wait_msg()
    #distancia = sensor.read()
    #distancia_json = json.dumps({"distancia": distancia})
    #client.publish(b"sensores/distancia", distancia_json)
    sleep(1)
