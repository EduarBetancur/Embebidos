import network
from time import sleep
import json
from umqtt.robust import MQTTClient
from main import Robot

SSID = "EBM"
PWD = "Rv186502"

def wifi_connect ():
    wlan = network.WLAN(network.STA_IF)
    wlan.active (True)
    wlan.connect(SSID, PWD)
    while not wlan.isconnected():
        sleep(1)
    print ("connected to WiFi")
        
def message (topic, msg):
    print(msg.decode())
    
client =MQTTClient("ESP_robot", "192.168.210.212")
client.set_callback(message)

wifi_connect()
client.connect()
print("Client connected")
client.subscribe(b"test/topic")
robot = Robot()

while True:
    client.wait_msg()
    frontal = robot._medir_distancia(robot._FRONTAL_ADDR)
    data =json.dumps({"distance": frontal})
    client.publish(b"rpi/topic", data)
    
    
    