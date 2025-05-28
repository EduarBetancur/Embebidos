import paho.mqtt.client as mqtt
from time import sleep
import json

# Variable para evitar multiples giros seguidos sin haber avanzado
esperando_giro = False

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe("sensores")
    client.publish("control/motores", "ENCENDER_MATRIZ")
    print("? Comando inicial enviado: mover_adelante")

def on_message(client, userdata, msg):
    global esperando_giro

    data = msg.payload.decode()
    try:
        # Intentamos cargar como JSON, pero puede ser un numero directo
        try:
            data = json.loads(data)
        except json.JSONDecodeError:
            data = int(data)

        # Si es un dict, sacamos 'distance'; si no, asumimos que data es distancia
        if isinstance(data, dict):
            distancia = data['distance']
        else:
            distancia = data

        print(f"?? Distance: {distancia} mm")

        if distancia < 75 and not esperando_giro:
            esperando_giro = True
            print("?? Objeto cercano detectado. Iniciando giro...")
            client.publish("control/m", "ENCENDER_MATRIZ")

            # Espera para que el giro se realice (ajusta segun tu sistema)
            sleep(2)  # Por ejemplo: 2 segundos para completar el giro

            client.publish("control/motores", "mover_adelante")
            print("? Giro completado. Reanudando avance...")
            esperando_giro = False

    except (ValueError, KeyError) as e:
        print(f"?? Error procesando mensaje: {e}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("localhost", 1883, 60)
client.loop_forever()
