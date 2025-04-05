from machine import Pin, I2C
import time
import VL53L0X  # Asegúrate de tener la librería VL53L0X para MicroPython

# Configuración del bus I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21))  # Ajusta los pines según tu placa

# Crear objeto para el sensor VL53L0X
sensor = VL53L0X.VL53L0X(i2c)

# Inicializar el sensor
if not sensor.begin():
    print("Error: No se encontró el sensor VL53L0X.")
    while True:  # Se detiene la ejecución si no se detecta el sensor
        pass

print("Sensor VL53L0X listo para medir distancias.")

while True:
    medida = sensor.ranging_test()  # Realizar medición

    if medida.RangeStatus != 4:  # El estado 4 indica un error en la medición
        print("Distancia: {} mm".format(medida.RangeMilliMeter))
    else:
        print("Error en la medición")

    time.sleep_ms(500)  # Pequeña pausa entre mediciones