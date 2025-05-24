import neopixel
import time
from machine import Pin  # No olvides importar Pin para usarlo

MATRIX_PIN = 23   # Cambia al pin que uses
NUM_LEDS = 64     # Número de LEDs en la matriz
matrix = neopixel.NeoPixel(Pin(MATRIX_PIN), NUM_LEDS)

class Matriz:

    @staticmethod
    def set_matrix_color(r, g, b, intensity=0.5):
        intensity = max(0.0, min(intensity, 1.0))
        r = int(r * intensity)
        g = int(g * intensity)
        b = int(b * intensity)
        for i in range(NUM_LEDS):
            matrix[i] = (r, g, b)
        matrix.write()

    @staticmethod
    def encender_matriz():
        # Cyan claro: bastante verde y azul, poco rojo
        Matriz.set_matrix_color(0, 115, 115, intensity=0.2)

    @staticmethod
    def apagar_matriz():
        Matriz.set_matrix_color(0, 0, 0, intensity=1.0)

 
