from machine import Pin, PWM, I2C
import time

class Robot:
    def __init__(self):
        # Configuración de pines del puente H L298N
        self._ENA = PWM(Pin(5))   # PWM Motor A
        self._IN1 = Pin(18, Pin.OUT)  # Dirección Motor A
        self._IN2 = Pin(19, Pin.OUT)  # Dirección Motor A
        self._ENB = PWM(Pin(25))  # PWM Motor B
        self._IN3 = Pin(4, Pin.OUT)  # Dirección Motor B
        self._IN4 = Pin(23, Pin.OUT)  # Dirección Motor B

        # Configuración de pines XSHUT de los sensores
        self._XSHUT_FRONTAL = Pin(32, Pin.OUT)
        self._XSHUT_LATERAL = Pin(33, Pin.OUT)

        # Potencia del motor (0-1023 en MicroPython)
        self._potencia = 512  # Ajustado para MicroPython (0-1023)

        # Inicialización de I2C para los sensores

        # Direcciones I2C de los sensores
        self._FRONTAL_ADDR = 0x30
        self._LATERAL_ADDR = 0x31

        # Configuración inicial de los sensores
        self._configurar_sensores()

    def _configurar_sensores(self):
        # Apagar ambos sensores antes de inicializar
        self._XSHUT_FRONTAL.value(0)
        self._XSHUT_LATERAL.value(0)
        time.sleep_ms(10)

        # Encender e inicializar sensor frontal
        self._XSHUT_FRONTAL.value(1)
        time.sleep_ms(10)
        # Aquí debes inicializar el sensor frontal con la dirección FRONTAL_ADDR

        # Encender e inicializar sensor lateral
        self._XSHUT_LATERAL.value(1)
        time.sleep_ms(10)
        # Aquí debes inicializar el sensor lateral con la dirección LATERAL_ADDR

    def _medir_distancia(self, sensor_addr):
        # Aquí debes implementar la lectura del sensor VL53L0X
        # Esto depende de la biblioteca que uses para el sensor en MicroPython
        # Por ahora, devolvemos un valor fijo para pruebas
        return 200  # Valor de ejemplo

    def adelante(self):
        self._ENA.duty(self._potencia)
        self._IN1.value(1)
        self._IN2.value(0)
        self._ENB.duty(self._potencia)
        self._IN3.value(1)
        self._IN4.value(0)

    def atras(self):
        self._ENA.duty(self._potencia)
        self._IN1.value(0)
        self._IN2.value(1)
        self._ENB.duty(self._potencia)
        self._IN3.value(0)
        self._IN4.value(1)

    def izquierda(self):
        self._ENA.duty(self._potencia)
        self._IN1.value(1)
        self._IN2.value(0)
        self._ENB.duty(self._potencia)
        self._IN3.value(0)
        self._IN4.value(1)

    def derecha(self):
        self._ENA.duty(self._potencia)
        self._IN1.value(0)
        self._IN2.value(1)
        self._ENB.duty(self._potencia)
        self._IN3.value(1)
        self._IN4.value(0)

    def detener(self):
        self._ENA.duty(0)
        self._IN1.value(0)
        self._IN2.value(0)
        self._ENB.duty(0)
        self._IN3.value(0)
        self._IN4.value(0)

    def ejecutar(self):
        while True:
            distancia_frontal = self._medir_distancia(self._FRONTAL_ADDR)
            distancia_lateral = self._medir_distancia(self._LATERAL_ADDR)
            
            print("Distancia Frontal:", distancia_frontal)
            print("Distancia Lateral:", distancia_lateral)
            
            if distancia_frontal > 100:
                self.adelante()
                print("Avanzando")
            else:
                self.detener()
                time.sleep_ms(5)
                
                if distancia_frontal <= 100 and distancia_lateral <= 100:
                    self.izquierda()
                    print("Girando a la izquierda")
                elif distancia_lateral > 100:
                    self.derecha()
                    print("Girando a la derecha")
                else:
                    self.izquierda()
                    print("Girando a la izquierda")
            
            time.sleep_ms(5)

if __name__ == "__main__":
    # Crear una instancia del robot y ejecutar el bucle principal
    robot = Robot()
    robot.ejecutar()
