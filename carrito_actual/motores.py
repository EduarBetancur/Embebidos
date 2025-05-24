# motores.py
from machine import Pin, PWM

class Motores:
    def __init__(self):
        # Definición de pines para el puente H L9110
        # Asegúrate de que estos pines coincidan con tu configuración
        # Motor Izquierdo (A)
        self.Motor_A1_pin = 17  # Control de dirección 1 del Motor A (ej. Adelante)
        self.Motor_A2_pin = 19  # Control de dirección 2 del Motor A (ej. Atrás)
        # Motor Derecho (B)
        self.Motor_B1_pin = 2   # Control de dirección 1 del Motor B (ej. Atrás)
        self.Motor_B2_pin = 4   # Control de dirección 2 del Motor B (ej. Adelante)

        self.Motor_A1 = PWM(Pin(self.Motor_A1_pin))
        self.Motor_A2 = PWM(Pin(self.Motor_A2_pin))
        self.Motor_B1 = PWM(Pin(self.Motor_B1_pin))
        self.Motor_B2 = PWM(Pin(self.Motor_B2_pin))

        pwm_freq = 500
        self.Motor_A1.freq(pwm_freq)
        self.Motor_A2.freq(pwm_freq)
        self.Motor_B1.freq(pwm_freq)
        self.Motor_B2.freq(pwm_freq)
        self.detener_motores()

    def motor_izquierdo(self, velocidad):
        velocidad = int(velocidad) # Asegurarse que es entero para duty
        if velocidad > 0:
            self.Motor_A1.duty(min(abs(velocidad), 1023))
            self.Motor_A2.duty(0)
        elif velocidad < 0:
            self.Motor_A1.duty(0)
            self.Motor_A2.duty(min(abs(velocidad), 1023))
        else:
            self.Motor_A1.duty(0)
            self.Motor_A2.duty(0)

    def motor_derecho(self, velocidad):
        velocidad = int(velocidad) # Asegurarse que es entero para duty
        if velocidad > 0:
            self.Motor_B1.duty(0)
            self.Motor_B2.duty(min(abs(velocidad), 1023))
        elif velocidad < 0:
            self.Motor_B1.duty(min(abs(velocidad), 1023))
            self.Motor_B2.duty(0)
        else:
            self.Motor_B1.duty(0)
            self.Motor_B2.duty(0)

    def mover_adelante(self, velocidad):
        self.motor_izquierdo(velocidad)
        self.motor_derecho(1.2*velocidad)

    def mover_atras(self, velocidad):
        self.motor_izquierdo(-velocidad)
        self.motor_derecho(-velocidad)

    def girar_izquierda_pivot(self, velocidad): # Robot gira sobre su centro
        self.motor_izquierdo(-velocidad)
        self.motor_derecho(velocidad)

    def girar_derecha_pivot(self, velocidad): # Robot gira sobre su centro
        self.motor_izquierdo(velocidad)
        self.motor_derecho(-velocidad)

    def detener_motores(self):
        self.motor_izquierdo(0)
        self.motor_derecho(0)
        
if __name__ == "__main__":
    motor = Motores()
    motor.motor_izquierdo(200)
    motor.motor_derecho(-250)
    while True:
        pass