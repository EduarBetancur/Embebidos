import time
import machine
# Asumiendo que estas clases están definidas en archivos .py correspondientes
from MPU6050 import MPU6050         # Debes tener este archivo
from pid import PID                 # Código proporcionado anteriormente
from sensores import SensoresLaser # Debes tener tu clase SensoresLaser.py
from matriz import Matriz           # Debes tener tu clase Matriz.py
from motores import Motores         # Importar la clase Motores para control directo

# --- Configuración de Parámetros ---
VELOCIDAD_BASE_AVANCE    = 250
VELOCIDAD_GIRO           = 300   # Velocidad para el giro principal (0-1023)
VELOCIDAD_CORRECCION_GIRO= 250   # Velocidad para el ajuste fino del giro
UMBRAL_OBSTACULO_MM      = 150   # Distancia frontal para considerar obstáculo
UMBRAL_LATERAL_MM        = 100   # Distancia lateral para considerar espacio suficiente para girar
ANGULO_GIRO_GRADOS       = 90
# Constantes PID (AJUSTAR ESTOS VALORES EXPERIMENTALMENTE)
PID_KP = 1
PID_KI = 0.08
PID_KD = 0.55

# --- Inicialización de Componentes ---
mpu        = MPU6050()
pid_ctrl   = PID(mpu, Kp=PID_KP, Ki=PID_KI, Kd=PID_KD)
sensores   = SensoresLaser()
motores    = Motores()           # Instancia de Motores para control de hardware

# --- Función de Prueba de Deriva ---
def prueba_deriva(velocidad=VELOCIDAD_BASE_AVANCE, duracion_s=15):
    """
    Mide la deriva angular al avanzar en línea recta sin corrección.
    Retorna el ángulo de deriva en grados.
    """
    print("Iniciando prueba de deriva...")
    # Asegurarse de estar detenidos antes de empezar
    motores.detener_motores()
    time.sleep_ms(500)

    yaw_inicio = mpu.actualizarYaw()
    print(f"Yaw inicial: {yaw_inicio:.2f}°")

    # Avanzar sin corrección PID
    motores.mover_adelante(velocidad)
    time.sleep(duracion_s)
    motores.detener_motores()

    yaw_final = mpu.actualizarYaw()
    deriva = yaw_final - yaw_inicio
    print(f"Yaw final: {yaw_final:.2f}° → deriva de {deriva:.2f}° en {duracion_s}s")
    return deriva

# --- Calibración y Estado Inicial ---
print("Calibrando MPU6050...")
mpu.calibrarGiroscopio()
print("Calibración completada.")

pid_ctrl.reset_pid()
yaw_referencia_actual = mpu.actualizarYaw()
print(f"Yaw de referencia inicial: {yaw_referencia_actual:.2f}°")

# Ejecutar prueba de deriva antes de la navegación principal
deriva = prueba_deriva()
print(f"Deriva medida: {deriva:.2f}°")

# --- Función Principal de Movimiento ---
def mover_controlado():
    global yaw_referencia_actual
    try:
        print("Iniciando prueba de movimiento con PID y esquiva de obstáculos...")
        while True:
            # Lectura de sensores y yaw
            dist_frontal, dist_derecha = sensores.leer_distancias()
            yaw_actual = mpu.actualizarYaw()

            if dist_frontal <= UMBRAL_OBSTACULO_MM:
                print(f"Obstáculo frontal detectado ({dist_frontal}mm). Deteniendo y decidiendo giro...")
                motores.detener_motores()
                time.sleep_ms(100)

                # Decidir sentido de giro
                if dist_derecha > UMBRAL_LATERAL_MM:
                    angulo_a_girar = -ANGULO_GIRO_GRADOS
                    print(f"Espacio detectado a la derecha. Girando {angulo_a_girar}°.")
                else:
                    angulo_a_girar = ANGULO_GIRO_GRADOS
                    print(f"Derecha obstruida ({dist_derecha}mm). Girando {angulo_a_girar}° a la izquierda.")

                # Realizar giro con PID
                yaw_objetivo = (yaw_referencia_actual + angulo_a_girar + 360) % 360
                pid_ctrl.turn_to(angulo_a_girar, VELOCIDAD_GIRO, umbral_angulo=3.0, timeout_ms=700)
                pid_ctrl.fine_tune_angle(
                    yaw_objetivo_absoluto=yaw_objetivo,
                    velocidad_correccion=VELOCIDAD_CORRECCION_GIRO,
                    umbral_correccion=3,
                    timeout_ms=300
                )

                # Actualizar referencia y reset PID
                yaw_referencia_actual = mpu.actualizarYaw()
                print(f"Nuevo yaw de referencia: {yaw_referencia_actual:.2f}°")
                pid_ctrl.reset_pid()
                time.sleep_ms(100)

            else:
                pid_ctrl.mover_recto_con_pid(VELOCIDAD_BASE_AVANCE, yaw_actual, yaw_referencia_actual)

            time.sleep_ms(50)

    except KeyboardInterrupt:
        print("Prueba detenida por el usuario.")
    except Exception as e:
        print(f"Error crítico: {e}")
        import sys; sys.print_exception(e)
    finally:
        print("Deteniendo motores y apagando componentes...")
        motores.detener_motores()
        Matriz.apagar_matriz()

if __name__ == "__main__":
    mover_controlado()
