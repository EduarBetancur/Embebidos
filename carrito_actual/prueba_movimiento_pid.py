# Prueba_movimiento_pid.py
import time
import machine
# Asumiendo que estas clases están definidas en archivos .py correspondientes
from MPU6050 import MPU6050         # Debes tener este archivo
from pid import PID                 # Código proporcionado anteriormente
from sensores import SensoresLaser # Debes tener tu clase SensoresLaser.py
from matriz import Matriz           # Debes tener tu clase Matriz.py

# --- Configuración de Parámetros ---
VELOCIDAD_BASE_AVANCE = 250
VELOCIDAD_GIRO        = 300   # Velocidad para el giro principal (0-1023)
VELOCIDAD_CORRECCION_GIRO = 250 # Velocidad para el ajuste fino del giro
UMBRAL_OBSTACULO_MM   = 150   # Distancia frontal para considerar obstáculo
UMBRAL_LATERAL_MM     = 100   # Distancia lateral para considerar espacio suficiente para girar (ajusta según necesidad)
ANGULO_GIRO_GRADOS    = 90
# Constantes PID (AJUSTAR ESTOS VALORES EXPERIMENTALMENTE)
PID_KP = 1
PID_KI = 0.08
PID_KD = 0.55

# --- Inicialización de Componentes ---
mpu = MPU6050()
pid_ctrl = PID(mpu, Kp=PID_KP, Ki=PID_KI, Kd=PID_KD)
sensores = SensoresLaser() # Asume que leer_distancias() devuelve (frontal, derecha)
# matriz_led = Matriz() # Si la clase Matriz no es estática

# --- Calibración y Estado Inicial ---
print("Calibrando MPU6050...")
mpu.calibrarGiroscopio()
print("Calibración completada.")

pid_ctrl.reset_pid()
#Matriz.encender_matriz() # Asumiendo método estático
yaw_referencia_actual = mpu.actualizarYaw()
print(f"Yaw de referencia inicial: {yaw_referencia_actual:.2f}°")

# --- Función Principal de Movimiento ---
def mover_controlado():
    global yaw_referencia_actual
    try:
        print("Iniciando prueba de movimiento con PID y esquiva de obstáculos...")
        while True:
            # Lectura de sensores al inicio de cada ciclo del bucle principal
            dist_frontal, dist_derecha = sensores.leer_distancias()
            # Si tuvieras un sensor izquierdo, sería: dist_frontal, dist_derecha, dist_izquierda = sensores.leer_distancias_completas()
            yaw_actual = mpu.actualizarYaw()
            
            # Descomenta para depuración detallada:
            # print(f"DistF: {dist_frontal}mm, DistR: {dist_derecha}mm | Yaw: {yaw_actual:.2f}°, RefYaw: {yaw_referencia_actual:.2f}°")

            if dist_frontal <= UMBRAL_OBSTACULO_MM:
                print(f"Obstáculo frontal detectado ({dist_frontal}mm). Deteniendo y decidiendo giro...")
                pid_ctrl.detener_motores()
                time.sleep_ms(200) # Pausa para estabilizar antes de decidir

                angulo_a_girar = 0
                decision_tomada = False

                # Decisión de giro:
                # Prioridad 1: Intentar girar a la DERECHA si hay espacio.
                # Usamos dist_derecha que se leyó al inicio del bucle.
                # Puedes añadir una lectura fresca aquí si crees que es necesario:
                # _, dist_derecha_actual_para_decision = sensores.leer_distancias()
                
                print(f"Evaluando giro: Distancia derecha actual para decisión: {dist_derecha}mm")
                if dist_derecha > UMBRAL_LATERAL_MM  : # Se necesita un poco más de espacio lateral que el umbral frontal
                    angulo_a_girar = -ANGULO_GIRO_GRADOS  # Girar a la derecha (negativo)
                    print(f"Espacio detectado a la derecha. Girando {angulo_a_girar}°.")
                    decision_tomada = True
                else:
                    # Prioridad 2: Si la derecha no está libre, intentar girar a la IZQUIERDA.
                    # (Aquí se asume que si la derecha está bloqueada, la izquierda es la alternativa)
                    # Si tuvieras un sensor izquierdo, aquí lo consultarías.
                    angulo_a_girar = ANGULO_GIRO_GRADOS   # Girar a la izquierda (positivo)
                    print(f"Derecha obstruida o con poco espacio ({dist_derecha}mm). Intentando giro a la izquierda {angulo_a_girar}°.")
                    decision_tomada = True
                
                # Si, por alguna razón más compleja, no se pudiera tomar una decisión (ej. ambos lados medidos como bloqueados)
                # 'decision_tomada' podría ser False. Con la lógica actual, siempre se toma una.

                if decision_tomada and angulo_a_girar != 0:
                    # --- Realizar el giro decidido ---
                    print(f"Realizando giro de {angulo_a_girar}°.")
                    # El yaw de referencia para el cálculo del objetivo es el actual ANTES del giro.
                    yaw_objetivo_calculado_para_giro = (yaw_referencia_actual + angulo_a_girar + 360) % 360
                    
                    pid_ctrl.turn_to(angulo_a_girar, VELOCIDAD_GIRO, umbral_angulo=5.0, timeout_ms=7000) # Ajusta umbral_angulo si es necesario
                    
                    print(f"Giro principal hecho. Iniciando ajuste fino hacia: {yaw_objetivo_calculado_para_giro:.2f}° (Yaw ref actual: {yaw_referencia_actual:.2f}°)")
                    pid_ctrl.fine_tune_angle(
                        yaw_objetivo_absoluto=yaw_objetivo_calculado_para_giro,
                        velocidad_correccion=VELOCIDAD_CORRECCION_GIRO,
                        umbral_correccion=0.7, # Ajusta umbral_correccion si es necesario
                        timeout_ms=3000
                    )
                    
                    # Actualizar el yaw de referencia global DESPUÉS de la maniobra de giro y ajuste
                    yaw_referencia_actual = mpu.actualizarYaw()
                    print(f"Maniobra de giro completada. Nuevo Yaw de referencia: {yaw_referencia_actual:.2f}°")
                    pid_ctrl.reset_pid() # Importante resetear el PID para el siguiente movimiento
                    
                    # Pausa después del giro para que los sensores se estabilicen
                    # y para que el robot no reaccione instantáneamente si sigue viendo un obstáculo
                    # debido a la inercia o al tiempo de procesamiento.
                    print("Pausa post-giro antes de reevaluar.")
                    time.sleep_ms(500) 
                else:
                    # Esto solo se alcanzaría si la lógica de decisión no asigna un angulo_a_girar.
                    # Con la lógica actual (derecha o, si no, izquierda), siempre se asigna uno.
                    print("No se pudo decidir un giro (situación inesperada). Robot detenido momentáneamente.")
                    pid_ctrl.detener_motores()
                    time.sleep_ms(1000) # Pausa más larga si está en una situación indefinida

            else: # No hay obstáculo frontal
                # print("Camino despejado. Avanzando...") # Descomenta para depuración
                pid_ctrl.mover_recto_con_pid(VELOCIDAD_BASE_AVANCE, yaw_actual, yaw_referencia_actual)
            
            time.sleep_ms(50) # Pequeña pausa en el bucle principal para no sobrecargar el procesador

    except KeyboardInterrupt:
        print("Prueba detenida por el usuario.")
    except Exception as e:
        print(f"Error crítico en el bucle principal: {e}")
        import sys
        sys.print_exception(e)
    finally:
        print("Deteniendo motores y apagando componentes...")
        pid_ctrl.detener_motores()
        Matriz.apagar_matriz() # Asumiendo método estático

# --- Punto de Entrada Principal ---
if __name__ == "__main__":
    mover_controlado()