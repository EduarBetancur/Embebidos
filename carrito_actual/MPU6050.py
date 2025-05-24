from machine import I2C, Pin
import time

class MPU6050:
    def __init__(self):
        # Inicializa I²C en los pines SDA=21 y SCL=22 a 400 kHz
        self.i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400000)
        self.address = 0x68

        # Verifica que el dispositivo responda en la dirección esperada
        devices = self.i2c.scan()
        if self.address not in devices:
            raise RuntimeError(f"MPU6050 no encontrado en la dirección {hex(self.address)}")

        # Despertar el sensor y configurar rangos
        self.i2c.writeto_mem(self.address, 0x6B, bytes([0]))    # PWR_MGMT_1: salir de sleep
        self.i2c.writeto_mem(self.address, 0x1B, bytes([0x00])) # GYRO_CONFIG: ±250°/s
        self.i2c.writeto_mem(self.address, 0x1C, bytes([0x00])) # ACCEL_CONFIG: ±2 g

        # Variables para integración del ángulo
        self.yaw = 0.0
        self.last_time = time.ticks_ms()
        self.offset_z = 0.0

    def LeerRegistro(self, Direccion):
        """Lee un registro de 16 bits con signo."""
        try:
            data = self.i2c.readfrom_mem(self.address, Direccion, 2)
            Registro = (data[0] << 8) | data[1]
            if Registro > 32767:
                Registro -= 65536
            return Registro
        except OSError as e:
            print(f"Error al leer el registro {hex(Direccion)}: {e}")
            return 0

    def LeerGiroscopio(self):
        """Devuelve (Gx, Gy, Gz) en °/s, aplicando offset en Z."""
        try:
            Gx = self.LeerRegistro(0x43) / 131.0
            Gy = self.LeerRegistro(0x45) / 131.0
            Gz = (self.LeerRegistro(0x47) / 131.0) - self.offset_z
            return (Gx, Gy, Gz)
        except OSError as e:
            print(f"Error al leer el giroscopio: {e}")
            return (0, 0, 0)

    def calibrarGiroscopio(self, muestras=300):
        """Calcula y almacena el offset de Gz promediando en reposo."""
        print("Calibrando giroscopio... No muevas el robot.")
        offset_z = 0.0
        for _ in range(muestras):
            _, _, Gz = self.LeerGiroscopio()
            offset_z += Gz
            time.sleep_ms(10)
        self.offset_z = offset_z / muestras
        print(f"Calibración completada. Offset Z: {self.offset_z:.2f}°/s")

    def actualizarYaw(self):
        """Integra Gz en el tiempo para obtener el ángulo de yaw."""
        try:
            current_time = time.ticks_ms()
            dt = (current_time - self.last_time) / 1000.0
            self.last_time = current_time

            Gz = self.LeerGiroscopio()[2]
            # Filtro de ruido: ignora pequeñas vibraciones
            if abs(Gz) < 0.5:
                Gz = 0.0

            self.yaw += Gz * dt
            return self.yaw
        except OSError as e:
            print(f"Error al actualizar el yaw: {e}")
            return 0.0
