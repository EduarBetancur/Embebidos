from machine import I2C, Pin
import time

class MPU6050:
    def _init_(self):
        self.i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400000)
        self.address = 0x68
        devices = self.i2c.scan()
        if self.address not in devices:
            raise RuntimeError(f"MPU6050 no encontrado en la dirección {hex(self.address)}")

        self.i2c.writeto_mem(self.address, 0x6B, bytes([0]))  # Despertar
        self.i2c.writeto_mem(self.address, 0x1B, bytes([0x00]))  # Giro ±250°/s
        self.i2c.writeto_mem(self.address, 0x1C, bytes([0x00]))  # Acel ±2g

        self.yaw = 0.0
        self.last_time = time.ticks_ms()
        self.offset_z = 0.0

    def LeerRegistro(self, Direccion):
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
        try:
            Gx = self.LeerRegistro(0x43) / 131.0
            Gy = self.LeerRegistro(0x45) / 131.0
            Gz = (self.LeerRegistro(0x47) / 131.0) - self.offset_z
            return (Gx, Gy, Gz)
        except OSError as e:
            print(f"Error al leer el giroscopio: {e}")
            return (0, 0, 0)

    def calibrarGiroscopio(self, muestras=300):
        print("Calibrando giroscopio... No muevas el robot.")
        offset_z = 0.0
        for _ in range(muestras):
            Gx, Gy, Gz = self.LeerGiroscopio()
            offset_z += Gz
            time.sleep_ms(10)
        self.offset_z = offset_z / muestras
        print(f"Calibración completada. Offset Z: {self.offset_z:.2f}°/s")

    def actualizarYaw(self):
        try:
            current_time = time.ticks_ms()
            dt = (current_time - self.last_time) / 1000.0
            self.last_time = current_time
            
            Gz = self.LeerGiroscopio()[2]
            # --- Añade este filtro de ruido ---
            if abs(Gz) < 0.5:  # Ignorar ruido pequeño
                Gz = 0.0
            
            self.yaw += Gz * dt
            # --- Normalización opcional ---
            
            
            return self.yaw
        except OSError as e:
            print(f"Error al actualizar el yaw: {e}")
            return 0.0
