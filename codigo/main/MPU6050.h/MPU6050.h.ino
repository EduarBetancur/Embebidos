#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

// Dirección I2C del MPU6050 (por defecto 0x68)
#define MPU6050_ADDRESS       0x68
// Registro para gestionar la alimentación (Power Management 1)
#define MPU6050_PWR_MGMT_1    0x6B
// Registro donde se inician las lecturas del giroscopio (eje Z)
#define MPU6050_GYRO_ZOUT_H   0x47

class MPU6050 {
public:
  MPU6050();
  // Inicializa el bus I2C y configura el sensor
  void begin();
  // Realiza la calibración para obtener el offset del giro en Z
  void calibrar();
  // Actualiza el ángulo (integrando la velocidad angular del eje Z)
  // dt: intervalo de tiempo en segundos
  float actualizarFiltro(float dt);
  // Funciones de giro: se espera hasta que se alcance una diferencia de 90°.
  void girar90Izquierda();
  void girar90Derecha();
  float getAngulo() { return anguloActual; }
private:
  float anguloActual;   // Ángulo acumulado (en grados)
  float gyroZ_offset;   // Offset calculado del giro en Z
  // Lee el valor bruto del giroscopio en Z (dos bytes) y lo devuelve como entero con signo
  int16_t readGyroZ();
};

#endif // MPU6050_H

// -------------------------
// Implementación de MPU6050
// -------------------------

MPU6050::MPU6050() : anguloActual(0), gyroZ_offset(0) { }

void MPU6050::begin() {
  Wire.begin();
  // Se "despierta" al MPU6050 escribiendo 0 en el registro de administración de energía
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
}

void MPU6050::calibrar() {
  // Realiza varias lecturas para promediar el offset del giroscopio (eje Z)
  long sum = 0;
  const int numReadings = 100;
  for (int i = 0; i < numReadings; i++) {
    int16_t gz = readGyroZ();
    sum += gz;
    delay(5);
  }
  gyroZ_offset = (float)sum / numReadings;
}

int16_t MPU6050::readGyroZ() {
  // Inicia la comunicación para leer 2 bytes a partir del registro GYRO_ZOUT_H
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 2, true);
  int16_t gz = Wire.read() << 8 | Wire.read();
  return gz;
}

float MPU6050::actualizarFiltro(float dt) {
  // Lee el valor bruto del giroscopio en Z
  int16_t rawGz = readGyroZ();
  // Convertir el valor bruto a grados por segundo (escala ±250°/s; factor 131 LSB/(°/s))
  float gz = (float)(rawGz - gyroZ_offset) / 131.0;
  // Integra la velocidad angular en el tiempo para obtener el ángulo
  anguloActual += gz * dt;
  return anguloActual;
}

void MPU6050::girar90Izquierda() {
  // Se desea disminuir el ángulo en 90° (giro a la izquierda)
  float target = anguloActual - 90.0;
  while (anguloActual > target) {
    actualizarFiltro(0.01);  // Actualiza el ángulo cada 10 ms
    delay(10);
  }
}

void MPU6050::girar90Derecha() {
  // Se desea aumentar el ángulo en 90° (giro a la derecha)
  float target = anguloActual + 90.0;
  while (anguloActual < target) {
    actualizarFiltro(0.01);
    delay(10);
  }
}
