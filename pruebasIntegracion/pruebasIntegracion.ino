#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <MPU6050_tockn.h>

// Instancias de los sensores
Adafruit_VL53L0X sensor_frontal = Adafruit_VL53L0X();
MPU6050 mpu6050(Wire);

// Pines del puente H L298N
#define ENA 25  // PWM Motor A
#define IN1 27  // Dirección Motor A
#define IN2 26  // Dirección Motor A
#define ENB 32  // PWM Motor B
#define IN3 14  // Dirección Motor B
#define IN4 15  // Dirección Motor B

// Pines de control de encendido del sensor
#define XSHUT_FRONTAL 33

// Potencia del motor (0-255)
int potencia = 200;

// Variables del filtro complementario
float angleX = 0, angleY = 0;
float alpha = 0.98;  // Constante de mezcla del filtro
long timer = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Inicializar el sensor VL53L0X
    pinMode(XSHUT_FRONTAL, OUTPUT);
    digitalWrite(XSHUT_FRONTAL, LOW);
    delay(10);
    digitalWrite(XSHUT_FRONTAL, HIGH);
    delay(10);
    if (!sensor_frontal.begin(0x30)) {
        Serial.println("Error al iniciar el sensor frontal");
    }

    // Configurar pines del motor
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Inicializar MPU6050
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
}

void loop() {
    mpu6050.update();
    actualizarAngulo();  // Aplica el filtro complementario

    int distancia_frontal = Medir_VL53L0X(sensor_frontal);

    Serial.print("Distancia Frontal: "); Serial.println(distancia_frontal);
    Serial.print("Ángulo X: "); Serial.print(angleX);
    Serial.print("\tÁngulo Y: "); Serial.println(angleY);

    if (distancia_frontal > 100) {
        adelante();
        Serial.println("Avanzando");
    } else {
        girar90('D');  // Gira 90° a la derecha si hay obstáculo
        delay(500);
    }
    delay(500);
}

// Función de medición con VL53L0X
float Medir_VL53L0X(Adafruit_VL53L0X &sensor) {
    VL53L0X_RangingMeasurementData_t medida;
    sensor.rangingTest(&medida, false);
    return (medida.RangeStatus != 4) ? medida.RangeMilliMeter : 9999;
}

// Función para actualizar el ángulo con filtro complementario
void actualizarAngulo() {
    float dt = (millis() - timer) / 1000.0;  // Delta tiempo en segundos
    timer = millis();

    float accAngleX = mpu6050.getAccAngleX();
    float accAngleY = mpu6050.getAccAngleY();
    float gyroX = mpu6050.getGyroX();
    float gyroY = mpu6050.getGyroY();

    // Aplicar filtro complementario
    angleX = alpha * (angleX + gyroX * dt) + (1 - alpha) * accAngleX;
    angleY = alpha * (angleY + gyroY * dt) + (1 - alpha) * accAngleY;
}

// Función para girar 90° en la dirección especificada ('D' = derecha, 'I' = izquierda)
void girar90(char direccion) {
    float anguloInicial = angleZ();

    if (direccion == 'D') {
        derecha();
        while (fabs(angleZ() - anguloInicial) < 90) {
            mpu6050.update();
            actualizarAngulo();
        }
    } else if (direccion == 'I') {
        izquierda();
        while (fabs(angleZ() - anguloInicial) < 90) {
            mpu6050.update();
            actualizarAngulo();
        }
    }

    detener();
}

// Funciones de movimiento
void adelante() {
    setMotor(ENA, IN1, IN2, potencia, HIGH, LOW);
    setMotor(ENB, IN3, IN4, potencia, HIGH, LOW);
}

void atras() {
    setMotor(ENA, IN1, IN2, potencia, LOW, HIGH);
    setMotor(ENB, IN3, IN4, potencia, LOW, HIGH);
}

void izquierda() {
    setMotor(ENA, IN1, IN2, potencia, LOW, HIGH);
    setMotor(ENB, IN3, IN4, potencia, HIGH, LOW);
}

void derecha() {
    setMotor(ENA, IN1, IN2, potencia, HIGH, LOW);
    setMotor(ENB, IN3, IN4, potencia, LOW, HIGH);
}

void detener() {
    setMotor(ENA, IN1, IN2, 0, LOW, LOW);
    setMotor(ENB, IN3, IN4, 0, LOW, LOW);
}

// Función para manejar los motores en ESP32
void setMotor(int pwmPin, int dirPin1, int dirPin2, int speed, int dir1, int dir2) {
    digitalWrite(dirPin1, dir1);
    digitalWrite(dirPin2, dir2);
    ledcWrite(pwmPin, speed);
}

// Función para obtener el ángulo Z del MPU6050
float angleZ() {
    return mpu6050.getAngleZ();
}
