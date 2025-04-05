#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "MPU6050.h"

// Instancia del sensor VL53L0X frontal y del MPU6050
Adafruit_VL53L0X sensor_frontal = Adafruit_VL53L0X();
MPU6050 mpu;

// Pines del puente H L298N
#define ENA 25   // PWM Motor A
#define IN1 27   // Dirección Motor A
#define IN2 26   // Dirección Motor A
#define ENB 32   // PWM Motor B
#define IN3 14   // Dirección Motor B
#define IN4 15   // Dirección Motor B

// Pin de control del sensor VL53L0X (XSHUT)
#define XSHUT_FRONTAL 33

// Potencia del motor (0-255)
int potencia = 200; // Ajusta según lo requieras

// Función para medir distancia con VL53L0X
float Medir_VL53L0X(Adafruit_VL53L0X &sensor) {
  VL53L0X_RangingMeasurementData_t medida;
  sensor.rangingTest(&medida, false);
  if (medida.RangeStatus == 0) {
    return medida.RangeMilliMeter;
  } else {
    return 9999; // Indica error o fuera de rango
  }
}

// Funciones de control de motores
void adelante() {
  analogWrite(ENA, potencia);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, potencia);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void atras() {
  analogWrite(ENA, potencia);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB, potencia);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void izquierda() {
  analogWrite(ENA, potencia);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB, potencia);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void derecha() {
  analogWrite(ENA, potencia);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, potencia);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void detener() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Funciones de giro de 90° (usando delay para simular el giro)
void giroDerecha90() {
  // Para girar 90° a la derecha: Motor A avanza y Motor B retrocede
  analogWrite(ENA, potencia);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, potencia);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(500);  // Ajusta este delay experimentalmente para un giro de 90°
  detener();
  Serial.println("Giro 90° a la derecha completado.");
}

void giroIzquierda90() {
  // Para girar 90° a la izquierda: Motor A retrocede y Motor B avanza
  analogWrite(ENA, potencia);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB, potencia);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(500);  // Ajusta este delay experimentalmente para un giro de 90°
  detener();
  Serial.println("Giro 90° a la izquierda completado.");
}

void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando VL53L0X y MPU6050...");

  // Configurar pines del motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Configurar pin XSHUT para el sensor frontal
  pinMode(XSHUT_FRONTAL, OUTPUT);
  
  Wire.begin();  // Iniciar I2C (SDA=21, SCL=22 por defecto)
  
  // Apagar el sensor frontal antes de inicializarlo
  digitalWrite(XSHUT_FRONTAL, LOW);
  delay(20);
  
  // Encender e inicializar el sensor frontal
  digitalWrite(XSHUT_FRONTAL, HIGH);
  delay(20);
  if (!sensor_frontal.begin()) {
    Serial.println("Error al iniciar el sensor frontal");
  } else {
    Serial.println("Sensor frontal iniciado correctamente.");
  }
  
  // Inicializar MPU6050
  Serial.println("Iniciando MPU6050...");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado correctamente.");
  } else {
    Serial.println("Error de conexión con MPU6050.");
  }
  
  // Dar tiempo para que se estabilice el bus I2C
  delay(50);
}

void loop() {
  // Primero, medir la distancia con el sensor VL53L0X
  float distancia_frontal = Medir_VL53L0X(sensor_frontal);
  Serial.print("Distancia Frontal: ");
  Serial.print(distancia_frontal);
  Serial.println(" mm");
  
  delay(20); // Pequeño delay para que se asiente el I2C
  
  // Luego, leer datos del MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  Serial.print("MPU Aceleración: X=");
  Serial.print(ax);
  Serial.print(" Y=");
  Serial.print(ay);
  Serial.print(" Z=");
  Serial.println(az);
  
  Serial.print("MPU Giroscopio: X=");
  Serial.print(gx);
  Serial.print(" Y=");
  Serial.print(gy);
  Serial.print(" Z=");
  Serial.println(gz);
  
  // Lógica de movimiento basada en la distancia medida
  if (distancia_frontal > 100) {
    adelante();
    Serial.println("Avanzando");
  } else {
    giroDerecha90();
    delay(50);
  }
  
  delay(50);
}
