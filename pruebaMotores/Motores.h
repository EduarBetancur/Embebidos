#ifndef MOTORES_H
#define MOTORES_H

#include <Arduino.h>
#include <stdint.h>

// Pines para el puente H (motores de 12V) en ESP32 con 6 pines: 3 para cada motor

// Motor A (usando pines seguros en ESP32)
#define PIN_MOTOR_A_PWM   25  // Pin PWM (enable) para controlar la velocidad del motor A
#define PIN_MOTOR_A_IN1   27  // Pin de dirección 1 del motor A
#define PIN_MOTOR_A_IN2   26  // Pin de dirección 2 del motor A

// Motor B (usando pines seguros en ESP32)
#define PIN_MOTOR_B_PWM   32  // Pin PWM (enable) para controlar la velocidad del motor B
#define PIN_MOTOR_B_IN1   14  // Pin de dirección 1 del motor B
#define PIN_MOTOR_B_IN2   15  // Pin de dirección 2 del motor B

// Definir una velocidad PWM (valor menor a 255 para no usar la velocidad máxima)
#define VELOCIDAD 50

class Motores {
public:
  Motores();
  void Adelante();
  void Atras();               // Nueva función para ir hacia atrás
  void GirarIzquierda(unsigned long duracion);  
  void GirarDerecha(unsigned long duracion);
  void Girar180(unsigned long duracion);
  void Alto();
};

#endif // MOTORES_H

// --------------------------
// Implementación de Motores
// --------------------------

Motores::Motores() {
  // Configurar pines para Motor A
  pinMode(PIN_MOTOR_A_PWM, OUTPUT);
  pinMode(PIN_MOTOR_A_IN1, OUTPUT);
  pinMode(PIN_MOTOR_A_IN2, OUTPUT);
  
  // Configurar pines para Motor B
  pinMode(PIN_MOTOR_B_PWM, OUTPUT);
  pinMode(PIN_MOTOR_B_IN1, OUTPUT);
  pinMode(PIN_MOTOR_B_IN2, OUTPUT);
  
  Alto();
}

void Motores::Adelante() {
  // Configurar ambos motores para avanzar:
  // Motor A: IN1 HIGH, IN2 LOW  
  // Motor B: IN1 HIGH, IN2 LOW  
  digitalWrite(PIN_MOTOR_A_IN1, HIGH);
  digitalWrite(PIN_MOTOR_A_IN2, LOW);
  digitalWrite(PIN_MOTOR_B_IN1, HIGH);
  digitalWrite(PIN_MOTOR_B_IN2, LOW);
  
  // Aplicar velocidad mediante PWM
  analogWrite(PIN_MOTOR_A_PWM, VELOCIDAD);
  analogWrite(PIN_MOTOR_B_PWM, VELOCIDAD);
}

void Motores::Atras() {
  // Configurar ambos motores para retroceder:
  // Motor A: IN1 LOW, IN2 HIGH  
  // Motor B: IN1 LOW, IN2 HIGH  
  digitalWrite(PIN_MOTOR_A_IN1, LOW);
  digitalWrite(PIN_MOTOR_A_IN2, HIGH);
  digitalWrite(PIN_MOTOR_B_IN1, LOW);
  digitalWrite(PIN_MOTOR_B_IN2, HIGH);
  
  // Aplicar velocidad mediante PWM
  analogWrite(PIN_MOTOR_A_PWM, VELOCIDAD);
  analogWrite(PIN_MOTOR_B_PWM, VELOCIDAD);
}

void Motores::GirarIzquierda(unsigned long duracion) {
  // Para girar a la izquierda:
  // Motor A avanza: IN1 HIGH, IN2 LOW  
  // Motor B retrocede: IN1 LOW, IN2 HIGH
  digitalWrite(PIN_MOTOR_A_IN1, HIGH);
  digitalWrite(PIN_MOTOR_A_IN2, LOW);
  digitalWrite(PIN_MOTOR_B_IN1, LOW);
  digitalWrite(PIN_MOTOR_B_IN2, HIGH);
  
  analogWrite(PIN_MOTOR_A_PWM, VELOCIDAD);
  analogWrite(PIN_MOTOR_B_PWM, VELOCIDAD);
  
  delay(duracion);
  Alto();
}

void Motores::GirarDerecha(unsigned long duracion) {
  // Para girar a la derecha:
  // Motor A retrocede: IN1 LOW, IN2 HIGH  
  // Motor B avanza: IN1 HIGH, IN2 LOW
  digitalWrite(PIN_MOTOR_A_IN1, LOW);
  digitalWrite(PIN_MOTOR_A_IN2, HIGH);
  digitalWrite(PIN_MOTOR_B_IN1, HIGH);
  digitalWrite(PIN_MOTOR_B_IN2, LOW);
  
  analogWrite(PIN_MOTOR_A_PWM, VELOCIDAD);
  analogWrite(PIN_MOTOR_B_PWM, VELOCIDAD);
  
  delay(duracion);
  Alto();
}

void Motores::Girar180(unsigned long duracion) {
  // Realiza dos giros de 90° consecutivos para efectuar un giro de 180°
  GirarDerecha(duracion);
  delay(150);
  GirarDerecha(duracion);
}

void Motores::Alto() {
  // Detener ambos motores: se apaga la señal PWM y se ponen LOW las señales de dirección
  analogWrite(PIN_MOTOR_A_PWM, 0);
  analogWrite(PIN_MOTOR_B_PWM, 0);
  
  digitalWrite(PIN_MOTOR_A_IN1, LOW);
  digitalWrite(PIN_MOTOR_A_IN2, LOW);
  digitalWrite(PIN_MOTOR_B_IN1, LOW);
  digitalWrite(PIN_MOTOR_B_IN2, LOW);
}
