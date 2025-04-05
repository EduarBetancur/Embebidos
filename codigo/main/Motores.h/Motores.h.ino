#ifndef MOTORES_H
#define MOTORES_H

#include <Arduino.h>
#include <stdint.h>

// Pines para el puente H (motores de 12V)
#define PIN_MOTOR_A_IN1 12
#define PIN_MOTOR_A_IN2 13
#define PIN_MOTOR_B_IN1 14
#define PIN_MOTOR_B_IN2 15

class Motores {
public:
  Motores();
  void Adelante();
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
  pinMode(PIN_MOTOR_A_IN1, OUTPUT);
  pinMode(PIN_MOTOR_A_IN2, OUTPUT);
  pinMode(PIN_MOTOR_B_IN1, OUTPUT);
  pinMode(PIN_MOTOR_B_IN2, OUTPUT);
  Alto();
}

void Motores::Adelante() {
  digitalWrite(PIN_MOTOR_A_IN1, HIGH);
  digitalWrite(PIN_MOTOR_A_IN2, LOW);
  digitalWrite(PIN_MOTOR_B_IN1, HIGH);
  digitalWrite(PIN_MOTOR_B_IN2, LOW);
}


void Motores::GirarIzquierda(unsigned long duracion) {
  // Un motor adelante y el otro atrás para girar a la izquierda
  digitalWrite(PIN_MOTOR_A_IN1, HIGH);
  digitalWrite(PIN_MOTOR_A_IN2, LOW);
  digitalWrite(PIN_MOTOR_B_IN1, LOW);
  digitalWrite(PIN_MOTOR_B_IN2, HIGH);
  delay(duracion);
  Alto();
}

void Motores::GirarDerecha(unsigned long duracion) {
  // Para girar a la derecha se invierten las direcciones
  digitalWrite(PIN_MOTOR_A_IN1, LOW);
  digitalWrite(PIN_MOTOR_A_IN2, HIGH);
  digitalWrite(PIN_MOTOR_B_IN1, HIGH);
  digitalWrite(PIN_MOTOR_B_IN2, LOW);
  delay(duracion);
  Alto();
}

void Motores::Girar180(unsigned long duracion) {
  // Se realizan dos giros de 90° consecutivos
  GirarDerecha( duracion);
  delay(100);
  GirarDerecha(duracion);
}

void Motores::Alto() {
  digitalWrite(PIN_MOTOR_A_IN1, LOW);
  digitalWrite(PIN_MOTOR_A_IN2, LOW);
  digitalWrite(PIN_MOTOR_B_IN1, LOW);
  digitalWrite(PIN_MOTOR_B_IN2, LOW);
}
