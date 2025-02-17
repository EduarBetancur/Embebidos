#include <Arduino.h>

// Definición de pines
#define PIN_MOTOR_DER_EN  5   // Enable motor derecho
#define PIN_MOTOR_DER_IN1 18  // Input 1 motor derecho
#define PIN_MOTOR_DER_IN2 19  // Input 2 motor derecho
#define PIN_MOTOR_IZQ_EN  13  // Enable motor izquierdo
#define PIN_MOTOR_IZQ_IN1 25  // Input 1 motor izquierdo
#define PIN_MOTOR_IZQ_IN2 26  // Input 2 motor izquierdo

class RobotMotors {
private:
    // Pines para el motor derecho
    const int motorDerEN;
    const int motorDerIN1;
    const int motorDerIN2;
    
    // Pines para el motor izquierdo
    const int motorIzqEN;
    const int motorIzqIN1;
    const int motorIzqIN2;
    
    // Velocidad predeterminada (0-255)
    const int velocidadDefault = 200;
    
public:
    RobotMotors(int pinDerEN, int pinDerIN1, int pinDerIN2, 
                int pinIzqEN, int pinIzqIN1, int pinIzqIN2) :
        motorDerEN(pinDerEN), motorDerIN1(pinDerIN1), motorDerIN2(pinDerIN2),
        motorIzqEN(pinIzqEN), motorIzqIN1(pinIzqIN1), motorIzqIN2(pinIzqIN2) {
        
        // Configuración de pines
        pinMode(motorDerEN, OUTPUT);
        pinMode(motorDerIN1, OUTPUT);
        pinMode(motorDerIN2, OUTPUT);
        pinMode(motorIzqEN, OUTPUT);
        pinMode(motorIzqIN1, OUTPUT);
        pinMode(motorIzqIN2, OUTPUT);
        
        // Inicializar motores detenidos
        detener();
    }
    
    void controlarMotorDerecho(int velocidad) {
        if (velocidad >= 0) {
            digitalWrite(motorDerIN1, HIGH);
            digitalWrite(motorDerIN2, LOW);
            analogWrite(motorDerEN, velocidad);
        } else {
            digitalWrite(motorDerIN1, LOW);
            digitalWrite(motorDerIN2, HIGH);
            analogWrite(motorDerEN, -velocidad);
        }
    }
    
    void controlarMotorIzquierdo(int velocidad) {
        if (velocidad >= 0) {
            digitalWrite(motorIzqIN1, HIGH);
            digitalWrite(motorIzqIN2, LOW);
            analogWrite(motorIzqEN, velocidad);
        } else {
            digitalWrite(motorIzqIN1, LOW);
            digitalWrite(motorIzqIN2, HIGH);
            analogWrite(motorIzqEN, -velocidad);
        }
    }

    void detener() {
        // Detener motor derecho
        digitalWrite(motorDerIN1, LOW);
        digitalWrite(motorDerIN2, LOW);
        analogWrite(motorDerEN, 0);
        
        // Detener motor izquierdo
        digitalWrite(motorIzqIN1, LOW);
        digitalWrite(motorIzqIN2, LOW);
        analogWrite(motorIzqEN, 0);
    }

    void procesarComandoSerial(String comando) {
        comando.toLowerCase();
        
        if (comando == "ad") {
            controlarMotorDerecho(velocidadDefault);
            Serial.println("Motor derecho adelante");
        }
        else if (comando == "ai") {
            controlarMotorIzquierdo(velocidadDefault);
            Serial.println("Motor izquierdo adelante");
        }
        else if (comando == "rd") {
            controlarMotorDerecho(-velocidadDefault);
            Serial.println("Motor derecho reversa");
        }
        else if (comando == "ri") {
            controlarMotorIzquierdo(-velocidadDefault);
            Serial.println("Motor izquierdo reversa");
        }
        else if (comando == "pd") {
            controlarMotorDerecho(0);
            Serial.println("Motor derecho detenido");
        }
        else if (comando == "pi") {
            controlarMotorIzquierdo(0);
            Serial.println("Motor izquierdo detenido");
        }
        else if (comando == "pp") {
            detener();
            Serial.println("Ambos motores detenidos");
        }
        else if (comando == "aa") { // Ambos adelante
            controlarMotorDerecho(velocidadDefault);
            controlarMotorIzquierdo(velocidadDefault);
            Serial.println("Ambos motores adelante");
        }
        else if (comando == "rr") { // Ambos reversa
            controlarMotorDerecho(-velocidadDefault);
            controlarMotorIzquierdo(-velocidadDefault);
            Serial.println("Ambos motores reversa");
        }
        else {
            Serial.println("Comando no reconocido");
        }
    }
};

// Instancia global de la clase RobotMotors
RobotMotors robot(
    PIN_MOTOR_DER_EN, PIN_MOTOR_DER_IN1, PIN_MOTOR_DER_IN2,
    PIN_MOTOR_IZQ_EN, PIN_MOTOR_IZQ_IN1, PIN_MOTOR_IZQ_IN2
);

void setup() {
    Serial.begin(115200);
    Serial.println("\nSistema de control de motores iniciado");
    Serial.println("Comandos disponibles:");
    Serial.println("ad - Adelante derecha");
    Serial.println("ai - Adelante izquierda");
    Serial.println("rd - Reversa derecha");
    Serial.println("ri - Reversa izquierda");
    Serial.println("pd - Parar derecha");
    Serial.println("pi - Parar izquierda");
    Serial.println("pp - Parar ambos");
    Serial.println("aa - Ambos adelante");
    Serial.println("rr - Ambos reversa");
}

void loop() {
    if (Serial.available() > 0) {
        String comando = Serial.readStringUntil('\n');
        comando.trim();
        robot.procesarComandoSerial(comando);
    }
}
