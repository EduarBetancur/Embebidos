#include <Arduino.h>
#include <Motores.h>
#include <Sensores.h>

enum Estados{
  Avanzar,
  Girar_izquierda,
  Girar_derecha,
  Girar_180,
  Parar
};

Estados estadoActual = Avanzar;

Motores motores;
Sensores sensores;
MPU6050 mpu;

unsigned long estadoTimer =0;


void setup() {
  mpu.begin();
  delay (100);
  mpu.calibrar();

  sensores.begin();

  estadoTimer =millis();

}

void loop() {
  int lecturaIzq = sensores.detectarSensor1();
  int lecturaCentro = sensores.detectarSensor2();
  int lecturaDer = sensores.detectarSensor3();

unsigned long tiempoActual = millis();

if (lecturaIzq==1 && lecturaCentro == 1 && lecturaDer ==1){
  estadoActual = Girar180;
  estadoTimer = estadoActual;
}
switch (estadoActual){
  case Avanzar:
  if(lecturaCentro ==1 ){
    motores.Alto();
    if (lecturaIzq==1){
      estadoActual = Girar_derecha;

    } else if(lecturaDer==1){
      estadoActual=Girar_izquierda;
    }else{
      estadoActual = Girar_derecha;
    }
    estadoTimer =tiempoActual;
    
  } else{
    motores.Adelante;
  }
break;

case Girar_izquierda:
motores.GirarIzquierda(mpu);
estadoActual = Avanzar;
estadoTimer=tiempoActual;
break;

case Cirar_derecha:
motores.GirarDerecha(mpu);
estadoActual=Avanzar;
estadoTimer=tiempoActual;
break;

case Girar_180:
motores.Girar180(mpu);
estadoActual=Avanzar;
estadoTimer=tiempoActual;
break;

case Parar:
motores.Alto();
if (tiempoActual-estadoTimer>=500){
  estadoActual=Avanzar;
  estadoTimer=tiempoActual;

}
break;
}
}