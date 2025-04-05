#include <Arduino.h>
#include <Sensores.h>
#include <Motores.h>

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

unsigned long estadoTimer =0;
const unsigned long duraciónGiro =400;

void setup() {

  sensores.begin();

  estadoTimer =millis();

}

void loop() {
  int lecturaIzq = sensores.detectarSensor1();
  int lecturaCentro = sensores.detectarSensor2();
  int lecturaDer = sensores.detectarSensor3();

unsigned long tiempoActual = millis();

if (lecturaIzq==1 && lecturaCentro == 1 && lecturaDer ==1){
  estadoActual = Girar_180;
  estadoTimer = tiempoActual;
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
    motores.Adelante();
  }
break;

case Girar_izquierda:
motores.GirarIzquierda(duraciónGiro);
estadoActual = Avanzar;
estadoTimer=tiempoActual;
break;

case Girar_derecha:
motores.GirarDerecha(duraciónGiro);
estadoActual=Avanzar;
estadoTimer=tiempoActual;
break;

case Girar_180:
motores.Girar180(2*duraciónGiro);
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