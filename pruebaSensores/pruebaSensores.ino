#include "Sensores.h"

Sensores sensores;

void setup() {
  Serial.begin(115200);
  sensores.begin();
}

void loop() {
  int obj1 = sensores.detectarSensor1();
  int obj2 = sensores.detectarSensor2();
  int obj3 = sensores.detectarSensor3();

  Serial.print("Sensor 1: "); Serial.println(obj1 ? "Objeto detectado" : "Libre");
  Serial.print("Sensor 2: "); Serial.println(obj2 ? "Objeto detectado" : "Libre");
  Serial.print("Sensor 3: "); Serial.println(obj3 ? "Objeto detectado" : "Libre");

  delay(50); // Espera medio segundo antes de la siguiente medición
}
