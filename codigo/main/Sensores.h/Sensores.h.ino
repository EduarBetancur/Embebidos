#ifndef SENSORES_H
#define SENSORES_H

#include <Arduino.h>
#include <wire.h>
#include <Adafruit_VL53L0X.h>

#define PIN_XSHUR_SENSOR1 25
#define PIN_XSHUR_SENSOR2 26
#define PIN_XSHUR_SENSOR3 27

class Sensores {
  public:
   Sensores();

   void begin();

   int detectarSensor1();
   int detectarSensor2();
   int detectarSensor3();
   int detectarObjeto (int sensorID);

   private:
   Adafruit_VL53L0X sensor1;
   Adafruit_VL53L0X sensor2;
   Adafruit_VL53L0X sensor3;
};

#endif 

Sensores::Sensores(){

}

void Sensores::begin(){
  pinMode(PIN_XSHUT_SENSOR1, OUTPUT);
  pinMode(PIN_XSHUT_SENSOR2, OUTPUT);
  pinMode(PIN_XSHUT_SENSOR3, OUTPUT);
  digitalWrite(PIN_XSHUT_SENSOR1, LOW);
  digitalWrite(PIN_XSHUT_SENSOR2, LOW);
  digitalWrite(PIN_XSHUT_SENSOR3, LOW);
  delay (10);

  digitalWrite(PIN_XSHUT_SENSOR1, HIGH);
  delay(10);
  if (!sensor1.begin()) {
    Serial.println("Sensor 1 no pudo inicializarse.");
  }
  
  // Enciende el sensor 2 y cambia su dirección a 0x30
  digitalWrite(PIN_XSHUT_SENSOR2, HIGH);
  delay(10);
  if (!sensor2.begin()) {
    Serial.println("Sensor 2 no pudo inicializarse.");
  }
  sensor2.setAddress(0x30);
  
  // Enciende el sensor 3 y cambia su dirección a 0x31
  digitalWrite(PIN_XSHUT_SENSOR3, HIGH);
  delay(10);
  if (!sensor3.begin()) {
    Serial.println("Sensor 3 no pudo inicializarse.");
  }
  sensor3.setAddress(0x31);
}

int Sensores::detectarSensor1() {
  VL53L0X_RangingMeasurementData_t measure;
  sensor1.rangingTest(&measure, false);
  // Si el sensor reporta medición correcta y la distancia es menor a 20 mm
  if (measure.RangeStatus == 0 && measure.RangeMilliMeter < 20) {
    return 1;
  }
  return 0;
}

int Sensores::detectarSensor2() {
  VL53L0X_RangingMeasurementData_t measure;
  sensor2.rangingTest(&measure, false);
  if (measure.RangeStatus == 0 && measure.RangeMilliMeter < 20) {
    return 1;
  }
  return 0;
}

int Sensores::detectarSensor3() {
  VL53L0X_RangingMeasurementData_t measure;
  sensor3.rangingTest(&measure, false);
  if (measure.RangeStatus == 0 && measure.RangeMilliMeter < 20) {
    return 1;
  }
  return 0;
}

int Sensores::detectarObjeto(int sensorId) {
  switch(sensorId) {
    case 1: return detectarSensor1();
    case 2: return detectarSensor2();
    case 3: return detectarSensor3();
    default: return 0;
  }
}
