#ifndef SENSORES_H
#define SENSORES_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define PIN_XSHUT_SENSOR1 33
#define PIN_XSHUT_SENSOR2 34
#define PIN_XSHUT_SENSOR3 35

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
  
  digitalWrite(PIN_XSHUT_SENSOR2, HIGH);
  delay(10);
  if (!sensor2.begin()) {
    Serial.println("Sensor 2 no pudo inicializarse.");
  }
  sensor2.setAddress(0x30);
  
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
  if (measure.RangeStatus == 0 && measure.RangeMilliMeter < 60) {
    return 1;
  }
  return 0;
}

int Sensores::detectarSensor2() {
  VL53L0X_RangingMeasurementData_t measure;
  sensor2.rangingTest(&measure, false);
  if (measure.RangeStatus == 0 && measure.RangeMilliMeter < 60) {
    return 1;
  }
  return 0;
}

int Sensores::detectarSensor3() {
  VL53L0X_RangingMeasurementData_t measure;
  sensor3.rangingTest(&measure, false);
  if (measure.RangeStatus == 0 && measure.RangeMilliMeter < 60) {
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
