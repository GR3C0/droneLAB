// Programa general para droneLAB

// Libreias generales
#include "Configuracion.h"
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

Servo motor;
float masa = 2; // Masa del drone

Adafruit_BNO055 bno = Adafruit_BNO055(55); // Llamamos al Sensor

void setup()
{
  Serial.begin(9600);
  if(!bno.begin()) //Prueba de conexión del Adafruit_Sensor
  {
    Serial.print("No detectado");
    while(1);
  }
  bno.setExtCrystalUse(true);
}

void loop()
{


}
