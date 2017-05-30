// Programa general para droneLAB

// Libreias generales
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

Servo motor;

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


float sensor() // Función del acelerometro
{
  Serial.println("Prubea de fuerza ejercida");
  imu::Vector<3> acelerometro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // Los datos del acelerometro
  for (size_t t = 0; t < 5000; t++) {
    Serial.println(t);
  }
  int x;
  while (x < 5000)
  {
    x++;
    if (x
       == 5000)
    {
      Serial.println("Han pasado 5 segundos");
    }
  }

}
