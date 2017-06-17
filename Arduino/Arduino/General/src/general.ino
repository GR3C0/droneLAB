// Programa general para droneLAB

// Libreias generales
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


float rozamiento_cero() // Función de rozamiento cero
{
  imu::Vector<3> acelerometro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // Obtiene la aceleración
  unsigned long t_inicio = millis(); // Obtiene el tiempo de ejecución del programa al iniciarse, este valor es 0
  unsigned long t_final; // Variable que almacenará el tiempo final para restarlo al inicial
  // CAMBIAR LOS VALORES SEGÚN LAS NECESIDADES
  float ax_min = 9.5;
  float ay_min = 9.5;
  float az_min = 9.5;

  Serial.println(acelerometro.x()); // Solo para pruebas
  if (acelerometro.x()>ax_min || acelerometro.y()>ay_min || acelerometro.z()>az_min) // Si uno de los tres ejes sufre una aceleración
  {
    t_final = millis();
  }
  // Aquí va el tema de motores
  unsigned long aceleracion = t_final-t_inicio;
  float Fuerza_motores = aceleracion * masa;
}
