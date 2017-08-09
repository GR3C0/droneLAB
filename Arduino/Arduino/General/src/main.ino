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

NewPing sonar[SONAR_NUM] = // Declaramos todos los sonars que hay
{
	NewPing(SONAR_1, TRIG_SONAR_1, MAX_DISTANCE), // Todos los pines están declarados en
	NewPing(SONAR_2, TRIG_SONAR_2, MAX_DISTANCE), // Configuracion.h
	NewPing(SONAR_3, TRIG_SONAR_3, MAX_DISTANCE),
  NewPing(SONAR_4, TRIG_SONAR_4, MAX_DISTANCE),
};


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
  // Dependiendo de lo que el arduino reciba que quiere hacer el usuario,
  // Usa una función o otra
  while (!Serial.available()); // Comprueba si el puerto para leer datos está abierto
  int respuesta;
  respuesta = Serial.read(); // Lectura de la respuesta del usuario
  Serial.println("Rozamiento cero 1");
  Serial.println("Sonars 2");

  if (respuesta == 1)
  {
    rozamiento_cero();
  }
  else if (respuesta == 2)
  {
      sonars();
  }
}

void rozamiento_cero()
{
  imu::Vector<3> acelerometro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // Obtiene la aceleración
  unsigned long t_inicio = millis(); // Obtiene el tiempo de ejecución del programa al iniciarse, este valor es 0
  unsigned long t_final; // Variable que almacenará el tiempo final para restarlo al inicial
  Serial.println(acelerometro.x()); // Solo para pruebas

  if (acelerometro.x()>AX_MIN || acelerometro.y()>AY_MIN || acelerometro.z()>AZ_MIN) // Si uno de los tres ejes sufre una aceleración
  {
    t_final = millis();
  }
  // Aquí va el tema de motores
  unsigned long aceleracion = t_final-t_inicio;
  float fuerza_motores;
  fuerza_motores = aceleracion * masa;
}

void sonars()
{
  // for(int i = 0; i < SONAR_NUM; i++)
  //{ // Ciclo para recorrer la lista de sonars
  //   int uS = sonar[i].ping_median(); // Lee el ping del sonar
  //   int distancia = uS / US_ROUNDTRIP_CM; // Lo transforma a cm
  //     Serial.print(distancia);
  //     Serial.print("cm");
  //     Serial.print("||");
  //}
  // int sonar1 = sonar[2].ping_median(); Así se coge solo los datos de un sonar
  // int distancia1 = sonar1 / US_ROUNDTRIP_CM;
  // Serial.println(distancia1);


}
