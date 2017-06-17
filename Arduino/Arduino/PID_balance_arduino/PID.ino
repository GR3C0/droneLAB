#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Declaración de motores
Servo right_prop;
Servo left_prop;

Adafruit_BNO055 bno = Adafruit_BNO055(55); // Iniciar el sensor

float Angulo_aceleracion[2];
float Angulo_giro[2];
float Angulo_total[2];

float T_transcurrido, time, timePrev;
int i;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=3.55;//3.55
double ki=0.005;//0.003
double kd=2.05;//2.05
///////////////////////////////////////////////

double velocidad=1300; // Velocidad inicial de los motores
float angulo_deseado = 0; // Angulo que queremos

void setup()
{
  Serial.begin(9600);
  right_prop.attach(3); //attatch the right motor to pin 3
  left_prop.attach(5);  //attatch the left motor to pin 5

  time = millis(); // Empieza a contar el tiempo
  left_prop.writeMicroseconds(1000); // Enviamos el valor minimo a los motores para que se enciendan
  right_prop.writeMicroseconds(1000);
  delay(7000); // Le damos 7 segundos para empezar

}

void loop()
{
  /////////////////////////////I M U/////////////////////////////////////
  timePrev = time;  // Almacenamos otra vez el tiempo
  time = millis();  // Lectura del tiempo actual
  T_transcurrido = (time - timePrev) / 1000; // Se calcula el tiempo pasado en segundos

  // Lectura de la aceleracion y los angulos
  imu::Vector<3> acelerometro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  

}
