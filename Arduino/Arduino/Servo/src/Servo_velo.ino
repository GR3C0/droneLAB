#include <Wire.h>
#include <NewPing.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define MOTOR_IZQ 5
#define MOTOR_DER 3
// Valores velocidad:
#define zero 2000
#define uno 1100
#define dos 1200
#define tres 1300
#define cuatro 1400
#define cinco 1500
#define seis 1600
#define siete 1700
#define ocho 1800
#define nueve 1900
#define esp 1000

Servo motor_izq;
Servo motor_der;
int respuesta;

void setup()
{
  Serial.begin(9600);
  Serial.println("Conectando motores");
  delay(2000);

  motor_der.attach(MOTOR_DER); //attatch the right motor to pin 3
  motor_izq.attach(MOTOR_IZQ);  //attatch the left motor to pin 5

  motor_izq.writeMicroseconds(1000); // Enviamos el valor minimo a los motores para que se enciendan
  motor_der.writeMicroseconds(1000);
  delay(1000); // Le damos 7 segundos para empezar

  // motor_izq.writeMicroseconds(2000);
  // motor_der.writeMicroseconds(2000);

}

void loop()
{

  while (!Serial.available()); // Comprueba si el puerto para leer datos está abierto
    respuesta = Serial.read(); // Lectura de la respuesta del usuario

  motor_izq.write(1500);
  motor_der.write(1500);

  if (respuesta == 48) // Valor en ASCII del número
  {
    motor_izq.writeMicroseconds(zero);
    motor_der.writeMicroseconds(zero);
    Serial.print("velocidad actual:");
    Serial.println(zero);
  }
  else if (respuesta == 49)
  {
    motor_izq.writeMicroseconds(uno);
    motor_der.writeMicroseconds(uno);
    Serial.print("velocidad actual:");
    Serial.println(uno);
  }
  else if (respuesta == 50)
  {
    motor_izq.writeMicroseconds(dos);
    motor_der.writeMicroseconds(dos);
    Serial.print("velocidad actual:");
    Serial.println(dos);
  }
  else if (respuesta == 51)
  {
    motor_izq.writeMicroseconds(tres);
    motor_der.writeMicroseconds(tres);
    Serial.print("velocidad actual:");
    Serial.println(tres);
  }
  else if (respuesta == 52)
  {
    motor_izq.writeMicroseconds(cuatro);
    motor_der.writeMicroseconds(cuatro);
    Serial.print("velocidad actual:");
    Serial.println(cuatro);
  }
  else if (respuesta == 53)
  {
    motor_izq.writeMicroseconds(cinco);
    motor_der.writeMicroseconds(cinco);
    Serial.print("velocidad actual:");
    Serial.println(cinco);
  }
  else if (respuesta == 54)
  {
    motor_izq.writeMicroseconds(seis);
    motor_der.writeMicroseconds(seis);
    Serial.print("velocidad actual:");
    Serial.println(seis);
  }
  else if (respuesta == 55)
  {
    motor_izq.writeMicroseconds(siete);
    motor_der.writeMicroseconds(siete);
    Serial.print("velocidad actual:");
    Serial.println(siete);
  }
  else if (respuesta == 56)
  {
    motor_izq.writeMicroseconds(ocho);
    motor_der.writeMicroseconds(ocho);
    Serial.print("velocidad actual:");
    Serial.println(ocho);
  }
  else if (respuesta == 57)
  {
    motor_izq.writeMicroseconds(nueve);
    motor_der.writeMicroseconds(nueve);
    Serial.print("velocidad actual:");
    Serial.println(nueve);
  }
  else if (respuesta == 32)
  {
    motor_izq.writeMicroseconds(esp);
    motor_der.writeMicroseconds(esp);
    Serial.print("velocidad actual:");
    Serial.println(esp);
  }

}
