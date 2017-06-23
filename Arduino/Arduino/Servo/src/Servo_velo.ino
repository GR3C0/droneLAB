#include <Servo.h>

#define MOTOR_IZQ 9
#define MOTOR_DER 3

Servo motor_izq;
Servo motor_der;

void setup()
{
  Serial.begin(9600);
  Serial.println("Conectando motores");
  delay(2000);

  motor_izq.attach(MOTOR_IZQ);
  motor_der.attach(MOTOR_DER);

  motor_izq.writeMicroseconds(1000);
  motor_der.writeMicroseconds(1000);

}

void loop()
{

  motor_izq.writeMicroseconds(1250);// DER
  motor_der.writeMicroseconds(1250);
  Serial.println("Motores conectados");

}
