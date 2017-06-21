#include <Servo.h>

#define MOTOR_IZQ 3
#define MOTOR_DER 5

Servo motor_izq;
Servo motor_der;

void setup()
{
  Serial.begin(9600);
  Serial.println("Conectando motores");
  delay(2000);

  motor_izq.attach(MOTOR_IZQ);
  motor_der.attach(MOTOR_DER);

  motor_izq.writeMicroseconds(1100);
  motor_der.writeMicroseconds(1100);

}

void loop()
{

  motor_izq.writeMicroseconds(1200);// DER
  motor_der.writeMicroseconds(1200);
  Serial.println("Motores conectados");

}
