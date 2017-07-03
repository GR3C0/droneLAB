#include <Servo.h>

#define MOTOR_IZQ 4
#define MOTOR_DER 3

Servo motor_izq;
Servo motor_der;

void setup()
{
  Serial.begin(9600);
  Serial.println("Conectando motores");
  delay(2000);

  pinMode(MOTOR_IZQ, OUTPUT);
  pinMode(MOTOR_DER, OUTPUT);

  // motor_izq.attach(MOTOR_IZQ);
  // motor_der.attach(MOTOR_DER);

  while (!Serial.available());
    Serial.read();

  // motor_izq.writeMicroseconds(2000);
  // motor_der.writeMicroseconds(2000);

}

void loop()
{

  analogWrite(MOTOR_IZQ, 2000);

  // motor_izq.write(1500);
  // motor_der.write(1500);
  Serial.println("Motores conectados");

}
