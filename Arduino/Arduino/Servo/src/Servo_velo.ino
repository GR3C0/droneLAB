#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1500
#define MEDIUM_SIGNAL 1750
#define MOTOR_PIN 10

Servo motor;

void setup()
{
  Serial.begin(9600);
  Serial.println("Conectando motores");
  delay(2000);

  motor.attach(MOTOR_PIN);

  Serial.println("Motor a velocidad minima");
  motor.writeMicroseconds(MIN_SIGNAL);
  delay(2000);
  Serial.println("Motor a velocidad media");
  motor.writeMicroseconds(MEDIUM_SIGNAL);
  delay(5000);
  Serial.println("Motor a velocidad maxima");
  motor.writeMicroseconds(MAX_SIGNAL);

}

void loop()
{


}
