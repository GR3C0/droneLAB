#include <Servo.h>

#define LMOTOR 3
#define RMOTOR 4

Servo lmotor;
Servo rmotor;

void setup()
{
  delay(500);
  lmotor.attach(LMOTOR);
  rmotor.attach(RMOTOR);
}

void loop()
{
  int i;
  while(i<=2000)
  {
    rmotor.writeMicroseconds(i);
    lmotor.writeMicroseconds(i);
    i++;
  }

}
