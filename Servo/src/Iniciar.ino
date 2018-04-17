#include<Servo.h>

Servo ESC; //Crear un objeto de clase servo
Servo ESC2;
Servo ESC3;
Servo ESC4;

int vel = 1000; //amplitud del pulso

void setup()
{
  //Asignar un pin al ESC
  ESC.attach(5);
  ESC2.attach(3);
  ESC3.attach(6);
  ESC4.attach(7);

  //Activar el ESC
  ESC.writeMicroseconds(1000); //1000 = 1ms
  ESC2.writeMicroseconds(1000);
  ESC3.writeMicroseconds(1000);
  ESC4.writeMicroseconds(1000);
  //Cambia el 1000 anterior por 2000 si
  //tu ESC se activa con un pulso de 2ms
  delay(2000); //Esperar 5 segundos para hacer la activacion

  //Iniciar puerto serial
  Serial.begin(9600);
  Serial.println("Iniciado");
  Serial.setTimeout(10);


}


void loop()
{
  if(Serial.available() >= 1)
  {
    vel = Serial.parseInt(); //Leer un entero por serial
    if(vel != 0)
    {
      ESC.writeMicroseconds(vel); //Generar un pulso con el numero recibido
      ESC2.writeMicroseconds(vel);
      ESC3.writeMicroseconds(vel);
      ESC4.writeMicroseconds(vel);
    }
  }
}
