#include <Arduino.h>

#include  <Servo.h>
 
Servo ESC; //Crear un objeto de clase servo
	 
void setup()
{
	//Asignar un pin al ESC
	ESC.attach(9);
	
	//Activar el ESC
	ESC.writeMicroseconds(2000); //1000 = 1ms
	//Cambia el 1000 anterior por 2000 si
	//tu ESC se activa con un pulso de 2ms
	 
	//Iniciar puerto serial
	Serial.begin(9600);
		 
}
 
void loop()
{
	ESC.writeMicroseconds(1200);
	delay(2000);
	ESC.writeMicroseconds(1400);
	delay(2000);
	ESC.writeMicroseconds(1500);
	delay(2000);
	ESC.writeMicroseconds(1990);

}

