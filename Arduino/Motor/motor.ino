// Uso del motor mediante el modulo L298N	
#include <NewPing.h>
const int pinENA = 6; // Cada pin tiene su respectiva conexion en el modulo
const int pinIN1 = 7;
const int pinIN2 = 14;
const int pinIN3 = 13;
const int pinIN4 = 12;
const int pinENB = 11;
const int pinLedRojo = 10; // Pins de leds
const int pinLedAmarillo = 9;
const int pintLedVerde = 8;
 
const int speed = 2; //velocidad de giro
 
const int pinMotorA[3] = { pinENA, pinIN1, pinIN2 }; // Agrupa todos los pines del primer motor en una lista
const int pinMotorB[3] = { pinENB, pinIN3, pinIN4 };

NewPing sonar(3,2,450); // Funcion para el sonar que recibe el trigger, el echo y la distancia maxima respectivamente
 
void setup()
{	// Inicia todos los pines
	Serial.begin(9600);
	pinMode(pinIN1, OUTPUT);
	pinMode(pinIN2, OUTPUT);
	pinMode(pinENA, OUTPUT);
	pinMode(pinIN3, OUTPUT);
	pinMode(pinIN4, OUTPUT);
	pinMode(pinENB, OUTPUT);
	pinMode(pinLedRojo, OUTPUT);
	pinMode(pinLedAmarillo, OUTPUT);
	pinMode(pintLedVerde, OUTPUT);
}
 
void loop()
{
	// Llamada de funciones
	moveForward(pinMotorA, 20);
	moveForward(pinMotorB, 20);
	delay(2000);
	
	moveBackward(pinMotorA, 20);
	moveBackward(pinMotorB, 20);
	delay(2000);
	
	fullStop(pinMotorA);
	fullStop(pinMotorB);
	delay(2000);
}
 
void moveForward(const int pinMotor[3], int speed)
{
 digitalWrite(pintLedVerde, HIGH);
 digitalWrite(pinLedAmarillo, LOW);
 digitalWrite(pinLedRojo, LOW);

 digitalWrite(pinMotor[1], HIGH);
 Serial.println("ADELANTE");
 digitalWrite(pinMotor[2], LOW);
 analogWrite(pinMotor[0], speed);
 Serial.println(speed);
}
 
void moveBackward(const int pinMotor[3], int speed)
{
 digitalWrite(pintLedVerde, LOW);
 digitalWrite(pinLedAmarillo, HIGH);
 digitalWrite(pinLedRojo, LOW);

 digitalWrite(pinMotor[1], LOW);
 digitalWrite(pinMotor[2], HIGH);
 Serial.println("ATRASSS");
 
 analogWrite(pinMotor[0], speed);
 Serial.println(speed);
}
 
void fullStop(const int pinMotor[3])
{
 digitalWrite(pintLedVerde, LOW);
 digitalWrite(pinLedAmarillo, LOW);
 digitalWrite(pinLedRojo, HIGH);

 digitalWrite(pinMotor[1], LOW);
 digitalWrite(pinMotor[2], LOW);
 Serial.println("QUIETO PARADO CHAVAAAL");
 analogWrite(pinMotor[0], 0);
}
