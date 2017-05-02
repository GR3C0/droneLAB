// Prueba de Leds + Adafruit + Sonar
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define ECHO_PIN  	  3
#define TRIGGER_PIN   4
#define MAX_DISTANCE  450

int pinENA = 6;
int pinIN1 = 7;
int pinIN2 = 14;
int pinIN3 = 13;
int pinIN4 = 12;
int pinENB = 11;
int LEDVERDE = 35;
int LEDROJO = 33;
int LEDAMARILLO = 31;
int speed = 200; // Velocidad de giro del motor

int pinMotorA[3] = { pinENA, pinIN1, pinIN2 }; // Agrupa todos los pines del primer motor en una lista
int pinMotorB[3] = { pinENB, pinIN3, pinIN4 };

Adafruit_BNO055 bno = Adafruit_BNO055(55); // Llamamos al Sensor
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup()
{
	Serial.begin(9600); //Canal de entrada
	Serial.println("Orientation Sensor Test"); Serial.println("");
    if(!bno.begin())
    {
    	Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    	while(1);
  	}
  
  	delay(1000);
	bno.setExtCrystalUse(true); 
	pinMode(pinIN1, OUTPUT);
	pinMode(pinIN2, OUTPUT);
	pinMode(pinENA, OUTPUT);
	pinMode(pinIN3, OUTPUT);
	pinMode(pinIN4, OUTPUT);
	pinMode(pinENB, OUTPUT);
	pinMode(LEDVERDE, OUTPUT); 
	pinMode(LEDROJO, OUTPUT);
	pinMode(LEDAMARILLO, OUTPUT);
}

void loop()
{
	if(Serial.available()) // Lectura de la raspi
	{
		char c = Serial.read();

	}
	sensor();
	Sonar();
}

float sensor()
{
	imu::Vector<3> acelerometro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // Los datos del acelerometro
	Serial.print("aX: ");
 	Serial.print(acelerometro.x());
 	Serial.print("||");
 	Serial.print(" aY: ");
 	Serial.print(acelerometro.y());
 	Serial.print("||");
 	Serial.print(" aZ: ");
 	Serial.print(acelerometro.z());
 	Serial.println();
}

float Sonar()
{
	delay(10); // Espera 10 ms entre mediciones
	int uS = sonar.ping_median(); // Calcula la velocidad del sonido
	Serial.print("Distancia:");
	int distancia = uS / US_ROUNDTRIP_CM; // Calcula la distancia
	Serial.print(distancia);
  	Serial.print("cm");

  	if(distancia<=50) // Cuando la distancia es menos o igual a 50 cm SOLO el led rojo se enciende
  	{	 
    	digitalWrite(LEDROJO, HIGH);
    	digitalWrite(LEDVERDE, LOW);
    	digitalWrite(LEDAMARILLO, LOW);
  	}
  	else if(distancia>50 & distancia<300) // Si la distancia esta entre 50 y 300 SOLO el led verde se enciende
  	{
    	digitalWrite(LEDROJO, LOW);
    	digitalWrite(LEDVERDE, HIGH);
    	digitalWrite(LEDAMARILLO, LOW);
  	}
  	else // Si la distancia es mayor a 300 cm SOLO el led amarillo se enciende
  	{
    	digitalWrite(LEDROJO, LOW);
    	digitalWrite(LEDVERDE, LOW);
    	digitalWrite(LEDAMARILLO, HIGH);
  	}
}

void moverAdelante(const int pinMotor[3], int speed)
{
	digitalWrite(LEDVERDE, HIGH);
	digitalWrite(LEDAMARILLO, LOW);
	digitalWrite(LEDROJO, LOW);

	digitalWrite(pinMotor[1], HIGH);
	Serial.println("ADELANTE");
	digitalWrite(pinMotor[2], LOW);
	analogWrite(pinMotor[0], speed);
	Serial.println(speed);
}
 
void moverAtras(const int pinMotor[3], int speed)
{
	digitalWrite(LEDVERDE, LOW);
	digitalWrite(LEDAMARILLO, HIGH);
	digitalWrite(LEDROJO, LOW);

	digitalWrite(pinMotor[1], LOW);
	digitalWrite(pinMotor[2], HIGH);
	Serial.println("ATRASSS");
	 
	analogWrite(pinMotor[0], speed);
	Serial.println(speed);
}
 
void parar(const int pinMotor[3])
{
	digitalWrite(LEDVERDE, LOW);
	digitalWrite(LEDAMARILLO, LOW);
	digitalWrite(LEDROJO, HIGH);

	digitalWrite(pinMotor[1], LOW);
	digitalWrite(pinMotor[2], LOW);
	Serial.println("QUIETO PARADO CHAVAAAL");
	analogWrite(pinMotor[0], 0);
}

