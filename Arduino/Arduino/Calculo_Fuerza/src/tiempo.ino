#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup()
{
	Serial.begin(9600);

	// Comprobación de los sensores
	if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);

  }

  delay(1000);

  bno.setExtCrystalUse(true);

}

void loop()
{

	imu::Vector<3> acelerometro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // Obtiene la aceleración

	// Imprime los diferentes valores
	Serial.print("aX: ");
	Serial.print(acelerometro.x());
	Serial.print(" aY: ");
	Serial.print(acelerometro.y());
	Serial.print(" aZ: ");
	Serial.print(acelerometro.z());

	//
	// Serial.print("Tiempo delay : ");
	// unsigned long time_ms = millis(); // Llamada a la función millis()
	// delay(100);
	// unsigned long resultado = micros()-time_ms; // Se resta los microsegundos a la función de millis()
	// Serial.println(resultado); // tiempo transcurrido en milisegundos

}
