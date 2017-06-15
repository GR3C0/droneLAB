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
	unsigned long inicio = millis(); // Al incio del programa se inicia millis()
	// Imprime los diferentes valores
	// Serial.print("aX: ");
	//Serial.println(acelerometro.x());
	// Serial.print(" aY: ");
	// Serial.println(acelerometro.y());
	// Serial.print(" aZ: ");
	// Serial.println(acelerometro.z());
	float a_min = 9.5;
	Serial.println(acelerometro.x());
	// while (acelerometro.x()>a_min)
	// {
	// 	unsigned long resultado = millis();
	// 	Serial.println(resultado);
	// }

	if(acelerometro.x()>=a_min)
	{
		unsigned long tiempo = millis(); // Se inicia otro millis() para calcular el tiempo de la aceleración
	}
	Serial.println(tiempo-inicio); // Se resta el tiempo de ejecucion - el tiempo del inicio
}

	// Serial.print("Tiempo delay : ");
	// unsigned long time_ms = millis(); // Llamada a la función millis()
	// delay(100);
	// unsigned long resultado = micros()-time_ms; // Se resta los microsegundos a la función de millis()
	// Serial.println(resultado); // tiempo transcurrido en milisegundos
