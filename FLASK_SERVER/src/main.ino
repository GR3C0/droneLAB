// Programa general para droneLAB
// TODO: Protocolos de conexion de todos los sensores
// Librerias generales
#include <Funciones_volar.h> // TODO: Prueba
//#include <Configuracion.h> // Archivo con todos los pines definidos, consultar en caso de duda
#include <NewPing.h>
#include <imumaths.h>

NewPing sonar[SONAR_NUM] =
{
	NewPing(SONAR_1, TRIG_SONAR_1, MAX_DISTANCE),
	NewPing(SONAR_2, TRIG_SONAR_2, MAX_DISTANCE),
	NewPing(SONAR_3, TRIG_SONAR_3, MAX_DISTANCE),
  NewPing(SONAR_4,TRIG_SONAR_4, MAX_DISTANCE),
	NewPing(SONAR_5,TRIG_SONAR_5,MAX_DISTANCE),
};

void setup()
{
  Serial.begin(9600);

  delay(1000);
  time = millis();

	// Armar motores
	motor_del_izq.write(180);
	motor_del_der.write(180);
	motor_tras_izq.write(180);
	motor_tras_der.write(180);

	delay(3000); // Delay de 3 segundos para que se lo piense

	motor_del_izq.write(0);
	motor_del_der.write(0);
	motor_tras_izq.write(0);
	motor_tras_der.write(0);

	time = millis(); // Medición del tiempo
	// calibracion del ESC
	//calibracion(); // Primero se calibra
}

void sonars() // TODO: implementar sonares a los movimientos, código de uso
{
	delay(10);
  for(int i = 0; i < SONAR_NUM; i++)
	{
    int uS = sonar[i].ping_median();
    int distancia = uS / US_ROUNDTRIP_CM;
      Serial.print(distancia);
      Serial.print("cm");
      Serial.print("||");
  }
	Serial.println();
}

void loop()
{
	if(Serial.available()) // Si está disponible la lectura de la raspi
	{
		int comando = Serial.read();// Leemos lo enviado por el usuario
		// Menú para elegir la operacion
		switch (comando)
		{
			case 48:
							{while (true) {
								pid();
							}
								break;
							}
		}
  }
	// Esto se envia a los motores, la velocidad despues de hacer los calculos
	motor_del_izq.writeMicroseconds(pwm_tras_der);
	motor_del_der.writeMicroseconds(pwm_tras_izq);
	motor_tras_izq.writeMicroseconds(pwm_del_izq);
	motor_tras_der.writeMicroseconds(pwm_del_der);
}
