// Programa general para droneLAB

// Libreias generales
#include "Configuracion.h"
#include <NewPing.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include <Servo.h>

Servo motor_der;
Servo motor_izq;
float masa = 2; // Masa del drone

NewPing sonar[SONAR_NUM] =
{
	NewPing(SONAR_1, TRIG_SONAR_1, MAX_DISTANCE),
	NewPing(SONAR_2, TRIG_SONAR_2, MAX_DISTANCE),
	NewPing(SONAR_3, TRIG_SONAR_3, MAX_DISTANCE),
  NewPing(SONAR_4,TRIG_SONAR_4, MAX_DISTANCE),
};


void setup()
{

}

void loop()
{
	if(Serial.available()) // Si está disponible la lectura de la raspi
	{
		char comando = Serial.read();
		if(comando == 'sonar')
		{
				Sonar();
		}
		else if(comando == 'sensor')
		{
				sensor();
		}
}

void rozamiento_cero()
{
  imu::Vector<3> acelerometro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // Obtiene la aceleración
  unsigned long t_inicio = millis(); // Obtiene el tiempo de ejecución del programa al iniciarse, este valor es 0
  unsigned long t_final; // Variable que almacenará el tiempo final para restarlo al inicial
  Serial.println(acelerometro.x()); // Solo para pruebas

  if (acelerometro.x()>AX_MIN || acelerometro.y()>AY_MIN || acelerometro.z()>AZ_MIN) // Si uno de los tres ejes
	{																																										// sufre una aceleración
    t_final = millis();
  }
  // Aquí va el tema de motores
  unsigned long aceleracion = t_final-t_inicio;
  float fuerza_motores;
  fuerza_motores = aceleracion * masa;
}

void sonars()
{
  for(int i = 0; i < SONAR_NUM; i++){
    int uS = sonar[i].ping_median();
    int distancia = uS / US_ROUNDTRIP_CM;
      Serial.print(distancia);
      Serial.print("cm");
      Serial.print("||");
  }
  Serial.println();

}

void pid()
{
	Serial.print("Activado PID");
		/////////////////////////////I M U/////////////////////////////////////
	timePrev = time;  // Almacenamos otra vez el tiempo
	time = millis();  // Lectura del tiempo actual
	T_transcurrido = (time - timePrev) / 1000; // Se calcula el tiempo pasado en segundos

	// Lectura de la aceleracion y los angulos
	imu::Vector<3> acelerometro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	imu::Vector<3> giroscopio = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);


	// Obtenemos los grados

	Serial.print(euler.x());
	Serial.print("||");
	Serial.print(euler.y());
	Serial.print("||");
	Serial.println(euler.z());

	// Datos del giroscopio pasados a grados/s

	// giro_z = giroscopio.z()*57.2958;
	// giro_y = giroscopio.y()*57.2958;
	giro_z = euler.z();
	giro_y = euler.y();

	acel_z = acelerometro.z();
	acel_y = acelerometro.y();

	Angulo_aceleracion[0] = acel_z;
	Angulo_aceleracion[1] = acel_y;
	Angulo_giro[0] = giro_z;
	Angulo_giro[1] = giro_y;

	/*---Ángulo Z---*/
	// Angulo_total[0] = 0.98 *(Angulo_total[0] + Angulo_giro[0]*T_transcurrido) + 0.02*Angulo_aceleracion[0];
	/*---Ángulo Y---*/
	// Angulo_total[1] = 0.98 *(Angulo_total[1] + Angulo_giro[1]*T_transcurrido) + 0.02*Angulo_aceleracion[1];

	// Almacenamos el error
	// error = Angulo_total[0]-angulo_deseado;
	error = Angulo_giro[0] - angulo_deseado;

	// Esta es la constante que solo hace falta multiplicarla por el error
	pid_p = kp*error;

	// La integral solo debe actuar si el error es mínimo
	if(-3 < error || error < 3)
	{
		pid_i = pid_i+(ki*error);
	}

	// La derivada
	pid_d = kd*((error - error_previo)/T_transcurrido);

	// Almacenamos todo el PID
	PID = pid_p + pid_i + pid_d;

	// Como la velocidad de los motores solo puede ir entre 1000 y 2000 hacemos unos ajustes para que no se vuelva loco
	if(PID < -1000)
	{
		PID=-1000;
	}
	if(PID > 1000)
	{
		PID=1000;
	}

	// Finalmente le decimos a los motores que hacer
	pwmIzq = velocidad + PID;
	pwmDer = velocidad - PID;

	// Reajustamos la velocidad por si acaso
	if(pwmDer < 1200)
	{
		pwmDer= 1200;
	}
	if(pwmDer > 2000)
	{
		pwmDer=2000;
	}

	if(pwmIzq < 1200)
	{
		pwmIzq= 1200;
	}
	if(pwmIzq > 2000)
	{
		pwmIzq=2000;
	}

	// Le enviamos los datos a los motores
	motor_der.writeMicroseconds(pwmIzq);
	Serial.print(pwmIzq);
	Serial.print("||");
	motor_izq.writeMicroseconds(pwmDer);
	Serial.println(pwmDer);
	error_previo = error; // Almacenamos el error

	motor_der.writeMicroseconds(1700);
	motor_izq.writeMicroseconds(1700);

}
