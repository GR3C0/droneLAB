// Programa general para droneLAB

// Librerias generales
#include "Configuracion.h" // Archivo con todos los pines definidos, consultar en caso de duda
#include <NewPing.h>
#include <Wire.h>
#include <imumaths.h>
#include <Servo.h>

NewPing sonar[SONAR_NUM] =
{
	NewPing(SONAR_1, TRIG_SONAR_1, MAX_DISTANCE),
	NewPing(SONAR_2, TRIG_SONAR_2, MAX_DISTANCE),
	NewPing(SONAR_3, TRIG_SONAR_3, MAX_DISTANCE),
  NewPing(SONAR_4,TRIG_SONAR_4, MAX_DISTANCE),
	NewPing(SONAR_5,TRIG_SONAR_5,MAX_DISTANCE),
};

// Declaración de motores
Servo motor_izq;
Servo motor_der;
Servo motor_izq_2;
Servo motor_der_2;

void setup()
{
	// Inicio del MPU6050
	Wire.begin();
	Wire.beginTransmission(0x68);
	Wire.write(0x6B);
	Wire.write(0);
	Wire.endTransmission(true);
	Serial.begin(250000);
	// Activacion de los motores y envio del valor minimo y maximo
	motor_del_izq.attach(MOTOR_DEL_IZQ, MOTOR_MAX_LEVEL, MOTOR_MIN_LEVEL);
	motor_del_der.attach(MOTOR_DEL_DER, MOTOR_MAX_LEVEL, MOTOR_MIN_LEVEL);
	motor_tras_izq.attach(MOTOR_TRAS_IZQ, MOTOR_MAX_LEVEL, MOTOR_MIN_LEVEL);
	motor_tras_der.attach(MOTOR_TRAS_DER, MOTOR_MIN_LEVEL, MOTOR_MAX_LEVEL);

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
	PID(); // Luego se activa el PID
}

void loop()
{
	if(Serial.available()) // Si está disponible la lectura de la raspi
	{
		int comando = Serial.read();// Leemos lo enviado por el usuario
		// Menu para elegir la operacion
		switch (comando)
		{
			case 48:
							{
								break;
							}

		}
  }
}

void sonars() // TODO: implementar sonares a los movimientos, código de uso
{
  for(int i = 0; i < SONAR_NUM; i++)
	{
    int uS = sonar[i].ping_median();
    int distancia = uS / US_ROUNDTRIP_CM;
      Serial.print(distancia);
      Serial.print("cm");
      Serial.print("||");

  }
}

void pid()
{

/////////////////////////////I M U/////////////////////////////////////
	timePrev = time;
  time = millis(); // Se lee otra vez el tiempo
  elapsedTime = (time - timePrev) / 1000; // Se pasa a segundos

   Wire.beginTransmission(0x68);
	 Wire.write(0x3B);
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,6,true);

   // Lectura de datos
   Acc_rawX=Wire.read()<<8|Wire.read(); // Cada valor necesita dos regristros
   Acc_rawY=Wire.read()<<8|Wire.read();
   Acc_rawZ=Wire.read()<<8|Wire.read();

   // -------------------Calculo de las ecuaciones de euler------------------

   // Dividimos los valores obtenidos con el sensor entre 16384 para que nos de la acceleracion
   // Calculamos los angulos en grados
   // Aplicamos la formula de Euler atan() = arctangente
   /*---X---*/
   Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
   /*---Y---*/
   Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

   Wire.beginTransmission(0x68);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); // 4 Registros

   Gyr_rawX=Wire.read()<<8|Wire.read();
   Gyr_rawY=Wire.read()<<8|Wire.read();

   // Dividimos los datos que recibimos entre 131 por que es el dato que nos da el sensor

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0;
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;

   // Multiplicamos el angulo de giro por el tiempo
   // Aplicamos el filtro, añadimos la aceleracion y lo multiplicamos por 0.98

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

   // Tenemos los angulos entre -100º y 100º
   //Serial.println(Total_angle[1]);

   /*///////////////////////////P I D///////////////////////////////////*/

   // Calculamos el error
   error = Total_angle[1] - desired_angle;

   // La constante multiplicada por el error
   pid_p = kp*error;

   // La integral solo debe funcionar si nos acercamos al angulo 0, por eso solo lo usamos
   // En un angulo entre -3 y 3.
   if(-3 <error <3)
   {
     pid_i = pid_i+(ki*error);
   }

  // La derivada se encarga de la velocidad de corrección del error

  pid_d = kd*((error - previous_error)/elapsedTime);

  // El PID es la suma de las tres operaciones
  PID = pid_p + pid_i + pid_d;

   if(PID < -1000)
   {
     PID=-1000;
 	 }
   if(PID > 1000)
   {
     PID=1000;
   }

   // Sumamos el valor minimo + el PID
   pwmLeft = throttle + PID;
   pwmRight = throttle - PID;
	 pwmLeft2 = throttle - PID;
	 pwmRight2 = throttle + PID;

   // Hacemos que no se pasen los datos
   //Right
   if(pwmRight < 1200)
   {
     pwmRight= 1200;
   }
   if(pwmRight > 1250)
   {
     pwmRight=1250;
   }
	 if(pwmRight2 < 1200)
   {
     pwmRight2 = 1200;
   }
   if(pwmRight2 > 1250)
   {
     pwmRight2 =1250;
   }
   //Left
   if(pwmLeft < 1200)
   {
     pwmLeft= 1200;
   }
   if(pwmLeft > 1250)
   {
     pwmLeft=1250;
   }
	 if(pwmLeft2 < 1200)
   {
     pwmLeft2= 1200;
   }
   if(pwmLeft2 > 1250)
   {
     pwmLeft2=1250;
   }
   // Mandamos los datos al motor
   motor_izq.writeMicroseconds(pwmLeft);
	 motor_izq_2.writeMicroseconds(pwmLeft2);
   Serial.print(pwmLeft);
   Serial.print("||");
   motor_der.writeMicroseconds(pwmRight);
	 motor_der_2.writeMicroseconds(pwmRight2);
   Serial.println(pwmRight);
   previous_error = error; // Almacena el error previo

}

void calibracion()
{
	Serial.println(" No conectes aun la bateria");
	// Enviamo el maximo de velocidad
	motor_izq.write(180);
	motor_der.write(180);
	motor_izq_2.write(180);
	motor_der_2.write(180);
	delay(5000)
	Serial.println(" Conecta la bateria");
	// Envio del minimo de velocidad
	motor_izq.write(0);
	motor_der.write(0);
	motor_izq_2.write(0);
	motor_der_2.write(0);
	delay(4000)

}
