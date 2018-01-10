// Programa general para droneLAB

// Librerias generales
#include "Configuracion.h"
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
Servo right_prop;
Servo left_prop;
Servo right_2_prop;
Servo left_2_prop;

void setup()
{
	Wire.begin();
	Wire.beginTransmission(0x68);
	Wire.write(0x6B);
	Wire.write(0);
	Wire.endTransmission(true);
	Serial.begin(250000);
	right_prop.attach(3); // Arratch al pin del motor derecho
	left_prop.attach(5);  // Attatch al pin del motor izquierdo
	right_2_prop.attach(4);
	left_2_prop.attach(6);

	time = millis(); // Medición del tiempo
	// Se envia el valor minimo al ESC para que se calibre
	left_prop.writeMicroseconds(1100);
	right_prop.writeMicroseconds(1100);
	right_2_prop.writeMicroseconds(1100);
	left_2_prop.writeMicroseconds(1100);
}

void loop()
{
	if(Serial.available()) // Si está disponible la lectura de la raspi
	{
		// char comando = Serial.read();
		// if(comando == 'sonar')
		// {
		// 		Sonar();
		// }
		// else if(comando == 'sensor')
		// {
		// 		sensor();
  }
}

void rozamiento_cero()
{
  // unsigned long t_inicio = millis(); // Obtiene el tiempo de ejecución del programa al iniciarse, este valor es 0
  // unsigned long t_final; // Variable que almacenará el tiempo final para restarlo al inicia
  // // Aquí va el tema de motores
  // unsigned long aceleracion = t_final-t_inicio;
  // float fuerza_motores;
  // fuerza_motores = aceleracion * MASA_DRONE;
}

void sonars()
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
	  elapsedTime = (time - timePrev) / 1000; // Se para a segundos

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
	   left_prop.writeMicroseconds(pwmLeft);
		 left_2_prop.writeMicroseconds(pwmLeft2);
	   Serial.print(pwmLeft);
	   Serial.print("||");
	   right_prop.writeMicroseconds(pwmRight);
		 right_2_prop.writeMicroseconds(pwmRight2);
	   Serial.println(pwmRight);
	   previous_error = error; //Remember to store the previous error.

}
