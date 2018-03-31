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
Servo motor_del_izq;
Servo motor_del_der;
Servo motor_tras_izq;
Servo motor_tras_der;

void setup()
{
	time = millis(); // Se mide el tiempo
	// Activacion de los motores y envio del valor minimo y maximo
	motor_del_izq.attach(MOTOR_DEL_IZQ, MOTOR_MAX_LEVEL, MOTOR_MIN_LEVEL);
	motor_del_der.attach(MOTOR_DEL_DER, MOTOR_MAX_LEVEL, MOTOR_MIN_LEVEL);
	motor_tras_izq.attach(MOTOR_TRAS_IZQ, MOTOR_MAX_LEVEL, MOTOR_MIN_LEVEL);
	motor_tras_der.attach(MOTOR_TRAS_DER, MOTOR_MIN_LEVEL, MOTOR_MAX_LEVEL);

	// Conexion con el MPU6050
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission(true);

  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);

  Serial.begin(9600);

  delay(1000);
  time = millis();

  // Calculamos el error del giroscopio antes del loop
  // Le damos un valor de 200, será suficiente
  if(gyro_error==0)
  {
    for(int i=0; i<200; i++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,4,true);

      Gyr_rawX=Wire.read()<<8|Wire.read();
      Gyr_rawY=Wire.read()<<8|Wire.read();

      /*---X---*/
      Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX/32.8);
      /*---Y---*/
      Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY/32.8);
      if(i==199)
      {
        Gyro_raw_error_x = Gyro_raw_error_x/200;
        Gyro_raw_error_y = Gyro_raw_error_y/200;
        gyro_error=1;
      }
    }
  }

  // Calculamos el error del acelerometro antes del loop
  // Le damos un valor de 200, con eso será suficiente

  if(acc_error==0)
  {
    for(int a=0; a<200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true);

      Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ;  // Cada valor necesita dos registros
      Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
      Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ;


      /*---X---*/
      Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg));
      /*---Y---*/
      Acc_angle_error_y = Acc_angle_error_y + ((atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg));

      if(a==199)
      {
        Acc_angle_error_x = Acc_angle_error_x/200;
        Acc_angle_error_y = Acc_angle_error_y/200;
        acc_error=1;
      }
    }
  }

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

void loop()
{
	if(Serial.available()) // Si está disponible la lectura de la raspi
	{
		int comando = Serial.read();// Leemos lo enviado por el usuario
		// Menú para elegir la operacion
		switch (comando)
		{
			case 48:
							{
								PID();
								break;
							}
		}
  }
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

void pid()
{
	/////////////////////////////I M U/////////////////////////////////////
	    timePrev = time;
	    time = millis(); // Se lee otra vez el tiempo
	    elapsedTime = (time - timePrev) / 1000; // Se para a segundos

	    // Lectura del giroscopio
	     Wire.beginTransmission(0x68);
	     Wire.write(0x43);
	     Wire.endTransmission(false);
	     Wire.requestFrom(0x68,4,true);

	     Gyr_rawX = Wire.read()<<8|Wire.read();
	     Gyr_rawY = Wire.read()<<8|Wire.read();

	     /*---X---*/
	     Gyr_rawX = (Gyr_rawX/32.8) - Gyro_raw_error_x;
	     /*---Y---*/
	     Gyr_rawY = (Gyr_rawY/32.8) - Gyro_raw_error_y;

	     /*---X---*/
	     Gyro_angle_x = Gyr_rawX*elapsedTime;
	     /*---X---*/
	     Gyro_angle_y = Gyr_rawY*elapsedTime;

	     Wire.beginTransmission(0x68);
	     Wire.write(0x3B);
	     Wire.endTransmission(false);
	     Wire.requestFrom(0x68,6,true);

	     // Lectura de datos
	     Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ;
	     Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
	     Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ;

	     /*---X---*/
	     Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_x;
	     /*---Y---*/
	     Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_y;

	     //////////////////////////////////////Total angle and filter/////////////////////////////////////
	     /*---X axis angle---*/
	     Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
	     /*---Y axis angle---*/
	     Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;


	     /*///////////////////////////P I D///////////////////////////////////*/
	     roll_desired_angle = map(input_ROLL,1000,2000,-10,10);
	     pitch_desired_angle = map(input_PITCH,1000,2000,-10,10);

	     // Calculamos el error
	     roll_error = Total_angle_y - roll_desired_angle;
	     pitch_error = Total_angle_x - pitch_desired_angle;

	     // Calculamos la kp
	     roll_pid_p = roll_kp*roll_error;
	     pitch_pid_p = pitch_kp*pitch_error;

	     if(-3 < roll_error and roll_error <3)
	     {
	       roll_pid_i = roll_pid_i+(roll_ki*roll_error);
	     }
	     if(-3 < pitch_error and pitch_error <3)
	     {
	       pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error);
	     }

	     // Calculamos la derivada
	     roll_pid_d = roll_kd*((roll_error - roll_previous_error)/elapsedTime);
	     pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error)/elapsedTime);

	     // Calculamos el PID global
	     roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
	     pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;

	     if(roll_PID < -1000){roll_PID=-1000;}
	     if(roll_PID > 2000) {roll_PID=2000;}
	     if(pitch_PID < -1000){pitch_PID=-1000;}
	     if(pitch_PID > 2000) {pitch_PID=2000;}


	     // Finalmente calculamos la velocidad global
	     pwm_del_izq  = input_THROTTLE - roll_PID - pitch_PID;
	     pwm_del_der  = input_THROTTLE - roll_PID + pitch_PID;
	     pwm_tras_izq  = input_THROTTLE + roll_PID + pitch_PID;
	     pwm_tras_der  = input_THROTTLE + roll_PID - pitch_PID;

	     if(pwm_del_der < MIN)
	     {
	       pwm_del_der = MIN;
	     }
	     if(pwm_del_der > MAX)
	     {
	       pwm_del_der = MAX;
	     }

	     //Left front
	     if(pwm_del_izq < MIN)
	     {
	       pwm_del_izq = MIN;
	     }
	     if(pwm_del_izq > MAX)
	     {
	       pwm_del_izq = MAX;
	     }

	     //Right back
	     if(pwm_tras_der < MIN)
	     {
	       pwm_tras_der = MIN;
	     }
	     if(pwm_tras_der > MAX)
	     {
	       pwm_tras_der = MAX;
	     }

	     //Left back
	     if(pwm_tras_izq < MIN)
	     {
	       pwm_tras_izq = MIN;
	     }
	     if(pwm_tras_izq > MAX)
	     {
	       pwm_tras_izq = MAX;
	     }

	     Serial.print(pwm_del_der);
	     Serial.print(" | | ");
	     Serial.print(pwm_del_izq);
	     Serial.print(" | | ");
	     Serial.print(pwm_tras_der);
	     Serial.print(" | | ");
	     Serial.println(pwm_tras_izq);

	     roll_previous_error = roll_error; //Remember to store the previous error.
	     pitch_previous_error = pitch_error; //Remember to store the previous error.

	     motor_del_izq.writeMicroseconds(pwm_del_izq);
	     motor_del_der.writeMicroseconds(pwm_del_der);
	     motor_tras_izq.writeMicroseconds(pwm_tras_izq);
	     motor_tras_der.writeMicroseconds(pwm_tras_der);


// /////////////////////////////I M U/////////////////////////////////////
// 	timePrev = time;
//   time = millis(); // Se lee otra vez el tiempo
//   elapsedTime = (time - timePrev) / 1000; // Se pasa a segundos
//
//    Wire.beginTransmission(0x68);
// 	 Wire.write(0x3B);
//    Wire.endTransmission(false);
//    Wire.requestFrom(0x68,6,true);
//
//    // Lectura de datos
//    Acc_rawX=Wire.read()<<8|Wire.read(); // Cada valor necesita dos regristros
//    Acc_rawY=Wire.read()<<8|Wire.read();
//    Acc_rawZ=Wire.read()<<8|Wire.read();
//
//    // -------------------Calculo de las ecuaciones de euler------------------
//
//    // Dividimos los valores obtenidos con el sensor entre 16384 para que nos de la acceleracion
//    // Calculamos los angulos en grados
//    // Aplicamos la formula de Euler atan() = arctangente
//    /*---X---*/
//    Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
//    /*---Y---*/
//    Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
//
//    Wire.beginTransmission(0x68);
//    Wire.write(0x43);
//    Wire.endTransmission(false);
//    Wire.requestFrom(0x68,4,true); // 4 Registros
//
//    Gyr_rawX=Wire.read()<<8|Wire.read();
//    Gyr_rawY=Wire.read()<<8|Wire.read();
//
//    // Dividimos los datos que recibimos entre 131 por que es el dato que nos da el sensor
//
//    /*---X---*/
//    Gyro_angle[0] = Gyr_rawX/131.0;
//    /*---Y---*/
//    Gyro_angle[1] = Gyr_rawY/131.0;
//
//    // Multiplicamos el angulo de giro por el tiempo
//    // Aplicamos el filtro, añadimos la aceleracion y lo multiplicamos por 0.98
//
//    /*---X axis angle---*/
//    Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
//    /*---Y axis angle---*/
//    Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
//
//    // Tenemos los angulos entre -100º y 100º
//    //Serial.println(Total_angle[1]);
//
//    /*///////////////////////////P I D///////////////////////////////////*/
//
//    // Calculamos el error
//    error = Total_angle[1] - desired_angle;
//
//    // La constante multiplicada por el error
//    pid_p = kp*error;
//
//    // La integral solo debe funcionar si nos acercamos al angulo 0, por eso solo lo usamos
//    // En un angulo entre -3 y 3.
//    if(-3 <error <3)
//    {
//      pid_i = pid_i+(ki*error);
//    }
//
//   // La derivada se encarga de la velocidad de corrección del error
//
//   pid_d = kd*((error - previous_error)/elapsedTime);
//
//   // El PID es la suma de las tres operaciones
//   PID = pid_p + pid_i + pid_d;
//
//    if(PID < -1000)
//    {
//      PID=-1000;
//  	 }
//    if(PID > 1000)
//    {
//      PID=1000;
//    }
//
//    // Sumamos el valor minimo + el PID
//    pwmLeft = throttle + PID;
//    pwmRight = throttle - PID;
// 	 pwmLeft2 = throttle - PID;
// 	 pwmRight2 = throttle + PID;
//
//    // Hacemos que no se pasen los datos
//    //Right
//    if(pwmRight < 1200)
//    {
//      pwmRight= 1200;
//    }
//    if(pwmRight > 1250)
//    {
//      pwmRight=1250;
//    }
// 	 if(pwmRight2 < 1200)
//    {
//      pwmRight2 = 1200;
//    }
//    if(pwmRight2 > 1250)
//    {
//      pwmRight2 =1250;
//    }
//    //Left
//    if(pwmLeft < 1200)
//    {
//      pwmLeft= 1200;
//    }
//    if(pwmLeft > 1250)
//    {
//      pwmLeft=1250;
//    }
// 	 if(pwmLeft2 < 1200)
//    {
//      pwmLeft2= 1200;
//    }
//    if(pwmLeft2 > 1250)
//    {
//      pwmLeft2=1250;
//    }
//    // Mandamos los datos al motor
//    motor_del_izq.writeMicroseconds(pwmLeft);
// 	 motor_tras_izq.writeMicroseconds(pwmLeft2);
//    Serial.print(pwmLeft);
//    Serial.print("||");
//    motor_del_der.writeMicroseconds(pwmRight);
// 	 motor_tras_der.writeMicroseconds(pwmRight2);
//    Serial.println(pwmRight);
//    previous_error = error; // Almacena el error previo

}

void calibracion()
{
	Serial.println(" No conectes aun la bateria");
	// Enviamo el maximo de velocidad
	motor_del_izq.write(180);
	motor_del_der.write(180);
	motor_tras_izq.write(180);
	motor_tras_der.write(180);
	delay(5000)
	Serial.println(" Conecta la bateria");
	// Envio del minimo de velocidad
	motor_del_izq.write(0);
	motor_del_der.write(0);
	motor_tras_izq.write(0);
	motor_tras_der.write(0);
	delay(4000)

}
