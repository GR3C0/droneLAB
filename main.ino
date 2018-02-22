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
Servo tras;
Servo del;
Servo der;
Servo izq;

void setup()
{
	time = millis(); // Medición del tiempo
  // Se envia el valor minimo al ESC para que se calibre
  tras.attach(TRAS, MIN, MAX);
  del.attach(DEL, MIN, MAX);
  der.attach(IZQ, MIN, MAX);
  izq.attach(DER, MIN, MAX);

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
}

void loop()
{
	if(Serial.available()) // Si está disponible la lectura de la raspi
	{
		int comando = Serial.read();// Leemos lo enviado por el usuario
		// Menu para elegir la operación
		}
  }
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
	// while (!Serial.available()); // Comprueba si el puerto para leer datos está abierto

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


	 // Calculamos la velocidad global
	 pwm_tras  = input_THROTTLE - roll_PID - pitch_PID;
	 pwm_der  = input_THROTTLE - roll_PID + pitch_PID;
	 pwm_izq  = input_THROTTLE + roll_PID + pitch_PID;
	 pwm_del  = input_THROTTLE + roll_PID - pitch_PID;

	 if(pwm_tras < MIN)
	 {
		 pwm_tras= MIN;
	 }
	 if(pwm_tras > MAX)
	 {
		 pwm_tras=MAX;
	 }

	 //Left front
	 if(pwm_der < MIN)
	 {
		 pwm_der= MIN;
	 }
	 if(pwm_der > MAX)
	 {
		 pwm_der = MAX;
	 }

	 //Right back
	 if(pwm_del < MIN)
	 {
		 pwm_del= MIN;
	 }
	 if(pwm_del > MAX)
	 {
		 pwm_del=MAX;
	 }

	 //Left back
	 if(pwm_izq < MIN)
	 {
		 pwm_izq= MIN;
	 }
	 if(pwm_izq > MAX)
	 {
		 pwm_izq=MAX;
	 }

	 Serial.print(pwm_izq);
	 Serial.print(" | | ");
	 Serial.print(pwm_der);
	 Serial.print(" | | ");
	 Serial.print(pwm_del);
	 Serial.print(" | | ");
	 Serial.println(pwm_tras);

	 roll_previous_error = roll_error; //Remember to store the previous error.
	 pitch_previous_error = pitch_error; //Remember to store the previous error.

	 tras.writeMicroseconds(pwm_der);
	 del.writeMicroseconds(pwm_izq);
	 der.writeMicroseconds(pwm_tras);
	 izq.writeMicroseconds(pwm_del);
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
