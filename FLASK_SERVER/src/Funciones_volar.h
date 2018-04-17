//-------------ARCHIVO PARA TODO EL VUELO-----------

#include <Arduino.h>
#include "Configuracion.h" // Acceder a todos los parametros
#include <Wire.h>
#include <Servo.h>
#include <imumaths.h>

//Declaración de motores
Servo motor_del_izq;
Servo motor_del_der;
Servo motor_tras_izq;
Servo motor_tras_der;

//#include "main.ino"
// TODO: PID del Yaw

//-----------------PID ESPECIFICACIÓNES-----------
// El MPU6050 envia los datos en int16_t

// TODO: Pruebas:
#define MIN 1650
#define MAX 2000

int input_YAW;
int input_PITCH;
int input_ROLL;
int input_THROTTLE = 1400;

float elapsedTime, time, timePrev;
int gyro_error = 0;
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Gyro_angle_x, Gyro_angle_y;
float Gyro_raw_error_x, Gyro_raw_error_y;

// Variables del acelerometro
int acc_error = 0;
float rad_to_deg = 180/3.141592654;
float Acc_rawX, Acc_rawY, Acc_rawZ;
float Acc_angle_x, Acc_angle_y;
float Acc_angle_error_x, Acc_angle_error_y;

float Total_angle_x, Total_angle_y;

int i;
int mot_activated = 0;
long activate_count = 0;
long des_activate_count = 0;

//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_del_izq, pwm_del_der, pwm_tras_izq, pwm_tras_der, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp=3.90;//3.55
double roll_ki=0.007;//0.003
double roll_kd=2.5;//2.05
float roll_desired_angle = 0;

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp=3.90;//3.55
double pitch_ki=0.007;//0.003
double pitch_kd=2.5;//2.05
float pitch_desired_angle = 0;

//////////////////////////////PID FOR YAW//////////////////////////
float yaw_PID=0, yaw_error, yaw_previous_error;
float yaw_pid_p=0;
float yaw_pid_i=0;
float yaw_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double yaw_kp=3.90;//3.55
double yaw_ki=0.007;//0.003
double yaw_kd=2.5;//2.05
float yaw_desired_angle = 0;


void activarMotores()
{
  motor_del_izq.attach(MOTOR_DEL_IZQ, MOTOR_MAX_LEVEL, MOTOR_MIN_LEVEL);
  motor_del_der.attach(MOTOR_DEL_DER, MOTOR_MAX_LEVEL, MOTOR_MIN_LEVEL);
  motor_tras_izq.attach(MOTOR_TRAS_IZQ, MOTOR_MAX_LEVEL, MOTOR_MIN_LEVEL);
  motor_tras_der.attach(MOTOR_TRAS_DER, MOTOR_MIN_LEVEL, MOTOR_MAX_LEVEL);

}

void pid() // La velocidad de los motores son variables globales
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

	     if(pwm_del_izq < MIN)
	     {
	       pwm_del_izq= MIN;
	     }
	     if(pwm_del_izq > MAX)
	     {
	       pwm_del_izq=MAX;
	     }

	     //Left front
	     if(pwm_tras_der < MIN)
	     {
	       pwm_tras_der= MIN;
	     }
	     if(pwm_tras_der > MAX)
	     {
	       pwm_tras_der = MAX;
	     }

	     //Right back
	     if(pwm_del_der < MIN)
	     {
	       pwm_del_der= MIN;
	     }
	     if(pwm_del_der > MAX)
	     {
	       pwm_del_der=MAX;
	     }

	     //Left back
	     if(pwm_tras_izq < MIN)
	     {
	       pwm_tras_izq= MIN;
	     }
	     if(pwm_tras_izq > MAX)
	     {
	       pwm_tras_izq=MAX;
	     }

	     Serial.print(pwm_tras_izq);
	     Serial.print(" | | ");
	     Serial.print(pwm_tras_der);
	     Serial.print(" | | ");
	     Serial.print(pwm_del_der);
	     Serial.print(" | | ");
	     Serial.println(pwm_del_izq);

	     motor_del_izq.writeMicroseconds(pwm_tras_der);
	     motor_del_der.writeMicroseconds(pwm_tras_izq);
	     motor_tras_izq.writeMicroseconds(pwm_del_izq);
	     motor_tras_der.writeMicroseconds(pwm_del_der);

			 roll_previous_error = roll_error; //Remember to store the previous error.
			 pitch_previous_error = pitch_error; //Remember to store the previous error.

}

void calibracion()
{
	Serial.println(" No conectes aun la bateria");
	// Enviamo el maximo de velocidad
	motor_del_izq.write(180);
	motor_del_der.write(180);
	motor_tras_izq.write(180);
	motor_tras_der.write(180);
	delay(5000);
	Serial.println(" Conecta la bateria");
	// Envio del minimo de velocidad
	motor_del_izq.write(0);
	motor_del_der.write(0);
	motor_tras_izq.write(0);
	motor_tras_der.write(0);
	delay(4000);

}

void girarDerecha()
{
  pwm_del_izq  = input_THROTTLE - roll_PID;
  pwm_del_der  = input_THROTTLE + roll_PID;
  pwm_tras_izq  = input_THROTTLE - roll_PID;
  pwm_tras_der  = input_THROTTLE + roll_PID;
}

void girarIzquierda()
{
  pwm_del_izq  = input_THROTTLE + roll_PID;
  pwm_del_der  = input_THROTTLE - roll_PID;
  pwm_tras_izq  = input_THROTTLE + roll_PID;
  pwm_tras_der  = input_THROTTLE - roll_PID;
}

void rotarDerecha()
{
  pwm_del_izq  = input_THROTTLE + yaw_PID;
  pwm_del_der  = input_THROTTLE - yaw_PID;
  pwm_tras_izq  = input_THROTTLE - yaw_PID;
  pwm_tras_der  = input_THROTTLE + yaw_PID;
}

void rotarIzquierda()
{
  pwm_del_izq  = input_THROTTLE - yaw_PID;
  pwm_del_der  = input_THROTTLE + yaw_PID;
  pwm_tras_izq  = input_THROTTLE + yaw_PID;
  pwm_tras_der  = input_THROTTLE - yaw_PID;
}

void avanzar()
{
  pwm_del_izq  = input_THROTTLE - pitch_PID;
  pwm_del_der  = input_THROTTLE - pitch_PID;
  pwm_tras_izq  = input_THROTTLE + pitch_PID;
  pwm_tras_der  = input_THROTTLE + pitch_PID;

}

void retroceder()
{
  pwm_del_izq  = input_THROTTLE + pitch_PID;
  pwm_del_der  = input_THROTTLE + pitch_PID;
  pwm_tras_izq  = input_THROTTLE - pitch_PID;
  pwm_tras_der  = input_THROTTLE - pitch_PID;
}
