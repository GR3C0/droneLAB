
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Servo.h"

Servo motor_del_izq;
Servo motor_del_der;
Servo motor_tras_izq;
Servo motor_tras_der;

int input_YAW;
int input_PITCH;
int input_ROLL;
int input_THROTTLE = 1500;
int MIN = 1550;
int MAX = 1650;

float elapsedTime, time, timePrev;

//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_tras_der, pwm_tras_izq, pwm_del_izq, pwm_del_der, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp=2.90;//3.55
double roll_ki=0.003;//0.003
double roll_kd=2.05;//2.05
float roll_desired_angle = 0;

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp=2.90;//3.55
double pitch_ki=0.003;//0.003
double pitch_kd=2.05;//2.05
float pitch_desired_angle = 0;

const int mpuAddress = 0x68;
MPU6050 mpu(mpuAddress);

int ax, ay, az;
int gx, gy, gz;

long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;
int angulo_normal = 0;

void updateFiltered()
{
   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();

   //Calcular los ángulos con acelerometro
   float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2)))*(180.0 / 3.14);
   float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0 / 3.14);

   //Calcular angulo de rotación con giroscopio y filtro complementario
   ang_x = 0.98*(ang_x_prev + (gx / 131)*dt) + 0.02*accel_ang_x;
   ang_y = 0.98*(ang_y_prev + (gy / 131)*dt) + 0.02*accel_ang_y;

   ang_x_prev = ang_x;
   ang_y_prev = ang_y;
}

void setup()
{
   Serial.begin(9600);
   Wire.begin();
   mpu.initialize();
   Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
   motor_del_izq.attach(5, 1450, 2000); // Motor 2
   motor_del_der.attach(7, 1450, 2000); // Motor 1
   motor_tras_izq.attach(3, 1450, 2000); // Motor 3
   motor_tras_der.attach(6, 1450, 2000); // Motor 4
}

void loop()
{
   timePrev = time;
   time = millis(); // Se lee otra vez el tiempo
   elapsedTime = (time - timePrev) / 1000; // Se para a segundos

   // Leer las aceleraciones y velocidades angulares
   mpu.getAcceleration(&ax, &ay, &az);
   mpu.getRotation(&gx, &gy, &gz);

   updateFiltered();

   //------------PID---------
   roll_error = ang_x - angulo_normal;
   pitch_error = ang_y - angulo_normal;

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
   pwm_tras_der  = input_THROTTLE + roll_PID + pitch_PID;
   pwm_tras_izq  = input_THROTTLE + roll_PID - pitch_PID;

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

   Serial.print(F("Rotacion en X:  "));
   //Serial.print(ang_x);
   Serial.print(pwm_del_der);
   Serial.print("  ");
   Serial.print(roll_PID);
   Serial.print(F("\t Rotacion en Y: "));
   //Serial.println(ang_y);
   Serial.print(pwm_tras_der);
   Serial.print("  ");
   Serial.print(pitch_PID);
   Serial.println();

   motor_del_izq.writeMicroseconds(pwm_del_izq);
   motor_del_der.writeMicroseconds(pwm_del_der);
   motor_tras_izq.writeMicroseconds(pwm_tras_izq);
   motor_tras_der.writeMicroseconds(pwm_tras_der);

   roll_previous_error = roll_error; //Remember to store the previous error.
   pitch_previous_error = pitch_error; //Remember to store the previous error.

   delay(10);
}
