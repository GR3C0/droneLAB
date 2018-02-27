#include <Wire.h>
#include <Servo.h>
#define MIN 1400
#define MAX 2000

Servo L_F_prop;
Servo L_B_prop;
Servo R_F_prop;
Servo R_B_prop;

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
float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
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


void setup() {

  time = millis(); // Medición del tiempo
  // Se envia el valor minimo al ESC para que se calibre
  L_F_prop.attach(3, 1300, 2000);
  L_B_prop.attach(5, 1300, 2000);
  R_F_prop.attach(6, 1300, 2000);
  R_B_prop.attach(7, 1300, 2000);

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
}// Fin del setup


void loop() {

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
     pwm_R_F  = input_THROTTLE - roll_PID - pitch_PID;
     pwm_R_B  = input_THROTTLE - roll_PID + pitch_PID;
     pwm_L_B  = input_THROTTLE + roll_PID + pitch_PID;
     pwm_L_F  = input_THROTTLE + roll_PID - pitch_PID;

     if(pwm_R_F < MIN)
     {
       pwm_R_F= MIN;
     }
     if(pwm_R_F > MAX)
     {
       pwm_R_F=MAX;
     }

     //Left front
     if(pwm_L_F < MIN)
     {
       pwm_L_F= MIN;
     }
     if(pwm_L_F > MAX)
     {
       pwm_L_F = MAX;
     }

     //Right back
     if(pwm_R_B < MIN)
     {
       pwm_R_B= MIN;
     }
     if(pwm_R_B > MAX)
     {
       pwm_R_B=MAX;
     }

     //Left back
     if(pwm_L_B < MIN)
     {
       pwm_L_B= MIN;
     }
     if(pwm_L_B > MAX)
     {
       pwm_L_B=MAX;
     }

     Serial.print(pwm_L_B);
     Serial.print(" | | ");
     Serial.print(pwm_L_F);
     Serial.print(" | | ");
     Serial.print(pwm_R_B);
     Serial.print(" | | ");
     Serial.println(pwm_R_F);

     roll_previous_error = roll_error; //Remember to store the previous error.
     pitch_previous_error = pitch_error; //Remember to store the previous error.

     L_F_prop.writeMicroseconds(pwm_L_F);
     L_B_prop.writeMicroseconds(pwm_L_B);
     R_F_prop.writeMicroseconds(pwm_R_F);
     R_B_prop.writeMicroseconds(pwm_R_B);

     // if(respuesta == "giro derecha")
     // {
     //   del_izq.writeMicroseconds(throttle - pwmDel_izq); //MP1
     //   del_der.writeMicroseconds(throttle + pwmDel_der); //MP2
     //   tras_izq.writeMicroseconds(throttle - pwmTras_der); //MP3
     //   tras_der.writeMicroseconds(throttle + pwmTras_izq); //MP4
     // }
     // if(respuesta == "giro izquierda")
     // {
     //   del_izq.writeMicroseconds(throttle + pwmDel_izq); //MP1
     //   del_der.writeMicroseconds(throttle - pwmDel_der); //MP2
     //   tras_izq.writeMicroseconds(throttle + pwmTras_der); //MP3
     //   tras_der.writeMicroseconds(throttle - pwmTras_izq); //MP4
     // }
     // if(respuesta == "rotar derecha")
     // {
     //   del_izq.writeMicroseconds(throttle - pwmDel_izq); //MP1
     //   del_der.writeMicroseconds(throttle - pwmDel_der); //MP2
     //   tras_izq.writeMicroseconds(throttle + pwmTras_der); //MP3
     //   tras_der.writeMicroseconds(throttle + pwmTras_izq); //MP4
     // }
     // if(respuesta == "rotar izquierda")
     // {
     //   del_izq.writeMicroseconds(throttle + pwmDel_izq); //MP1
     //   del_der.writeMicroseconds(throttle + pwmDel_der); //MP2
     //   tras_izq.writeMicroseconds(throttle - pwmTras_der); //MP3
     //   tras_der.writeMicroseconds(throttle - pwmTras_izq); //MP4
     // }
     // if(respuesta == "avanzar")
     // {
     //   del_izq.writeMicroseconds(throttle - pwmDel_izq); //MP1
     //   del_der.writeMicroseconds(throttle + pwmDel_der); //MP2
     //   tras_izq.writeMicroseconds(throttle + pwmTras_der); //MP3
     //   tras_der.writeMicroseconds(throttle - pwmTras_izq); //MP4
     // }
     // if(respuesta == "retroceder")
     // {
     //   del_izq.writeMicroseconds(throttle + pwmDel_izq); //MP1
     //   del_der.writeMicroseconds(throttle - pwmDel_der); //MP2
     //   tras_izq.writeMicroseconds(throttle - pwmTras_der); //MP3
     //   tras_der.writeMicroseconds(throttle + pwmTras_izq); //MP4
     // }

}
