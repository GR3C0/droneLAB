#include <Wire.h>
#include <Servo.h>


Servo right_prop;
Servo left_prop;
Servo right_2_prop;
Servo left_2_prop;


// El MPU6050 envia los datos es int16_t

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;


float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];


float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmLeft2, pwmRight2, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=3.0;//3.55
double ki=0.003;//0.003
double kd=2.0;//2.05
///////////////////////////////////////////////

double throttle=1200; // Valor inicial de arranque de los motores
float desired_angle = 0;


void setup() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);
  right_prop.attach(3); // Arratch el pin del motor derecho
  left_prop.attach(5);  // Attatch el pin del motor izquierdo

  time = millis(); // Medición del tiempo
  // Se envia el valor minimo al ESC para que se calibre
  left_prop.writeMicroseconds(1000);
  right_prop.writeMicroseconds(1000);
}

void loop() {

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
     //Left
     if(pwmLeft < 1200)
     {
       pwmLeft= 1200;
     }
     if(pwmLeft > 1250)
     {
       pwmLeft=1250;
     }
     // Mandamos los datos al motor
     left_prop.writeMicroseconds(pwmLeft);
     Serial.print(pwmLeft);
     Serial.print("||");
     right_prop.writeMicroseconds(pwmRight);
     Serial.println(pwmRight);
     previous_error = error; //Remember to store the previous error.

}//end of loop void
