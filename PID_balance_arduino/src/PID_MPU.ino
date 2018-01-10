#include <Wire.h>
#include <Servo.h>


Servo del_der;
Servo del_izq;
Servo tras_der;
Servo tras_izq;


// El MPU6050 envia los datos es int16_t

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;


float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];


float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmDel_der, pwmDel_izq, pwmTras_der, pwmTras_izq, error, previous_error;
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
  del_der.attach(3); // Arratch el pin del motor derecho
  tras_izq.attach(4);  // Attatch el pin del motor izquierdo
  del_izq.attach(5);
  tras_der.attach(6);

  time = millis(); // Medición del tiempo
  // Se envia el valor minimo al ESC para que se calibre
  del_der.writeMicroseconds(1000);
  del_izq.writeMicroseconds(1000);
  tras_der.writeMicroseconds(1000);
  tras_izq.writeMicroseconds(1000);
}

void loop() {

    while (!Serial.available()); // Comprueba si el puerto para leer datos está abierto
    char respuesta;
    respuesta = Serial.read(); // Lectura de la respuesta del usuario

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
     if(-3 < error and error <3)
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
     // pwmDel_der = throttle + PID;
     // pwmDel_izq = throttle - PID;
     // pwmTras_der = throttle + PID;
     // pwmTras_izq = throttle - PID;


     // Hacemos que no se pasen los datos
     //Delantero Derecho
     if(pwmDel_der < 1200)
     {
       pwmDel_der = 1200;
     }
     if(pwmDel_der > 2000)
     {
       pwmDel_der = 2000;
     }
     //Delantero izquierdo
     if(pwmDel_izq < 1200)
     {
       pwmDel_izq = 2000;
     }
     if(pwmDel_izq > 1200)
     {
       pwmDel_izq = 2000;
     }
     //Trasero Derecho
     if(pwmTras_der < 1200)
     {
       pwmTras_der = 2000;
     }
     if(pwmTras_der > 1200)
     {
       pwmTras_der = 2000;
     }
     //Trasero izquierdo
     if(pwmTras_izq < 1200)
     {
       pwmTras_izq = 2000;
     }
     if(pwmTras_izq > 1200)
     {
       pwmTras_izq = 2000;
     }
     // Mandamos los datos al motor
     tras_izq.writeMicroseconds(pwmTras_izq);
     Serial.print(pwmTras_izq);
     Serial.print("||");

     tras_der.writeMicroseconds(pwmTras_der);
     Serial.println(pwmTras_der);
     Serial.print("||");

     del_izq.writeMicroseconds(pwmDel_izq);
     Serial.print(pwmDel_izq);
     Serial.print("||");

     del_der.writeMicroseconds(pwmDel_der);
     Serial.print(pwmDel_der);
     Serial.print("||");

     previous_error = error; //Remember to store the previous error.

     if(respuesta == "giro derecha")
     {
       del_izq.writeMicroseconds(throttle - pwmDel_izq); //MP1
       del_der.writeMicroseconds(throttle + pwmDel_der); //MP2
       tras_izq.writeMicroseconds(throttle - pwmTras_der); //MP3
       tras_der.writeMicroseconds(throttle + pwmTras_izq); //MP4
     }
     if(respuesta == "giro izquierda")
     {
       del_izq.writeMicroseconds(throttle + pwmDel_izq); //MP1
       del_der.writeMicroseconds(throttle - pwmDel_der); //MP2
       tras_izq.writeMicroseconds(throttle + pwmTras_der); //MP3
       tras_der.writeMicroseconds(throttle - pwmTras_izq); //MP4
     }
     if(respuesta == "rotar derecha")
     {
       del_izq.writeMicroseconds(throttle - pwmDel_izq); //MP1
       del_der.writeMicroseconds(throttle - pwmDel_der); //MP2
       tras_izq.writeMicroseconds(throttle + pwmTras_der); //MP3
       tras_der.writeMicroseconds(throttle + pwmTras_izq); //MP4
     }
     if(respuesta == "rotar izquierda")
     {
       del_izq.writeMicroseconds(throttle + pwmDel_izq); //MP1
       del_der.writeMicroseconds(throttle + pwmDel_der); //MP2
       tras_izq.writeMicroseconds(throttle - pwmTras_der); //MP3
       tras_der.writeMicroseconds(throttle - pwmTras_izq); //MP4
     }
     if(respuesta == "avanzar")
     {
       del_izq.writeMicroseconds(throttle - pwmDel_izq); //MP1
       del_der.writeMicroseconds(throttle + pwmDel_der); //MP2
       tras_izq.writeMicroseconds(throttle + pwmTras_der); //MP3
       tras_der.writeMicroseconds(throttle - pwmTras_izq); //MP4
     }
     if(respuesta == "retroceder")
     {
       del_izq.writeMicroseconds(throttle + pwmDel_izq); //MP1
       del_der.writeMicroseconds(throttle - pwmDel_der); //MP2
       tras_izq.writeMicroseconds(throttle - pwmTras_der); //MP3
       tras_der.writeMicroseconds(throttle + pwmTras_izq); //MP4
     }

}//end of loop void
