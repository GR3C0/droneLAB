// #include <Wire.h>
// #include <Servo.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>
//
// #define MOTOR_IZQ 4
// #define MOTOR_DER 3
//
// // Declaración de motores
// Servo motor_der;
// Servo motor_izq;
//
// Adafruit_BNO055 bno = Adafruit_BNO055(55); // Iniciar el sensor
//
// float Angulo_aceleracion[2];
// float Angulo_giro[2];
// float Angulo_total[2];
// float giro_z, giro_y, acel_z, acel_y;
//
// float T_transcurrido, time, timePrev;
// int i;
//
// float PID, pwmIzq, pwmDer, error, error_previo;
// float pid_p=0;
// float pid_i=0;
// float pid_d=0;
// /////////////////PID CONSTANTS/////////////////
// double kp=3.55;//3.55 Amplia o reduce el error
// double ki=0.003;//0.003 Acumula el error
// double kd=2.05;//2.05 Evalua el incremento del error
// ///////////////////////////////////////////////
//
// double velocidad=1300; // Velocidad inicial de los motores
// float angulo_deseado = 0; // Angulo que queremos
//
// void setup()
// {
//   Serial.begin(9600);
//   if(!bno.begin()) //Prueba de conexión del Adafruit_Sensor
//   {
//     Serial.print("No detectado");
//     while(1);
//   }
//   bno.setExtCrystalUse(true);
//   motor_der.attach(MOTOR_DER); //attatch the right motor to pin 3
//   motor_izq.attach(MOTOR_IZQ);  //attatch the left motor to pin 5
//
//   time = millis(); // Empieza a contar el tiempo
//   motor_izq.writeMicroseconds(1000); // Enviamos el valor minimo a los motores para que se enciendan
//   motor_der.writeMicroseconds(1000);
//   delay(1000); // Le damos 7 segundos para empezar
//
// }
//
// void loop()
// {
//   /////////////////////////////I M U/////////////////////////////////////
//   timePrev = time;  // Almacenamos otra vez el tiempo
//   time = millis();  // Lectura del tiempo actual
//   T_transcurrido = (time - timePrev) / 1000; // Se calcula el tiempo pasado en segundos
//
//   // Lectura de la aceleracion y los angulos
//   imu::Vector<3> acelerometro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//   imu::Vector<3> giroscopio = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//
//
//   // Obtenemos los grados
//
//   Serial.print(euler.x());
//   Serial.print("||");
//   Serial.print(euler.y());
//   Serial.print("||");
//   Serial.println(euler.z());
//
//   // Datos del giroscopio pasados a grados/s
//
//   // giro_z = giroscopio.z()*57.2958;
//   // giro_y = giroscopio.y()*57.2958;
//   giro_z = euler.z();
//   giro_y = euler.y();
//
//   acel_z = acelerometro.z();
//   acel_y = acelerometro.y();
//
//   Angulo_aceleracion[0] = acel_z;
//   Angulo_aceleracion[1] = acel_y;
//   Angulo_giro[0] = giro_z;
//   Angulo_giro[1] = giro_y;
//
//   /*---Ángulo Z---*/
//   // Angulo_total[0] = 0.98 *(Angulo_total[0] + Angulo_giro[0]*T_transcurrido) + 0.02*Angulo_aceleracion[0];
//   /*---Ángulo Y---*/
//   // Angulo_total[1] = 0.98 *(Angulo_total[1] + Angulo_giro[1]*T_transcurrido) + 0.02*Angulo_aceleracion[1];
//
//   // Almacenamos el error
//   // error = Angulo_total[0]-angulo_deseado;
//   error = Angulo_giro[0] - angulo_deseado;
//
//   // Esta es la constante que solo hace falta multiplicarla por el error
//   pid_p = kp*error;
//
//   // La integral solo debe actuar si el error es mínimo
//   if(-3 < error || error < 3)
//   {
//     pid_i = pid_i+(ki*error);
//   }
//
//   // La derivada
//   pid_d = kd*((error - error_previo)/T_transcurrido);
//
//   // Almacenamos todo el PID
//   PID = pid_p + pid_i + pid_d;
//
//   // Como la velocidad de los motores solo puede ir entre 1000 y 2000 hacemos unos ajustes para que no se vuelva loco
//   if(PID < -1000)
//   {
//     PID=-1000;
//   }
//   if(PID > 1000)
//   {
//     PID=1000;
//   }
//
//   // Finalmente le decimos a los motores que hacer
//   pwmIzq = velocidad + PID;
//   pwmDer = velocidad - PID;
//
//   // Reajustamos la velocidad por si acaso
//   if(pwmDer < 1200)
//   {
//     pwmDer= 1200;
//   }
//   if(pwmDer > 2000)
//   {
//     pwmDer=2000;
//   }
//
//   if(pwmIzq < 1200)
//   {
//     pwmIzq= 1200;
//   }
//   if(pwmIzq > 2000)
//   {
//     pwmIzq=2000;
//   }
//
//   // Le enviamos los datos a los motores
//   motor_der.writeMicroseconds(pwmIzq);
//   Serial.print(pwmIzq);
//   Serial.print("||");
//   motor_izq.writeMicroseconds(pwmDer);
//   Serial.println(pwmDer);
//   error_previo = error; // Almacenamos el error
//
//   motor_der.writeMicroseconds(1700);
//   motor_izq.writeMicroseconds(1700);
//  }
