#include <Wire.h>
#include <NewPing.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define MOTOR_IZQ 3
#define MOTOR_DER 4
#define MOTOR_IZQ_2 5
#define MOTOR_DER_2 6
// Valores velocidad:
#define zero 2000
#define uno 1100
#define dos 1200
#define tres 1300
#define cuatro 1400
#define cinco 1500
#define seis 1600
#define siete 1700
#define ocho 1800

#define nueve 1900
#define esp 1000

Servo motor_izq;
Servo motor_der;
Servo motor_izq_2;
Servo motor_der_2;

int respuesta;

void setup()
{
  Serial.begin(9600);
  Serial.println("Conectando motores");

  motor_der.attach(MOTOR_DER); //attatch the right motor to pin 3
  motor_izq.attach(MOTOR_IZQ);  //attatch the left motor to pin 5
  motor_izq_2.attach(MOTOR_IZQ_2);
  motor_der_2.attach(MOTOR_DER_2);

  motor_izq.writeMicroseconds(2000); // Enviamos el valor minimo a los motores para que se enciendan
  motor_der.writeMicroseconds(2000);
  motor_izq_2.writeMicroseconds(2000);
  motor_der_2.writeMicroseconds(2000);


  motor_izq.writeMicroseconds(1300); // Enviamos el valor minimo a los motores para que se enciendan
  motor_der.writeMicroseconds(1300);
  motor_izq_2.writeMicroseconds(1300);
  motor_der_2.writeMicroseconds(1300);

  delay(1000); // Le damos 7 segundos para empezar

  // motor_izq.writeMicroseconds(2000);
  // motor_der.writeMicroseconds(2000);

}

void loop()
{

  while (!Serial.available()); // Comprueba si el puerto para leer datos está abierto
    respuesta = Serial.read(); // Lectura de la respuesta del usuario

  motor_izq.write(1200);
  motor_der.write(1200);
  motor_izq_2.write(1200);
  motor_der_2.write(1200);

  if (respuesta == 48) // Valor en ASCII del número
  {
    motor_izq.writeMicroseconds(zero);
    motor_der.writeMicroseconds(zero);
    motor_izq_2.writeMicroseconds(zero);
    motor_der_2.writeMicroseconds(zero);
    Serial.print("velocidad actual:");
    Serial.println(zero);
  }
  else if (respuesta == 49)
  {
    motor_izq.writeMicroseconds(uno);
    motor_der.writeMicroseconds(uno);
    motor_izq_2.writeMicroseconds(uno);
    motor_der_2.writeMicroseconds(uno);
    Serial.print("velocidad actual:");
    Serial.println(uno);
  }
  else if (respuesta == 50)
  {
    motor_izq.writeMicroseconds(dos);
    motor_der.writeMicroseconds(dos);
    motor_izq_2.writeMicroseconds(dos);
    motor_der_2.writeMicroseconds(dos);
    Serial.print("velocidad actual:");
    Serial.println(dos);
  }
  else if (respuesta == 51)
  {
    motor_izq.writeMicroseconds(tres);
    motor_der.writeMicroseconds(tres);
    motor_izq_2.writeMicroseconds(tres);
    motor_der_2.writeMicroseconds(tres);
    Serial.print("velocidad actual:");
    Serial.println(tres);
  }
  else if (respuesta == 52)
  {
    motor_izq.writeMicroseconds(cuatro);
    motor_der.writeMicroseconds(cuatro);
    motor_izq_2.writeMicroseconds(cuatro);
    motor_der_2.writeMicroseconds(cuatro);
    Serial.print("velocidad actual:");
    Serial.println(cuatro);
  }
  else if (respuesta == 53)
  {
    motor_izq.writeMicroseconds(cinco);
    motor_der.writeMicroseconds(cinco);
    motor_izq_2.writeMicroseconds(cinco);
    motor_der_2.writeMicroseconds(cinco);
    Serial.print("velocidad actual:");
    Serial.println(cinco);
  }
  else if (respuesta == 54)
  {
    motor_izq.writeMicroseconds(seis);
    motor_der.writeMicroseconds(seis);
    motor_izq_2.writeMicroseconds(seis);
    motor_der_2.writeMicroseconds(seis);
    Serial.print("velocidad actual:");
    Serial.println(seis);
  }
  else if (respuesta == 55)
  {
    motor_izq.writeMicroseconds(siete);
    motor_der.writeMicroseconds(siete);
    motor_izq_2.writeMicroseconds(siete);
    motor_der_2.writeMicroseconds(siete);
    Serial.print("velocidad actual:");
    Serial.println(siete);
  }
  else if (respuesta == 56)
  {
    motor_izq.writeMicroseconds(ocho);
    motor_der.writeMicroseconds(ocho);
    motor_izq_2.writeMicroseconds(ocho);
    motor_der_2.writeMicroseconds(ocho);
    Serial.print("velocidad actual:");
    Serial.println(ocho);
  }
  else if (respuesta == 57)
  {
    motor_izq.writeMicroseconds(nueve);
    motor_der.writeMicroseconds(nueve);
    motor_izq_2.writeMicroseconds(nueve);
    motor_der_2.writeMicroseconds(nueve);
    Serial.print("velocidad actual:");
    Serial.println(nueve);
  }
  else if (respuesta == 32)
  {
    motor_izq.writeMicroseconds(esp);
    motor_der.writeMicroseconds(esp);
    motor_izq_2.writeMicroseconds(esp);
    motor_der_2.writeMicroseconds(esp);
    Serial.print("velocidad actual:");
    Serial.println(esp);
  }
}

// /*ESC calibration sketch; author: ELECTRONOOBS */
// #include <Servo.h>
//
// #define MAX_SIGNAL 2000
// #define MIN_SIGNAL 1000
// #define MOTOR_PIN 9
// int DELAY = 1000;
//
// Servo motor;
//
// void setup() {
//   Serial.begin(9600);
//   Serial.println("Program begin...");
//   delay(1000);
//   Serial.println("This program will start the ESC.");
//
//   motor.attach(MOTOR_PIN);
//
//   Serial.print("Now writing maximum output: (");Serial.print(MAX_SIGNAL);Serial.print(" us in this case)");Serial.print("\n");
//   Serial.println("Turn on power source, then wait 2 seconds and press any key.");
//   motor.writeMicroseconds(MAX_SIGNAL);
//
//   // Wait for input
//   while (!Serial.available());
//   Serial.read();
//
//   // Send min output
//   Serial.println("\n");
//   Serial.println("\n");
//   Serial.print("Sending minimum output: (");Serial.print(MIN_SIGNAL);Serial.print(" us in this case)");Serial.print("\n");
//   motor.writeMicroseconds(MIN_SIGNAL);
//   Serial.println("The ESC is calibrated");
//   Serial.println("----");
//   Serial.println("Now, type a values between 1000 and 2000 and press enter");
//   Serial.println("and the motor will start rotating.");
//   Serial.println("Send 1000 to stop the motor and 2000 for full throttle");
//
// }
//
// void loop() {
//
//   if (Serial.available() > 0)
//   {
//     int DELAY = Serial.parseInt();
//     if (DELAY > 999)
//     {
//
//       motor.writeMicroseconds(DELAY);
//       float SPEED = (DELAY-1000)/10;
//       Serial.print("\n");
//       Serial.println("Motor speed:"); Serial.print("  "); Serial.print(SPEED); Serial.print("%");
//     }
//   }
// }
