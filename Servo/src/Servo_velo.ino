// Script para la calibracion del ESC
#include <Servo.h>

#define MOTOR_IZQ 3
#define MOTOR_DER 5
#define MOTOR_IZQ_2 6
#define MOTOR_DER_2 7
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
  // Se envian valores minimos y máximos a cada ESC
  motor_izq.attach(MOTOR_IZQ, 1200, 2000);
  motor_der.attach(MOTOR_DER, 1200, 2000);
  motor_izq_2.attach(MOTOR_IZQ_2, 1200, 2000);
  motor_der_2.attach(MOTOR_DER_2, 1200, 2000);

}

// void loop()
// {
//
//   motor_izq.write(180);
//   motor_der.write(180);
//   motor_izq_2.write(180);
//   motor_der_2.write(180);
//
//   delay(5000);
//
//   motor_izq.write(0);
//   motor_der.write(0);
//   motor_izq_2.write(0);
//   motor_der_2.write(0);
//
//   delay(5000);
//
//   motor_izq.write(1500);
//   motor_der.write(1500);
//   motor_izq_2.write(1500);
//   motor_der_2.write(1500);

  // if (Serial.available())
  // {
  //     respuesta = Serial.read();
  //
  //     switch (respuesta)
  //     {
  //
  //         // 1, Ejecutar el primero, al escuchar un sonido pasar al siguiente
  //         case 49 : Serial.println("Velocidad maxima"); // El maximo
  //                   motor_izq.write(180);
  //                   motor_der.write(180);
  //                   motor_izq_2.write(180);
  //                   motor_der_2.write(180);
  //         break;
  //
  //         // 2, Ejecutar el segundo, al escuchar el sonido pasar al siguiente
  //         case 50 : Serial.println("Velocidad minima"); // El minimo
  //                   motor_izq.write(0);
  //                   motor_der.write(0);
  //                   motor_izq_2.write(0);
  //                   motor_der_2.write(0);
  //         break;
  //
  //         // 3
  //         case 51 : Serial.println("Velocidad media");
  //                   motor_izq.write(90);
  //                   motor_der.write(90);
  //                   motor_izq_2.write(90);
  //                   motor_der_2.write(90);
  //         break;
  //
  //         // 9
  //         case 57: Serial.println("Velocidad a 2000");
  //                   motor_izq.write(2000);
  //                   motor_der.write(2000);
  //                   motor_izq_2.write(2000);
  //                   motor_der_2.write(2000);
  //         break;
  //
  //         // 0
  //         case 48: Serial.println("Velocidad a 1200");
  //                  motor_izq.write(1200);
  //                  motor_der.write(1200);
  //                  motor_izq_2.write(1200);
  //                  motor_der_2.write(1200);
  //
  //         break;
  //     }
  //     int velocidad = 0;
  //     while(respuesta == 119) // Mientras esté la W pulsada
  //     {
  //       motor_izq.write(velocidad);
  //       motor_der.write(velocidad);
  //       motor_izq_2.write(velocidad);
  //       motor_der_2.write(velocidad);
  //       velocidad+= 1;
  //     }
  //     while(respuesta == 115) // Mientras esté la S pulsada
  //     {
  //       motor_izq.write(velocidad);
  //       motor_der.write(velocidad);
  //       motor_izq_2.write(velocidad);
  //       motor_der_2.write(velocidad);
  //       velocidad-= 1;
  //     }
  // }
//}


/**
 * Usage, according to documentation(https://www.firediy.fr/files/drone/HW-01-V4.pdf) :
 *     1. Plug your Arduino to your computer with USB cable, open terminal, then type 1 to send max throttle to every ESC to enter programming mode
 *     2. Power up your ESCs. You must hear "beep1 beep2 beep3" tones meaning the power supply is OK
 *     3. After 2sec, "beep beep" tone emits, meaning the throttle highest point has been correctly confirmed
 *     4. Type 0 to send 0 throttle
 *     5. Several "beep" tones emits, wich means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo)
 *     6. A long beep tone emits meaning the throttle lowest point has been correctly confirmed
 *     7. Type 2 to launch test function. This will send 0 to 180 throttle to ESCs to test them
 */

// #include <Servo.h>
//
// Servo motor_izq, motor_der, motor_izq_2, motor_der_2;
// char data;
//
// /**
//  * Initialisation routine
//  */
// void setup() {
//     Serial.begin(9600);
//
//     motor_izq.attach(3, 1200, 2000); // 1000 and 2000 are very important ! Values can be different with other ESCs.
//     motor_der.attach(4, 1200, 2000);
//     motor_izq_2.attach(5, 1200, 2000);
//     motor_der_2.attach(6, 1200, 2000);
//
//     displayInstructions();
// }
//
// /**
//  * Main function
//  */
void loop() {
    if (Serial.available()) {
       char data;
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : Serial.println("Sending 0 throttle");
                      motor_izq.write(0);
                      motor_der.write(0);
                      motor_izq_2.write(0);
                      motor_der_2.write(0);
            break;

            // 1
            case 49 : Serial.println("Sending 180 throttle");
                      motor_izq.write(180);
                      motor_der.write(180);
                      motor_izq_2.write(180);
                      motor_der_2.write(180);
            break;

            // 2
            case 50 : Serial.print("Running test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test();
            break;
        }
    }


}
//
// /**
//  * Test function sending angle to the ESCs from 0 to 180 degrees
//  */
void test()
{
    for (int i=1400; i<=1500; i++) {
        Serial.print("Speed = ");
        Serial.println(i);

        motor_izq.writeMicroseconds(i);
        motor_der.writeMicroseconds(i);
        motor_izq_2.writeMicroseconds(i);
        motor_der_2.writeMicroseconds(i);

        delay(200);
    }

    Serial.println("STOP");
    motor_izq.write(0);
    motor_der.write(0);
    motor_izq_2.write(0);
    motor_der_2.write(0);
}
//
// /**
//  * Displays instructions to user
//  */
// void displayInstructions()
// {
//     Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
//     Serial.println("\t0 : Sends 0 throttle");
//     Serial.println("\t1 : Sends 180 throttle");
//     Serial.println("\t2 : Runs test function\n");
// }
