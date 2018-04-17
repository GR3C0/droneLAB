// // Script para la calibracion del ESC
// #include <Servo.h>
//
// // Valores velocidad:
// #define zero 2000
// #define uno 1100
// #define dos 1200
// #define tres 1300
// #define cuatro 1400
// #define cinco 1500
// #define seis 1600
// #define siete 1700
// #define ocho 1800
// #define nueve 1900
// #define esp 1000
//
// Servo motor_del_izq;
// Servo motor_del_der;
// Servo motor_tras_izq;
// Servo motor_tras_der;
//
// int respuesta;
//
// void setup()
// {
//   Serial.begin(9600);
//   Serial.println("Conectando motores");
//   // Se envian valores mínimos y máximos a cada ESC
//   motor_del_izq.attach(3, 1600, 2000);
//   motor_del_der.attach(5, 1600, 2000);
//   motor_tras_izq.attach(6, 1600, 2000);
//   motor_tras_der.attach(7, 1600, 2000);
//
// }
//
// void loop() {
//     if (Serial.available()) {
//        char data;
//         data = Serial.read();
//
//         switch (data) {
//             // 0
//             case 48 : Serial.println("Sending 0 throttle");
//                       motor_del_izq.write(0);
//                       motor_del_der.write(0);
//                       motor_tras_izq.write(0);
//                       motor_tras_der.write(0);
//             break;
//
//             // 1
//             case 49 : Serial.println("Sending 180 throttle");
//                       motor_del_izq.write(180);
//                       motor_del_der.write(180);
//                       motor_tras_izq.write(180);
//                       motor_tras_der.write(180);
//                       delay(2000);
//                       motor_del_izq.write(90);
//                       motor_del_der.write(90);
//                       motor_tras_izq.write(90);
//                       motor_tras_der.write(90);
//             break;
//
//             // 2
//             case 50 : Serial.print("Running test in 3");
//                       delay(1000);
//                       Serial.print(" 2");
//                       delay(1000);
//                       Serial.println(" 1...");
//                       delay(1000);
//                       test();
//             break;
//         }
//     }
//
//
// }
// //
// // /**
// //  * Test function sending angle to the ESCs from 0 to 180 degrees
// //  */
// void test()
// {
//     for (int i=1600; i<=1700; i++) {
//         Serial.print("Speed = ");
//         Serial.println(i);
//
//         motor_del_izq.writeMicroseconds(i);
//         motor_del_der.writeMicroseconds(i);
//         motor_tras_izq.writeMicroseconds(i);
//         motor_tras_der.writeMicroseconds(i);
//
//         delay(100);
//     }
//
//     Serial.println("STOP");
//     motor_del_izq.write(0);
//     motor_del_der.write(0);
//     motor_tras_izq.write(0);
//     motor_tras_der.write(0);
// }
// //
// // /**
// //  * Displays instructions to user
// //  */
// // void displayInstructions()
// // {
// //     Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
// //     Serial.println("\t0 : Sends 0 throttle");
// //     Serial.println("\t1 : Sends 180 throttle");
// //     Serial.println("\t2 : Runs test function\n");
// // }
