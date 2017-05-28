#include <Arduino.h>

// #include <Servo.h>

// Servo esc;

// int ACELERAMAX;
// int ACELERAMED;
// int ACELERAMIN;
// void setup(){
//   pinMode (2,INPUT);
//   pinMode (3,INPUT);
//   pinMode (4,INPUT);
//   esc.attach (9);
//   // esc.writeMicroseconds(0);
//   Serial.begin (9600);
// }
// void loop() {
//   ACELERAMIN = digitalRead(2);
//   ACELERAMED = digitalRead(3);
//   ACELERAMAX = digitalRead(4);

//   if(ACELERAMAX == 0) {
//     esc.writeMicroseconds(1990);
//     Serial.println ("Max");
//   }

//   else if(ACELERAMED == 0) {
//    esc.writeMicroseconds(1470);
//     Serial.println ("Med");
//   }

//   else if(ACELERAMIN == 0) {
//    esc.writeMicroseconds(1130);
//    Serial.println ("Min");
//   }

// }