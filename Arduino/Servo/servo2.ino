
#include <Servo.h>

Servo myservo;         //Declaramos un objeto de tipo servo
int velocidad = 0;     //Declaramos y inicializamos la variable que controlará la velocidad

void setup()
{
 Serial.begin(9600);                                              //Utilizo el monitor serie para comprovar los valores que está recibiendo el motor (ésta velocidad es por el tema del mando).
 myservo.writeMicroseconds(2000);
 myservo.attach(9);
 arm();                                                             // Importantísima funcion: Es la funcion con la que conseguimos el armado (Y pitido) del ESC (Está al final del programa).
}
void loop()
{
 {
     velocidad = map(0, 118 , 0, 40, 90);         //Con ésto yo controlo el motor: hago un mapeado que consta de 118-(valor con el que empiezo) 0-(Valor de gas al maximo) 40- (VALOR MINIMO QUE NECESITA EL MOTOR PARA MOVERSE) 90-(Valor maximo que le dejo girar al motor.)
     myservo.write(velocidad);                                              //En el mapeado anterior meto cada valor a la variable velocidad constantemente y ahora se la paso al ESC.
     delay(15);                                                             // delay necesario
 }
}
void arm(){                                                                   // Funcion del armado del ESC
 myservo.write(0);
 delay(100);
 myservo.write(20);
 delay(100);
}