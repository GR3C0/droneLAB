#include <Arduino.h>

#include <NewPing.h>
 
/*Aqui se configuran los pines donde debemos conectar el sensor*/
#define LEDROJO        23
#define LEDVERDE       13
#define LEDAMARILLO    17
#define TRIGGER_PIN    4
#define ECHO_PIN       3 
#define MAX_DISTANCE   450

/*Crear el objeto de la clase NewPing*/
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
 
void setup() {
  Serial.begin(9600);
  pinMode(LEDVERDE, OUTPUT);
  pinMode(LEDROJO, OUTPUT);
  pinMode(LEDAMARILLO, OUTPUT);
}
 
void loop() {
  // Esperar 10 milisegundos entre mediciones
  delay(10);
  // Obtener medicion de tiempo de viaje del sonido y guardar en variable uS
  int uS = sonar.ping_median();
  // Imprimir la distancia medida a la consola serial
  Serial.print("Distancia: ");
  // Calcular la distancia con base en una constante
  int distancia = uS / US_ROUNDTRIP_CM;

  Serial.print(distancia);
  Serial.print("cm");
  Serial.println();
  if(distancia<=50) // Cuando la distancia es menos o igual a 50 cm SOLO el led rojo se enciende
  { 
    digitalWrite(LEDROJO, HIGH);
    digitalWrite(LEDVERDE, LOW);
    digitalWrite(LEDAMARILLO, LOW);
  }
  else if(distancia>50 & distancia<300) // Si la distancia esta entre 50 y 300 SOLO el led verde se enciende
  {
    digitalWrite(LEDROJO, LOW);
    digitalWrite(LEDVERDE, HIGH);
    digitalWrite(LEDAMARILLO, LOW);
  }
  else // Si la distancia es mayor a 300 cm SOLO el led amarillo se enciende
  {
    digitalWrite(LEDROJO, LOW);
    digitalWrite(LEDVERDE, LOW);
    digitalWrite(LEDAMARILLO, HIGH);
  }

}