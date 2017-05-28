// Libreria para el manejo de motores
// Desarrollada por Diego Morell // droneLAB

//Definimos las librerias base
#ifndef droneLAB_Motores_h
#define droneLAB_Motores_h

// Incluimos las librerias según el modelo de arduino
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class ControlMotor
{
  public:
    ControlMotor(int MD1,int MD2,int MI1,int MI2,int PWMD,int PWMI);  // Asignamos los pines
    void Motor(int velocidad, int giro);  // Carga los valores en los motores
  private:
    void CalcularVelocidad(int velocidad, int giro, int *vel_izq, int *vel_der );  // Calcula la velocidad de cada motor
    int MD1;    // Esta variable la usa internamente la librería para el enable 1 motor derecho
    int MD2;    // Esta variable la usa internamente la librería para el enable 2 motor derecho
    int MI1;    // Esta variable la usa internamente la librería para el enable 1 motor izquierdo
    int MI2;    // Esta variable la usa internamente la librería para el enable 2 motor izquierdo
    int pwmD;       // Esta variable la usa internamente la librería para el PWM del motor Derecho
    int pwmI;       // Esta variable la usa internamente la librería para el PWM del motor Izquierdo
};

#endif
