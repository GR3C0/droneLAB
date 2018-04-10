//-------------ARCHIVO PARA TODO EL VUELO-----------

#include "Configuracion.h" // Acceder a todos los parametros
//#include "main.ino"
// TODO: PID del Yaw

void girarDerecha()
{
  pwm_del_izq  = input_THROTTLE - roll_PID;
  pwm_del_der  = input_THROTTLE + roll_PID;
  pwm_tras_izq  = input_THROTTLE - roll_PID;
  pwm_tras_der  = input_THROTTLE + roll_PID;
}

void girarIzquierda()
{
  pwm_del_izq  = input_THROTTLE + roll_PID;
  pwm_del_der  = input_THROTTLE - roll_PID;
  pwm_tras_izq  = input_THROTTLE + roll_PID;
  pwm_tras_der  = input_THROTTLE - roll_PID;
}

void rotarDerecha()
{
  pwm_del_izq  = input_THROTTLE + yaw_PID;
  pwm_del_der  = input_THROTTLE - yaw_PID;
  pwm_tras_izq  = input_THROTTLE - yaw_PID;
  pwm_tras_der  = input_THROTTLE + yaw_PID;
}

void rotarIzquierda()
{
  pwm_del_izq  = input_THROTTLE - yaw_PID;
  pwm_del_der  = input_THROTTLE + yaw_PID;
  pwm_tras_izq  = input_THROTTLE + yaw_PID;
  pwm_tras_der  = input_THROTTLE - yaw_PID;
}

void avanzar()
{
  pwm_del_izq  = input_THROTTLE - pitch_PID;
  pwm_del_der  = input_THROTTLE - pitch_PID;
  pwm_tras_izq  = input_THROTTLE + pitch_PID;
  pwm_tras_der  = input_THROTTLE + pitch_PID;

}

void retroceder()
{
  pwm_del_izq  = input_THROTTLE + pitch_PID;
  pwm_del_der  = input_THROTTLE + pitch_PID;
  pwm_tras_izq  = input_THROTTLE - pitch_PID;
  pwm_tras_der  = input_THROTTLE - pitch_PID;
}
