/*--------MASA DRONE--------------------------*/
#define MASA_DRONE 5
/*---------------MINIMOS PID------------------*/
#define AX_MIN 9.9
#define AY_MIN 9.9
#define AZ_MIN 9.9

/*---------NIVELES VELOCIDAD MOTORES----------*/
#define MOTOR_ZERO_LEVEL 1000
#define MOTOR_MIN_LEVEL 1100
#define MOTOR_MEDIUM_LEVEL 1500
#define MOTOR_MAX_LEVEL 2000

//---------------PINES MOTORES----------------
#define MOTOR_DEL_IZQ 2
#define MOTOR_DEL_DER 3
#define MOTOR_TRAS_IZQ 4
#define MOTOR_TRAS_DER 5

//----------------PINES SONARS-------------------
#define SONAR_1 6 // Sonar delantero izquierdo
#define SONAR_2 9 // Sonar delantero derecho
#define SONAR_3 12 // Sonar trasero izquierdo
#define SONAR_4 15 // Sonar trasero derecho

#define TRIG_SONAR_1 7 // Trigger sonar del_izq
#define TRIG_SONAR_2 10 // Triger sonar del_der
#define TRIG_SONAR_3 13 // Triger sonar tras_izq
#define TRIG_SONAR_4 16 // Triger sonar tras_der

#define ECHO_SONAR_1 8 // ECHO sonar del_izq
#define ECHO_SONAR_2 11 // ECHO sonar del_der
#define ECHO_SONAR_3 14 // ECHO sonar tras_izq
#define ECHO_SONAR_4 17 // ECHO sonar tras_izq

#define SONAR_NUM 	 4 // Numero de Sonars
#define MAX_DISTANCE 450 // Distancia máxima para los sonars

//-----------------PINES LEDS---------------------
#define LED_VERDE 36
#define LED_ROJO 37
#define LED_AMARILLO 38

//-----------------PID ESPECIFICACIÓNES-----------
float Angulo_aceleracion[2];
float Angulo_giro[2];
float Angulo_total[2];
float giro_z, giro_y, acel_z, acel_y;

float T_transcurrido, time, timePrev;
int i;

float PID, pwmIzq, pwmDer, error, error_previo;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=3.55;//3.55 Amplia o reduce el error
double ki=0.003;//0.003 Acumula el error
double kd=2.05;//2.05 Evalua el incremento del error
///////////////////////////////////////////////

double velocidad=1300; // Velocidad inicial de los motores
float angulo_deseado = 0; // Angulo que queremos
