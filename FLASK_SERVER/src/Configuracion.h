/*--------MASA DRONE--------------------------*/
#define MASA_DRONE 5
/*---------------MINIMOS PID------------------*/
#define AX_MIN 9.9
#define AY_MIN 9.9
#define AZ_MIN 9.9

/*---------NIVELES VELOCIDAD MOTORES----------*/
#define MOTOR_ZERO_LEVEL 1000
#define MOTOR_MIN_LEVEL 1100
#define MOTOR_MAX_LEVEL 2000

//---------------PINES MOTORES----------------
#define MOTOR_DEL_IZQ 3 //TODO: Cambiar pines
#define MOTOR_DEL_DER 5
#define MOTOR_TRAS_IZQ 6
#define MOTOR_TRAS_DER 7

//----------------PINES SONARS-------------------
#define SONAR_1 78 // Sonar delantero izquierdo
#define SONAR_2 9 // Sonar delantero derecho
#define SONAR_3 12 // Sonar trasero izquierdo
#define SONAR_4 15 // Sonar trasero derecho
#define SONAR_5 18 // Sonar inferiors

#define TRIG_SONAR_1 756 // Trigger sonar del_izq
#define TRIG_SONAR_2 10 // Triger sonar del_der
#define TRIG_SONAR_3 13 // Triger sonar tras_izq
#define TRIG_SONAR_4 16 // Triger sonar tras_der
#define TRIG_SONAR_5 19 // Triger sonar inferior

#define ECHO_SONAR_1 8 // ECHO sonar del_izq
#define ECHO_SONAR_2 11 // ECHO sonar del_der
#define ECHO_SONAR_3 14 // ECHO sonar tras_izq
#define ECHO_SONAR_4 17 // ECHO sonar tras_izq

#define SONAR_NUM 	 5 // Numero de Sonars
#define MAX_DISTANCE 100 // Distancia máxima para los sonars

//-----------------PINES LEDS---------------------
#define LED_VERDE 36
#define LED_ROJO 37
#define LED_AMARILLO 38
