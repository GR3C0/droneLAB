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
#define MOTOR_DEL_IZQ 2
#define MOTOR_DEL_DER 3
#define MOTOR_TRAS_IZQ 4
#define MOTOR_TRAS_DER 5

//----------------PINES SONARS-------------------
#define SONAR_1 6 // Sonar delantero izquierdo
#define SONAR_2 9 // Sonar delantero derecho
#define SONAR_3 12 // Sonar trasero izquierdo
#define SONAR_4 15 // Sonar trasero derecho
#define SONAR_5 18 // Sonar inferiors

#define TRIG_SONAR_1 7 // Trigger sonar del_izq
#define TRIG_SONAR_2 10 // Triger sonar del_der
#define TRIG_SONAR_3 13 // Triger sonar tras_izq
#define TRIG_SONAR_4 16 // Triger sonar tras_der
#define TRIG_SONAR_5 19 // Triger sonar inferior

#define ECHO_SONAR_1 8 // ECHO sonar del_izq
#define ECHO_SONAR_2 11 // ECHO sonar del_der
#define ECHO_SONAR_3 14 // ECHO sonar tras_izq
#define ECHO_SONAR_4 17 // ECHO sonar tras_izq

#define SONAR_NUM 	 5 // Numero de Sonars
#define MAX_DISTANCE 450 // Distancia máxima para los sonars

//-----------------PINES LEDS---------------------
#define LED_VERDE 36
#define LED_ROJO 37
#define LED_AMARILLO 38

//-----------------PID ESPECIFICACIÓNES-----------
// El MPU6050 envia los datos es int16_t

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;


float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];


float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmLeft2, pwmRight2, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=3.0;//3.55
double ki=0.003;//0.003
double kd=2.0;//2.05
///////////////////////////////////////////////

double throttle=1200; // Valor inicial de arranque de los motores
float desired_angle = 0;
