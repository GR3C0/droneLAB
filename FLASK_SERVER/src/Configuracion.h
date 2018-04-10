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

//-----------------PID ESPECIFICACIÓNES-----------
// El MPU6050 envia los datos en int16_t

// TODO: Pruebas:
#define MIN 1650
#define MAX 2000

int input_YAW;
int input_PITCH;
int input_ROLL;
int input_THROTTLE = 1400;

float elapsedTime, time, timePrev;
int gyro_error = 0;
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Gyro_angle_x, Gyro_angle_y;
float Gyro_raw_error_x, Gyro_raw_error_y;

// Variables del acelerometro
int acc_error = 0;
float rad_to_deg = 180/3.141592654;
float Acc_rawX, Acc_rawY, Acc_rawZ;
float Acc_angle_x, Acc_angle_y;
float Acc_angle_error_x, Acc_angle_error_y;

float Total_angle_x, Total_angle_y;

int i;
int mot_activated = 0;
long activate_count = 0;
long des_activate_count = 0;

//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_del_izq, pwm_del_der, pwm_tras_izq, pwm_tras_der, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp=3.90;//3.55
double roll_ki=0.007;//0.003
double roll_kd=2.5;//2.05
float roll_desired_angle = 0;

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp=3.90;//3.55
double pitch_ki=0.007;//0.003
double pitch_kd=2.5;//2.05
float pitch_desired_angle = 0;

//////////////////////////////PID FOR YAW//////////////////////////
float yaw_PID=0, yaw_error, yaw_previous_error;
float yaw_pid_p=0;
float yaw_pid_i=0;
float yaw_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double yaw_kp=3.90;//3.55
double yaw_ki=0.007;//0.003
double yaw_kd=2.5;//2.05
float yaw_desired_angle = 0;
