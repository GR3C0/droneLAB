//Funciones principales en arduino, setup y loop

// LOS PINES EN ARDUINO ESTAN POR DEFECTO EN INPUT

// SIEMPRE PONER EN LA FUNCION SETUP SERIAL.BEGIN(9600)

char variableGlobal = "Hola"; //

void setup()
{
	// Se utiliza para definir los pinMode o las comunicaciones entre componentes
	pinMode(11, OUTPUT);
}

void loop()
{
	// Es el corazón de todos los programas
	digitalWrite(11, HIGH); // Activa el pin 11, HIGH indica que el voltaje llevado es alto, 5 V
	delay(1000); // Espera 1 segundo
	digitalWrite(11, LOW); // Desactiva el pin 11, LOW indica que el voltaje llevado es bajo, 0 V
	delay(1000);
}

// -------------------------Creación de funciones--------------------------

int delayVal()
{
	int v; // Crea la variable 'v'
	v = analogRead(pot); // Lee el valor del potenciometro
	v /= 4; // Divide 'v' entre 4
	return v; // Devulve el valor 'v'
}

// ------------------------------Variables---------------------------------

int inputVariable = 0; // Crea la variable inputVariable
inputVariable = analogRead(2); // Le da a esa variable el valor del pin 2

if(inputVariable < 100) 
{
	inputVariable = 100 // Si inputVariable es menor a 100, se le asigna 100 de valor
}

delay(inputVariable); // Utiliza ese valor como delay

for(int i=0; i<20; i++)
{
	inputVariable = 2;    
}


// ----------------------Tipos de Variables---------------------------------

byte algo = 180; // Almacena un valor numerico de 8 bits de rango de 0 a 255

int algo = 2; // Numero entero

long asd = 900000; // Sirve para enteros largos almacenados en 32 bits

float as = 2.2; // Valor con decimales

int miarray[] = {2,2,2}

int myArray[5]; // Declara un array de enteros con 6 posiciones
myArray[3] = 10; // Asigna a la cuarta posicion del array el valor 10

x = myArray[3]; // Ahora 'x' vale 10 

// -----------------------------Aritmetica---------------------------------

	y = y+3;
    x = x-7;
    i = j*6;
    r = r/5;

	x++;    //lo mismo que x = x+1
    x--;    //lo mismo que x = x-1
    x += y; //lo mismo que x = x+y
    x -= y; //lo mismo que x = x-y
    x *= y; //lo mismo que x = x*y
    x /= y; //lo mismo que x = x/y

	x == y; //x es igual a y
	x != y; //x no es igual a y
	x<y; //xesmenorquey
	x>y; //xesmayorquey
	x <= y; //x es menor o igual que y x >= y; //x es mayor o igual que y

	//AND logico:
    if(x>0 && x<5)  //verdadero sólo si las dos expresiones son ciertas
    //OR logico:
    if(x>0 || y>0)  //verdadero si al menos una expresion es cierta
    //NOT logico:
    if(!(x>0))      //verdadero sólo si la expresión es falsa




//Ejemplo de programa
int led = 13;   //conecta 'led' al pin 13
int pin = 7;    //conecta 'pushbutton' al pin 7
int value = 0;  //variable para almacenar el valor leido

void setup()
{
	//ajusta el pin 13 como salida
	//ajusta el pin 7 como entrada
	pinMode(led, OUTPUT);
	pinMode(pin, INPUT);
}

void loop() {
	//ajusta 'value' igual al pin de entrada
	//ajusta 'led' al valor del boton
	value = digitalRead(pin);
	digitalWrite(led, value);
}


// ------------------------------Serial-------------------------------------

void setup()
{
	Serial.begin(9600) // Abre el puerto serie y ajusta los banadios a 9600
	Serial.println(analogValue) // Imprime datos en pantalla
}

void loop()
{
	Serial.println(analogRead(0)); // Envia el valor analogico
	delay(1000); 
}