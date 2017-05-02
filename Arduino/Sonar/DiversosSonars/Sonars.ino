#include <NewPing.h>

#define SONAR_NUM 	 3
#define MAX_DISTANCE 450

NewPing sonar[SONAR_NUM] = 
{
	NewPing(20, 21, MAX_DISTANCE),
	NewPing(32, 33, MAX_DISTANCE),
	NewPing(48, 49, MAX_DISTANCE),
};

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	delay(10); 
	for(int i = 0; i < SONAR_NUM; i++){
		int uS = sonar[i].ping_median();
		int distancia = uS / US_ROUNDTRIP_CM;
	    Serial.print(distancia);
	    Serial.print("cm");
	    Serial.print("||");
	}
	Serial.println();
}