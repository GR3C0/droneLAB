#include <Arduino.h>

#include <Time.h>

void setup()
{
	int ti = 0;
	Serial.begin(9600);
	setTime(0,0,ti,0,0,0);
}

void setup()
{
	time_t t = now();
}