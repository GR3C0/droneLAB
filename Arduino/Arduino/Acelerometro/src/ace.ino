#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


int LedVerde = 7;
int LedAmarillo = 8;
int LedRojo = 9;
// float a = 0.9;
// float b = 0.5;
// float epsi = sqrt(1-pow((b/a),2));

Adafruit_BNO055 bno = Adafruit_BNO055(55);


// float Kepler()
// {
//   float angulo = 12;
//   float r = b/sqrt(1-pow(epsi,2)*pow(cos(angulo),2));
//   Serial.print(r);
// }


void displayCalStatus(void)
{

  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}

void setup()
{
  Serial.begin(9600);
  pinMode(LedVerde, OUTPUT);
  pinMode(LedAmarillo, OUTPUT);
  pinMode(LedRojo, OUTPUT);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);

  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop()
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> acelerometro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("aX: ");
  Serial.print(acelerometro.x());
  Serial.print(" aY: ");
  Serial.print(acelerometro.y());
  Serial.print(" aZ: ");
  Serial.print(acelerometro.z());
  // Serial.print("||");
  // Serial.print(" eX: ");
  // Serial.print(euler.x());
  // Serial.print(" eY: ");
  // Serial.print(euler.y());
  // Serial.print(" eZ: ");
  // Serial.print(euler.z());
  Serial.println();

  if(euler.x() <= 20)
  {
    digitalWrite(LedRojo, HIGH);
    digitalWrite(LedVerde, LOW);
    digitalWrite(LedAmarillo, LOW);
    Serial.print("Hola\n");
  }
  else if(euler.x() >= 180)
  {
    digitalWrite(LedRojo, LOW);
    digitalWrite(LedVerde, HIGH);
    digitalWrite(LedAmarillo, LOW);
  }
  else
  {
    digitalWrite(LedRojo, LOW);
    digitalWrite(LedVerde, LOW);
    digitalWrite(LedAmarillo, HIGH);
  }

  delay(100);
}
