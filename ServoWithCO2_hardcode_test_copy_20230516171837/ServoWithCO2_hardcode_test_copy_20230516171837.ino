/*
  Library for the Sensirion SGP30 Indoor Air Quality Sensor
  By: Ciara Jekel
  SparkFun Electronics
  Date: June 28th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SGP30 Datasheet: https://cdn.sparkfun.com/assets/c/0/a/2/e/Sensirion_Gas_Sensors_SGP30_Datasheet.pdf

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14813

  This example reads the sensors calculated CO2 and TVOC values
*/

#include "SparkFun_SGP30_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SGP30
#include <Wire.h>
#include <Servo.h>

Servo myservo; // create servo object to control a servo
int pos = 0;    // variable to store the servo position
int startTime;
SGP30 mySensor; //create an object of the SGP30 class
//
float LOWER = 1000;
float UPPER = 2000;
float FULLY_OPENED_ANGLE = 90;
float factor = FULLY_OPENED_ANGLE / (UPPER - LOWER);
int open_close = 0;

void setup() {
  startTime = millis();
  Serial.begin(9600);
  Wire.begin();
  //Initialize sensor
  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
    while (1);
  }
  //Initializes sensor for air quality readings
  //measureAirQuality should be called in one second increments after a call to initAirQuality
  mySensor.initAirQuality();
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(0);
}

void loop() {
  //First fifteen readings will be
  //CO2: 400 ppm  TVOC: 0 ppb
  delay(5000); //Wait 1 second
  //measure CO2 and TVOC levels
  mySensor.measureAirQuality();
  Serial.print("CO2: ");
  Serial.print(mySensor.CO2);
  Serial.print(" ppm\t");
  if (open_close == 0){
      open_close = 1;
  } else {
    open_close = 0;
  }
  // int co2 = mySensor.CO2 - 400;
  float co2_measured = mySensor.CO2;
  // float calcServoPos = factor * (co2);
  float servo_pos = (co2_measured - 1000) * factor;
  Serial.print("current CO2: ");
  Serial.println(co2_measured);
  Serial.print("current Servo: ");
  Serial.println(myservo.read());
  float test_degree = 0;
  float test_degree_2 = 90;
  if (open_close == 0){
      myservo.write(test_degree);
  } else {
      myservo.write(test_degree_2);
  }
  
  //if(mySensor.CO2 > 1000){
  //    pos = int(calcServoPos); 
  //    myservo.write(pos);
  //}

  

  //Debugging
  //.print("factor: ");
  //Serial.print(factor);
  //Serial.print("\t Co2 value: ");
  //Serial.print(co2);
  //Serial.print("\t calc position: ");
  //Serial.print(calcServoPos);
  //Serial.print("\t servo position: ");
  //Serial.println(pos);
}
