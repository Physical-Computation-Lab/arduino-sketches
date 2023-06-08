#include "SparkFun_SGP30_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SGP30
#include <Wire.h>
#include <Servo.h>
#include "pitches.h"


// General
#define BAUD_RATE 9600

// SGP30 Air Quality Sensor
SGP30 mySensor; // create an object of the SGP30 class
float current_co2_avg;
#define MS_PER_MEASUREMENT 1000
#define CALIBRATION_MEASUREMENTS 15
// trying to set a variables' value to 20 (for example)
// produces an error when trying to initialize the array
// with it, so just hardcode the value.
// this value should be adapted empirically.
int measurements[20];

// Servo motor
#define SERVO 9
#define FULLY_OPENED_MOTOR_ANGLE 180
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
int startTime;

// Window
#define LOWER 600  // default: 1000
#define UPPER 800  // default: 2000
float WINDOW_FACTOR = FULLY_OPENED_ANGLE / (UPPER - LOWER);

// Touch
bool status = false;
int out = LOW;
#define touchIn 2
#define touchOut 12
 
// RGB LED and Delay Time
#define GREEN 11 
#define BLUE 5 
#define RED 6 
#define delayTime 20

// Air Fan
#define AIR_FAN 10
#define AIR_FAN_SPEED = 1023

// Sound
#define TONE 13

void setup() {
  // Air fan
  pinMode(AIR_FAN, OUTPUT);
  digitalWrite(AIR_FAN, AIR_FAN_SPEED);
  // Sound / tone
  pinMode(TONE, OUTPUT);
  // LED
  pinMode(GREEN, OUTPUT); 
  pinMode(BLUE, OUTPUT); 
  pinMode(RED, OUTPUT);
  digitalWrite(GREEN, 0);
  digitalWrite(BLUE, 0);
  digitalWrite(RED, HIGH); 
  // Servo motor
  startTime = millis();
  Serial.begin(BAUD_RATE);
  Wire.begin();
  // Initialize sensor
  if (mySensor.begin() == false) {
    // Loop forever when there is no SGP30 detected
    Serial.println("No SGP30 Detected. Check connections.");
    while (1);
  }
  // Initializes sensor for air quality readings
  // measureAirQuality should be called in one second increments after a call to initAirQuality
  mySensor.initAirQuality();
  myservo.attach(SERVO);  // attaches the servo on pin 9 to the servo object
  myservo.write(180);
  // Touchsensor
  pinMode(touchOut, OUTPUT);
  pinMode(touchIn, INPUT);
  digitalWrite(touchOut, out);
}

void loop() {
  delay(MS_PER_MEASUREMENT); //Wait 1 second
  // CO2 levels
  set_co2_level();
  // Servo motor
  set_servor_motor_position();
  // Touch
  touch_sensor();
  // Sound
  // _debug();
}

//
// CO2 functions
//

void calibrate_sensor(){
  // measureAirQuality should be called in one second increments after a call to initAirQuality
  for (int i = 0; i < CALIBRATION_MEASUREMENTS; i++){
    Serial.println("calibrating sensor...please wait. ");
    delay(ms_per_measurement); 
    mySensor.measureAirQuality();
  }
}

void set_co2_level(){
  mySensor.measureAirQuality();
  // CO2
  float co2 = mySensor.CO2;
  Serial.print("Currently measured CO2: ");
  Serial.println(co2);
  // update
  update_measurements(co2);
  // compute average
  current_co2_avg = get_current_co2_average();
  Serial.print("The current average co2 is ");
  Serial.println(current_co2_avg);
}

void update_measurements(float new_measurement){
  // determine array length (see comment at beginning of file)
  int measurements_length = sizeof(measurements)/sizeof(measurements[0]);
  // "move" every element one to the left
  for (int i = 1; i < measurements_length; i++){
    measurements[i - 1] = measurements[i];
  }
  // finally, add new measurement value to array
  measurements[measurements_length - 1] = new_measurement;
  Serial.print("current measurements: ");
  for (int i = 0; i < measurements_length; i++){
    Serial.print(measurements[i]);
  }

}

float get_current_co2_average(){
  // compute and return the average value of
  // all elements in measurements array.
  int measurements_length = sizeof(measurements)/sizeof(measurements[0]);
  float sum = 0;
  for (int i = 0; i < measurements_length; i++){
    sum = sum + measurements[i];
  }
  return sum / measurements_length;
}

//
// Servo motor functions
//

void set_servor_motor_position(){
  float current_servo_pos = FULLY_OPENED_MOTOR_ANGLE - (current_avg_co2 - LOWER) * WINDOW_FACTOR;
  if (current_servo_pos > 180){
    current_servo_pos = 180
  } else if (current_servo_pos < 0){
    current_servo_pos = 0
  }
  Serial.print("current servo motor angle: ");
  Serial.println(myservo.read());
  pos = int(current_servo_pos);
  Serial.print("new position is computed as ");
  Serial.println(pos);
  if (pos != current_servo_pos){
    Serial.println("Setting new servo motor position.")
    myservo.write(pos);
  } else {
    Serial.println("Servo motor position did not change; doing nothing.")
  }
}

//
// Window functions
// (there are no distinct window functions?!)

//
// Touch sensor functions
//

void touch_sensor(){
  out = (out == LOW) ? HIGH : LOW;
  digitalWrite(touchOut, out);
  long int i = 0;
  while(digitalRead(touchIn) != out) i++;
  if(status == 0 && i > 0){
    status = 1;
    Serial.print("Angefasst ");
    Serial.println(i);
  } else if(status == 1 && i == 0){
    status = 0;
    Serial.print("Losgelassen ");
    Serial.println(i); 
  }
}

//
// RGB LED functions
//

//
// Sound / tone functions
//

void _debug(){
  //Tone testen
  //tone(13,1000);
  //delay(1000);
  //noTone(13);
  //delay(1000);
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

//
// Air fan functions
//




