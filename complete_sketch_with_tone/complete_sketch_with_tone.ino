#include "SparkFun_SGP30_Arduino_Library.h"  // Click here to get the library: http://librarymanager/All#SparkFun_SGP30
#include <Wire.h>
#include <Servo.h>
#include "pitches.h"


// General
#define BAUD_RATE 9600

// SGP30 Air Quality Sensor
SGP30 mySensor;  // create an object of the SGP30 class
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
#define WINDOW_OPENING_SPEED 2
#define WINDOW_CLOSING_SPEED 2
Servo myservo;  // create servo object to control a servo
int startTime;  // is this variable needed?

// Window
#define LOWER 450.0  // default: 1000
#define UPPER 800.0  // default: 2000
#define WINDOW_TOLERANCE 2
float WINDOW_FACTOR = FULLY_OPENED_MOTOR_ANGLE / (UPPER - LOWER);

// Touch
boolean status = false;
int out = LOW;
unsigned long int IDLE_DURATION = 1000UL * 60UL;  // 3 Minutes
boolean idleMode = false;
unsigned long int idleTime;  //last Time, when the Idle Mode was activated
#define touchIn 2
#define touchOut 12

// Tone
unsigned long int toneTime;
unsigned long int TONE_DURATION = 1000UL;  // 1 Sekunde
bool playTone = false;

// RGB LED and Delay Time
#define GREEN 11
#define BLUE 5
#define RED 6
#define delayTime 20

// Temperatur
#define LM35 A0

// Air Fan
#define AIR_FAN 10
#define AIR_FAN_SPEED 1023

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
  analogWrite(GREEN, 255);
  analogWrite(BLUE, 0);
  analogWrite(RED, 0);
  // Servo motor
  startTime = millis();  // is this needed?
  Serial.begin(BAUD_RATE);
  Wire.begin();
  // Initialize sensor
  if (mySensor.begin() == false) {
    // Loop forever when there is no SGP30 detected
    Serial.println("No SGP30 Detected. Check connections.");
    while (1)
      ;
  }
  // Initializes sensor for air quality readings
  // measureAirQuality should be called in one second increments after a call to initAirQuality
  mySensor.initAirQuality();
  calibrate_sensor();
  // servo motor
  myservo.attach(SERVO);  // attaches the servo on pin 9 to the servo object
  myservo.write(180);
  // Touchsensor
  pinMode(touchOut, OUTPUT);
  pinMode(touchIn, INPUT);
  digitalWrite(touchOut, out);
}

void loop() {
  //delay(MS_PER_MEASUREMENT); //Wait 1 second
  if (!idleMode) {
    // CO2 levels
    set_co2_level();
    // Servo motor
    set_servor_motor_position();
    // Touch
    touch_sensor();
    //Temperatur
    showTemperatur();
    // RGB LED
    //controlLED();
    // Sound
    toneManagement();
  } else {
    // CO2 levels
    set_co2_level();

    updateIdleMode();
  }
}

// Temperatur messen
void showTemperatur() {
  float lmvalue = analogRead(LM35);
  float tmp = (lmvalue * 500) / 1023;
  Serial.print("Temperatur measurment in celsius ");
  Serial.println(tmp);
}


//
// CO2 functions
//

void calibrate_sensor() {
  // measureAirQuality should be called in one second increments after a call to initAirQuality
  // add some RGB LED effects to the loop?
  for (int i = 0; i < CALIBRATION_MEASUREMENTS; i++) {
    Serial.println("calibrating sensor...please wait. ");
    delay(MS_PER_MEASUREMENT);
    set_co2_level();
    //mySensor.measureAirQuality();
  }
}

void set_co2_level() {
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

void update_measurements(float new_measurement) {
  // determine array length (see comment at beginning of file)
  int measurements_length = sizeof(measurements) / sizeof(measurements[0]);
  // "move" every element one to the left
  for (int i = 1; i < measurements_length; i++) {
    measurements[i - 1] = measurements[i];
  }
  // finally, add new measurement value to array
  measurements[measurements_length - 1] = new_measurement;
  Serial.print("current measurements: ");
  for (int i = 0; i < measurements_length; i++) {
    Serial.print(measurements[i]);
    Serial.print(" ");
  }
  Serial.println();
}

float get_current_co2_average() {
  // compute and return the average value of
  // all elements in measurements array.
  int measurements_length = sizeof(measurements) / sizeof(measurements[0]);
  float sum = 0;
  for (int i = 0; i < measurements_length; i++) {
    sum = sum + measurements[i];
  }
  return sum / measurements_length;
}

//
// Servo motor / window functions
//

void set_servor_motor_position() {
  float new_servo_pos = FULLY_OPENED_MOTOR_ANGLE - (current_co2_avg - LOWER) * WINDOW_FACTOR;
  if (new_servo_pos > 180) {
    new_servo_pos = 180;
  } else if (new_servo_pos <= 2) {
    new_servo_pos = 2;
  }
  int current_servo_pos = myservo.read();
  Serial.print("current servo motor angle: ");
  Serial.println(current_servo_pos);
  int pos = int(new_servo_pos);  // are floats allowed?
  Serial.print("new position is computed as ");
  Serial.println(pos);
  // set new position when new and current
  // position are not equal
  if (pos != current_servo_pos) {
    Serial.print("Setting new servo motor position.");
    //myservo.write(pos);
    Serial.println(pos);
  } else {
    Serial.println("Servo motor position did not change; doing nothing.");
  }
  // Servo motor speed controlling

  if (new_servo_pos - WINDOW_TOLERANCE > current_servo_pos) {
    Serial.println("Setting new servo motor position: opening.");
    myservo.write(int(current_servo_pos + WINDOW_OPENING_SPEED));  // can floats be used?
  } else if (new_servo_pos + WINDOW_TOLERANCE < current_servo_pos) {
    Serial.println("Setting new servo motor position: closing.");
    myservo.write(int(current_servo_pos - WINDOW_CLOSING_SPEED));  // can floats be used?
  } else {
    Serial.println("Not setting new servo motor position.");
  }
  /*
  while (new_servo_pos != 0){
    myservo.write(current_servo_pos - 1);
    delay(20);
    current_servo_pos = myservo.read();
  }*/
}

//
// Touch sensor functions
//

void touch_sensor() {
  out = (out == LOW) ? HIGH : LOW;
  digitalWrite(touchOut, out);
  long int i = 0;
  while (digitalRead(touchIn) != out) i++;
  if (status == 0 && i > 0) {
    status = 1;
    Serial.print("Angefasst ");
    Serial.println(i);
    idleTime = millis();
    idleMode = true;
    myservo.write(FULLY_OPENED_MOTOR_ANGLE);
  } else if (status == 1 && i == 0) {
    status = 0;
    Serial.print("Losgelassen ");
    Serial.println(i);
  }
}

//
// RGB LED functions
void controlLED() {  // make green to red
  if (current_co2_avg > LOWER) {
    int redVal = 255;
    int blueVal = 0;
    int greenVal = 0;
    for (int i = 0; i < 255; i += 1) {
      greenVal += 1;
      redVal -= 1;
      analogWrite(GREEN, 255 - greenVal);
      analogWrite(RED, 255 - redVal);

      delay(delayTime);
    }
  }
  /*delay(2000);
  int redVal = 255;
  int blueVal = 0;
  int greenVal = 0;
  for( int i = 0 ; i < 255 ; i += 1 ){
    greenVal += 1;
    redVal -= 1;
    analogWrite( GREEN, 255 - greenVal );
    analogWrite( RED, 255 - redVal );

    delay( delayTime );
  }
delay(2000);
   redVal = 0;
   blueVal = 0;
   greenVal = 255;
  for( int i = 0 ; i < 255 ; i += 1 ){
    greenVal -= 1;
    redVal += 1;
    analogWrite( GREEN, 255 - greenVal );
    analogWrite( RED, 255 - redVal );

    delay( delayTime );
  }*/
}
//

//
// Sound / tone functions
//

//
// Idle Mode
//

void updateIdleMode() {
  if (idleMode) {
    Serial.println(millis() - idleTime);
    if ((millis() - idleTime) >= IDLE_DURATION) {
      idleMode = false;
    }
  }
}

void toneManagement() {
  //Tone testen
  if ((millis() - toneTime) >= TONE_DURATION && playTone) {
      
    if(myservo.read() <= 5){
      tone(TONE, 1000);
      toneTime = millis();
      playTone = !playTone;
    }
  } else if ((millis() - toneTime) >= TONE_DURATION && !playTone) {
    noTone(TONE);
    toneTime = millis();
    playTone = !playTone;
  }
  //delay(1000);

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
