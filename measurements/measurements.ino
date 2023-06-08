#include "SparkFun_SGP30_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SGP30
#include <Wire.h>
 
SGP30 mySensor; //create an object of the SGP30 class
// trying to set a variables' value to 20 (for example)
// produces an error, so just hardcode the value.
int measurements[20];


/*
First try at implementing a function
that maintains an array of measurements and
keeps them up to date. 
The mean of these values is considered to be
the actual value the system should consider.
*/

void setup() {
  // set data rate in bits / seconds
  Serial.begin(9600);
  // intialize Wire
  Wire.begin();
  // sensor connected?
  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
    while (1);
  }
  // initialize sensor for air quality readings
  mySensor.initAirQuality();
  calibrate_sensor();
  // fill up measurements with 400 as a reasonable value to start with
  measurements[0] = 400;
  int measurements_length = sizeof(measurements)/sizeof(measurements[0]);
  for (int i = 0; i < measurements_length; i++){
    measurements[i] = 400;
  }
}

void calibrate_sensor(){
  // measureAirQuality should be called in one second increments after a call to initAirQuality
  int wait_ms = 1000; // wait 1 second
  for (int i = 0; i < 15; i++){
    Serial.println("calibrating sensor.");
    delay(wait_ms); 
    mySensor.measureAirQuality();
  }
}

void loop() {
  // main loop.
  delay(1000); // Wait 1 second
  mySensor.measureAirQuality();
  // CO2
  float co2 = mySensor.CO2;
  Serial.print("Currently measured CO2: ");
  Serial.println(co2);
  // update
  update_measurements(co2);
  // compute average
  avg = get_current_co2_average();
  Serial.print("The current average co2 is ");
  Serial.println(avg);
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
}

float get_current_co2_average(){
  int measurements_length = sizeof(measurements)/sizeof(measurements[0]);
  float sum = 0;
  for (int i = 1; i < measurements_length; i++){
    sum = sum + measurements[i];
  }
  return sum / measurements_length;
}