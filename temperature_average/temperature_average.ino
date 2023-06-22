#include "SparkFun_SGP30_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SGP30
#include <Wire.h>

// General
#define BAUD_RATE 9600
int temp_measurements[20];

// Temperature
#define LM35 A0
float current_temp_avg;


void setup() {
  // temperature
  pinMode(LM35, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  // temperature
  set_temperature_measurement();
}

void set_temperature_measurement(){
  float lmvalue = analogRead(LM35);
  float temperature = (lmvalue * 500) / 1023;
  Serial.print("Currently measured temperature in celsius ");
  Serial.println(temperature);
  // update
  update_temp_measurements(temperature);
  //
  current_temp_avg = get_current_temp_average();
  Serial.print("The current average temperature in celsius is ");
  Serial.println(current_temp_average);
}

float get_current_temp_average(){
  // compute and return the average value of
  // all elements in measurements array.
  int temp_measurements_length = sizeof(temp_measurements)/sizeof(temp_measurements[0]);
  float sum = 0;
  for (int i = 0; i < temp_measurements_length; i++){
    sum = sum + temp_measurements[i];
  }
  return sum / temp_measurements_length;
}

void update_temp_measurements(float new_temp_measurement){
  // determine array length (see comment at beginning of file)
  int temp_measurements_length = sizeof(temp_measurements)/sizeof(temp_measurements[0]);
  // "move" every element one to the left
  for (int i = 1; i < temp_measurements_length; i++){
    temp_measurements[i - 1] = temp_measurements[i];
  }
  // finally, add new temp_measurement value to array
  temp_measurements[temp_measurements_length - 1] = new_temp_measurement;
  Serial.print("current temp_measurements: ");
  for (int i = 0; i < temp_measurements_length; i++){
    Serial.print(temp_measurements[i]);
  }