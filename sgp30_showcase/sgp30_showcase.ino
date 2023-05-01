#include "SparkFun_SGP30_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SGP30
#include <Wire.h>
 
SGP30 mySensor; //create an object of the SGP30 class

/*
Showcase how to access the measurement data and others
when using the AirQualitySensor SGP30.
Can also be used for debugging.
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
  /*
  show which variables are accessible in the SGP30.
  */
  delay(1000); // Wait 1 second
  mySensor.measureAirQuality();
  // CO2
  float co2 = mySensor.CO2;
  Serial.print("CO2: ");
  Serial.println(co2);
  // TVOC
  float tvoc = mySensor.TVOC;
  Serial.print("TVOC: ");
  Serial.println(tvoc);
  // baselineCO2
  float baselineco2 = mySensor.baselineCO2;
  Serial.print("baselineCO2: ");
  Serial.println(baselineco2);
  // baselineTVOC
  float baselinetvoc = mySensor.baselineTVOC;
  Serial.print("baselineTVOC: ");
  Serial.println(baselinetvoc);
  // featureSetVersion
  float featuresetversion = mySensor.featureSetVersion;
  Serial.print("featureSetVersion: ");
  Serial.println(featuresetversion);
  // H2
  float h2 = mySensor.H2;
  Serial.print("H2: ");
  Serial.println(h2);
  // ethanol
  float ethanol = mySensor.ethanol;
  Serial.print("ethanol: ");
  Serial.println(ethanol);
  // serialID
  float serialid = mySensor.serialID;
  Serial.print("serialID: ");
  Serial.println(serialid);
}
