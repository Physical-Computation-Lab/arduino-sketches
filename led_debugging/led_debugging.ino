#include "SparkFun_SGP30_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SGP30
/*
Wire.h is used for accessing certain ports on the
Arduino, but these are not needed in this sketch.
*/
// #include <Wire.h>

/*
This sketch was made for debugging purposes.
Will set the pins to HIGH - LOW - HIGH - LOW...
periodically.
This can be used for checking if whether LEDs
are still working or not.
NOTE: use appropriate resistors!
*/

int LED_1 = 8;
int LED_2 = 9;
int LED_3 = 10;
int LED_4 = 11;
int LED_5 = 12;
int LED_6 = 13;

int LED_arr[] = {
  LED_1, 
  LED_2,
  LED_3,
  LED_4,
  LED_5,
  LED_6
};

int PERIOD = 500;

void setup() {
  /*
  call pinMode to set the mode to OUTPUT
  for each pin used.
  */
  
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);
  pinMode(LED_6, OUTPUT);
}

void loop() {
  /*
  Use digitalWrite() to write set outputs
  to HIGH (~True, 1, LED on) or LOW (~False, 0, LED off).
  Use delay(t_ms) to delay program execution
  for t_ms milliseconds.
  */
  for (int i = 0; i < sizeof(LED_arr); i++){
    digitalWrite(LED_arr[i], HIGH);
  }
  delay(PERIOD); //Wait 1 second
  for (int i = 0; i < sizeof(LED_arr); i++){
    digitalWrite(LED_arr[i], LOW);
  }
  delay(PERIOD); //Wait 1 second
}
