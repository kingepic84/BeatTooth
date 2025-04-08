// *Interfacing RGB LED with Arduino 
// * Author: Osama Ahmed 

#include <Arduino.h>
//Defining  variable and the GPIO pin on Arduino
int redPin= 5;
int greenPin = 9;
int bluePin = 7;

void setup() {
  //Defining the pins as OUTPUT
  pinMode(redPin,  OUTPUT);              
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void setColor(int redValue, int greenValue,  int blueValue) {
    analogWrite(redPin, redValue);
    analogWrite(greenPin,  greenValue);
    analogWrite(bluePin, blueValue);
}

void loop() {
  setColor(255, 0, 0); // Red Color
  delay(1000);
  setColor(0,  255, 0); // Green Color
  delay(1000);
  setColor(0,  0, 255); // Blue Color
  delay(1000);
}
