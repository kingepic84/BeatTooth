/*
  Arduino Nano BLE 33 Rev2 - Controller Code
  --------------------------------------------
  This sketch uses digital pin 2 as an output to send a trigger signal 
  to a connected Arduino Nano. When the trigger is sent, the peripheral code
  on the Nano will execute (e.g., blink its LED).

  Wiring:
    - Connect Digital Pin 2 of the Nano BLE 33 Rev2 to Digital Pin 2 on the Arduino Nano.
    - Connect the grounds of both boards.
*/

#include <Arduino.h>

const int controlPin = 2;  // Pin used to send the trigger signal

void setup() {
  Serial.begin(9600);
  pinMode(controlPin, OUTPUT);    // Set the control pin as output

  Serial.println("Controller ready. Waiting 2 seconds before sending trigger...");
  delay(2000);  // Wait for 2 seconds before sending signal
  
  // Send the trigger signal: HIGH for 1 second
  digitalWrite(controlPin, HIGH);
  Serial.println("Trigger sent: HIGH");
  delay(1000);
  digitalWrite(controlPin, LOW);
  Serial.println("Trigger ended: LOW");

  // Additional trigger code can be added in loop if needed.
}

void loop() {
  // For this example, the trigger signal is sent only once in setup.
  // Implement additional logic here to trigger more events if desired.
}
