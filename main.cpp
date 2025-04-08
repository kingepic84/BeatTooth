/*
  Arduino BMI270 - Accelerometer with Continuous Delta Calculation
  and LED Update Based on a 50ms Detection Window at the End of Each Second

  This example continuously reads acceleration values from the BMI270 sensor
  and computes a delta value (Euclidean distance) between consecutive readings.
  Every one second, only the readings from the last 50ms of that interval are
  considered: if any delta during that window exceeded 0.75, the green LED (pin 13)
  flashes for 100ms; otherwise, the red LED (pin 2) flashes for 100ms.
  In between flashes, no LED is lit.

  The circuit:
  - Arduino Nano 33 BLE Sense Rev2

  created 10 Jul 2019 (modified by ChatGPT)
  This example code is in the public domain.
*/

#include "Arduino_BMI270_BMM150.h"
#include <math.h> // For sqrt() and pow()

// Global variables for sensor data
float prevX = 0.0, prevY = 0.0, prevZ = 0.0;
bool firstReading = true;

// Timing and detection parameters
unsigned long lastLEDCheckTime = 0;
const unsigned long LED_CHECK_INTERVAL = 1000; // 1 second interval
const unsigned long DETECTION_WINDOW = 250;       // Last 50ms of the interval
const unsigned long LED_FLASH_DURATION = 100;    // 100ms LED flash duration
bool ledFlashing = false;
unsigned long ledFlashStartTime = 0;
bool recentChangeDetected = false;  // Set true if delta > 0.75 in the detection window

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  // Set LED pins as outputs
  pinMode(2, OUTPUT);    // Red LED
  pinMode(13, OUTPUT);   // Green LED

  // Ensure LEDs are off initially
  digitalWrite(2, LOW);
  digitalWrite(13, LOW);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration (X, Y, Z), Delta");
  Serial.println("X\tY\tZ\tDelta");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Continuously compute delta as soon as sensor data is available
  if (IMU.accelerationAvailable()) {
    float x, y, z;
    IMU.readAcceleration(x, y, z);
    
    float delta = 0.0;
    if (!firstReading) {
      // Compute Euclidean distance between the current and previous readings
      delta = sqrt(pow(x - prevX, 2) + pow(y - prevY, 2) + pow(z - prevZ, 2));
    } else {
      firstReading = false;
    }
    
    // Update previous readings for the next computation
    prevX = x;
    prevY = y;
    prevZ = z;
    
    // Print the current acceleration values and delta to the Serial Monitor
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.print('\t');
    Serial.println(delta);
    
    // Only during the last 50ms of the current interval, check if delta exceeds 0.75
    if ((currentMillis - lastLEDCheckTime >= LED_CHECK_INTERVAL - DETECTION_WINDOW) &&
        (currentMillis - lastLEDCheckTime < LED_CHECK_INTERVAL)) {
      if (delta > 0.75) {
        recentChangeDetected = true;
      }
    }
  }
  
  // At the end of each 1-second interval, flash the appropriate LED
  if ((currentMillis - lastLEDCheckTime >= LED_CHECK_INTERVAL) && (!ledFlashing)) {
    if (recentChangeDetected) {
      // Green LED indicates a significant change occurred in the last 50ms
      digitalWrite(13, HIGH);
      digitalWrite(2, LOW);
    } else {
      // Red LED indicates no significant change was detected in the window
      digitalWrite(13, LOW);
      digitalWrite(2, HIGH);
    }
    
    // Begin LED flash period and reset the detection flag
    ledFlashing = true;
    ledFlashStartTime = currentMillis;
    recentChangeDetected = false;
    
    // Advance the LED check time to maintain a 1-second cycle (avoiding drift)
    lastLEDCheckTime += LED_CHECK_INTERVAL;
  }
  
  // After 100ms, turn off the LED if flashing
  if (ledFlashing && (currentMillis - ledFlashStartTime >= LED_FLASH_DURATION)) {
    digitalWrite(13, LOW);
    digitalWrite(2, LOW);
    ledFlashing = false;
  }
}
