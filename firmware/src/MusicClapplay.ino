/*!
 *@file clap_music_rgb.ino
 *@brief Clap Detection, Music Playback, and RGB LED Beat Game
 *@details The DF1201S MP3 module starts playing music immediately. The user is expected to clap
 *         in time with the beat (roughly every 750 ms). The system continuously samples the ADXL3xx 
 *         accelerometer. When a sudden change (a clap) is detected, its time is recorded (subject to a
 *         1-second debounce). Then, at every 750‑ms beat, the code checks if the last clap occurred 
 *         in the final 50 ms of the beat (i.e. between 700 ms and 750 ms after the beat started). 
 *         If a clap is detected in that narrow window, the LED flashes green; if not, it flashes red.
 *@date 2023-10-09
 */

#include <DFRobot_DF1201S.h>
#include "SoftwareSerial.h"
SoftwareSerial DF1201SSerial(2, 3);  // RX, TX

DFRobot_DF1201S DF1201S;

//---------------------------------------------------------
// Accelerometer (ADXL3xx) Pin Definitions:
// - analog 0: self test (unused)
// - analog 1 (A1): z-axis
// - analog 2 (A2): y-axis
// - analog 3 (A3): x-axis
// - analog 4: ground  --> provided via digital pin 18
// - analog 5: vcc     --> provided via digital pin 19
//---------------------------------------------------------
const int groundpin = 18;
const int powerpin  = 19;
const int xpin      = A3;
const int ypin      = A2;
const int zpin      = A1;

//---------------------------------------------------------
// RGB LED Pins (LED is common cathode; longest lead is ground)
//---------------------------------------------------------
const int redPin   = 8;   // red channel (anode via resistor)
const int greenPin = 9;   // green channel (anode via resistor)
const int bluePin  = 10;  // blue channel (unused here)

//---------------------------------------------------------
// Clap Detection Parameters
//---------------------------------------------------------
const int CLAP_THRESHOLD = 100; // (Do not change) Adjust the threshold as needed

// Variables to store previous accelerometer readings:
int prevX = 0, prevY = 0, prevZ = 0;
bool firstReading = true;

// Timing variables:
// lastClapTime is updated when a clap is detected (subject to debounce).
unsigned long lastClapTime = 0;
const unsigned long CLAP_DEBOUNCE = 10; // 1 second debounce

// Beat timing:
unsigned long previousBeatMillis = 0;
const unsigned long BEAT_INTERVAL = 750;   // 750 ms beat period
const unsigned long VALID_CLAP_WINDOW = 125;  // Clap is valid if it occurs in the last 50 ms of the beat

// LED on duration:
const unsigned long LED_ON_DURATION = 200;   // LED lights for 200 ms

void setup(void) {
  Serial.begin(115200);
  
  // Setup accelerometer power pins:
  pinMode(groundpin, OUTPUT);
  pinMode(powerpin, OUTPUT);
  digitalWrite(groundpin, LOW);
  digitalWrite(powerpin, HIGH);
  
  // Setup RGB LED pins:
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);
  
  // Initialize the DF1201S music module:
  DF1201SSerial.begin(115200);
  while (!DF1201S.begin(DF1201SSerial)) {
    Serial.println("DF1201S init failed, please check the wire connection!");
    delay(1000);
  }
  
  DF1201S.setVol(1);
  Serial.print("Volume: ");
  Serial.println(DF1201S.getVol());
  
  DF1201S.switchFunction(DF1201S.MUSIC);
  delay(2000);  // Allow prompt tone to finish
  
  DF1201S.setPlayMode(DF1201S.ALLCYCLE);
  Serial.print("PlayMode: ");
  Serial.println(DF1201S.getPlayMode());
  
  // Start playing music immediately:
  DF1201S.start();
  
  Serial.println("Setup complete. Music playing; await claps to the beat...");
  
  previousBeatMillis = millis();
}

void loop(void) {
  unsigned long currentMillis = millis();
  
  // --- Accelerometer Sampling & Clap Detection ---
  // Sample the accelerometer continuously and record a clap if a sudden change occurs.
  int currX = analogRead(xpin);
  int currY = analogRead(ypin);
  int currZ = analogRead(zpin);
  
  if (!firstReading && (currentMillis - lastClapTime >= CLAP_DEBOUNCE)) {
    int deltaX = abs(currX - prevX);
    int deltaY = abs(currY - prevY);
    int deltaZ = abs(currZ - prevZ);
    
    if (deltaX > CLAP_THRESHOLD || deltaY > CLAP_THRESHOLD || deltaZ > CLAP_THRESHOLD) {
      Serial.println("yes");  // Clap detected
      lastClapTime = currentMillis;
    }
  } else {
    firstReading = false;
  }
  
  // Update previous accelerometer readings:
  prevX = currX;
  prevY = currY;
  prevZ = currZ;
  
  // --- Beat-Based LED Blink ---
  // Every BEAT_INTERVAL (750 ms), the code checks whether the last clap occurred in the final
  // VALID_CLAP_WINDOW (last 50 ms) of the beat window. If so, flash green; else, flash red.
  if (currentMillis - previousBeatMillis >= BEAT_INTERVAL) {
    // Save the start of the current beat window.
    unsigned long beatStart = previousBeatMillis;
    previousBeatMillis = currentMillis;  // update for the next beat
    
    // Check if the last clap occurred in the final 50 ms of the beat.
    if (lastClapTime >= (beatStart + BEAT_INTERVAL - VALID_CLAP_WINDOW) && lastClapTime <= (beatStart + BEAT_INTERVAL)) {
      // Valid clap detected in the final 50 ms; flash green.
      digitalWrite(greenPin, HIGH);
      delay(LED_ON_DURATION);
      digitalWrite(greenPin, LOW);
    } else {
      // No valid clap during the last 50 ms; flash red.
      digitalWrite(redPin, HIGH);
      delay(LED_ON_DURATION);
      digitalWrite(redPin, LOW);
    }
  }
  
  delay(30); // Short delay for frequent sampling.
}
