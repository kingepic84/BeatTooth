/*!
 * @file peripheral_music_trigger.ino
 * @brief Peripheral code for Arduino Nano that plays songs when a trigger signal is received.
 * 
 * The code initializes the DF1201S module (for playing songs) and listens for a trigger on digital pin 4.
 * When the trigger pin goes HIGH, it runs a sequence of commands (start, pause, next, previous, fast forward, etc.)
 * to demonstrate the music playing commands.
 * 
 * Wiring Notes:
 * - DF1201S Module:
 *    • Connect DF1201S RX to Arduino Nano pin 11 (TX of SoftwareSerial)
 *    • Connect DF1201S TX to Arduino Nano pin 10 (RX of SoftwareSerial)
 *    • Ensure DF1201S VCC and GND are powered appropriately.
 * - Trigger Signal:
 *    • Use digital pin 4 on the Nano as input for the trigger.
 *    • Connect a controller’s output pin to digital pin 4 on the Nano.
 * - All devices must share a common ground.
 */

#include <SoftwareSerial.h>
#include <DFRobot_DF1201S.h>

// Use SoftwareSerial on pins 10 (RX) and 11 (TX) for DF1201S communication.
SoftwareSerial DF1201SSerial(10, 11);  // (RX, TX)

DFRobot_DF1201S DF1201S;

// Define the trigger pin (make sure this does not conflict with DF1201S serial pins)
const int triggerPin = 4;

// Flag to ensure the DF1201S song routine is executed only once per trigger event.
bool hasExecuted = false;

void setup() {
  // Initialize Serial for debugging at 115200 baud.
  Serial.begin(115200);
  
  // Begin SoftwareSerial for the DF1201S module communication.
  DF1201SSerial.begin(115200);
  
  // Initialize the DF1201S module. This loop continues until the module is detected.
  while (!DF1201S.begin(DF1201SSerial)) {
    Serial.println("DF1201S init failed, please check the wire connection!");
    delay(1000);
  }
  
  // Set the volume.
  DF1201S.setVol(15);
  Serial.print("Volume: ");
  Serial.println(DF1201S.getVol());
  
  // Switch the DF1201S into music mode.
  DF1201S.switchFunction(DF1201S.MUSIC);
  delay(2000);
  
  // Set playback mode to "repeat all" (ALLCYCLE).
  DF1201S.setPlayMode(DF1201S.ALLCYCLE);
  Serial.print("Playback Mode: ");
  Serial.println(DF1201S.getPlayMode());
  
  // Configure the trigger pin.
  pinMode(triggerPin, INPUT);
  
  Serial.println("Peripheral ready. Waiting for trigger signal on pin 4...");
}

void loop() {
  // Read the state of the trigger pin.
  int triggerState = digitalRead(triggerPin);
  
  // When the trigger is HIGH and the song routine has not yet executed...
  if (triggerState == HIGH && !hasExecuted) {
    Serial.println("Trigger received. Executing DF1201S song sequence...");
    runPeripheralCode();  // Run the DF1201S music commands.
    hasExecuted = true;
  }
  
  // Allow retriggering: reset the flag when the trigger goes LOW.
  if (triggerState == LOW && hasExecuted) {
    hasExecuted = false;
  }
  
  delay(100);  // Small delay for input stability.
}

// runPeripheralCode() contains the sequence of DF1201S commands to play songs.
void runPeripheralCode() {
  Serial.println("Start playing...");
  DF1201S.start();
  delay(3000);
  
  Serial.println("Pause playing...");
  DF1201S.pause();
  delay(3000);
  
  Serial.println("Play next song...");
  DF1201S.next();
  delay(3000);
  
  Serial.println("Play previous song...");
  DF1201S.last();
  delay(3000);
  
  Serial.println("Fast forward 10 seconds...");
  DF1201S.fastForward(10);  // Fast forward 10 seconds.
  
  // Retrieve and print various status information.
  Serial.print("File number: ");
  Serial.println(DF1201S.getCurFileNumber());
  
  Serial.print("Total number of files: ");
  Serial.println(DF1201S.getTotalFile());
  
  Serial.print("Current play time: ");
  Serial.println(DF1201S.getCurTime());
  
  Serial.print("Total duration of the current song: ");
  Serial.println(DF1201S.getTotalTime());
  
  Serial.print("Current file name: ");
  Serial.println(DF1201S.getFileName());
  delay(3000);
  
  // Start playing a specific file (file number 1 in this case).
  Serial.println("Playing file number 1...");
  DF1201S.playFileNum(1);
  
  // At this point, the module will continue to handle the music play.
}
