#include <DFRobot_DF1201S.h>
#include "DFSerial.h"
#include "mbed.h"
#include <Arduino_BMI270_BMM150.h>   // built‑in BMI270 accelerometer

// DF1201S music module setup:
DFSerial* DF1201SSerial;
DFRobot_DF1201S DF1201S;

// RGB LED Pins
const int redPin   = 8, greenPin = 9, bluePin  = 10;

// Button pins
const int btnRestart   = 2, btnPlayPause = 3, btnSkip = 4;

// Two demo songs
const int songCount = 2;
int BPMs[songCount] = { 79, 120 };
const int patternLengths[songCount] = { 4 , 4};
bool clapPatterns[songCount][4] = {
  { true,  true,  true,  true  },
  { true,  false, true,  false }
};
int currentSong = 0;

// Beat & pattern
int BPM, patternLength;
unsigned long beatInterval, CLAP_LISTEN_DURATION, beatStartTime;
int patternIndex;
bool clapDetectedInBeat;

// Clap state‑machine (g’s & ms)
const float RISE_THRESHOLD_G     = 0.7;    // detect incoming strike
const float FALL_THRESHOLD_G     = 0.7;    // detect the “zero” at impact
const float OUT_THRESHOLD_G      = 0.4;    // detect rebound

const unsigned long MAX_RISE_DURATION = 40;   // ms to see the initial spike
const unsigned long MAX_OUT_DURATION  = 120;  // ms to see the rebound

const unsigned long CLAP_MARGIN_MS = 370;  // margin at end of beat

enum ClapState { WAIT_RISE, WAIT_FALL, WAIT_OUT };
ClapState clapState;
unsigned long phaseStart;

// previous acceleration
float prevAx, prevAy, prevAz;

// LED timing
const unsigned long LED_ON_DURATION = 150;
int currentLED = -1;
unsigned long ledOffTime;

// Playback
bool isPlaying;
unsigned long pauseTimestamp;

// Helpers
void triggerLED(int color) {
  currentLED = color;
  ledOffTime = millis() + LED_ON_DURATION;
  if (color == 0) {         // red
    digitalWrite(redPin,   HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin,  LOW);
  }
  else if (color == 1) {    // green
    digitalWrite(redPin,   LOW);
    digitalWrite(greenPin, HIGH);
    digitalWrite(bluePin,  LOW);
  }
  else if (color == 2) {    // blue
    digitalWrite(redPin,   LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin,  HIGH);
  }
}

void updateLED() {
  if (currentLED != -1 && millis() >= ledOffTime) {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
    currentLED = -1;
  }
}

void resetBeatTimer() {
  beatStartTime = millis();
  patternIndex = 0;
  clapDetectedInBeat = false;
  clapState = WAIT_RISE;
}

void loadSongParams() {
  BPM = BPMs[currentSong];
  patternLength = patternLengths[currentSong];
  beatInterval = 60000UL / BPM;
  CLAP_LISTEN_DURATION = (unsigned long)(beatInterval * 0.4);
  resetBeatTimer();
}

void setup() {
  Serial.begin(115200);

  // DF1201S init
  DF1201SSerial = new DFSerial(
    digitalPinToPinName(D1),
    digitalPinToPinName(D0),
    115200
  );
  DF1201SSerial->logging = false;
  Serial.println(DF1201S.begin(*DF1201SSerial));
  DF1201S.setVol(5);
  DF1201S.switchFunction(DF1201S.MUSIC);
  delay(2000);
  DF1201S.setPlayMode(DF1201S.ALLCYCLE);

  // Buttons
  pinMode(btnRestart,   INPUT_PULLUP);
  pinMode(btnPlayPause, INPUT_PULLUP);
  pinMode(btnSkip,      INPUT_PULLUP);

  // LEDs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  // IMU init
  if (!IMU.begin()) {
    Serial.println("BMI270 init failed!");
    while (1);
  }
  IMU.readAcceleration(prevAx, prevAy, prevAz);

  // Start first song
  loadSongParams();
  DF1201S.start();
  isPlaying = true;
  Serial.println("Demo song 0 started");
  delay(1000);
}

void loop() {
  unsigned long now = millis();

  // Button handling
  if (digitalRead(btnRestart) == LOW) {
    DF1201S.pause();
    DF1201S.setPlayTime(0);
    DF1201S.start();
    isPlaying = true;
    loadSongParams();
    Serial.println("Song restarted");
    delay(200);
  }
  if (digitalRead(btnPlayPause) == LOW) {
    if (isPlaying) {
      DF1201S.pause();
      isPlaying = false;
      pauseTimestamp = now;
      Serial.println("Paused");
    } else {
      DF1201S.start();
      isPlaying = true;
      unsigned long pausedFor = now - pauseTimestamp;
      beatStartTime += pausedFor;
      Serial.println("Resumed");
    }
    delay(200);
  }
  if (digitalRead(btnSkip) == LOW) {
    DF1201S.next();
    currentSong = (currentSong + 1) % songCount;
    loadSongParams();
    isPlaying = true;
    Serial.print("Switched to song ");
    Serial.println(currentSong);
    delay(200);
  }

  if (!isPlaying) return;

  // Clap detection via BMI270 & vector magnitude
  float ax, ay, az;
  IMU.readAcceleration(ax, ay, az);
  float dx = ax - prevAx, dy = ay - prevAy, dz = az - prevAz;
  float mag = sqrt(dx*dx + dy*dy + dz*dz);

  switch (clapState) {
    case WAIT_RISE:
      if (mag > RISE_THRESHOLD_G) {
        clapState = WAIT_FALL;
        phaseStart = now;
      }
      break;

    case WAIT_FALL:
      if (mag < FALL_THRESHOLD_G) {
        clapState = WAIT_OUT;
        phaseStart = now;
      } else if (now - phaseStart > MAX_RISE_DURATION) {
        clapState = WAIT_RISE;
      }
      break;

    case WAIT_OUT:
      if (mag > OUT_THRESHOLD_G) {
        unsigned long sinceBeat = now - beatStartTime;
        if (sinceBeat >= (beatInterval - CLAP_MARGIN_MS) && sinceBeat <= beatInterval) {
          // on‑beat: green later at beat
          clapDetectedInBeat = true;
          Serial.println("✅ TRUE CLAP on beat!");
        } else {
          // off‑beat within beat window: blue now
          Serial.println("🔵 Clap off‑beat");
          triggerLED(2);
        }
        clapState = WAIT_RISE;
      } else if (now - phaseStart > MAX_OUT_DURATION) {
        clapState = WAIT_RISE;
      }
      break;
  }

  prevAx = ax; prevAy = ay; prevAz = az;

  // Beat evaluation
  if (now - beatStartTime >= beatInterval) {
    bool expected = clapPatterns[currentSong][patternIndex];
    if (expected) {
      if (clapDetectedInBeat) {
        triggerLED(1);
        Serial.println("Good beat");
      } else {
        triggerLED(0);
        Serial.println("Missed beat");
      }
    } else if (clapDetectedInBeat) {
      triggerLED(0);
      Serial.println("Off‑beat clap");
    } else {
      Serial.println("Rest beat");
    }
    patternIndex = (patternIndex + 1) % patternLength;
    beatStartTime += beatInterval;
    clapDetectedInBeat = false;
  }

  updateLED();
}
