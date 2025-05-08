#include <DFRobot_DF1201S.h>
#include "DFSerial.h"
#include "BeatMap.h"
#include "mbed.h"
#include <SD.h>
#include <Arduino_BMI270_BMM150.h>   // built-in BMI270 accelerometer

// DF1201S music module setup:
DFSerial* DF1201SSerial;
DFRobot_DF1201S DF1201S;
using namespace std;
// RGB LED Pins
const int redPin   = 8, greenPin = 9, bluePin  = 10;
// Button pins
const int btnRestart   = 2, btnPlayPause = 3, btnSkip = 4;

// Two demo songs
const int songCount = 2;
array<int, 4> clapPatterns;

int currentSong = 0;

// Beat & pattern
int BPM, patternLength;
unsigned long beatInterval, CLAP_LISTEN_DURATION, beatStartTime;
int patternIndex;
int chipSelect = 5;
bool clapDetectedInBeat;

// Clap state-machine (g’s & ms)
const float RISE_THRESHOLD_G     = 0.7;
const float FALL_THRESHOLD_G     = 0.7;
const float OUT_THRESHOLD_G      = 0.4;
const unsigned long MAX_RISE_DURATION = 40;
const unsigned long MAX_OUT_DURATION  = 120;
const unsigned long CLAP_MARGIN_MS    = 370;

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
bool dfPausedLED = false;

// Volume + button-hold detection
int currentVol = 5;
const unsigned long LONG_PRESS_THRESHOLD = 500;
unsigned long restartPressStart, skipPressStart;
bool restartWasPressed = false, restartLongActive = false;
bool skipWasPressed    = false, skipLongActive    = false;
BeatMap beatMap;
// — Helpers —

void triggerLED(int color) {
  currentLED = color;
  ledOffTime = millis() + LED_ON_DURATION;
  digitalWrite(redPin,   LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin,  LOW);
  switch(color) {
    case 0: digitalWrite(redPin,   HIGH); break;  // red
    case 1: digitalWrite(greenPin, HIGH); break;  // green
    case 2: digitalWrite(bluePin,  HIGH); break;  // blue
  }
}

void updateLED() {
  if (dfPausedLED) return;
  if (currentLED != -1 && millis() >= ledOffTime) {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
    currentLED = -1;
  }
}

void resetBeatTimer() {
  beatStartTime       = millis();
  patternIndex        = 0;
  clapDetectedInBeat  = false;
  clapState           = WAIT_RISE;
}

void loadSongParams() {
  Serial.println("Before getbpm");
  BPM                  = beatMap.getBpm();
  patternLength        = beatMap.getTimeSignature();
  beatInterval         = 60000UL / BPM;
  CLAP_LISTEN_DURATION = (unsigned long)(beatInterval * 0.4);
  resetBeatTimer();
  return;
}

// — Setup —

void setup() {
  Serial.begin(115200);

  DF1201SSerial = new DFSerial(
    digitalPinToPinName(D1),
    digitalPinToPinName(D0),
    115200
  );
  bool sdcard = SD.begin(chipSelect);
  DF1201SSerial->logging = false;
  DF1201S.begin(*DF1201SSerial);
  DF1201S.setVol(currentVol);
  DF1201S.switchFunction(DF1201S.MUSIC);
  delay(2000);
  DF1201S.setPlayMode(DF1201S.ALLCYCLE);

  pinMode(btnRestart,   INPUT_PULLUP);
  pinMode(btnPlayPause, INPUT_PULLUP);
  pinMode(btnSkip,      INPUT_PULLUP);
  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);

  if (!IMU.begin()) {
    Serial.println("BMI270 init failed!");
    while (1);
  }
  IMU.readAcceleration(prevAx, prevAy, prevAz);
  Serial.println("Init beatMap");
  beatMap = BeatMap::loadFromCSV(DF1201S.getFileName());
  Serial.println("Init song params");
  loadSongParams();
  Serial.println("After song params");
  // don't auto-start; wait for play button
  isPlaying = false;
  pauseTimestamp = millis();
}

// — Main Loop —

void loop() {
  unsigned long now = millis();

  // --- Restart button: short=vol-down, long=restart song ---
  bool r = digitalRead(btnRestart)==LOW;
  if (r) {
    if (!restartWasPressed) {
      restartWasPressed = true;
      restartLongActive = false;
      restartPressStart = now;
    } else if (!restartLongActive && now - restartPressStart >= LONG_PRESS_THRESHOLD) {
      restartLongActive = true;
      // long-press → restart current
      DF1201S.pause();
      DF1201S.setPlayTime(0);
      beatMap = BeatMap::loadFromCSV(DF1201S.getFileName());
      loadSongParams();
      DF1201S.start();
      dfPausedLED = false;
      updateLED();
      isPlaying = true;
      Serial.println("🔄 Song restarted");
      delay(100);
      return;
    }
  } else {
    if (restartWasPressed && !restartLongActive) {
      // short-press → volume down
      if (currentVol > 0) {
        currentVol--;
        DF1201S.setVol(currentVol);
        Serial.print("🔉 Vol down to "); Serial.println(currentVol);
      }
    }
    restartWasPressed = false;
  }

  // --- Skip button: short=vol-up, long=skip song ---
  bool s = digitalRead(btnSkip)==LOW;
  if (s) {
    if (!skipWasPressed) {
      skipWasPressed = true;
      skipLongActive = false;
      skipPressStart = now;
    } else if (!skipLongActive && now - skipPressStart >= LONG_PRESS_THRESHOLD) {
      skipLongActive = true;
      // long-press → skip
      DF1201S.next();
      currentSong = (currentSong+1)%songCount;
      DF1201S.setPlayTime(0);
      if (currentSong==1) {DF1201S.setPlayTime(1);}
      beatMap = BeatMap::loadFromCSV(DF1201S.getFileName());
      loadSongParams();
      DF1201S.start();
      dfPausedLED = false;
      updateLED();
      isPlaying = true;
      Serial.print("⏭ Skipped to "); 
      Serial.println(currentSong);
      delay(100);
      return;
    }
  } else {
    if (skipWasPressed && !skipLongActive) {
      // short-press → volume up
      if (currentVol < 30) {
        currentVol++;
        DF1201S.setVol(currentVol);
        Serial.print("🔊 Vol up to "); 
        Serial.println(currentVol);
      }
    }
    skipWasPressed = false;
  }

  // --- Detect end-of-track: advance + pause at 0 + delay if song 2 ---
  if (isPlaying && DF1201S.getCurTime() >= DF1201S.getTotalTime()) {
    Serial.println("▶️ Song ended");
    // next song, paused at zero
    currentSong = (currentSong+1)%songCount;
    DF1201S.setPlayTime(0);
    DF1201S.pause();
    beatMap.loadFromCSV(DF1201S.getFileName());
    loadSongParams();
    clapPatterns = beatMap.nextLine(DF1201S.getCurTime());
    Serial.println("Next line...");
    Serial.println(clapPatterns.data()[0]);
    Serial.println(clapPatterns.data()[1]);
    Serial.println(clapPatterns.data()[2]);
    Serial.println(clapPatterns.data()[3]);
    isPlaying = false; // ← show purple until resumed
    dfPausedLED = true;
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, HIGH);
  }

  // --- Play/Pause toggle ---
  if (digitalRead(btnPlayPause)==LOW) {
    if (isPlaying) {
      DF1201S.pause();
      isPlaying = false;
      pauseTimestamp = now;
      // latch on solid purple:
      dfPausedLED = true;
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
      Serial.println("⏸ Paused");
    } else {
      DF1201S.start();
      isPlaying = true;
      // clear the latched purple:
      dfPausedLED = false;
      updateLED();   // immediately turn it off
      unsigned long pausedFor = now - pauseTimestamp;
      beatStartTime  += pausedFor;
      Serial.println("▶️ Resumed");
    }
    delay(400);
  }
  if (!isPlaying) return;

  // --- Clap detection & beat logic unchanged ---

  float ax, ay, az;
  IMU.readAcceleration(ax, ay, az);
  float dx = ax - prevAx, dy = ay - prevAy, dz = az - prevAz;
  float mag = sqrt(dx*dx + dy*dy + dz*dz);

  switch(clapState) {
    case WAIT_RISE:
      if (mag > RISE_THRESHOLD_G) { clapState = WAIT_FALL; phaseStart = now; }
      break;
    case WAIT_FALL:
      if (mag < FALL_THRESHOLD_G) { clapState = WAIT_OUT; phaseStart = now; }
      else if (now - phaseStart > MAX_RISE_DURATION) clapState = WAIT_RISE;
      break;
    case WAIT_OUT:
      if (mag > OUT_THRESHOLD_G) {
        unsigned long sinceBeat = now - beatStartTime;
        if (sinceBeat >= (beatInterval - CLAP_MARGIN_MS) && sinceBeat <= beatInterval) {
          clapDetectedInBeat = true;
          Serial.println("✅ TRUE CLAP");
        } else {
          Serial.println("🔵 Off-beat");
          triggerLED(2);
        }
        clapState = WAIT_RISE;
      } else if (now - phaseStart > MAX_OUT_DURATION) {
        clapState = WAIT_RISE;
      }
      break;
  }
  prevAx = ax; prevAy = ay; prevAz = az;
  // beat evaluation
  if (now - beatStartTime >= beatInterval) {
    if(patternIndex+1 == patternLength){
        clapPatterns = beatMap.nextLine(DF1201S.getCurTime());
        Serial.println("Next line...");
        Serial.println(clapPatterns.data()[0]);
        Serial.println(clapPatterns.data()[1]);
        Serial.println(clapPatterns.data()[2]);
        Serial.println(clapPatterns.data()[3]);
    }
    bool expected = clapPatterns.at(patternIndex);
    if (expected) {
      if (clapDetectedInBeat) {
        triggerLED(1); Serial.println("Good beat");
      } else {
        triggerLED(0); Serial.println("Missed beat");
      }
    } else if (clapDetectedInBeat) {
      triggerLED(0); Serial.println("Off-beat clap");
    } else {
      Serial.println("Rest beat");
    }
    patternIndex = (patternIndex + 1) % patternLength;
    beatStartTime += beatInterval;
    clapDetectedInBeat = false;
  }

  updateLED();
}
