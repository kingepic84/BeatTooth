#include <DFRobot_DF1201S.h>
#include "DFSerial.h"
#include "mbed.h"
#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>
#include <SD.h>
#include "BeatMap.h"

// Uncomment to enable debug prints
#define DEBUG

// DF1201S music module
DFSerial*        DF1201SSerial;
DFRobot_DF1201S  DF1201S;

// I/O pins
const int redPin   = 8;
const int greenPin = 9;
const int bluePin  = 10;
const int btnRestart   = 2;
const int btnPlayPause = 3;
const int btnSkip      = 4;

// Playback state
bool           isPlaying   = false;
bool           songPlaying = false;
unsigned long  pauseTimestamp;

// BeatMap & pattern
const int     chipSelect = 5;
BeatMap       beatMap;
uint8_t       currentPattern[4];
int           BPM;
int           patternLength;
int           patternIndex;
unsigned long beatInterval;
unsigned long CLAP_LISTEN_DURATION;
unsigned long beatStartTime;
bool          clapDetectedInBeat;

// Clap detector state
enum ClapState { WAIT_RISE, WAIT_FALL, WAIT_OUT };
ClapState     clapState;
unsigned long phaseStart;
float         prevAx, prevAy, prevAz;

// Thresholds
const float   RISE_THRESHOLD_G       = 0.7;
const float   FALL_THRESHOLD_G       = 0.7;
const float   OUT_THRESHOLD_G        = 0.4;
const unsigned long MAX_RISE_DURATION = 40;
const unsigned long MAX_OUT_DURATION  = 120;
const unsigned long CLAP_MARGIN_MS    = 370;

// LED timing
const unsigned long LED_ON_DURATION = 150;
int                  currentLED     = -1;
unsigned long        ledOffTime;

// Volume & long-press
int           currentVol           = 5;
const unsigned long LONG_PRESS_THRESHOLD = 500;
bool          restartWasPressed = false;
bool          restartLongActive = false;
bool          skipWasPressed    = false;
bool          skipLongActive    = false;
unsigned long restartPressStart;
unsigned long skipPressStart;

// BLE service & characteristics
BLEService        songService   ("12345678-1234-1234-1234-1234567890ab");
BLECharacteristic songNameChar ("12345678-1234-1234-1234-1234567890ac", BLERead | BLENotify, 32);
BLECharacteristic scoreChar    ("12345678-1234-1234-1234-1234567890ad", BLERead | BLENotify, 2);
BLECharacteristic timeChar     ("12345678-1234-1234-1234-1234567890ae", BLERead | BLENotify, 2);
BLECharacteristic totalTimeChar("12345678-1234-1234-1234-1234567890af", BLERead | BLENotify, 2);

int            score           = 0;
char           songNameBuf[32];
unsigned long  lastTimeNotify  = 0;
unsigned long  lastEOSCheck    = 0;

// — Event Handlers —
void onCentralConnect(BLEDevice central) {
#ifdef DEBUG
  Serial.print("BLE: Central connected: ");
  Serial.println(central.address());
#endif
  BLE.stopAdvertise();
}

void onCentralDisconnect(BLEDevice central) {
#ifdef DEBUG
  Serial.print("BLE: Central disconnected: ");
  Serial.println(central.address());
#endif
  BLE.advertise();
}

// — Helpers —
void triggerLED(int color) {
  currentLED = color;
  ledOffTime = millis() + LED_ON_DURATION;
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);
  if (color == 0) digitalWrite(redPin, HIGH);
  else if (color == 1) digitalWrite(greenPin, HIGH);
  else if (color == 2) digitalWrite(bluePin, HIGH);
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
  beatStartTime      = millis();
  patternIndex       = 0;
  clapDetectedInBeat = false;
  clapState          = WAIT_RISE;
}

void loadSongParams() {
#ifdef DEBUG
  Serial.println("Loading parameters from BeatMap");
#endif
  BPM                  = beatMap.getBpm();
  patternLength        = beatMap.getTimeSignature();
  beatInterval         = 60000UL / BPM;
  CLAP_LISTEN_DURATION = (unsigned long)(beatInterval * 0.4);
  resetBeatTimer();
  memcpy(currentPattern, beatMap.nextLine(0).data(), patternLength * sizeof(uint8_t));
}

void announceSongStart() {
  score       = 0;
  songPlaying = true;

  String name = beatMap.getSongName();
  name.toCharArray(songNameBuf, sizeof(songNameBuf));
  songNameChar.writeValue((uint8_t*)songNameBuf, name.length());
#ifdef DEBUG
  Serial.print("BLE: Announced song: "); Serial.println(songNameBuf);
#endif

  uint16_t tot = DF1201S.getTotalTime();
  uint8_t tbuf[2] = { lowByte(tot), highByte(tot) };
  totalTimeChar.writeValue(tbuf, 2);
#ifdef DEBUG
  Serial.print("BLE: Total time = "); Serial.println(tot);
#endif

  lastTimeNotify = millis();
}

// — Setup —
void setup() {
  Serial.begin(115200);

  // DFPlayer init
  DF1201SSerial = new DFSerial(digitalPinToPinName(D1), digitalPinToPinName(D0), 115200);
  DF1201SSerial->logging = false;
  DF1201S.begin(*DF1201SSerial);
  DF1201S.setVol(currentVol);
  DF1201S.switchFunction(DF1201S.MUSIC);
  delay(2000);
  DF1201S.setPlayMode(DF1201S.ALLCYCLE);

  // SD + BeatMap
  bool sdOK = SD.begin(chipSelect);
#ifdef DEBUG
  Serial.print("SD init: "); Serial.println(sdOK);
#endif
  beatMap = BeatMap::loadFromCSV(DF1201S.getFileName());
#ifdef DEBUG
  Serial.print("Loaded BeatMap: "); Serial.println(beatMap.getSongName());
#endif

  // I/O
  pinMode(btnRestart, INPUT_PULLUP);
  pinMode(btnPlayPause, INPUT_PULLUP);
  pinMode(btnSkip, INPUT_PULLUP);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // IMU init
  if (!IMU.begin()) while (1);
  IMU.readAcceleration(prevAx, prevAy, prevAz);

  loadSongParams();
  pauseTimestamp = millis();

  // BLE init
  if (!BLE.begin()) while (1);
  BLE.setLocalName("beatTooth");
  BLE.setAdvertisedService(songService);
  songService.addCharacteristic(songNameChar);
  songService.addCharacteristic(scoreChar);
  songService.addCharacteristic(timeChar);
  songService.addCharacteristic(totalTimeChar);
  BLE.addService(songService);

  songNameChar.writeValue((uint8_t*)"", 0);
  scoreChar.writeValue((uint8_t*)"\0\0", 2);
  timeChar.writeValue((uint8_t*)"\0\0", 2);
  totalTimeChar.writeValue((uint8_t*)"\0\0", 2);

  BLE.setEventHandler(BLEConnected, onCentralConnect);
  BLE.setEventHandler(BLEDisconnected, onCentralDisconnect);
  BLE.advertise();
}

// — Main Loop —
void loop() {
  unsigned long now = millis();

  // Process BLE events
  BLE.poll();

  // Restart button
  bool r = digitalRead(btnRestart) == LOW;
  if (r) {
    if (!restartWasPressed) {
      restartWasPressed   = true;
      restartLongActive   = false;
      restartPressStart   = now;
    } else if (!restartLongActive && now - restartPressStart >= LONG_PRESS_THRESHOLD) {
      restartLongActive = true;
      DF1201S.pause();
      DF1201S.setPlayTime(0);
      beatMap = BeatMap::loadFromCSV(DF1201S.getFileName());
      loadSongParams();
      DF1201S.start();
      announceSongStart();
      delay(50);
      return;
    }
  } else {
    if (restartWasPressed && !restartLongActive && currentVol > 0) {
      currentVol--;
      DF1201S.setVol(currentVol);
    }
    restartWasPressed = false;
  }

  // Skip button
  bool s = digitalRead(btnSkip) == LOW;
  if (s) {
    if (!skipWasPressed) {
      skipWasPressed  = true;
      skipLongActive  = false;
      skipPressStart  = now;
    } else if (!skipLongActive && now - skipPressStart >= LONG_PRESS_THRESHOLD) {
      skipLongActive = true;
      DF1201S.next();
      DF1201S.setPlayTime(0);
      beatMap = BeatMap::loadFromCSV(DF1201S.getFileName());
      loadSongParams();
      DF1201S.start();
      announceSongStart();
      delay(50);
      return;
    }
  } else {
    if (skipWasPressed && !skipLongActive && currentVol < 30) {
      currentVol++;
      DF1201S.setVol(currentVol);
    }
    skipWasPressed = false;
  }

  // Play/Pause toggle
  if (digitalRead(btnPlayPause) == LOW) {
    if (isPlaying) {
      DF1201S.pause();
      isPlaying    = false;
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
    } else {
      DF1201S.start();
      isPlaying    = true;
      beatStartTime += (now - pauseTimestamp);
      if (!songPlaying) announceSongStart();
    }
    delay(200);
  }
  if (!isPlaying) return;

  // Clap detection
  float ax, ay, az;
  IMU.readAcceleration(ax, ay, az);
  float dx = ax - prevAx, dy = ay - prevAy, dz = az - prevAz;
  float mag = sqrt(dx*dx + dy*dy + dz*dz);
  prevAx = ax; prevAy = ay; prevAz = az;

  switch (clapState) {
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
          score += 5;
          triggerLED(1);
        } else {
          clapDetectedInBeat = true;
          score += 1;
          triggerLED(2);
        }
        // update score characteristic
        uint8_t b[2] = { lowByte(score), highByte(score) };
        scoreChar.writeValue(b, 2);
#ifdef DEBUG
        Serial.print("Score update: "); Serial.println(score);
#endif
        clapState = WAIT_RISE;
      } else if (now - phaseStart > MAX_OUT_DURATION) clapState = WAIT_RISE;
      break;
  }

  // Beat evaluation
  if (now - beatStartTime >= beatInterval) {
    bool expected = (currentPattern[patternIndex] != 0);
    if (expected && !clapDetectedInBeat) {
      score = (score >= 2 ? score - 2 : 0);
      triggerLED(0);
      uint8_t b[2] = { lowByte(score), highByte(score) };
      scoreChar.writeValue(b, 2);
#ifdef DEBUG
      Serial.print("Missed beat, score: "); Serial.println(score);
#endif
    }
    patternIndex++;
    if (patternIndex >= patternLength) {
      patternIndex = 0;
      memcpy(currentPattern, beatMap.nextLine(DF1201S.getCurTime()).data(), patternLength * sizeof(uint8_t));
    }
    beatStartTime += beatInterval;
    clapDetectedInBeat = false;
  }

  updateLED();

  // Time notify every second
  if (songPlaying && now - lastTimeNotify >= 1000) {
    lastTimeNotify = now;
    uint16_t ct = DF1201S.getCurTime();
    uint8_t t2[2] = { lowByte(ct), highByte(ct) };
    timeChar.writeValue(t2, 2);
#ifdef DEBUG
    Serial.print("Time notify: "); Serial.println(ct);
#endif
  }

  // End-of-song
  if (songPlaying && now - lastEOSCheck >= 500) {
    lastEOSCheck = now;
    if (DF1201S.getCurTime() >= DF1201S.getTotalTime()) {
      songPlaying = false;
      uint8_t b[2] = { lowByte(score), highByte(score) };
      scoreChar.writeValue(b, 2);
#ifdef DEBUG
      Serial.print("Final score: "); Serial.println(score);
#endif
      beatMap = BeatMap::loadFromCSV(DF1201S.getFileName());
      loadSongParams();
      DF1201S.pause();
      DF1201S.setPlayTime(0);
      isPlaying = false;
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
    }
  }
}
