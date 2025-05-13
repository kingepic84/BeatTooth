#include <DFRobot_DF1201S.h>
#include "DFSerial.h"
#include "BeatMap.h"
#include "mbed.h"
#include <SD.h>
#include <Arduino_BMI270_BMM150.h>   // built-in BMI270 accelerometer
#include <ArduinoBLE.h>

// Uncomment to enable debug prints
#define DEBUG

// DF1201S music module setup:
DFSerial*        DF1201SSerial;
DFRobot_DF1201S  DF1201S;

// I/O pins
const int redPin       = 8;
const int greenPin     = 9;
const int bluePin      = 10;
const int btnRestart   = 2;
const int btnPlayPause = 3;
const int btnSkip      = 4;

// SD / BeatMap
const int chipSelect = 5;
BeatMap   beatMap;

// Pattern buffer
#include <array>
std::array<int,4> currentPattern;

// Playback & scoring
bool           isPlaying      = false;
bool           songPlaying    = false;
unsigned long  pauseTimestamp = 0;
int            score          = 0;

// Beat timing
int            BPM            = 0;
int            patternLength  = 0;
int            patternIndex   = 0;
unsigned long  beatInterval   = 0;
unsigned long  CLAP_LISTEN_DURATION = 0;
unsigned long  beatStartTime  = 0;
bool           clapDetectedInBeat = false;

// Clap detector state-machine
enum ClapState { WAIT_RISE, WAIT_FALL, WAIT_OUT };
ClapState     clapState = WAIT_RISE;
unsigned long phaseStart = 0;
float         prevAx=0, prevAy=0, prevAz=0;

// Thresholds
const float   RISE_THRESHOLD_G     = 0.7;
const float   FALL_THRESHOLD_G     = 0.7;
const float   OUT_THRESHOLD_G      = 0.4;
const unsigned long MAX_RISE_DURATION = 40;
const unsigned long MAX_OUT_DURATION  = 120;
const unsigned long CLAP_MARGIN_MS    = 370;

// LED timing
const unsigned long LED_ON_DURATION = 150;
int                  currentLED     = -1;
unsigned long        ledOffTime     = 0;

// Volume + long-press detection
int            currentVol           = 5;
const unsigned long LONG_PRESS_THRESHOLD = 500;
bool           restartWasPressed    = false;
bool           restartLongActive    = false;
unsigned long  restartPressStart    = 0;
bool           skipWasPressed       = false;
bool           skipLongActive       = false;
unsigned long  skipPressStart       = 0;

// BLE service & characteristics
BLEService        songService    ("12345678-1234-1234-1234-1234567890ab");
BLECharacteristic songNameChar   ("12345678-1234-1234-1234-1234567890ac", BLERead | BLENotify, 32);
BLECharacteristic scoreChar      ("12345678-1234-1234-1234-1234567890ad", BLERead | BLENotify, 2);
BLECharacteristic timeChar       ("12345678-1234-1234-1234-1234567890ae", BLERead | BLENotify, 2);
BLECharacteristic totalTimeChar  ("12345678-1234-1234-1234-1234567890af", BLERead | BLENotify, 2);

unsigned long lastTimeNotify = 0;
unsigned long lastEOSCheck   = 0;

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
    digitalWrite(redPin,   LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin,  LOW);
    currentLED = -1;
  }
}

void loadSongParams() {
#ifdef DEBUG
  Serial.println(">> loadSongParams()");
#endif

  BPM           = beatMap.getBpm();
  patternLength = beatMap.getTimeSignature();

  if (BPM <= 0 || patternLength <= 0) {
    Serial.println("ERROR: invalid BPM or patternLength");
    while (1); // lock up so you notice!
  }

  beatInterval         = 60000UL / BPM;
  CLAP_LISTEN_DURATION = (unsigned long)(beatInterval * 0.4);
  beatStartTime        = millis();
  patternIndex         = 0;
  clapDetectedInBeat   = false;
  clapState            = WAIT_RISE;

  // copy pattern line into our fixed buffer
  auto line = beatMap.nextLine(0);
  for (int i = 0; i < patternLength && i < 4; i++) {
    currentPattern[i] = line[i];
  }

#ifdef DEBUG
  Serial.print("Loaded pattern: ");
  for (int i = 0; i < patternLength && i < 4; i++) {
    Serial.print(currentPattern[i]); Serial.print(' ');
  }
  Serial.println();
#endif
}

void announceSongStart() {
  score       = 0;
  songPlaying = true;

  // Song name from BeatMap
  String name = beatMap.getSongName();
  name.toCharArray((char*)songNameChar.value(), name.length());
  songNameChar.writeValue((uint8_t*)name.c_str(), name.length());
#ifdef DEBUG
  Serial.print("BLE: Announced song: "); Serial.println(name);
#endif

  // total time
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

  // SD init guard
  if (!SD.begin(chipSelect)) {
    Serial.println("ERROR: SD init failed");
    while (1);
  }
#ifdef DEBUG
  Serial.println("SD initialized OK");
#endif

  // load BeatMap
  beatMap = BeatMap::loadFromCSV(DF1201S.getFileName());
#ifdef DEBUG
  Serial.print("Loaded BeatMap: ");       Serial.println(beatMap.getSongName());
  Serial.print(" → BPM: ");               Serial.println(beatMap.getBpm());
  Serial.print(" → Time signature: ");    Serial.println(beatMap.getTimeSignature());
#endif

  // I/O pins
  pinMode(btnRestart,   INPUT_PULLUP);
  pinMode(btnPlayPause, INPUT_PULLUP);
  pinMode(btnSkip,      INPUT_PULLUP);
  pinMode(redPin,        OUTPUT);
  pinMode(greenPin,      OUTPUT);
  pinMode(bluePin,       OUTPUT);

  // IMU init
  if (!IMU.begin()) {
    Serial.println("ERROR: BMI270 init failed");
    while (1);
  }
  IMU.readAcceleration(prevAx, prevAy, prevAz);

  // prepare timing & pattern
  loadSongParams();
  pauseTimestamp = millis();

  // BLE init
  if (!BLE.begin()) {
    Serial.println("ERROR: BLE.begin() failed");
    while (1);
  }
  BLE.setLocalName("beatTooth");
  BLE.setAdvertisedService(songService);
  songService.addCharacteristic(songNameChar);
  songService.addCharacteristic(scoreChar);
  songService.addCharacteristic(timeChar);
  songService.addCharacteristic(totalTimeChar);
  BLE.addService(songService);
  BLE.setEventHandler(BLEConnected,    [](BLEDevice){ BLE.stopAdvertise();  });
  BLE.setEventHandler(BLEDisconnected, [](BLEDevice){ BLE.advertise();      });

  // initialize to zero
  songNameChar.writeValue((uint8_t*)"", 0);
  scoreChar.writeValue((uint8_t*)"\0\0", 2);
  timeChar.writeValue((uint8_t*)"\0\0", 2);
  totalTimeChar.writeValue((uint8_t*)"\0\0", 2);
  BLE.advertise();

#ifdef DEBUG
  Serial.println("BLE advertising as beatTooth");
#endif
}

// — Main Loop —

void loop() {
  unsigned long now = millis();

  // pump BLE event loop
  BLE.poll();

  // --- Restart button: short=vol-down, long=restart song ---
  bool r = digitalRead(btnRestart) == LOW;
  if (r) {
    if (!restartWasPressed) {
      restartWasPressed = true;
      restartLongActive = false;
      restartPressStart = now;
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
    if (restartWasPressed && !restartLongActive && currentVol>0) {
      currentVol--;
      DF1201S.setVol(currentVol);
    }
    restartWasPressed = false;
  }

  // --- Skip button: short=vol-up, long=skip song ---
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
    if (skipWasPressed && !skipLongActive && currentVol<30) {
      currentVol++;
      DF1201S.setVol(currentVol);
    }
    skipWasPressed = false;
  }

  // --- Play/Pause toggle ---
  if (digitalRead(btnPlayPause)==LOW) {
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
  if (!isPlaying) {
    pauseTimestamp = now;
    return;
  }

  // --- Clap detection & scoring ---
  float ax, ay, az;
  IMU.readAcceleration(ax, ay, az);
  float dx = ax - prevAx, dy = ay - prevAy, dz = az - prevAz;
  float mag = sqrt(dx*dx + dy*dy + dz*dz);
  prevAx = ax; prevAy = ay; prevAz = az;

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
        if (sinceBeat >= beatInterval-CLAP_MARGIN_MS && sinceBeat <= beatInterval) {
          // on-beat
          clapDetectedInBeat = true;
          score += 5;
          triggerLED(1);
        } else {
          // off-beat
          clapDetectedInBeat = true;
          score += 1;
          triggerLED(2);
        }
        // push score update
        uint8_t b[2] = { lowByte(score), highByte(score) };
        scoreChar.writeValue(b,2);
#ifdef DEBUG
        Serial.print("Score="); Serial.println(score);
#endif
        clapState = WAIT_RISE;
      } else if (now - phaseStart > MAX_OUT_DURATION) {
        clapState = WAIT_RISE;
      }
      break;
  }

  // --- Beat evaluation ---
  if (now - beatStartTime >= beatInterval) {
    bool expect = (currentPattern[patternIndex] != 0);
    if (expect && !clapDetectedInBeat) {
      // missed
      score = max(0, score - 2);
      triggerLED(0);
      uint8_t b[2] = { lowByte(score), highByte(score) };
      scoreChar.writeValue(b,2);
#ifdef DEBUG
      Serial.print("Missed, score="); Serial.println(score);
#endif
    }
    patternIndex++;
    if (patternIndex >= patternLength) {
      patternIndex = 0;
      // load next line
      auto next = beatMap.nextLine(DF1201S.getCurTime());
      for (int i=0; i<patternLength && i<4; i++) currentPattern[i] = next[i];
    }
    beatStartTime += beatInterval;
    clapDetectedInBeat = false;
  }

  updateLED();

  // --- Time notify every second ---
  if (songPlaying && now - lastTimeNotify >= 1000) {
    lastTimeNotify = now;
    uint16_t ct = DF1201S.getCurTime();
    uint8_t t2[2] = { lowByte(ct), highByte(ct) };
    timeChar.writeValue(t2,2);
#ifdef DEBUG
    Serial.print("Time="); Serial.println(ct);
#endif
  }

  // --- End‐of‐song check ---
  if (songPlaying && now - lastEOSCheck >= 500) {
    lastEOSCheck = now;
    if (DF1201S.getCurTime() >= DF1201S.getTotalTime()) {
      songPlaying = false;
      uint8_t f[2] = { lowByte(score), highByte(score) };
      scoreChar.writeValue(f,2);
#ifdef DEBUG
      Serial.print("Final score="); Serial.println(score);
#endif
      // prepare for next track
      beatMap = BeatMap::loadFromCSV(DF1201S.getFileName());
      loadSongParams();
      DF1201S.pause();
      DF1201S.setPlayTime(0);
      isPlaying    = false;
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
    }
  }
}
