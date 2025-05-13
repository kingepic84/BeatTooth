#include <DFRobot_DF1201S.h>
#include "DFSerial.h"
#include "mbed.h"
#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>

// Uncomment to enable debug prints
 #define DEBUG

// DF1201S music module setup:
DFSerial* DF1201SSerial;
DFRobot_DF1201S DF1201S;

// RGB LED Pins
const int redPin   = 8;
const int greenPin = 9;
const int bluePin  = 10;
// Button pins
const int btnRestart   = 2;
const int btnPlayPause = 3;
const int btnSkip      = 4;

// Two demo songs
const int songCount = 2;
int BPMs[songCount] = { 79, 65 };
const int patternLengths[songCount] = { 4, 4 };
bool clapPatterns[songCount][4] = {
  { true,  true,  true,  true  },
  { true,  false, true,  false }
};
int currentSong = 0;

// Beat & pattern
int BPM, patternLength;
unsigned long beatInterval;
unsigned long CLAP_LISTEN_DURATION;
unsigned long beatStartTime;
int patternIndex;
bool clapDetectedInBeat;

// Clap state-machine thresholds & state
const float RISE_THRESHOLD_G = 0.7;
const float FALL_THRESHOLD_G = 0.7;
const float OUT_THRESHOLD_G  = 0.4;
const unsigned long MAX_RISE_DURATION = 40;
const unsigned long MAX_OUT_DURATION  = 120;
const unsigned long CLAP_MARGIN_MS    = 370;

enum ClapState { WAIT_RISE, WAIT_FALL, WAIT_OUT };
ClapState clapState;
unsigned long phaseStart;
float prevAx, prevAy, prevAz;

// LED timing
const unsigned long LED_ON_DURATION = 150;
int currentLED = -1;
unsigned long ledOffTime;

// Playback state
bool isPlaying = false;
unsigned long pauseTimestamp;
bool dfPausedLED = false;

// Volume + button-hold detection
int currentVol = 5;
const unsigned long LONG_PRESS_THRESHOLD = 500;
unsigned long restartPressStart, skipPressStart;
bool restartWasPressed = false, restartLongActive = false;
bool skipWasPressed    = false, skipLongActive    = false;

// BLE definitions: four characteristics
BLEService songService("12345678-1234-1234-1234-1234567890ab");
BLECharacteristic songNameChar(
  "12345678-1234-1234-1234-1234567890ac",
  BLERead | BLENotify, 32
);
BLECharacteristic scoreChar(
  "12345678-1234-1234-1234-1234567890ad",
  BLERead | BLENotify, 2
);
BLECharacteristic timeChar(
  "12345678-1234-1234-1234-1234567890ae",
  BLERead | BLENotify, 2
);
BLECharacteristic totalTimeChar(
  "12345678-1234-1234-1234-1234567890af",
  BLERead | BLENotify, 2
);

int score = 0;
bool songPlaying = false;
char songNameBuf[32];

// Throttles
unsigned long lastEOSCheck = 0;
unsigned long lastTimeNotify = 0;

// — Helper Functions —
void triggerLED(int color) {
  currentLED = color;
  ledOffTime = millis() + LED_ON_DURATION;
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);
  switch (color) {
    case 0: digitalWrite(redPin, HIGH);   break;  // missed
    case 1: digitalWrite(greenPin, HIGH); break;  // on-beat
    case 2: digitalWrite(bluePin, HIGH);  break;  // off-beat
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

// Announce song start via BLE
void announceSongStart() {
  score = 0;
  songPlaying = true;

  // Song name
  String fn = DF1201S.getFileName();
  fn.toCharArray(songNameBuf, sizeof(songNameBuf));
  songNameChar.writeValue((const uint8_t*)songNameBuf, fn.length());
  if (BLE.connected()) songNameChar.writeValue((const uint8_t*)songNameBuf, fn.length());
#ifdef DEBUG
  Serial.print("BLE: Announced song start: "); Serial.println(songNameBuf);
#endif

  // Total time
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
  DF1201SSerial = new DFSerial(
    digitalPinToPinName(D1), digitalPinToPinName(D0), 115200
  );
  DF1201SSerial->logging = false;
  DF1201S.begin(*DF1201SSerial);
  DF1201S.setVol(currentVol);
  DF1201S.switchFunction(DF1201S.MUSIC);
  delay(2000);
  DF1201S.setPlayMode(DF1201S.ALLCYCLE);

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

  songNameChar.writeValue((const uint8_t*)"", 0);
  scoreChar.writeValue((const uint8_t*)"\0\0", 2);
  timeChar.writeValue((const uint8_t*)"\0\0", 2);
  totalTimeChar.writeValue((const uint8_t*)"\0\0", 2);

  BLE.advertise();
#ifdef DEBUG
  Serial.println("BLE advertising as beatTooth");
#endif
}

// — Main Loop —
void loop() {
  unsigned long now = millis();

  // Re-advertise if disconnected
  if (!BLE.connected()) {
    BLE.advertise();
#ifdef DEBUG
    Serial.println("BLE re-advertising");
#endif
  }

  //------------------------------------------------------------------
  // Restart button: short=vol-down, long=restart song
  //------------------------------------------------------------------
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
      loadSongParams();
      DF1201S.start();
      dfPausedLED = false;
      updateLED();
      isPlaying = true;
      announceSongStart();
      lastTimeNotify = now;
#ifdef DEBUG
      Serial.println("🔄 Song restarted");
#endif
      delay(50);
      return;
    }
  } else {
    if (restartWasPressed && !restartLongActive) {
      if (currentVol > 0) {
        currentVol--; DF1201S.setVol(currentVol);
#ifdef DEBUG
        Serial.print("🔉 Vol down to "); Serial.println(currentVol);
#endif
      }
    }
    restartWasPressed = false;
  }

  //------------------------------------------------------------------
  // Skip button: short=vol-up, long=skip song
  //------------------------------------------------------------------
  bool s = digitalRead(btnSkip) == LOW;
  if (s) {
    if (!skipWasPressed) {
      skipWasPressed = true;
      skipLongActive = false;
      skipPressStart = now;
    } else if (!skipLongActive && now - skipPressStart >= LONG_PRESS_THRESHOLD) {
      skipLongActive = true;
      DF1201S.next();
      currentSong = (currentSong + 1) % songCount;
      DF1201S.setPlayTime(0);
      if (currentSong == 1) DF1201S.setPlayTime(1);
      loadSongParams();
      DF1201S.start();
      dfPausedLED = false;
      updateLED();
      announceSongStart();
      lastTimeNotify = now;
#ifdef DEBUG
      Serial.print("⏭ Skipped to "); Serial.println(currentSong);
#endif
      delay(50);
      return;
    }
  } else {
    if (skipWasPressed && !skipLongActive) {
      if (currentVol < 30) {
        currentVol++; DF1201S.setVol(currentVol);
#ifdef DEBUG
        Serial.print("🔊 Vol up to "); Serial.println(currentVol);
#endif
      }
    }
    skipWasPressed = false;
  }

  //------------------------------------------------------------------
  // Play/Pause toggle
  //------------------------------------------------------------------
  if (digitalRead(btnPlayPause) == LOW) {
    if (isPlaying) {
      DF1201S.pause(); isPlaying = false;
      pauseTimestamp = now; dfPausedLED = true;
      digitalWrite(redPin, HIGH); digitalWrite(greenPin, LOW); digitalWrite(bluePin, HIGH);
#ifdef DEBUG
      Serial.println("⏸ Paused");
#endif
    } else {
      DF1201S.start(); isPlaying = true; dfPausedLED = false; updateLED();
      beatStartTime += (now - pauseTimestamp);
      if (!songPlaying) {
        announceSongStart();
        lastTimeNotify = now;
      }
#ifdef DEBUG
      Serial.println("▶️ Resumed");
#endif
    }
    delay(200);
  }
  if (!isPlaying) return;

  //------------------------------------------------------------------
  // Clap detection & scoring
  //------------------------------------------------------------------
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
          score += 2;
          triggerLED(1);
          uint8_t b[2] = { lowByte(score), highByte(score) };
          scoreChar.writeValue(b, 2);
#ifdef DEBUG
          Serial.println("Good beat");
#endif
        } else {
          score += 1;
          triggerLED(2);
          uint8_t b[2] = { lowByte(score), highByte(score) };
          scoreChar.writeValue(b, 2);
#ifdef DEBUG
          Serial.println("Off-beat clap");
#endif
        }
        clapState = WAIT_RISE;
      } else if (now - phaseStart > MAX_OUT_DURATION) clapState = WAIT_RISE;
      break;
  }

  //------------------------------------------------------------------
  // Beat evaluation
  //------------------------------------------------------------------
  if (now - beatStartTime >= beatInterval) {
    bool expected = clapPatterns[currentSong][patternIndex];
    if (expected && !clapDetectedInBeat) {
      if (score!=0){
        score--;
      }
      triggerLED(0);
      uint8_t b[2] = { lowByte(score), highByte(score) };
      scoreChar.writeValue(b, 2);
#ifdef DEBUG
      Serial.println("Missed beat");
#endif
    }
    patternIndex = (patternIndex + 1) % patternLength;
    beatStartTime += beatInterval;
    clapDetectedInBeat = false;
  }

  updateLED();

  //------------------------------------------------------------------
  // Notify current play time every second
  //------------------------------------------------------------------
  if (songPlaying && now - lastTimeNotify >= 1000) {
    lastTimeNotify = now;
    uint16_t ct = DF1201S.getCurTime();
    uint8_t buf[2] = { lowByte(ct), highByte(ct) };
    timeChar.writeValue(buf, 2);
#ifdef DEBUG
    Serial.print("BLE: Time notify = "); Serial.println(ct);
#endif
  }

  //------------------------------------------------------------------
  // End-of-song → Score notification and advance
  //------------------------------------------------------------------
  if (songPlaying && now - lastEOSCheck >= 500) {
    lastEOSCheck = now;
    if (DF1201S.getCurTime() >= DF1201S.getTotalTime()) {
      songPlaying = false;
      uint8_t b[2] = { lowByte(score), highByte(score) };
      scoreChar.writeValue(b, 2);
#ifdef DEBUG
      Serial.print("BLE: Final score = "); Serial.println(score);
#endif
      // Pause and prepare next
      DF1201S.pause();
      DF1201S.setPlayTime(0);
      currentSong = (currentSong + 1) % songCount;
      loadSongParams();
      isPlaying = false;
      dfPausedLED = true;
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
    }
  }
}

Here is a second iteration of the Arduino code with a new boatman system:

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
int BPMs[songCount] = { 79, 65 };
const int patternLengths[songCount] = { 4 , 4};
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
  Serial.println("After getbpm: ");
  Serial.println(beatMap.getBpm());
  patternLength        = beatMap.getTimeSignature();
  Serial.println("After patternlength: ");
  Serial.println(beatMap.getTimeSignature());
  beatInterval         = 60000UL / BPM;
  Serial.println("After beatinterval");
  CLAP_LISTEN_DURATION = (unsigned long)(beatInterval * 0.4);
  Serial.println("After clapduration");
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
  Serial.println("SD CARD: " + sdcard);
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
    isPlaying = false;
    // ← show purple until resumed
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
