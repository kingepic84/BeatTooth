#BeatTooth Project

##Overview

BeatTooth is a rhythm-based interactive project using an Arduino-compatible board, a DFPlayer MP3 module, an IMU sensor (BMI270), an SD card for beatmap storage, and Bluetooth® Low Energy (BLE) communication. It plays demo songs, detects claps on the beat, scores the user, and streams real-time song and score data to a macOS GUI application.

LED Feedback:
	•	Green LED lights on an on‑beat clap (good clap).
	•	Blue LED lights on an off‑beat clap (clap detected but off beat).
	•	Red LED lights for a missed beat (no clap when expected).

├── README.md
└── firmware
   └── src
       ├── BeatMap.h
       ├── BeatMap.cpp
       ├── complete.ino
       ├── MusicClapplay.ino
       ├── BLEclap.ino
       ├── any other Arduino .ino, .h, .cpp, and SD/DFPlayer helpers
       └── beat_scan.py          ← macOS GUI (PyQt + Bleak)
Components
	•	complete.ino: Combines music playback (DFPlayer), clap detection (BMI270), dynamic beatmap loading (SD + BeatMap), and BLE service for streaming song name, score, playback time, and total duration. LEDs indicate clap feedback: green for on‑beat clap, blue for off‑beat clap, red for missed beat.
	•	BLE_clap.ino: Simplified BLE-enabled sketch demonstrating clap detection and BLE notifications.
	•	beat_scan.py: Python utility for parsing and analyzing recorded accelerometer data against beat intervals.
	•	BeatMap.cpp / BeatMap.h: C++ classes that load beatmaps (time signatures, BPM, patterns, song metadata) from CSV files on the SD card.
	•	MusicClapplay.ino: The original Arduino demo playing back two hardcoded patterns without BLE or SD card.
	•	BeatToothMacApp.py (in src/): macOS Qt application to visualize and record leaderboard, receiving BLE notifications from the Arduino.

Prerequisites
	•	Hardware:
	•	Arduino-compatible board with BLE support (e.g., Nano 33 BLE Sense)
	•	DFRobot DF1201S (DFPlayer) module
	•	SD card module and microSD card with beatmap CSV files
	•	BMI270/BMM150 IMU or compatible accelerometer
	•	RGB LEDs and pushbuttons wired as per pin definitions (LEDs on pins 8,9,10; buttons on 2,3,4)
	•	Software:
	•	Arduino IDE (>=1.8) or PlatformIO
	•	Python 3.7+ with bleak, PyQt5, sqlite3 for the macOS app

Setup & Usage

Arduino Sketch
	1.	Copy complete.ino, BeatMap.cpp, and BeatMap.h into an Arduino sketch folder.
	2.	Insert CSV beatmap files into the SD card root. Example CSV format:

# song_name,bpm,time_signature
demo_song.mp3,80,4
# then each line: pattern of 0/1 for each beat
1,0,1,0
1,1,1,1


	3.	Wire hardware:
	•	DFPlayer TX/RX to D1/D0
	•	SD module CS to pin 5
	•	IMU on I²C pins
	•	Buttons on 2,3,4 with INPUT_PULLUP
	•	LEDs on 8 (red), 9 (green), 10 (blue)
	4.	Compile & upload via Arduino IDE.
	5.	Open Serial Monitor at 115200 baud to see debug logs (enable #define DEBUG).

macOS GUI
	1.	Create a Python virtual environment:

python3 -m venv venv
source venv/bin/activate
pip install bleak PyQt5


	2.	Run the GUI:

python src/BeatToothMacApp.py


	3.	The app will scan for BLE device named beatTooth, subscribe to characteristics, and display:
	•	Connection indicator (red/green dot)
	•	Now playing song title
	•	Progress bar (current time / total duration)
	•	Score and name entry
	•	Leaderboard (SQLite database leaderboard.db)

Troubleshooting
	•	SD Card fails to initialize: Check CS pin wiring (5) and card formatting (FAT16/32). If initialization fails, the sketch prints SD card init failed to Serial.
	•	BLE not connecting: Ensure Arduino advertises service UUID 12345678-1234-1234-1234-1234567890ab and characteristic UUIDs match the Mac app.
	•	Clap detection unstable: Tune RISE_THRESHOLD_G, FALL_THRESHOLD_G, and timing margins in sketch.

License

MIT © Your Name
