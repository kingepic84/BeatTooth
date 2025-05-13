# MusicClapplay.cpp

## Overview

**MusicClapplay.cpp** is an Arduino sketch that integrates:

* **DFRobot DF1201S** music module (DFPlayer) for audio playback
* **BMI270** accelerometer for clap detection
* **SD card** with the `BeatMap` class for dynamic beatmap loading
* **RGB LED feedback** on pins 8 (red), 9 (green), and 10 (blue)
* **Pushbuttons** on pins 2 (Restart), 3 (Play/Pause), and 4 (Skip)

This sketch performs the following:

1. Initializes the DFPlayer and SD card.
2. Loads beatmap data (BPM, time signature, and beat patterns) from a CSV file via `BeatMap`.
3. Plays the selected song via the DFPlayer module.
4. Detects claps using the BMI270 accelerometer, applying thresholds to distinguish on‑beat and off‑beat claps.
5. Updates and displays a running score.
6. Provides LED feedback:

   * **Green LED** for an on‑beat clap (good clap).
   * **Blue LED** for an off‑beat clap (clap detected but off beat).
   * **Red LED** for missed beats (no clap when expected).

---

# ble\_sd\_clap.cpp

## Overview

**ble\_sd\_clap.cpp** extends `MusicClapplay.cpp` by adding BLE server functionality with **ArduinoBLE** and robust beatmap support:

* **SD Library** (`<SD.h>`) initialization and error reporting on failure.
* **BeatMap** integration (`BeatMap.h` / `BeatMap.cpp`) to load song metadata, BPM, time signature, and dynamic beat patterns from CSV.
* **BLEService** and four **BLECharacteristic** objects for:

  * Song name
  * Real‑time score updates
  * Current playback time
  * Total song duration
* BLE **connect** and **disconnect** event handlers using `BLE.setEventHandler` to start/stop advertising.
* **Pattern buffer** using `std::array<int,4>` for fixed‑length beat patterns, refreshed each measure.

Key workflow:

1. Initialize DFPlayer, SD card, and load `BeatMap` from CSV on SD card root.
2. Set up BLE advertising with custom service and characteristics.
3. Respond to button presses (restart, skip, play/pause) to control playback and beatmap state.
4. Detect claps, update score, trigger LED feedback, and write to `scoreChar`.
5. Notify connected BLE central of playback time and final score.

---

# BLE\_clap.ino

## Overview

**BLE\_clap.ino** is a minimal demonstration of BLE notifications tied to clap detection:

* Configures **ArduinoBLE** to advertise a custom service UUID.
* Defines a single characteristic for clap event notifications.
* Detects a simple clap via the accelerometer (BMI270).
* Sends a BLE notification when a clap is detected.

This file highlights the core BLE setup and notification mechanism without full beatmap, SD card, or scoring logic.

---

# beat\_scan.py

## Overview

**beat\_scan.py** is a Python utility for analyzing accelerometer logs against expected beat intervals:

* Parses CSV or raw data files containing timestamped accelerometer readings.
* Accepts **BPM** and **time signature** parameters to compute expected beat times.
* Compares detected acceleration peaks (claps) to beat times.
* Outputs statistics on hit/miss counts and timing offsets.

### Usage

```bash
python beat_scan.py --input data.csv --bpm 80 --time-signature 4
```

Output includes:

* Total hits and misses
* Histogram of timing offsets
