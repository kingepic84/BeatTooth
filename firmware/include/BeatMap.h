#ifndef BeatMap_h
#define BeatMap_h

#include <Arduino.h>
#include <SD.h>
#include <vector>
#include <array>
#include <utility>

using namespace std;
class BeatMap {
public:
  BeatMap();
  
  static BeatMap loadFromCSV(const String &songFileName);

  void setSongName(const String &name);
  String getSongName() const;

  void setBpm(int b);
  int getBpm() const;

  void setTimeSignature(int num, int den);
  int getTimeSignature() const;

  void setBpmMap(const vector<pair<int,int>> &map);
  const vector<pair<int,int>>& getBpmMap() const;

  void setBeatMap(const vector<array<int,4>> &map);
  const vector<array<int,4>>& getBeatMap() const;

  bool isSongOver() const;
  array<int,4> nextLine(int currentTimeSec);

private:
  String songName;
  int bpm;
  int timeSig[2];
  vector<pair<int,int>> bpmMap;
  vector<array<int,4>> beatmap;
  size_t currentIndex;
};

#endif // BeatMap_h