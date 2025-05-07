#include "BeatMap.h"

using namespace std;


static vector<String> splitString(const String &str, char delim)
{
    vector<String> elems;
    int start = 0;
    int idx;
    while ((idx = str.indexOf(delim, start)) >= 0)
    {
        elems.push_back(str.substring(start, idx));
        start = idx + 1;
    }
    if (start < str.length())
    {
        elems.push_back(str.substring(start));
    }
    return elems;
}

BeatMap::BeatMap() : songName(""), bpm(0), timeSig{0, 0}, currentIndex(0) {}

BeatMap BeatMap::loadFromCSV(const String &songFileName)
{
    BeatMap bm;
    String songFName = songFileName.substring(0, songFileName.lastIndexOf("."));
    songFName.trim();
    String path = "/" + songFName + ".csv";
    File f = SD.open(path);
    if (!f){
        Serial.println("Inside not file return!");
        return bm;
    }
    String header = f.readStringUntil('\n');
    Serial.println(header);
    header.trim();
    auto tokens = splitString(header, ',');
    int idx = 0;
    for (int i = 0; i < 4; i++){
        Serial.println(tokens[i]);
    }
    bm.setBpm(tokens[idx++].toInt());
    int num = tokens[idx++].toInt();
    int den = tokens[idx++].toInt();
    bm.setTimeSignature(num, den);
    Serial.println(bm.getBpm());
    Serial.println(bm.getTimeSignature());
    bm.setSongName(tokens[idx++]);
    vector<pair<int, int>> changes;
    while (idx + 1 < (int)tokens.size())
    {
        changes.emplace_back(tokens[idx++].toInt(), tokens[idx++].toInt());
    }
    bm.setBpmMap(changes);
    vector<array<int, 4>> measures;
    while (f.available())
    {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0)
            continue;
            auto parts = splitString(line, ',');
            if (parts.size() < 4)
            continue;
            array<int, 4> m;
        for (int i = 0; i < 4; ++i)
        m[i] = parts[i].toInt();
        measures.push_back(m);
    }
    f.close();
    bm.setBeatMap(measures);
    return bm;
}

void BeatMap::setSongName(const String &name) { this->songName = name; }
String BeatMap::getSongName() const { return this->songName; }

void BeatMap::setBpm(int b) { this->bpm = b; }
int BeatMap::getBpm() { return this->bpm; }

void BeatMap::setTimeSignature(int num, int den)
{
    this->timeSig[0] = num;
    this->timeSig[1] = den;
}

int BeatMap::getTimeSignature() {
    return this->timeSig[0];
}

void BeatMap::setBpmMap(const vector<pair<int, int>> &map) { this->bpmMap = map; }
const vector<pair<int, int>> &BeatMap::getBpmMap() const { return this->bpmMap; }

void BeatMap::setBeatMap(const vector<array<int, 4>> &map) { this->beatmap = map; }
const vector<array<int, 4>> &BeatMap::getBeatMap() const { return this->beatmap; }

bool BeatMap::isSongOver() const { return currentIndex >= this->beatmap.size(); }

array<int, 4> BeatMap::nextLine(int currentTimeSec)
{
    for (auto it = this->bpmMap.begin(); it != this->bpmMap.end(); ++it)
    {
        if (currentTimeSec >= it->first)
        {
            setBpm(it->second);
            this->bpmMap.erase(it);
            break;
        }
    }
    if (isSongOver())
        return {0, 0, 0, 0};
    return this->beatmap[currentIndex++];
}