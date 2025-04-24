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
    String path = "/" + songFileName + ".csv";
    File f = SD.open(path);
    if (!f)
        return bm;

    String header = f.readStringUntil('\n');
    header.trim();
    auto tokens = splitString(header, ',');
    int idx = 0;
    bm.setBpm(tokens[idx++].toInt());
    int num = tokens[idx++].toInt();
    int den = tokens[idx++].toInt();
    bm.setTimeSignature(num, den);
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

void BeatMap::setSongName(const String &name) { songName = name; }
String BeatMap::getSongName() const { return songName; }

void BeatMap::setBpm(int b) { bpm = b; }
int BeatMap::getBpm() const { return bpm; }

void BeatMap::setTimeSignature(int num, int den)
{
    timeSig[0] = num;
    timeSig[1] = den;
}
void BeatMap::getTimeSignature(int &num, int &den) const
{
    num = timeSig[0];
    den = timeSig[1];
}

void BeatMap::setBpmMap(const vector<pair<int, int>> &map) { bpmMap = map; }
const vector<pair<int, int>> &BeatMap::getBpmMap() const { return bpmMap; }

void BeatMap::setBeatMap(const vector<array<int, 4>> &map) { beatmap = map; }
const vector<array<int, 4>> &BeatMap::getBeatMap() const { return beatmap; }

bool BeatMap::isSongOver() const { return currentIndex >= beatmap.size(); }

array<int, 4> BeatMap::nextLine(int currentTimeSec)
{
    for (auto it = bpmMap.begin(); it != bpmMap.end(); ++it)
    {
        if (currentTimeSec >= it->first)
        {
            setBpm(it->second);
            bpmMap.erase(it);
            break;
        }
    }
    if (isSongOver())
        return {0, 0, 0, 0};
    return beatmap[currentIndex++];
}

int main()
{
    BeatMap bm = BeatMap();
}