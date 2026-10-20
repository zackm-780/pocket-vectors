#pragma once
// X1: METAR parser. Reads one raw METAR/SPECI line as served by
// https://aviationweather.gov/api/data/metar?ids=KPDX&format=raw and extracts what the sim uses:
// wind (incl. VRB and gusts), visibility, cloud layers, temperature/dewpoint and altimeter.
// Free of I/O and GL; tolerant of missing or unknown groups (they are skipped).

#include <string>
#include <vector>

namespace fsim::world {

struct MetarCloud {
    enum class Cover { Few, Scattered, Broken, Overcast, VerticalVis };
    Cover cover = Cover::Few;
    float baseAglFt = 0;
};

struct Metar {
    std::string station;  // "KPDX"
    int day = 0, hour = 0, minute = 0;

    bool hasWind = false;
    bool windVariable = false;  // VRB
    float windFromTrueDeg = 0;  // METAR winds are true
    float windKt = 0;
    float gustKt = 0;  // 0 = no gusts reported

    bool hasVisibility = false;      // false: no group seen
    float visibilityM = 0;           // 9999 and CAVOK are reported as 10000 (">= 10 km")
    bool visibilityAtLeast = false;  // 9999 / CAVOK / P6SM: "or more"
    bool cavok = false;

    std::vector<MetarCloud> clouds;
    float ceilingFt = -1;  // lowest BKN/OVC/VV base, -1 = none

    bool hasTemp = false;
    float tempC = 0, dewPointC = 0;
    bool hasDewPoint = false;

    bool hasAltimeter = false;
    float altimeterHpa = 0;  // from A (inHg) or Q (hPa)

    bool valid() const { return !station.empty() && (hasWind || hasVisibility || hasAltimeter); }
};

// Parses the first METAR/SPECI report found in `text` (extra lines, "METAR" prefix and the
// remarks section are ignored). Returns false when no usable report was found.
bool parseMetar(const std::string& text, Metar& out);

}  // namespace fsim::world
