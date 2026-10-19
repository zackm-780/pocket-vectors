#pragma once
// X-Plane 11 ".fms" (v1100) flight plan text format: parser and writer. Pure text <-> struct; no
// navigation database is involved here (see FlightPlan.h for building a FlightPlan from this).
// Format details and the conventions we rely on are in src/nav/README_FMS.md.

#include <string>
#include <string_view>
#include <vector>

namespace fsim {

enum class FmsEntryType { Airport = 1, Ndb = 2, Vor = 3, Fix = 11, LatLon = 28 };

struct FmsEntry {
    FmsEntryType type = FmsEntryType::Fix;
    std::string ident;
    std::string via = "DRCT";  // ADEP, ADES, DRCT or an airway name
    float altFt = 0;           // 0 = none (airports: elevation)
    double latDeg = 0, lonDeg = 0;
    int line = 0;  // 1-based source line when parsed, 0 when constructed in code
};

struct FmsFile {
    int version = 1100;
    std::string cycle;  // four digit AIRAC cycle, may be empty
    std::string adep, deprwy, sid, sidTrans;
    std::string ades, desrwy, star, starTrans, app, appTrans;
    std::string dep, des;  // DEP/DES: named nav point instead of an airport
    int numEnr = -1;       // NUMENR as read, -1 when absent
    std::vector<FmsEntry> entries;
};

struct FmsParseResult {
    bool ok = false;
    FmsFile file;
    std::vector<std::string> errors;    // "line N: ..." (empty when ok)
    std::vector<std::string> warnings;  // tolerated oddities (unknown keys, count mismatch)
};

// Parses .fms text (LF or CRLF, tabs or spaces, optional UTF-8 BOM). Collects every error it can
// find; ok is true only if there were none.
FmsParseResult parseFms(std::string_view text);

// Writes v1100 text with LF line endings. NUMENR is the number of entries.
std::string writeFms(const FmsFile& file);

// Type 28 idents are written as "+36.000_-121.000". Decodes that form (also "+36.0,-121.0" and
// "+36.0-121.0"); returns false if the ident is not a lat/lon ident.
bool decodeLatLonIdent(std::string_view ident, double& outLatDeg, double& outLonDeg);
std::string encodeLatLonIdent(double latDeg, double lonDeg);

// "RW28L" <-> "28L" (runway idents in the nav database have no "RW" prefix).
std::string fmsRunwayToDb(std::string_view fmsRunway);
std::string dbRunwayToFms(std::string_view dbRunway);

}  // namespace fsim
