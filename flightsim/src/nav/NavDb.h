#pragma once
// CsvNavDatabase: the runtime NavDatabase (ARCHITECTURE section 5) backed by the CSV nav pack in
// <data root>/nav (DATA.md section 5) and magvar.bin (section 6).
//
//  - The synthetic airport XTST is always merged in, with or without a data pack.
//  - Real ILS rows (ils.csv) are attached to runways by airport + runway ident. Hard, exact
//    (non-`approx`) runways >= 3,000 ft without a real ILS get a synthesized one
//    (NAVIGATION section 3.2, `Ils::synthesized = true`).
//  - Airports are indexed on a 1 degree grid for nearest / box queries.
//  - Airways are loaded lazily on first use (airway()); everything else loads in load().
//
// After load() returns, the database is read-only: every query is const and thread-safe, and
// returned `const Airport*` / `const Navaid*` stay valid for the life of the object. Do not call
// load() again while pointers are in use.

#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "core/NavDatabase.h"
#include "core/SyntheticNavDb.h"
#include "nav/MagVar.h"

namespace fsim {

struct NavDbStats {
    size_t airports = 0, runways = 0, navaids = 0, fixes = 0;
    size_t realIls = 0, synthIls = 0;
    size_t skippedRows = 0;  // rows with missing or unusable data, or that reference unknown rows
    double loadMs = 0;
};

struct AirwayPoint {
    int seq = 0;  // 1..n, restarting at 1001, 2001, ... for each disjoint segment (DATA 5.7)
    std::string fixIdent;
    double latDeg = 0, lonDeg = 0;
    float meaFt = 0;                            // 0 = unknown
    int segment() const { return seq / 1000; }  // points connect only within one segment
};

class CsvNavDatabase : public NavDatabase {
public:
    CsvNavDatabase();  // XTST only until load() succeeds

    // Loads airports.csv, runways.csv, navaids.csv, ils.csv, fixes.csv and magvar.bin from
    // `navDir`. airports.csv and runways.csv are required; the others are optional (logged).
    // On failure returns false and leaves only XTST in the database.
    bool load(const std::string& navDir, std::string& error);
    const NavDbStats& stats() const { return stats_; }
    bool hasMagVar() const { return magvar_.valid(); }

    // NavDatabase
    const Airport* findAirport(std::string_view ident) const override;
    std::vector<const Airport*> nearestAirports(double latDeg, double lonDeg, int maxCount,
                                                float maxDistNm) const override;
    std::vector<const Airport*> searchAirports(std::string_view text, int maxCount) const override;
    std::vector<const Navaid*> findNavaids(std::string_view ident) const override;
    std::vector<Waypoint> findFixes(std::string_view ident) const override;
    std::vector<const Airport*> airportsInBox(double south, double west, double north,
                                              double east) const override;
    float magVarDeg(double latDeg, double lonDeg) const override;

    // Points of an airway designator ("V23"), ordered by seq; empty if unknown or there is no
    // airways.csv. The file is read on first call (thread-safe).
    const std::vector<AirwayPoint>& airway(std::string_view designator) const;

    // Number of airports (including XTST).
    size_t airportCount() const { return airports_.size(); }

private:
    struct AirportExtra {
        std::string identUpper, iataUpper, nameLower, muniLower;
    };

    void reset();
    void addAirport(Airport airport, std::string iata, std::string municipality);
    void buildIndexes();
    void attachIls(const std::string& path);
    void synthesizeMissingIls();
    bool loadAirports(const std::string& path, std::string& error);
    bool loadRunways(const std::string& path, std::string& error);
    void loadNavaids(const std::string& path);
    void loadFixes(const std::string& path);

    std::vector<Airport> airports_;
    std::vector<AirportExtra> extra_;
    std::unordered_map<std::string, int> byIdent_;  // upper-case ident -> index
    std::vector<std::vector<int>> grid_;            // 1 degree cells -> airport indexes
    std::vector<Navaid> navaids_;                   // sorted by ident
    std::vector<Waypoint> fixes_;                   // sorted by ident
    MagVar magvar_;
    NavDbStats stats_;
    std::string navDir_;

    mutable std::once_flag airwaysOnce_;
    mutable std::unordered_map<std::string, std::vector<AirwayPoint>> airways_;
};

}  // namespace fsim
