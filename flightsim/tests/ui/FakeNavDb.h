#pragma once
// A tiny multi-airport NavDatabase for the menu tests (the synthetic database only has XTST).
// KAAA (large, IATA "AAA"), KBBB (medium, near KAAA), KOTH-like "KOTX" (small, IATA "OTX"),
// XTST (via SyntheticNavDb), and "NORW" (no runways, must never be offered).

#include <algorithm>
#include <cctype>
#include <map>
#include <string>
#include <vector>

#include "core/Geo.h"
#include "core/NavDatabase.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"

namespace fsim::test {

class FakeNavDb : public NavDatabase {
public:
    FakeNavDb() {
        add("KAAA", "Alpha International", "AAA", AirportType::Large, 45.5, -122.6, 30,
            {runway("10L", "28R", 100, 11000, true, true),
             runway("28R", "10L", 280, 11000, true, true),
             runway("10R", "28L", 100, 8000, true, false),
             runway("28L", "10R", 280, 8000, true, false),
             runway("03", "21", 30, 6000, false, false)});
        add("KBBB", "Bravo Municipal", "BBB", AirportType::Medium, 45.3, -122.9, 60,
            {runway("16", "34", 160, 5000, true, false),
             runway("34", "16", 340, 5000, true, false)});
        add("KOTX", "Otx Regional", "OTX", AirportType::Small, 43.4, -124.2, 17,
            {runway("05", "23", 50, 5300, true, true), runway("23", "05", 230, 5300, true, true)});
        add("OTHR", "Kotx Field", "", AirportType::Small, 43.5, -124.3, 20,
            {runway("09", "27", 90, 3000, true, false),
             runway("27", "09", 270, 3000, true, false)});
        add("NORW", "No Runways", "", AirportType::Small, 45.0, -122.0, 10, {});
    }

    const Airport* findAirport(std::string_view ident) const override {
        const std::string id = upper(std::string(ident));
        if (id == SyntheticNavDb::kIdent) return &xtst_.airport();
        for (const Airport& a : airports_)
            if (a.ident == id) return &a;
        return nullptr;
    }
    std::vector<const Airport*> nearestAirports(double lat, double lon, int maxCount,
                                                float maxDistNm) const override {
        std::vector<const Airport*> out;
        for (const Airport& a : airports_)
            if (mToNm(greatCircleDistM(lat, lon, a.latDeg, a.lonDeg)) <= maxDistNm)
                out.push_back(&a);
        std::sort(out.begin(), out.end(), [&](const Airport* a, const Airport* b) {
            return greatCircleDistM(lat, lon, a->latDeg, a->lonDeg) <
                   greatCircleDistM(lat, lon, b->latDeg, b->lonDeg);
        });
        if (static_cast<int>(out.size()) > maxCount) out.resize(static_cast<size_t>(maxCount));
        return out;
    }
    std::vector<const Airport*> searchAirports(std::string_view text, int maxCount) const override {
        const std::string q = upper(std::string(text));
        std::vector<const Airport*> out;
        // Deliberately the "wrong" order for exact matches: name matches before idents, so the
        // UI-side ordering has something to fix.
        for (const Airport& a : airports_)
            if (upper(a.name).find(q) != std::string::npos) out.push_back(&a);
        for (const Airport& a : airports_) {
            const bool hit = a.ident.rfind(q, 0) == 0 || iata_.at(a.ident) == q;
            if (hit && std::find(out.begin(), out.end(), &a) == out.end()) out.push_back(&a);
        }
        if (static_cast<int>(out.size()) > maxCount) out.resize(static_cast<size_t>(maxCount));
        return out;
    }
    std::vector<const Navaid*> findNavaids(std::string_view) const override { return {}; }
    std::vector<Waypoint> findFixes(std::string_view) const override { return {}; }
    std::vector<const Airport*> airportsInBox(double, double, double, double) const override {
        return {};
    }
    float magVarDeg(double, double) const override { return 0.0f; }

private:
    static std::string upper(std::string s) {
        for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        return s;
    }
    static Runway runway(const char* ident, const char* opposite, float hdgDeg, float lengthFt,
                         bool hard, bool ils) {
        Runway r;
        r.ident = ident;
        r.oppositeIdent = opposite;
        r.headingTrueDeg = hdgDeg;
        r.lengthFt = lengthFt;
        r.widthFt = 150;
        r.hard = hard;
        if (ils) r.ils = Ils{};
        return r;
    }
    void add(const char* ident, const char* name, const char* iata, AirportType type, double lat,
             double lon, float elevFt, std::vector<Runway> rwys) {
        Airport a;
        a.ident = ident;
        a.name = name;
        a.latDeg = lat;
        a.lonDeg = lon;
        a.elevFt = elevFt;
        a.type = type;
        for (Runway& r : rwys) {
            r.airportIdent = ident;
            r.thrLatDeg = lat;
            r.thrLonDeg = lon;
            r.thrElevFt = elevFt;
        }
        a.runways = std::move(rwys);
        airports_.push_back(std::move(a));
        iata_[ident] = iata;
    }

    std::vector<Airport> airports_;
    std::map<std::string, std::string> iata_;
    SyntheticNavDb xtst_;
};

}  // namespace fsim::test
