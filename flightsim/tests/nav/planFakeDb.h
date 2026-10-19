#pragma once
// A small fake NavDatabase for flight plan tests: two airports, a few navaids and fixes with
// deliberately duplicated idents (CVO exists in Oregon and Egypt, ABC is a VOR and an NDB).
#include <cmath>
#include <string>
#include <vector>

#include "core/NavDatabase.h"

namespace fsim::test {

class PlanFakeDb : public NavDatabase {
public:
    PlanFakeDb() {
        // KAAA: 45N 122W, runways 10R (heading east-southeast) and 28L.
        Airport a;
        a.ident = "KAAA";
        a.name = "Alpha Intl";
        a.latDeg = 45.0;
        a.lonDeg = -122.0;
        a.elevFt = 30;
        a.runways.push_back(makeRwy("KAAA", "10R", "28L", 45.0, -122.02, 45.0, -121.98));
        a.runways.push_back(makeRwy("KAAA", "28L", "10R", 45.0, -121.98, 45.0, -122.02));
        airports_.push_back(a);
        Airport b;
        b.ident = "KBBB";
        b.name = "Bravo Regional";
        b.latDeg = 43.0;
        b.lonDeg = -124.0;
        b.elevFt = 15;
        b.runways.push_back(makeRwy("KBBB", "05", "23", 43.0, -124.02, 43.01, -124.0));
        b.runways.push_back(makeRwy("KBBB", "23", "05", 43.01, -124.0, 43.0, -124.02));
        airports_.push_back(b);

        navaids_.push_back(nav("CVO", "Corvallis", NavaidType::VorDme, 44.5, -123.3));
        navaids_.push_back(nav("CVO", "Cairo", NavaidType::VorDme, 30.09, 31.39));
        navaids_.push_back(nav("ABC", "Abc VOR", NavaidType::Vor, 44.0, -123.0));
        navaids_.push_back(nav("ABC", "Abc NDB", NavaidType::Ndb, 44.05, -123.05));
        navaids_.push_back(nav("MID", "Midway VOR", NavaidType::Vortac, 44.0, -123.5));
        navaids_.push_back(nav("FAR", "Far VOR", NavaidType::Vor, 40.0, -118.0));
        fixes_.push_back(Waypoint{"WPT01", WaypointKind::Fix, 44.2, -122.8});
        fixes_.push_back(Waypoint{"WPT01", WaypointKind::Fix, 44.9, -110.0});
    }

    const Airport* findAirport(std::string_view ident) const override {
        for (const Airport& a : airports_)
            if (a.ident == ident) return &a;
        return nullptr;
    }
    std::vector<const Airport*> nearestAirports(double, double, int, float) const override {
        std::vector<const Airport*> v;
        for (const Airport& a : airports_) v.push_back(&a);
        return v;
    }
    std::vector<const Airport*> searchAirports(std::string_view, int) const override { return {}; }
    std::vector<const Navaid*> findNavaids(std::string_view ident) const override {
        std::vector<const Navaid*> v;
        for (const Navaid& n : navaids_)
            if (n.ident == ident) v.push_back(&n);
        return v;
    }
    std::vector<Waypoint> findFixes(std::string_view ident) const override {
        std::vector<Waypoint> v;
        for (const Waypoint& w : fixes_)
            if (w.ident == ident) v.push_back(w);
        return v;
    }
    std::vector<const Airport*> airportsInBox(double, double, double, double) const override {
        return {};
    }
    float magVarDeg(double, double) const override { return 15.0f; }  // east

    std::vector<const Navaid*> navaidsNear(double latDeg, double lonDeg, float radiusNm) const {
        std::vector<const Navaid*> v;
        for (const Navaid& n : navaids_) {
            double dLat = (n.latDeg - latDeg) * 60.0;
            double dLon = (n.lonDeg - lonDeg) * 60.0 * std::cos(latDeg * 3.14159265 / 180.0);
            if (std::sqrt(dLat * dLat + dLon * dLon) <= radiusNm) v.push_back(&n);
        }
        return v;
    }

    std::vector<Airport> airports_;
    std::vector<Navaid> navaids_;
    std::vector<Waypoint> fixes_;

private:
    static Runway makeRwy(const char* apt, const char* id, const char* opp, double thrLat,
                          double thrLon, double endLat, double endLon) {
        Runway r;
        r.airportIdent = apt;
        r.ident = id;
        r.oppositeIdent = opp;
        r.thrLatDeg = r.startLatDeg = thrLat;
        r.thrLonDeg = r.startLonDeg = thrLon;
        r.endLatDeg = endLat;
        r.endLonDeg = endLon;
        return r;
    }
    static Navaid nav(const char* id, const char* name, NavaidType t, double lat, double lon) {
        Navaid n;
        n.ident = id;
        n.name = name;
        n.type = t;
        n.latDeg = lat;
        n.lonDeg = lon;
        return n;
    }
};

}  // namespace fsim::test
