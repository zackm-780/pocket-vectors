// CsvNavDatabase over the small fixture pack in tests/fixtures/nav (real rows from OurAirports /
// FAA NASR, plus a synthetic magvar grid). No data pack or network is needed.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <set>
#include <string>

#include "app/Scenario.h"
#include "core/Geo.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "nav/NavDb.h"

using namespace fsim;

namespace {

std::string fixtureDir() {
    std::string f = __FILE__;  // .../tests/nav/navdb_test.cpp
    f = f.substr(0, f.find_last_of('/'));
    f = f.substr(0, f.find_last_of('/'));
    return f + "/fixtures/nav";
}

const CsvNavDatabase& fixtureDb() {
    static CsvNavDatabase db;
    static bool loaded = false;
    if (!loaded) {
        std::string err;
        REQUIRE_MESSAGE(db.load(fixtureDir(), err), err);
        loaded = true;
    }
    return db;
}

std::vector<std::string> idents(const std::vector<const Airport*>& v) {
    std::vector<std::string> out;
    for (const Airport* a : v) out.push_back(a->ident);
    return out;
}

bool contains(const std::vector<std::string>& v, const std::string& s) {
    return std::find(v.begin(), v.end(), s) != v.end();
}

const Runway& runway(const char* apt, const char* rwy) {
    const Airport* a = fixtureDb().findAirport(apt);
    REQUIRE(a != nullptr);
    const Runway* r = findRunway(*a, rwy);
    REQUIRE_MESSAGE(r != nullptr, apt << " " << rwy);
    return *r;
}

}  // namespace

TEST_CASE("nav: ident lookup, zero-padded runways, XTST merged") {
    const CsvNavDatabase& db = fixtureDb();
    const Airport* pdx = db.findAirport("KPDX");
    REQUIRE(pdx != nullptr);
    CHECK(db.findAirport("kpdx") == pdx);
    CHECK(pdx->name == "Portland International Airport");
    CHECK(pdx->type == AirportType::Large);
    CHECK(pdx->iso == "US");
    CHECK(pdx->elevFt == doctest::Approx(31.0));
    CHECK(pdx->latDeg == doctest::Approx(45.588699));
    CHECK(pdx->runways.size() == 6);

    const Runway& r = runway("KPDX", "28L");
    CHECK(r.oppositeIdent == "10R");
    CHECK(r.airportIdent == "KPDX");
    CHECK(r.thrElevFt == doctest::Approx(23.0));
    CHECK(r.headingTrueDeg == doctest::Approx(299.1));
    CHECK(r.lengthFt == doctest::Approx(11000.0));
    CHECK(r.hard);
    CHECK(r.lighted);
    CHECK_FALSE(r.approxGeometry);
    // Displaced threshold: start of pavement differs from the landing threshold.
    const Runway& r10l = runway("KPDX", "10L");
    CHECK(r10l.displacedFt == doctest::Approx(1290.0));
    CHECK(greatCircleDistM(r10l.startLatDeg, r10l.startLonDeg, r10l.thrLatDeg, r10l.thrLonDeg) ==
          doctest::Approx(ftToM(1290.0)).epsilon(0.02));

    // Zero-padded idents: "05" exists, "5" does not.
    CHECK(findRunway(*db.findAirport("KOTH"), "05") != nullptr);
    CHECK(findRunway(*db.findAirport("KOTH"), "5") == nullptr);

    CHECK(db.findAirport("ZZZZ") == nullptr);
    CHECK(db.findAirport("") == nullptr);
    const Airport* xtst = db.findAirport("XTST");
    REQUIRE(xtst != nullptr);
    CHECK(xtst->runways.size() == 2);
    CHECK(xtst->runways[0].ils.has_value());
    CHECK(db.stats().airports == 13);  // 12 fixture airports + XTST
}

TEST_CASE("nav: no data pack still gives XTST, failed load falls back") {
    CsvNavDatabase db;
    CHECK(db.findAirport("XTST") != nullptr);
    CHECK(db.findAirport("KPDX") == nullptr);
    CHECK(db.magVarDeg(45.0, -122.0) == 0.0f);
    std::string err;
    CHECK_FALSE(db.load("/nonexistent/nav", err));
    CHECK_FALSE(err.empty());
    CHECK(db.findAirport("XTST") != nullptr);
    CHECK(db.airportCount() == 1);
}

TEST_CASE("nav: navaid and fix ambiguity") {
    const CsvNavDatabase& db = fixtureDb();
    const auto pdx = db.findNavaids("pdx");
    REQUIRE(pdx.size() == 1);
    CHECK(pdx[0]->type == NavaidType::VorDme);
    CHECK(pdx[0]->freqKhz == doctest::Approx(111800.0));
    CHECK(pdx[0]->magVarDeg == doctest::Approx(17.04));
    CHECK(db.findNavaids("OTH")[0]->type == NavaidType::Vortac);

    const auto twok = db.findNavaids("2K");  // an NDB and a DME with the same ident
    REQUIRE(twok.size() == 2);
    CHECK(twok[0]->type != twok[1]->type);
    CHECK(db.findNavaids("NOPE").empty());

    const auto aaron = db.findFixes("aaron");
    REQUIRE(aaron.size() == 2);
    CHECK(aaron[0].kind == WaypointKind::Fix);
    CHECK(aaron[0].latDeg != doctest::Approx(aaron[1].latDeg));
    CHECK(db.findFixes("AASEN").size() == 1);
    CHECK(db.findFixes("NOPEX").empty());
}

TEST_CASE("nav: nearest airports") {
    const CsvNavDatabase& db = fixtureDb();
    const Airport* pdx = db.findAirport("KPDX");

    auto near = db.nearestAirports(pdx->latDeg, pdx->lonDeg, 10, 30.0f);
    REQUIRE(near.size() >= 3);
    CHECK(near[0] == pdx);
    CHECK(contains(idents(near), "KTTD"));
    CHECK(contains(idents(near), "KVUO"));
    CHECK(contains(idents(near), "KHIO"));
    CHECK_FALSE(contains(idents(near), "KOTH"));  // ~150 NM away
    double prev = 0;
    for (const Airport* a : near) {
        const double d = greatCircleDistM(pdx->latDeg, pdx->lonDeg, a->latDeg, a->lonDeg);
        CHECK(d >= prev);
        CHECK(mToNm(d) <= 30.0 + 1e-6);
        prev = d;
    }

    // maxCount limits, distance limit of ~0 keeps only an airport at the exact position.
    CHECK(db.nearestAirports(pdx->latDeg, pdx->lonDeg, 2, 30.0f).size() == 2);
    CHECK(db.nearestAirports(pdx->latDeg, pdx->lonDeg, 10, 0.0f).size() == 1);
    CHECK(db.nearestAirports(pdx->latDeg, pdx->lonDeg, 0, 30.0f).empty());
    CHECK(db.nearestAirports(0.0, 0.0, 5, 100.0f).empty());

    // A large radius finds KOTH (~150 NM) and reaches XTST (~1100 NM).
    const auto wide = idents(db.nearestAirports(pdx->latDeg, pdx->lonDeg, 50, 2000.0f));
    CHECK(contains(wide, "KOTH"));
    CHECK(contains(wide, "XTST"));
    // Every airport within the radius must be returned (no grid-edge misses).
    for (const char* id : {"KOTH", "KSEA", "KSFO", "KLAX", "KEUG", "04W", "04WN", "00AL"}) {
        const Airport* a = db.findAirport(id);
        const double dNm = mToNm(greatCircleDistM(pdx->latDeg, pdx->lonDeg, a->latDeg, a->lonDeg));
        CHECK(contains(wide, id) == (dNm <= 2000.0));
    }

    // Longitude wrap: a query just east of the antimeridian is fine and finds nothing.
    CHECK(db.nearestAirports(60.0, 179.9, 5, 300.0f).empty());
    // Pole queries do not crash.
    CHECK(db.nearestAirports(89.99, 10.0, 5, 300.0f).empty());
}

TEST_CASE("nav: airports in box") {
    const CsvNavDatabase& db = fixtureDb();
    const auto portland = idents(db.airportsInBox(45.4, -123.0, 45.7, -122.3));
    CHECK(portland == std::vector<std::string>{"KHIO", "KPDX", "KTTD", "KVUO"});  // sorted by ident

    CHECK(idents(db.airportsInBox(43.0, -125.0, 44.0, -124.0)) == std::vector<std::string>{"KOTH"});
    CHECK(db.airportsInBox(10.0, 10.0, 11.0, 11.0).empty());
    CHECK(db.airportsInBox(46.0, -123.0, 45.0, -122.0).empty());  // south > north
    // A box crossing the antimeridian (west > east) is accepted and finds nothing here.
    CHECK(db.airportsInBox(-10.0, 170.0, 10.0, -170.0).empty());
    // Whole world.
    CHECK(db.airportsInBox(-90.0, -180.0, 90.0, 180.0).size() == db.airportCount());
    // Exact edges are inclusive.
    const Airport* koth = db.findAirport("KOTH");
    CHECK(db.airportsInBox(koth->latDeg, koth->lonDeg, koth->latDeg, koth->lonDeg).size() == 1);
}

TEST_CASE("nav: search by ident, IATA, municipality and name") {
    const CsvNavDatabase& db = fixtureDb();
    auto first = [&](const char* q) {
        const auto r = db.searchAirports(q, 5);
        return r.empty() ? std::string() : r[0]->ident;
    };
    CHECK(first("KPDX") == "KPDX");
    CHECK(first("kpdx") == "KPDX");
    CHECK(first("PDX") == "KPDX");  // IATA
    CHECK(first("OTH") == "KOTH");
    CHECK(first("north bend") == "KOTH");  // municipality prefix
    CHECK(first("Southwest") == "KOTH");   // name prefix
    CHECK(first("Hillsboro") == "KHIO");   // word inside the name
    CHECK(first("xt") == "XTST");          // ident prefix
    CHECK(first("Synthetic") == "XTST");

    // Municipality "Portland": large airports first, then by ident; maxCount applies.
    const auto portland = idents(db.searchAirports("portland", 10));
    CHECK(portland == std::vector<std::string>{"KPDX", "KHIO", "KTTD"});
    CHECK(db.searchAirports("portland", 2).size() == 2);
    CHECK(db.searchAirports("portland", 0).empty());

    CHECK(db.searchAirports("", 5).empty());
    CHECK(db.searchAirports("zzzzzz", 5).empty());
    // Ident prefix beats name word match.
    CHECK(first("KS") == "KSEA");  // large, ident prefix; KSEA before KSFO by ident
    CHECK(contains(idents(db.searchAirports("International", 10)), "KSFO"));
}

TEST_CASE("nav: airways load lazily and keep segment numbering") {
    const CsvNavDatabase& db = fixtureDb();
    const auto& v23 = db.airway("v23");
    REQUIRE(v23.size() > 3);
    CHECK(v23.front().seq == 1);
    CHECK(v23.front().fixIdent == "MZB");
    CHECK(v23.front().segment() == 0);
    CHECK(v23.front().meaFt == doctest::Approx(3000.0));
    for (size_t i = 1; i < v23.size(); ++i) CHECK(v23[i].seq > v23[i - 1].seq);
    CHECK(v23.back().segment() >= 1);  // V23 has a disjoint second segment (seq 1001..)
    CHECK(db.airway("Q99").empty());
}

TEST_CASE("nav: magnetic variation from the fixture grid") {
    const CsvNavDatabase& db = fixtureDb();
    REQUIRE(db.hasMagVar());
    // The fixture grid holds lat/10 + lon/30 degrees, which bilinear interpolation reproduces.
    CHECK(db.magVarDeg(45.6, -122.6) == doctest::Approx(45.6 / 10 - 122.6 / 30).epsilon(0.01));
    CHECK(db.magVarDeg(-33.3, 151.2) == doctest::Approx(-33.3 / 10 + 151.2 / 30).epsilon(0.01));
    CHECK(db.magVarDeg(0.0, 0.0) == doctest::Approx(0.0).epsilon(0.01));
    // Longitude wraps; latitude clamps.
    CHECK(db.magVarDeg(10.0, 190.0) == doctest::Approx(db.magVarDeg(10.0, -170.0)));
    CHECK(db.magVarDeg(95.0, 10.0) == doctest::Approx(db.magVarDeg(90.0, 10.0)));
}

TEST_CASE("nav: scenario start resolves real runways from the database") {
    const CsvNavDatabase& db = fixtureDb();
    AircraftProfile profile;
    profile.id = "c172p";
    profile.vappKt = 65;
    profile.approachFlapDetent = 2;
    profile.flapDetents.resize(4);
    Scenario sc;
    std::string err;
    REQUIRE(parseScenario("[scenario]\nairport = KOTH\nrunway = 05\nstart = final\n"
                          "distance_nm = 6\n",
                          sc, err));
    StartCondition start;
    REQUIRE_MESSAGE(resolveStart(sc, db, profile, start, err), err);
    const Runway& r05 = runway("KOTH", "05");
    CHECK(start.headingTrueDeg == doctest::Approx(r05.headingTrueDeg));
    CHECK(start.fieldElevFt == doctest::Approx(r05.thrElevFt));
    CHECK(mToNm(greatCircleDistM(start.latDeg, start.lonDeg, r05.thrLatDeg, r05.thrLonDeg)) ==
          doctest::Approx(6.0).epsilon(0.001));
    // 3 degree path to the threshold crossing height of the real ILS (50 ft default).
    CHECK(start.altMslFt ==
          doctest::Approx(r05.thrElevFt + 50.0 + nmToFt(6.0) * std::tan(degToRad(3.0))));

    REQUIRE(parseScenario("[scenario]\nairport = KPDX\nrunway = 28L\nstart = runway\n", sc, err));
    REQUIRE_MESSAGE(resolveStart(sc, db, profile, start, err), err);
    CHECK(start.headingTrueDeg == doctest::Approx(299.1));
    CHECK(start.fieldElevFt == doctest::Approx(23.0));
}

TEST_CASE("nav: full-database checks (manual, set FSIM_NAV_DATA_DIR)") {
    const char* dir = std::getenv("FSIM_NAV_DATA_DIR");
    if (!dir) return;  // not run in CI: needs the real data pack
    CsvNavDatabase db;
    std::string err;
    REQUIRE_MESSAGE(db.load(dir, err), err);
    MESSAGE("load " << db.stats().loadMs << " ms, airports " << db.stats().airports << ", runways "
                    << db.stats().runways << ", navaids " << db.stats().navaids << ", fixes "
                    << db.stats().fixes << ", real ILS " << db.stats().realIls
                    << ", synthesized ILS " << db.stats().synthIls);
    CHECK(db.stats().loadMs < 3000.0);
    REQUIRE(db.findAirport("KPDX") != nullptr);
    REQUIRE(db.findAirport("KOTH") != nullptr);
    CHECK(db.airway("V23").size() > 5);
    // WMM2025 values near the Oregon coast and Portland (~15-16 deg east in 2026).
    const float varPdx = db.magVarDeg(45.5887, -122.598);
    const float varOth = db.magVarDeg(43.4171, -124.246);
    MESSAGE("magvar KPDX " << varPdx << " KOTH " << varOth);
    CHECK(varPdx > 14.0f);
    CHECK(varPdx < 17.0f);
    CHECK(varOth > 14.0f);
    CHECK(varOth < 17.0f);
}

TEST_CASE("nav: navaidsNear returns navaids within the radius, nearest first") {
    const NavDatabase& db = fixtureDb();
    // KPDX field: PDX VOR is ~0.4 NM away, BTG ~9.5 NM, UBG ~21 NM, OTH far south.
    const auto near10 = db.navaidsNear(45.5887, -122.598, 10.0f);
    REQUIRE(near10.size() == 2);
    CHECK(near10[0]->ident == "PDX");
    CHECK(near10[1]->ident == "BTG");
    const auto near30 = db.navaidsNear(45.5887, -122.598, 30.0f);
    REQUIRE(near30.size() == 3);
    CHECK(near30[2]->ident == "UBG");
    CHECK(db.navaidsNear(0.0, 0.0, 50.0f).empty());
}
