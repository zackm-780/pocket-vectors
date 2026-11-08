// U2: flight log. The recorder over synthetic state sequences, the CSV file, the open-flight
// recovery and the arrival lookup. No simulation, no data pack.
#include <doctest/doctest.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

#include "app/FlightLog.h"
#include "app/Scenario.h"
#include "core/Geo.h"
#include "core/Paths.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"

using namespace fsim;

namespace {

std::string tempPath(const char* leaf) {
    const std::string dir = "flightlog_test";
    makeDirs(dir);
    const std::string p = dir + "/" + leaf;
    std::remove(p.c_str());
    return p;
}

Scenario xtstScenario() {
    Scenario sc;
    sc.aircraftId = "c172p";
    sc.airportIdent = "XTST";
    sc.runwayIdent = "27";
    sc.startKind = "runway";
    sc.planPath = "assets/plans/XTST-lnav.fms";
    return sc;
}

AircraftState ground(double simS, float gsKt = 0, float fuel = 200) {
    AircraftState s;
    s.valid = true;
    s.simTimeS = simS;
    s.wowAny = s.wowAll = true;
    s.gsKt = gsKt;
    s.altMslFt = 2000;
    s.fuelTotalLbs = fuel;
    s.latDeg = SyntheticNavDb::kLatDeg;
    s.lonDeg = SyntheticNavDb::kLonDeg;
    return s;
}

AircraftState air(double simS, double altFt, float vsFpm = 0, float gsKt = 100, float fuel = 200) {
    AircraftState s = ground(simS, gsKt, fuel);
    s.wowAny = s.wowAll = false;
    s.altMslFt = altFt;
    s.vsFpm = vsFpm;
    return s;
}

// Steps the recorder through `seconds` of a constant state at 10 Hz.
void hold(FlightRecorder& r, AircraftState s, double& t, double seconds, bool paused = false) {
    for (int i = 0; i < static_cast<int>(seconds * 10 + 0.5); ++i) {
        if (!paused) t += 0.1;
        s.simTimeS = t;
        r.update(s, 0.1, paused);
    }
}

}  // namespace

TEST_CASE("app: flight log records a takeoff, a flight and a landing") {
    FlightRecorder r;
    double t = 0;
    r.begin(xtstScenario(), "c172p", "2026-11-07T14:02:00Z", ground(0));
    CHECK(r.active());
    hold(r, ground(0, 40, 199), t, 20);            // takeoff roll: 20 s at 40 kt
    hold(r, air(0, 3000, 500, 100, 190), t, 600);  // 10 min airborne at 100 kt
    hold(r, air(0, 2010, -420, 70, 185), t, 5);    // short final, 420 fpm down
    hold(r, ground(0, 50, 185), t, 30);            // touchdown and rollout
    hold(r, ground(0, 0, 185), t, 10);             // stopped
    const FlightLogEntry e = r.end(FlightEnd::NewFlight);
    CHECK_FALSE(r.active());
    CHECK(e.outcome == "LANDED");
    CHECK(e.landings == 1);
    CHECK(e.touchdownFpm == doctest::Approx(420));
    CHECK(e.flightS == doctest::Approx(665).epsilon(0.001));
    CHECK(e.airborneS == doctest::Approx(605).epsilon(0.001));
    CHECK(e.distNm ==
          doctest::Approx((20 * 40 + 600 * 100 + 5 * 70 + 30 * 50) / 3600.0).epsilon(0.002));
    CHECK(e.maxAltFt == doctest::Approx(3000));
    CHECK(e.fuelStartLbs == doctest::Approx(200));
    CHECK(e.fuelUsedLbs() == doctest::Approx(15));
    CHECK(e.depAirport == "XTST");
    CHECK(e.depRunway == "27");
    CHECK(e.plan == "XTST-lnav.fms");
    CHECK(e.approach == "-");
}

TEST_CASE("app: flight log counts a bounced landing once, and a takeoff hop not at all") {
    FlightRecorder r;
    double t = 0;
    r.begin(xtstScenario(), "c172p", "2026-11-07T14:02:00Z", air(0, 2100, -300));
    hold(r, air(0, 2050, -600), t, 10);
    hold(r, ground(0, 60), t, 0.5);          // touchdown, 600 fpm
    hold(r, air(0, 2003, 200, 58), t, 1.5);  // bounce: 1.5 s in the air
    hold(r, ground(0, 50), t, 20);           // down again: the same landing
    CHECK(r.entry().landings == 1);
    CHECK(r.entry().touchdownFpm == doctest::Approx(600));
    // A takeoff that skips once on the roll is not a landing either.
    hold(r, air(0, 2002, 100, 60), t, 1.0);
    hold(r, ground(0, 61), t, 1.0);
    hold(r, air(0, 2100, 500, 70), t, 30);
    CHECK(r.entry().landings == 1);
    CHECK(r.end(FlightEnd::Quit).outcome == "IN FLIGHT");  // QUIT while airborne
}

TEST_CASE("app: flight log outcomes: crash, never airborne, pause excluded, checkpoints") {
    FlightRecorder r;
    double t = 0;
    r.begin(xtstScenario(), "c172p", "2026-11-07T14:02:00Z", air(0, 2500));
    hold(r, air(0, 2400, -900), t, 30);
    AircraftState crash = ground(t, 80);
    crash.crashed = true;
    std::strncpy(crash.crashReason, "terrain impact", sizeof crash.crashReason - 1);
    r.update(crash, 0.1, false);
    CHECK(r.end(FlightEnd::Crash).outcome == "CRASHED: TERRAIN IMPACT");

    FlightRecorder g;
    t = 0;
    g.begin(xtstScenario(), "c172p", "2026-11-07T15:00:00Z", ground(0));
    hold(g, ground(0, 10), t, 30);
    hold(g, ground(0, 10), t, 120, true);  // two minutes in the pause menu
    const FlightLogEntry e = g.end(FlightEnd::NewFlight);
    CHECK(e.outcome == "ON GROUND");
    CHECK(e.flightS == doctest::Approx(30).epsilon(0.001));
    CHECK(e.landings == 0);

    // The open-flight file is due every 60 s of flight and at each touchdown.
    FlightRecorder c;
    t = 0;
    c.begin(xtstScenario(), "c172p", "2026-11-07T16:00:00Z", air(0, 3000));
    hold(c, air(0, 3000), t, 59);
    CHECK_FALSE(c.checkpointDue());
    hold(c, air(0, 3000), t, 1.5);
    CHECK(c.checkpointDue());
    c.checkpointDone();
    CHECK_FALSE(c.checkpointDue());
    hold(c, air(0, 2010, -300), t, 10);
    hold(c, ground(0, 50), t, 0.2);
    CHECK(c.checkpointDue());  // touchdown
}

TEST_CASE("app: flight log CSV round trip, damaged lines skipped, newest first") {
    const std::string path = tempPath("flightlog.csv");
    FlightLogEntry a;
    a.startUtc = "2026-11-07T14:02:00Z";
    a.aircraftId = "c172p";
    a.depAirport = "KPDX";
    a.depRunway = "28L";
    a.startKind = "runway";
    a.plan = "KPDX,KOTH.fms";  // a comma in a name becomes a space
    a.arrAirport = "KOTH";
    a.arrRunway = "04";
    a.flightS = 4320;
    a.airborneS = 4100;
    a.distNm = 160.4;
    a.maxAltFt = 8020;
    a.landings = 1;
    a.touchdownFpm = 210;
    a.fuelStartLbs = 240;
    a.fuelEndLbs = 170;
    a.outcome = "LANDED";
    REQUIRE(appendFlightLog(path, a));
    {
        std::ofstream f(path, std::ios::app);
        f << "this,is,not,a,flight\n"
          << "2026-11-07T15:00:00Z,c172p,KPDX\n";  // damaged
    }
    FlightLogEntry b = a;
    b.startUtc = "2026-11-08T09:30:00Z";
    b.outcome = "CRASHED: TERRAIN";
    b.landings = 0;
    REQUIRE(appendFlightLog(path, b));

    std::ifstream in(path);
    std::string first;
    std::getline(in, first);
    CHECK(first == flightLogHeader());  // one header, written with the first row

    const std::vector<FlightLogEntry> got = readFlightLog(path);
    REQUIRE(got.size() == 2);
    CHECK(got[0].startUtc == b.startUtc);  // newest first
    CHECK(got[0].outcome == "CRASHED: TERRAIN");
    const FlightLogEntry& g = got[1];
    CHECK(g.plan == "KPDX KOTH.fms");
    CHECK(g.arrRunway == "04");
    CHECK(g.flightS == doctest::Approx(4320));
    CHECK(g.distNm == doctest::Approx(160.4));
    CHECK(g.landings == 1);
    CHECK(g.fuelUsedLbs() == doctest::Approx(70));
    CHECK(flightLogListLine(g) == "11-07 14:02  C172P  KPDX 28L > KOTH 04  1:12  LANDED");
    CHECK(readFlightLog(path, 1).size() == 1);
    CHECK(readFlightLog(tempPath("missing.csv")).empty());  // a missing file is not an error
}

TEST_CASE("app: a flight the game did not close is recovered as GAME CLOSED") {
    const std::string log = tempPath("recover_log.csv");
    const std::string open = tempPath("recover_open.csv");
    CHECK_FALSE(recoverOpenFlight(open, log));  // nothing to recover
    FlightRecorder r;
    double t = 0;
    r.begin(xtstScenario(), "c172p", "2026-11-08T10:00:00Z", air(0, 4000));
    hold(r, air(0, 4500, 0, 110), t, 61);
    FlightLogEntry e = r.entry();
    e.outcome = "IN FLIGHT";
    REQUIRE(writeOpenFlight(open, e));
    CHECK(fileExists(open));
    CHECK(recoverOpenFlight(open, log));
    CHECK_FALSE(fileExists(open));
    const std::vector<FlightLogEntry> got = readFlightLog(log);
    REQUIRE(got.size() == 1);
    CHECK(got[0].outcome == "GAME CLOSED");
    CHECK(got[0].maxAltFt == doctest::Approx(4500));
    CHECK(got[0].flightS == doctest::Approx(61).epsilon(0.01));
}

TEST_CASE("app: the arrival is the runway nearest the last touchdown, within 2 NM") {
    SyntheticNavDb db;
    const Runway* r27 = findRunway(*db.findAirport("XTST"), "27");
    REQUIRE(r27);
    FlightLogEntry e;
    e.landings = 1;
    double lat = 0, lon = 0;  // 300 m past the 27 threshold, 5 m right of the centerline
    destinationPoint(r27->thrLatDeg, r27->thrLonDeg, degToRad(r27->headingTrueDeg), 300.0, lat,
                     lon);
    fillArrival(e, lat, lon, db);
    CHECK(e.arrAirport == "XTST");
    CHECK(e.arrRunway == "27");
    FlightLogEntry far;
    far.landings = 1;
    fillArrival(far, SyntheticNavDb::kLatDeg + 0.2, SyntheticNavDb::kLonDeg, db);  // 12 NM north
    CHECK(far.arrAirport == "-");
    FlightLogEntry none;  // never landed
    fillArrival(none, lat, lon, db);
    CHECK(none.arrAirport == "-");
}
