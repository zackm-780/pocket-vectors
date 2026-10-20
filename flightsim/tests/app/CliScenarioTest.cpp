#include <doctest/doctest.h>

#include <cmath>

#include "app/Cli.h"
#include "app/Scenario.h"
#include "core/Geo.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "sim/ProfileLoader.h"

using namespace fsim;

TEST_CASE("cli: flags parse") {
    CliOptions o;
    std::string err;
    REQUIRE(parseCli({"--headless", "--scenario", "a.ini", "--aircraft", "737", "--duration",
                      "12.5", "--trace", "t.csv", "--frames", "90", "--screenshot", "s.png",
                      "--window", "640x480", "--fullscreen", "--show-fps", "--data-dir", "/d"},
                     o, err));
    CHECK(o.headless);
    CHECK(o.scenarioPath == "a.ini");
    CHECK(o.aircraftId == "737");
    CHECK(o.durationS == doctest::Approx(12.5));
    CHECK(o.tracePath == "t.csv");
    CHECK(o.frames == 90);
    CHECK(o.screenshotPath == "s.png");
    CHECK(o.windowWidth == 640);
    CHECK(o.windowHeight == 480);
    CHECK(o.fullscreen);
    CHECK(o.showFps);
    CHECK(o.dataDir == "/d");
}

TEST_CASE("cli: bad input is rejected with a message") {
    CliOptions o;
    std::string err;
    CHECK_FALSE(parseCli({"--bogus"}, o, err));
    CHECK(err.find("--bogus") != std::string::npos);
    CHECK_FALSE(parseCli({"--scenario"}, o, err));  // missing value
    CHECK_FALSE(parseCli({"--duration", "abc"}, o, err));
    CHECK_FALSE(parseCli({"--frames", "-3"}, o, err));
    CHECK_FALSE(parseCli({"--window", "800"}, o, err));
    CHECK(parseCli({}, o, err));
    CHECK_FALSE(cliUsage().empty());
}

namespace {

const char* kArchitectureExample = "[scenario]\n"
                                   "aircraft = c172p\n"
                                   "airport = XTST\n"
                                   "runway = 27\n"
                                   "; start = runway | final | cruise\n"
                                   "start = final\n"
                                   "distance_nm = 6\n"
                                   "altitude_ft = 0\n"
                                   "time_utc = 2026-06-21T18:00:00\n"
                                   "[wind]\n"
                                   "from_true_deg = 270\n"
                                   "speed_kt = 10\n"
                                   "[inputs]\n"
                                   "12.0 = elevator 0.3\n"
                                   "0.0  = throttle 1.0\n"
                                   "[autopilot]\n"
                                   "engage = hdg alt\n"
                                   "hdg_bug_mag_deg = 270\n"
                                   "alt_bug_ft = 4000\n";

AircraftProfile c172Profile() {
    AircraftProfile p;
    std::string err;
    p.id = "c172p";
    p.vappKt = 65;
    p.takeoffFlapDetent = 0;
    p.approachFlapDetent = 2;
    p.flapDetents.resize(4);
    p.cruiseAltFt = 6000;
    p.cruiseKt = 110;
    return p;
}

}  // namespace

TEST_CASE("scenario: the ARCHITECTURE section 9 example parses") {
    Scenario s;
    std::string err;
    REQUIRE_MESSAGE(parseScenario(kArchitectureExample, s, err), err);
    CHECK(s.aircraftId == "c172p");
    CHECK(s.airportIdent == "XTST");
    CHECK(s.runwayIdent == "27");
    CHECK(s.startKind == "final");
    CHECK(s.distanceNm == doctest::Approx(6.0));
    CHECK(s.windFromTrueDeg == doctest::Approx(270.0f));
    CHECK(s.windSpeedKt == doctest::Approx(10.0f));
    REQUIRE(s.inputs.size() == 2);
    CHECK(s.inputs[0].timeS == doctest::Approx(0.0));  // sorted by time
    CHECK(s.inputs[0].command == ScriptCommand::Throttle);
    CHECK(s.inputs[1].timeS == doctest::Approx(12.0));
    CHECK(s.inputs[1].command == ScriptCommand::Elevator);
    CHECK(s.inputs[1].value == doctest::Approx(0.3f));
    REQUIRE(s.apEngage.size() == 2);
    CHECK(s.apEngage[1] == "alt");
    CHECK(s.hasHdgBug);
    CHECK(s.hdgBugMagDeg == doctest::Approx(270.0f));
    CHECK(s.altBugFt == doctest::Approx(4000.0f));
}

TEST_CASE("scenario: errors are reported") {
    Scenario s;
    std::string err;
    CHECK_FALSE(parseScenario("[scenario]\nstart = hover\n", s, err));
    CHECK_FALSE(parseScenario("[scenario]\nstart = final\ndistance_nm = 0\n", s, err));
    CHECK_FALSE(parseScenario("[inputs]\nsoon = throttle 1\n", s, err));
    CHECK_FALSE(parseScenario("[inputs]\n1.0 = warp 1\n", s, err));
    CHECK_FALSE(parseScenario("[inputs]\n1.0 = throttle\n", s, err));
    CHECK(parseScenario("", s, err));  // all defaults
    CHECK(s.aircraftId == "c172p");
    CHECK_FALSE(loadScenario("/no/such/file.ini", s, err));
}

TEST_CASE("scenario: final start lands on the 3 degree path") {
    SyntheticNavDb nav;
    Scenario s;
    std::string err;
    REQUIRE(parseScenario(kArchitectureExample, s, err));
    StartCondition start;
    REQUIRE_MESSAGE(resolveStart(s, nav, c172Profile(), start, err), err);

    const Runway* r27 = findRunway(*nav.findAirport("XTST"), "27");
    REQUIRE(r27 != nullptr);
    CHECK(start.kind == StartCondition::Kind::Final);
    CHECK(start.headingTrueDeg == doctest::Approx(270.0));
    CHECK(start.iasKt == doctest::Approx(65.0));
    CHECK(start.flightPathDeg == doctest::Approx(-3.0));
    CHECK(start.flapDetent == 2);
    CHECK(start.gearDown);

    // 6 NM east of the 27 threshold (final for a westbound landing), on the extended centerline.
    CHECK(greatCircleDistM(start.latDeg, start.lonDeg, r27->thrLatDeg, r27->thrLonDeg) ==
          doctest::Approx(nmToM(6.0)).epsilon(1e-6));
    CHECK(start.lonDeg > r27->thrLonDeg);
    CHECK(start.latDeg == doctest::Approx(40.0).epsilon(1e-5));  // a parallel is not a great circle
    // Altitude = threshold elevation + 50 ft TCH + 6 NM * tan(3 deg).
    const double expected = 2000.0 + 50.0 + nmToFt(6.0) * std::tan(degToRad(3.0));
    CHECK(start.altMslFt == doctest::Approx(expected).epsilon(1e-9));
    CHECK(start.altMslFt == doctest::Approx(3960.7).epsilon(1e-4));
}

TEST_CASE("scenario: runway and cruise starts") {
    SyntheticNavDb nav;
    Scenario s;
    std::string err;
    StartCondition start;

    REQUIRE(parseScenario("[scenario]\nstart = runway\nrunway = 09\n", s, err));
    REQUIRE(resolveStart(s, nav, c172Profile(), start, err));
    CHECK(start.kind == StartCondition::Kind::Runway);
    CHECK(start.headingTrueDeg == doctest::Approx(90.0));
    CHECK(start.fieldElevFt == doctest::Approx(2000.0));
    const Runway* r09 = findRunway(*nav.findAirport("XTST"), "09");
    CHECK(greatCircleDistM(start.latDeg, start.lonDeg, r09->thrLatDeg, r09->thrLonDeg) ==
          doctest::Approx(ftToM(60.0)).epsilon(1e-6));

    REQUIRE(parseScenario("[scenario]\nstart = cruise\n", s, err));
    REQUIRE(resolveStart(s, nav, c172Profile(), start, err));
    CHECK(start.kind == StartCondition::Kind::Cruise);
    CHECK(start.altMslFt == doctest::Approx(6000.0));  // profile cruise altitude
    CHECK(start.iasKt == doctest::Approx(110.0));

    REQUIRE(parseScenario("[scenario]\nstart = cruise\naltitude_ft = 4500\n", s, err));
    REQUIRE(resolveStart(s, nav, c172Profile(), start, err));
    CHECK(start.altMslFt == doctest::Approx(4500.0));
}

TEST_CASE("scenario: unknown airport or runway fails") {
    SyntheticNavDb nav;
    Scenario s;
    std::string err;
    StartCondition start;
    REQUIRE(parseScenario("[scenario]\nairport = KSFO\n", s, err));
    CHECK_FALSE(resolveStart(s, nav, c172Profile(), start, err));
    CHECK(err.find("KSFO") != std::string::npos);
    REQUIRE(parseScenario("[scenario]\nrunway = 36\n", s, err));
    CHECK_FALSE(resolveStart(s, nav, c172Profile(), start, err));
    CHECK(err.find("36") != std::string::npos);
}

TEST_CASE("scenario: scripted inputs apply once, in time order") {
    ScriptedInputs script;
    script.setEvents({{0.0, ScriptCommand::Throttle, 1.0f},
                      {2.0, ScriptCommand::Elevator, 0.3f},
                      {2.0, ScriptCommand::Gear, 0.0f},
                      {5.0, ScriptCommand::Flaps, 2.0f}});
    ControlInputs c;
    c.gearDown = true;
    script.apply(0.0, 2, c);
    CHECK(c.throttleNorm[0] == doctest::Approx(1.0f));
    CHECK(c.throttleNorm[1] == doctest::Approx(1.0f));
    CHECK(c.throttleNorm[2] == doctest::Approx(0.0f));  // only engineCount engines
    CHECK(c.elevatorNorm == doctest::Approx(0.0f));
    script.apply(1.99, 2, c);
    CHECK(c.elevatorNorm == doctest::Approx(0.0f));
    script.apply(2.0, 2, c);
    CHECK(c.elevatorNorm == doctest::Approx(0.3f));
    CHECK_FALSE(c.gearDown);
    c.elevatorNorm = 0;  // an event never fires twice, so edits made since then survive
    script.apply(3.0, 2, c);
    CHECK(c.elevatorNorm == doctest::Approx(0.0f));
    script.apply(10.0, 2, c);
    CHECK(c.flapDetent == 2);
}

TEST_CASE("profile: placeholder profiles parse") {
    for (const char* id : {"c172p", "737"}) {
        AircraftProfile p;
        std::string err;
        REQUIRE_MESSAGE(loadAircraftProfile(std::string("assets/aircraft/") + id + ".ini", p, err),
                        err);
        CHECK(p.id == id);
        CHECK(p.engineCount >= 1);
        CHECK(p.flapDetents.size() >= 2);
        CHECK(p.vappKt > 0);
        CHECK(p.approachFlapDetent < static_cast<int>(p.flapDetents.size()));
    }
    AircraftProfile p;
    std::string err;
    REQUIRE(loadAircraftProfile("assets/aircraft/c172p.ini", p, err));
    CHECK(p.category == AircraftCategory::Piston);
    CHECK(p.jsbsimModel == "c172p");
    CHECK(p.flapDetents[2].cmdNorm == doctest::Approx(0.6667f));
    CHECK(std::string(p.flapDetents[0].label) == "UP");
    CHECK_FALSE(p.hasRetractableGear);

    CHECK_FALSE(parseAircraftProfile("[aircraft]\nid = x\nengines = 9\n", p, err));
    CHECK_FALSE(parseAircraftProfile("[other]\n", p, err));
    CHECK_FALSE(loadAircraftProfile("/no/such.ini", p, err));
}

TEST_CASE("scenario: end_agl_ft is parsed, and the ILS demos end above the flare") {
    Scenario sc;
    std::string err;
    REQUIRE_MESSAGE(parseScenario("[scenario]\nairport = XTST\nend_agl_ft = 100\n", sc, err), err);
    CHECK(sc.endAglFt == doctest::Approx(100.0f));
    REQUIRE(parseScenario("[scenario]\nairport = XTST\n", sc, err));
    CHECK(sc.endAglFt == 0.0f);
}
