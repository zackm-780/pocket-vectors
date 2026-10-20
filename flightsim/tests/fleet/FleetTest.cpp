// WP F2: flyability suite for the core fleet (c172p, c310, global5000, 737, B747).
//
// Every test flies the real JSBSim model headless through FlightModel with the scripted pilot in
// FleetPilot.h. Test names are "sim: fleet <id> <phase>", so ctest's sim_tests picks them up and
// one aircraft or phase can be selected with --test-case="sim: fleet 737 *". Each test also prints
// a "FLEET <id> <key> <value>" line (visible with -s) that feeds the table in README.md.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "FleetPhases.h"
#include "core/Log.h"

using namespace fsim;
using namespace fsim::test;
using namespace fsim::test::fleet;

// The overlay fixes (737 input sockets, global5000 csv output, c310 propeller) leave every core
// aircraft loading, starting and trimming without a JSBSim error or warning, and without stray
// output files.
TEST_CASE("sim: fleet overlay fixes load all aircraft without errors") {
    const std::string logPath = "fleet_overlay_log.txt";
    std::remove("global5000.csv");
    REQUIRE(log::openFile(logPath));
    log::setLevel(LogLevel::Warn);
    for (const FleetSpec& sp : kFleet) {
        for (const char* kind : {"runway", "cruise", "final"}) {
            Flight f;
            INFO(sp.id << " " << kind);
            REQUIRE_MESSAGE(f.begin(sp.id, kind), f.error);
            f.run(2.0);
            CHECK_FALSE(f.s().crashed);
        }
    }
    log::setLevel(LogLevel::Info);
    log::closeFile();
    std::ifstream in(logPath);
    std::stringstream text;
    text << in.rdbuf();
    INFO("log:\n" << text.str());
    CHECK(text.str().find("ERROR") == std::string::npos);
    CHECK(text.str().find("WARN") == std::string::npos);
    std::remove(logPath.c_str());
    CHECK_FALSE(std::ifstream("global5000.csv").good());
    // The overlay copies replaced the stock files in the runtime layout.
    for (const char* path :
         {"jsbsim/aircraft/737/737.xml", "jsbsim/aircraft/global5000/global5000.xml",
          "jsbsim/aircraft/c310/c310.xml"}) {
        std::ifstream xml(path);
        REQUIRE_MESSAGE(xml.good(), path);
        std::stringstream s;
        s << xml.rdbuf();
        CHECK_MESSAGE(s.str().find("OVERLAY (flightsim WP F2)") != std::string::npos, path);
    }
}

#define FLEET_CASES(PHASE, FN)                                                                     \
    TEST_CASE("sim: fleet c172p " PHASE) {                                                         \
        FN(kFleet[0]);                                                                             \
    }                                                                                              \
    TEST_CASE("sim: fleet c310 " PHASE) {                                                          \
        FN(kFleet[1]);                                                                             \
    }                                                                                              \
    TEST_CASE("sim: fleet global5000 " PHASE) {                                                    \
        FN(kFleet[2]);                                                                             \
    }                                                                                              \
    TEST_CASE("sim: fleet 737 " PHASE) {                                                           \
        FN(kFleet[3]);                                                                             \
    }                                                                                              \
    TEST_CASE("sim: fleet B747 " PHASE) {                                                          \
        FN(kFleet[4]);                                                                             \
    }

FLEET_CASES("profile", profileTest)
FLEET_CASES("takeoff and climb", takeoffClimbTest)
FLEET_CASES("cruise", cruiseTest)
FLEET_CASES("configuration changes", configTest)
FLEET_CASES("approach", approachTest)
FLEET_CASES("hands-off final", handsOffTest)
FLEET_CASES("landing", landingTest)
FLEET_CASES("stall", stallTest)
