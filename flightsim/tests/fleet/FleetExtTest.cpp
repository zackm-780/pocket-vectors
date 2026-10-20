// WP F4: flyability suite for the extended fleet (pa28, c182, J3Cub, DHC6, A320, 787-8, MD11).
// Same phases and pilot as the core fleet (FleetPhases.h); test names are
// "sim: fleet <id> <phase>". Autopilot checks for these aircraft are in tests/autopilot.
#include <doctest/doctest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "FleetPhases.h"
#include "FleetSpecsExt.h"
#include "core/Log.h"

using namespace fsim;
using namespace fsim::test;
using namespace fsim::test::fleet;

// Every extended aircraft loads, starts and trims in all three start kinds without a JSBSim
// error or warning (the log file sink is open for the duration).
TEST_CASE("sim: fleet extended fleet loads without errors") {
    const std::string logPath = "fleet_ext_overlay_log.txt";
    REQUIRE(log::openFile(logPath));
    log::setLevel(LogLevel::Warn);
    for (const FleetSpec& sp : kFleetExt) {
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
}

namespace {
// The stall deceleration starts from a trim in a changed configuration; heavy aircraft need about
// 30 s to settle before the load-factor drop means a stall.
void stallTestExt(const FleetSpec& sp) {
    stallTest(sp, 30.0);
}
}  // namespace

#define FLEET_EXT_CASES(PHASE, FN)                                                                 \
    TEST_CASE("sim: fleet A320 " PHASE) {                                                          \
        FN(kFleetExt[0]);                                                                          \
    }                                                                                              \
    TEST_CASE("sim: fleet 787-8 " PHASE) {                                                         \
        FN(kFleetExt[1]);                                                                          \
    }                                                                                              \
    TEST_CASE("sim: fleet MD11 " PHASE) {                                                          \
        FN(kFleetExt[2]);                                                                          \
    }

FLEET_EXT_CASES("profile", profileTest)
FLEET_EXT_CASES("takeoff and climb", takeoffClimbTest)
FLEET_EXT_CASES("cruise", cruiseTest)
FLEET_EXT_CASES("configuration changes", configTest)
FLEET_EXT_CASES("approach", approachTest)
FLEET_EXT_CASES("hands-off final", handsOffTest)
FLEET_EXT_CASES("landing", landingTest)
FLEET_EXT_CASES("stall", stallTestExt)
