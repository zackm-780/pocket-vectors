// Scenario keys `plan` / `approach` wire the Navigator into the simulation (they fly JSBSim, hence
// the "sim:" prefix that ctest groups).
#include <doctest/doctest.h>

#include <cstring>

#include "SimFixture.h"

using namespace fsim;
using namespace fsim::test;

TEST_CASE("sim: scenario approach key loads the ILS and the navigator guides the final") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initFile("c172_ils27_xtst.ini"), f.error);
    f.run(2.0);
    const NavState& n = f.sim.navState();
    CHECK(n.phase == NavPhase::Approach);
    CHECK(n.source == NavSource::Loc);
    CHECK(n.locValid);
    CHECK(std::strcmp(n.approachName, "ILS 27") == 0);
    CHECK(std::fabs(n.cdiDeflection) < 0.3f);
    CHECK(n.gsValid);
    CHECK(std::fabs(n.gsDeflection) < 0.6f);
    CHECK(n.distToThresholdNm == doctest::Approx(6.0).epsilon(0.05));
    CHECK(n.minimumsFt == doctest::Approx(2200.0f));
}

TEST_CASE("sim: scenario plan and approach keys load a plan and pick the leg near the aircraft") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initText("[scenario]\nairport = XTST\nrunway = 09\nstart = final\n"
                               "distance_nm = 6\nplan = assets/plans/XTST-pattern.fms\n"
                               "approach = R09\n"),
                    f.error);
    f.run(1.0);
    const FlightPlan& p = f.sim.navigator().plan();
    CHECK(p.approachId == "R09");
    CHECK(p.approachKind == ApproachKind::Rnav);
    CHECK(f.sim.navState().phase == NavPhase::Approach);
    CHECK(f.sim.navState().source == NavSource::Gps);
    CHECK(f.sim.navState().cdiFullScaleNm > 0.0f);
    CHECK(f.sim.navState().activeLegIndex >= p.approachStartLeg);

    SimFixture bad;
    CHECK_FALSE(bad.initText("[scenario]\nstart = final\napproach = I99\n"));
    CHECK_FALSE(bad.error.empty());
    SimFixture noPlan;
    CHECK_FALSE(noPlan.initText("[scenario]\nplan = assets/plans/does-not-exist.fms\n"));
}
