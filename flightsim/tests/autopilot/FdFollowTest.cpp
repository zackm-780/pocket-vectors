// 1.3 (owner, 2026-11-08): "Flight director points the wrong way. If I follow it, I get off the
// GPS track." A scripted pilot who flies the FPV onto the FD cue (AP off, FD on, NAV) must track a
// GPS plan with turns as closely as the autopilot does, from the entry the device used: NAV pressed
// with the AP and the FD off (the FD comes on in ROL with NAV armed) while off course and moving
// away from it.
//
// The real-data case (KPDX-KOTH cruise start) runs only with FSIM_REAL_DATA_DIR set:
//   FSIM_REAL_DATA_DIR=/path/to/data ./flightsim_tests --test-case='*fd real data*'
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include "SimFixture.h"
#include "autopilot/FdPilot.h"
#include "core/Units.h"
#include "nav/NavDb.h"
#include "world/terrain/TerrainService.h"

using namespace fsim;
using namespace fsim::test;

namespace {

ActionState pressed(std::initializer_list<Action> actions) {
    ActionState a;
    for (Action x : actions) a.pressed[static_cast<int>(x)] = true;
    return a;
}

struct Track {
    bool intercepted = false;
    double interceptS = -1;
    bool joinedPressLeg = false;  // the first intercept is on the leg active at the press
    double maxXtkNm = 0;          // after the intercept, through four turns
    double rmsXtkNm = 0;          // from the first intercept to the end, turns included
    int legChanges = 0;
    bool crashed = false;
    LatMode endLat = LatMode::Off;
};

// XTST cruise start (heading 270, on the first leg of XTST-lnav.fms), then HDG 300 with the AP for
// offS seconds: right of course and diverging. Then the AP and the FD go off and the pilot presses
// NAV and ALT (fd) or AP, NAV and ALT (ap). Measures the intercept and the cross-track after it.
Track flyFromTrap(const char* aircraft, bool fdPilot, const std::string& wind) {
    const bool jet = std::string(aircraft) == "737";
    const double altFt = jet ? 6000 : 4500;
    SimFixture f;
    const std::string scenario =
        "[scenario]\naircraft = " + std::string(aircraft) +
        "\nairport = XTST\nrunway = 27\nstart = cruise\naltitude_ft = " + std::to_string(altFt) +
        "\nplan = assets/plans/XTST-lnav.fms\n[wind]\n" + wind +
        "\n[autopilot]\nengage = ap hdg alt" + (jet ? " at" : "") +
        "\nhdg_bug_mag_deg = 300\nalt_bug_ft = " + std::to_string(altFt) +
        (jet ? "\nspd_bug_kt = 250\n" : "\n");
    REQUIRE_MESSAGE(f.initText(scenario), f.error);
    Autopilot& ap = f.sim.autopilot();
    for (int i = 0; i < 300 && f.sim.navState().crossTrackNm < 0.5f; ++i) f.run(1.0);
    REQUIRE(f.sim.navState().crossTrackNm >= 0.5f);  // right of course, heading away
    ap.disengageAp(false);
    ap.setFd(false, f.sim.aircraft());
    REQUIRE_FALSE(ap.state().fdOn);
    const double pressS = f.sim.aircraft().simTimeS;
    if (fdPilot)
        ap.handleActions(pressed({Action::NavMode, Action::AltMode}), f.sim.aircraft(),
                         f.sim.navState(), 1.0 / 60);
    else
        ap.handleActions(pressed({Action::ApToggle, Action::NavMode, Action::AltMode}),
                         f.sim.aircraft(), f.sim.navState(), 1.0 / 60);
    REQUIRE(ap.state().fdOn == fdPilot);
    REQUIRE(ap.state().apEngaged != fdPilot);

    FdPilot pilot;
    Track t;
    int lastLeg = f.sim.navState().activeLegIndex;
    bool joined = false;  // the active leg has been joined (|XTK| < 0.1 NM)
    double sumSq = 0;
    long samples = 0;
    for (long i = 0; i < 1500L * kSimHz; ++i) {
        if (fdPilot) pilot.step(f.sim);
        f.sim.stepOnce();
        const NavState& n = f.sim.navState();
        const AircraftState& s = f.sim.aircraft();
        // The cross-track jumps at a leg change (the new leg starts at the fix the aircraft is
        // still short of); measure each leg once it has been joined, as the N6 LNAV flight does.
        if (n.activeLegIndex != lastLeg) {
            lastLeg = n.activeLegIndex;
            ++t.legChanges;
            joined = false;
        }
        if (std::fabs(n.crossTrackNm) < 0.1f) {
            if (!t.intercepted) {
                t.intercepted = true;
                t.interceptS = s.simTimeS - pressS;
                t.joinedPressLeg = t.legChanges == 0;
            }
            joined = true;
        }
        if (t.intercepted && joined && t.legChanges <= 4)
            t.maxXtkNm = std::max(t.maxXtkNm, static_cast<double>(std::fabs(n.crossTrackNm)));
        if (t.intercepted) {
            sumSq += static_cast<double>(n.crossTrackNm) * n.crossTrackNm;
            ++samples;
        }
        if (s.crashed) {
            t.crashed = true;
            break;
        }
        if (t.legChanges >= 5) break;
    }
    t.endLat = f.sim.autopilotState().latActive;
    t.rmsXtkNm = samples ? std::sqrt(sumSq / static_cast<double>(samples)) : 0;
    return t;
}

}  // namespace

TEST_CASE("sim: FD-following pilot joins and tracks the GPS plan like the AP (c172p, 737)") {
    struct Case {
        const char* aircraft;
        const char* wind;
        const char* name;
    };
    const Case cases[] = {
        {"c172p", "from_true_deg = 0\nspeed_kt = 10\n", "10 kt crosswind"},
        {"c172p", "from_true_deg = 0\nspeed_kt = 25\n", "25 kt crosswind"},
        {"737", "from_true_deg = 0\nspeed_kt = 10\n", "10 kt crosswind"},
        {"737", "from_true_deg = 30\nspeed_kt = 40\n", "40 kt quartering"},
    };
    for (const Case& c : cases) {
        CAPTURE(c.aircraft);
        CAPTURE(c.name);
        const Track ap = flyFromTrap(c.aircraft, false, c.wind);
        const Track fd = flyFromTrap(c.aircraft, true, c.wind);
        std::printf("[FD] %s %s: AP intercept %.0f s, max XTK %.3f NM, rms %.3f | FD pilot "
                    "intercept %.0f s, max XTK %.3f NM, rms %.3f, %d legs\n",
                    c.aircraft, c.name, ap.interceptS, ap.maxXtkNm, ap.rmsXtkNm, fd.interceptS,
                    fd.maxXtkNm, fd.rmsXtkNm, fd.legChanges);
        CHECK_FALSE(fd.crashed);
        CHECK(ap.intercepted);
        CHECK(fd.intercepted);  // before 1.3: ROL (NAV) held the wings level, never joined
        const bool c172 = std::string(c.aircraft) == "c172p";
        if (c172) {  // the 737 reaches the end of the 6.9 NM first leg before it can join
            CHECK(ap.joinedPressLeg);
            CHECK(fd.joinedPressLeg);  // before 1.3: only once the leg sequenced past its fix
            CHECK(fd.interceptS < 120.0);
        }
        CHECK(fd.legChanges >= 5);  // flew the plan's turns
        CHECK(fd.endLat == LatMode::Nav);
        // The c172p finishes its 90 degree turns inside the 4.8 NM legs; the 737 at 250 kt cannot
        // (both the AP and the FD overshoot alike), which the fly-by turn arcs address.
        if (c172) CHECK(fd.maxXtkNm < 0.3);
        // As close to the track as the AP: within 25 % or 0.05 NM, whichever is larger.
        CHECK(fd.maxXtkNm <= std::max(ap.maxXtkNm * 1.25, ap.maxXtkNm + 0.05));
        CHECK(fd.rmsXtkNm <= std::max(ap.rmsXtkNm * 1.25, ap.rmsXtkNm + 0.02));
    }
}

// Manual: the device entry on the real KPDX-KOTH plan. The cruise start is about 0.8 NM right of
// the first leg (KPDX -> UBG, 231 true) heading 299, away from it. Before 1.3 an FD-following
// pilot was 26 NM off course 15 minutes later.
TEST_CASE("sim: fd real data: KPDX-KOTH, NAV pressed off course, FD-following pilot joins") {
    const char* env = std::getenv("FSIM_REAL_DATA_DIR");
    if (!env || !*env) return;
    Paths paths = Paths::resolve({});
    CsvNavDatabase nav;
    std::string err;
    REQUIRE_MESSAGE(nav.load(Paths::join(env, "nav"), err), err);
    terrain::TerrainService svc(Paths::join(env, "terrain"));
    REQUIRE(svc.isOpen());
    for (bool fdPilot : {false, true}) {
        Simulation sim(paths, nav, svc);
        sim.setTerrainPump([&](double la, double lo) { svc.update(la, lo); });
        Scenario s;
        REQUIRE_MESSAGE(
            loadScenario(paths.asset("assets/scenarios/c172_cruise_kpdx_koth_plan.ini"), s, err),
            err);
        REQUIRE_MESSAGE(sim.init(s, "", err), err);
        const NavState n0 = sim.navState();
        sim.autopilot().handleActions(
            fdPilot ? pressed({Action::NavMode, Action::AltMode})
                    : pressed({Action::ApToggle, Action::NavMode, Action::AltMode}),
            sim.aircraft(), n0, 1.0 / 60);
        FdPilot pilot;
        double interceptS = -1, maxAfterNm = 0;
        int startLeg = n0.activeLegIndex;
        for (long i = 0; i < 900L * kSimHz; ++i) {
            if (fdPilot) pilot.step(sim);
            sim.stepOnce();
            svc.update(sim.aircraft().latDeg, sim.aircraft().lonDeg);
            const NavState& n = sim.navState();
            if (interceptS < 0 && std::fabs(n.crossTrackNm) < 0.1f)
                interceptS = sim.aircraft().simTimeS;
            if (interceptS >= 0 && n.activeLegIndex == startLeg)
                maxAfterNm = std::max(maxAfterNm, static_cast<double>(std::fabs(n.crossTrackNm)));
        }
        const NavState& n = sim.navState();
        std::printf("[FD real] %s: start XTK %+.2f NM, intercept at %.0f s, max XTK after %.3f NM, "
                    "now leg %d XTK %+.3f NM\n",
                    fdPilot ? "FD pilot" : "AP      ", n0.crossTrackNm, interceptS, maxAfterNm,
                    n.activeLegIndex, n.crossTrackNm);
        CHECK(interceptS > 0);
        CHECK(maxAfterNm < 0.3);
        CHECK(std::fabs(n.crossTrackNm) < 0.3f);
    }
}
