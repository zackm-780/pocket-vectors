// N10: fly-by turns flown on the real JSBSim models. XTST-turns.fms has a 30 degree right turn,
// a 90 degree left turn and a 130 degree right turn on 12 NM legs (the 737's 130 degree arc at
// 20 degrees of bank needs 5.6 NM on each side). The AP in NAV (and a pilot
// following the FD) must fly each turn along the arc and roll out on the next leg without
// overshooting it.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "SimFixture.h"
#include "autopilot/FdPilot.h"
#include "core/Units.h"

using namespace fsim;
using namespace fsim::test;

namespace {

struct TurnResult {
    double maxArcXtkNm = 0;  // during the turn, from the arc
    double overshootNm = 0;  // after it, past the outbound leg (outside the turn)
    double maxBankDeg = 0;
    double turnDeg = 0;
    bool seen = false;
};

struct Flight {
    TurnResult turns[4];  // at the fixes ending legs 1, 2, 3 (the three turns)
    int legChanges = 0;
    bool crashed = false;
};

Flight flyTurns(const char* aircraft, const std::string& wind, bool fdPilot) {
    const bool jet = std::string(aircraft) == "737";
    const std::string alt = jet ? "6000" : "4500";
    SimFixture f;
    const std::string sc = "[scenario]\naircraft = " + std::string(aircraft) +
                           "\nairport = XTST\nrunway = 27\nstart = cruise\naltitude_ft = " + alt +
                           "\nplan = assets/plans/XTST-turns.fms\n[wind]\n" + wind +
                           "\n[autopilot]\nengage = " +
                           (fdPilot ? "fd ap nav alt" : (jet ? "ap at nav alt" : "ap nav alt")) +
                           "\nalt_bug_ft = " + alt + (jet ? "\nspd_bug_kt = 250\n" : "\n");
    REQUIRE_MESSAGE(f.initText(sc), f.error);
    if (fdPilot) f.sim.autopilot().disengageAp(false);  // the FD keeps NAV and ALT
    FdPilot pilot;
    Flight out;
    int lastLeg = f.sim.navState().activeLegIndex;
    int turnIdx = -1;         // which turn is being flown (0..2), -1 = none
    double afterUntilS = -1;  // overshoot window after a turn
    double turnSign = 0;
    for (long i = 0; i < 1800L * kSimHz; ++i) {
        if (fdPilot) pilot.step(f.sim);
        f.sim.stepOnce();
        const NavState& n = f.sim.navState();
        const AircraftState& s = f.sim.aircraft();
        if (n.activeLegIndex != lastLeg) {
            lastLeg = n.activeLegIndex;
            ++out.legChanges;
            if (n.turnActive && n.activeLegIndex >= 2 && n.activeLegIndex <= 4) {
                turnIdx = n.activeLegIndex - 2;
                turnSign = n.turnBankDeg > 0 ? 1.0 : -1.0;
                out.turns[turnIdx].seen = true;
            }
        }
        if (turnIdx >= 0) {
            TurnResult& t = out.turns[turnIdx];
            if (n.turnActive) {
                t.maxArcXtkNm =
                    std::max(t.maxArcXtkNm, static_cast<double>(std::fabs(n.crossTrackNm)));
                t.maxBankDeg = std::max(t.maxBankDeg, std::fabs(radToDeg(s.rollRad)));
                afterUntilS = s.simTimeS + 90.0;
            } else if (s.simTimeS < afterUntilS) {
                // Outside a right turn is left of the new leg (XTK < 0), and the reverse.
                t.overshootNm = std::max(t.overshootNm, -turnSign * n.crossTrackNm);
            } else {
                turnIdx = -1;
            }
        }
        if (s.crashed) {
            out.crashed = true;
            break;
        }
        if (n.activeLegIndex >= 5) break;  // the leg back to XTST
    }
    return out;
}

}  // namespace

TEST_CASE("sim: N10 fly-by turns of 30, 90 and 130 degrees: on the arc, no overshoot") {
    struct Case {
        const char* aircraft;
        const char* wind;
        const char* name;
    };
    const Case cases[] = {
        {"c172p", "", "calm"},
        {"c172p", "from_true_deg = 0\nspeed_kt = 20\n", "20 kt from the north"},
        {"737", "", "calm"},
        {"737", "from_true_deg = 0\nspeed_kt = 20\n", "20 kt from the north"},
    };
    const double turnDeg[3] = {30, -90, 130};
    for (const Case& c : cases) {
        const bool jet = std::string(c.aircraft) == "737";
        const Flight ap = flyTurns(c.aircraft, c.wind, false);
        const Flight fd = flyTurns(c.aircraft, c.wind, true);
        CAPTURE(c.aircraft);
        CAPTURE(c.name);
        CHECK_FALSE(ap.crashed);
        CHECK_FALSE(fd.crashed);
        for (int k = 0; k < 3; ++k) {
            const TurnResult& a = ap.turns[k];
            const TurnResult& d = fd.turns[k];
            std::printf(
                "[N10] %s %s, %+.0f deg turn: AP arc XTK %.3f NM, overshoot %.3f NM, bank %.1f | "
                "FD pilot arc XTK %.3f, overshoot %.3f\n",
                c.aircraft, c.name, turnDeg[k], a.maxArcXtkNm, a.overshootNm, a.maxBankDeg,
                d.maxArcXtkNm, d.overshootNm);
            CAPTURE(k);
            CHECK(a.seen);
            CHECK(d.seen);
            CHECK(a.maxArcXtkNm < (jet ? 0.2 : 0.1));
            CHECK(a.overshootNm < 0.05);
            // A pilot following the FD stays within the same bounds. (Where the turn needs the
            // whole bank limit, the 737's 130 degrees with the wind behind it at 26 degrees, the
            // pilot's 0.6 s reaction lag costs about 0.1 NM more than the AP.)
            CHECK(d.maxArcXtkNm < (jet ? 0.2 : 0.1));
            CHECK(d.overshootNm < 0.05);
        }
    }
}
