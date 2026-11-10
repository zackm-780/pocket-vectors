// N10b: the AP in NAV flies the new fly-by arcs: onto an IF in mid-plan (the approach's first
// leg), onto a straight CF, and through a same-fix leg (the approach's IF on the last enroute
// fix). Closed loop: the simulation, the navigator and the autopilot over XTST.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "SimFixture.h"
#include "core/Geo.h"
#include "core/Units.h"
#include "nav/FlyBy.h"
#include "nav/navigatorTestUtil.h"

using namespace fsim;
using namespace fsim::test;

namespace {

Leg legTo(const char* ident, const LatLon& p, LegType type) {
    Leg l;
    l.type = type;
    l.to = Waypoint{ident, WaypointKind::Fix, p.latDeg, p.lonDeg};
    return l;
}

double courseDeg(const LatLon& a, const LatLon& b) {
    return wrap360(radToDeg(initialBearingTrueRad(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg)));
}

struct FlightResult {
    int arcTurns = 0;        // turns flown with turnActive
    double maxArcXtkNm = 0;  // during the arcs
    double maxLegXtkNm = 0;  // after each arc, on the legs
    bool reachedLast = false;
};

// Present position (IF), then 4 NM ahead A (TF), a 60 degree right turn onto B (an IF: the
// approach's first leg), a 45 degree left turn onto C (a CF whose course is the line B -> C),
// and D straight on. With `sameFix`, an IF at A repeats A (the approach starts on the last
// enroute fix) and B is a TF.
FlightResult fly(const char* aircraft, bool sameFix) {
    SimFixture f;
    REQUIRE_MESSAGE(f.initText(std::string("[scenario]\naircraft = ") + aircraft +
                               "\nairport = XTST\nrunway = 27\nstart = cruise\n"
                               "altitude_ft = 4500\nplan = assets/plans/XTST-lnav.fms\n"
                               "[autopilot]\nengage = ap nav alt\nalt_bug_ft = 4500\n"),
                    f.error);
    f.run(1.0);
    const AircraftState& s0 = f.sim.aircraft();
    const double hdg = radToDeg(s0.headingTrueRad);
    const LatLon p0{s0.latDeg, s0.lonDeg};
    const LatLon a = pointAtNm(p0.latDeg, p0.lonDeg, hdg, 4.0);
    const LatLon b = pointAtNm(a.latDeg, a.lonDeg, hdg + 60.0, 5.0);
    const LatLon c = pointAtNm(b.latDeg, b.lonDeg, hdg + 15.0, 5.0);
    const LatLon d = pointAtNm(c.latDeg, c.lonDeg, hdg + 15.0, 5.0);
    FlightPlan plan;
    plan.legs.push_back(legTo("PPOS", p0, LegType::IF));
    plan.legs.push_back(legTo("A", a, LegType::TF));
    if (sameFix) plan.legs.push_back(legTo("A", a, LegType::IF));
    plan.legs.push_back(legTo("B", b, sameFix ? LegType::TF : LegType::IF));
    Leg cf = legTo("C", c, LegType::CF);
    cf.courseTrueDeg = static_cast<float>(courseDeg(b, c));
    plan.legs.push_back(cf);
    plan.legs.push_back(legTo("D", d, LegType::TF));
    plan.activeLeg = 1;
    f.sim.navigator().setPlan(plan);

    FlightResult r;
    bool inArc = false;
    const int last = static_cast<int>(plan.legs.size()) - 1;
    for (long i = 0; i < static_cast<long>(600 * kSimHz) && !f.sim.aircraft().crashed; ++i) {
        f.sim.stepOnce();
        const NavState& n = f.sim.navState();
        const double xtk = std::fabs(static_cast<double>(n.crossTrackNm));
        if (n.turnActive) {
            if (!inArc) ++r.arcTurns;
            r.maxArcXtkNm = std::max(r.maxArcXtkNm, xtk);
        } else if (inArc || r.arcTurns > 0) {
            r.maxLegXtkNm = std::max(r.maxLegXtkNm, xtk);
        }
        inArc = n.turnActive;
        if (n.activeLegIndex == last) {
            r.reachedLast = true;
            break;
        }
    }
    CHECK(f.sim.autopilotState().latActive == LatMode::Nav);
    std::printf("[N10b] %s%s: %d arc turns, max XTK on the arcs %.3f NM, on the legs %.3f NM\n",
                aircraft, sameFix ? " (same-fix IF)" : "", r.arcTurns, r.maxArcXtkNm,
                r.maxLegXtkNm);
    return r;
}

}  // namespace

TEST_CASE("sim: N10b the AP in NAV flies arcs onto a mid-plan IF and a straight CF") {
    for (const char* ac : {"c172p", "737"}) {
        for (const bool sameFix : {false, true}) {
            CAPTURE(ac);
            CAPTURE(sameFix);
            const FlightResult r = fly(ac, sameFix);
            CHECK(r.reachedLast);
            CHECK(r.arcTurns >= 2);  // at A onto B, and at B onto the CF to C
            CHECK(r.maxArcXtkNm < (std::string(ac) == "737" ? 0.2 : 0.1));
        }
    }
}
