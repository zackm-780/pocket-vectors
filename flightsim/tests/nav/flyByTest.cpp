// N10: fly-by turn arcs. Geometry (radius, anticipation, tangency, arc guidance) and the navigator
// guiding along the arc between two TF legs. No simulation.
#include <doctest/doctest.h>

#include <cmath>
#include <cstring>

#include "core/SyntheticNavDb.h"
#include "nav/FlyBy.h"
#include "nav/Navigator.h"
#include "nav/navigatorTestUtil.h"

using namespace fsim;
using namespace fsim::test;

namespace {

constexpr double kLat = 45.0, kLon = -100.0;

// Course (true deg) of the leg that leaves `fix` and the fix `turnDeg` (+ = right) after an
// inbound leg flown at `inDeg`, each leg `legNm` long.
struct Corner {
    LatLon from, fix, to;
};
Corner corner(double inDeg, double turnDeg, double legNm = 10.0) {
    Corner c;
    c.fix = {kLat, kLon};
    c.from = pointAtNm(kLat, kLon, inDeg + 180.0, legNm);
    c.to = pointAtNm(kLat, kLon, inDeg + turnDeg, legNm);
    return c;
}

double distNm(const LatLon& a, const LatLon& b) {
    return mToNm(greatCircleDistM(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg));
}

double courseDeg(const LatLon& a, const LatLon& b) {
    double c = radToDeg(initialBearingTrueRad(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg));
    return c < 0 ? c + 360.0 : c;
}

Leg tf(const char* ident, const LatLon& p, LegType type = LegType::TF) {
    Leg l;
    l.type = type;
    l.to = Waypoint{ident, WaypointKind::Fix, p.latDeg, p.lonDeg};
    return l;
}

}  // namespace

TEST_CASE("nav: fly-by radius is V^2 / (g tan(bank)), the bank is min(25, limit - 5)") {
    const double v100 = ktToMps(100.0), v250 = ktToMps(250.0);
    CHECK(flyByRadiusNm(100, 15) ==
          doctest::Approx(mToNm(v100 * v100 / (kStdGravityMps2 * std::tan(degToRad(15.0)))))
              .epsilon(1e-9));
    CHECK(flyByRadiusNm(250, 20) ==
          doctest::Approx(mToNm(v250 * v250 / (kStdGravityMps2 * std::tan(degToRad(20.0)))))
              .epsilon(1e-9));
    CHECK(flyByRadiusNm(10, 15) == doctest::Approx(flyByRadiusNm(40, 15)));  // 40 kt minimum
    CHECK(flyByBankDeg(20) == doctest::Approx(15));
    CHECK(flyByBankDeg(25) == doctest::Approx(20));
    CHECK(flyByBankDeg(35) == doctest::Approx(25));
    CHECK(flyByBankForRadiusDeg(250, flyByRadiusNm(250, 20)) == doctest::Approx(20).epsilon(1e-6));
}

TEST_CASE("nav: fly-by arcs are tangent to both legs at R tan(|turn|/2) from the fix") {
    const double r = 1.0;
    for (const double inDeg : {90.0, 0.0, 225.0}) {
        for (const double turnDeg : {30.0, 90.0, 134.0, -30.0, -90.0, -134.0}) {
            CAPTURE(inDeg);
            CAPTURE(turnDeg);
            const Corner c = corner(inDeg, turnDeg);
            const FlyByTurn t = flyByTurn(c.from, c.fix, c.to, r);
            REQUIRE(t.valid);
            // The legs are great circles: the in-course at the fix differs from inDeg by the
            // meridian convergence (about 0.2 degree over 10 NM at 45 N).
            CHECK(std::fabs(t.turnDeg - turnDeg) < 0.3);
            CHECK(std::fabs(wrap180(t.inCourseTrueDeg - courseDeg(c.from, c.fix))) < 0.3);
            CHECK(std::fabs(wrap180(t.outCourseTrueDeg - courseDeg(c.fix, c.to))) < 0.15);
            const double d = r * std::tan(degToRad(std::fabs(t.turnDeg)) * 0.5);
            CHECK(t.anticipationNm == doctest::Approx(d).epsilon(1e-6));
            CHECK(distNm(t.start, c.fix) == doctest::Approx(d).epsilon(0.002));
            CHECK(distNm(t.end, c.fix) == doctest::Approx(d).epsilon(0.002));
            // T1 on the inbound leg, T2 on the outbound one.
            CHECK(std::fabs(wrap180(courseDeg(t.start, c.fix) - t.inCourseTrueDeg)) < 0.1);
            CHECK(std::fabs(wrap180(courseDeg(c.fix, t.end) - t.outCourseTrueDeg)) < 0.1);
            // Both at R from the centre, on the turn side.
            CHECK(distNm(t.center, t.start) == doctest::Approx(r).epsilon(0.002));
            CHECK(distNm(t.center, t.end) == doctest::Approx(r).epsilon(0.002));
            // The arc's tangent at T1 is the in-course, at T2 the out-course, XTK 0 on the arc.
            const ArcGuide g1 = arcGuide(t, t.start), g2 = arcGuide(t, t.end);
            CHECK(std::fabs(wrap180(g1.dtkTrueDeg - t.inCourseTrueDeg)) < 0.1);
            CHECK(std::fabs(wrap180(g2.dtkTrueDeg - t.outCourseTrueDeg)) < 0.1);
            CHECK(std::fabs(g1.xtkNm) < 0.002);
            CHECK(g1.progress01 == doctest::Approx(0).epsilon(0.002));
            CHECK(g2.progress01 == doctest::Approx(1).epsilon(0.002));
            // Points along the arc: every one at R, from T1 to T2.
            LatLon pts[32];
            const int n = arcPoints(t, pts, 32);
            CHECK(n >= 4);
            for (int k = 0; k < n; ++k) {
                CHECK(distNm(t.center, pts[k]) == doctest::Approx(r).epsilon(0.002));
                CHECK(std::fabs(arcGuide(t, pts[k]).xtkNm) < 0.002);
            }
        }
    }
    // Too small or too large a turn: no arc.
    CHECK_FALSE(flyByTurn(corner(90, 0.5).from, corner(90, 0.5).fix, corner(90, 0.5).to, r).valid);
    CHECK_FALSE(flyByTurn(corner(90, 150).from, corner(90, 150).fix, corner(90, 150).to, r).valid);
}

TEST_CASE("nav: arc guidance: XTK sign, DTK along the tangent, done past T2") {
    for (const double turnDeg : {90.0, -90.0}) {
        CAPTURE(turnDeg);
        const Corner c = corner(90.0, turnDeg);  // inbound east, then south (right) or north (left)
        const FlyByTurn t = flyByTurn(c.from, c.fix, c.to, 1.0);
        REQUIRE(t.valid);
        LatLon pts[16];
        const int n = arcPoints(t, pts, 16);
        const LatLon mid = pts[n / 2];
        // 0.1 NM toward the centre: inside the circle. Right of a right turn, left of a left one.
        const double toCenter = courseDeg(mid, t.center);
        const LatLon inside = pointAtNm(mid.latDeg, mid.lonDeg, toCenter, 0.1);
        const LatLon outside = pointAtNm(mid.latDeg, mid.lonDeg, toCenter + 180.0, 0.1);
        const ArcGuide gi = arcGuide(t, inside), go = arcGuide(t, outside);
        CHECK(gi.xtkNm == doctest::Approx(turnDeg > 0 ? 0.1 : -0.1).epsilon(0.02));
        CHECK(go.xtkNm == doctest::Approx(turnDeg > 0 ? -0.1 : 0.1).epsilon(0.02));
        // Along the arc the DTK turns with the swept angle.
        const ArcGuide gm = arcGuide(t, mid);
        CHECK(std::fabs(wrap180(gm.dtkTrueDeg -
                                (t.inCourseTrueDeg + t.turnDeg * gm.progress01))) < 0.2);
        CHECK(gm.progress01 > 0.3);
        CHECK(gm.progress01 < 0.7);
        CHECK_FALSE(arcGuide(t, mid).done);
        const LatLon past = pointAtNm(t.end.latDeg, t.end.lonDeg, 90.0 + turnDeg, 0.2);
        CHECK(arcGuide(t, past).done);
    }
}

TEST_CASE("nav: a fly-by arc shrinks to fit half of the shorter leg") {
    const Corner c = corner(90.0, 90.0, 2.0);  // 2 NM legs, a 3 NM radius would not fit
    const FlyByTurn t = flyByTurn(c.from, c.fix, c.to, 3.0);
    REQUIRE(t.valid);
    CHECK(t.anticipationNm == doctest::Approx(1.0).epsilon(0.002));
    CHECK(t.radiusNm == doctest::Approx(1.0).epsilon(0.002));
}

TEST_CASE("nav: plan legs: fly-by arcs at interior TF fixes only, spans end at the arcs") {
    const Corner c = corner(90.0, 90.0);
    FlightPlan plan;
    plan.legs = {tf("A", c.from, LegType::IF), tf("B", c.fix), tf("C", c.to)};
    plan.activeLeg = 1;
    CHECK_FALSE(flyByEligible(plan, 0));  // the origin
    CHECK(flyByEligible(plan, 1));
    CHECK_FALSE(flyByEligible(plan, 2));  // the last fix
    const LegSpan s1 = legSpan(plan, 1, 1.0), s2 = legSpan(plan, 2, 1.0);
    CHECK_FALSE(s1.turnIn.valid);
    CHECK(s2.turnIn.valid);
    CHECK(distNm(s1.end, c.fix) == doctest::Approx(1.0).epsilon(0.002));    // T1
    CHECK(distNm(s2.start, c.fix) == doctest::Approx(1.0).epsilon(0.002));  // T2
    plan.legs[1].flyOver = true;
    CHECK_FALSE(flyByEligible(plan, 1));
    CHECK_FALSE(legSpan(plan, 2, 1.0).turnIn.valid);
    plan.legs[1].flyOver = false;
    plan.legs[2].type = LegType::CF;  // a course leg next: flown over
    CHECK_FALSE(flyByEligible(plan, 1));
    CHECK_FALSE(legSpan(plan, 2, 0.0).turnIn.valid);  // radius 0: sharp corners
}

TEST_CASE("nav: the navigator guides along the arc from T1 to T2, then along the next leg") {
    SyntheticNavDb db;
    const Corner c = corner(90.0, 90.0);  // east, then south
    FlightPlan plan;
    plan.legs = {tf("A", c.from, LegType::IF), tf("B", c.fix), tf("C", c.to)};
    plan.activeLeg = 1;
    Navigator nav(db);
    nav.setPlan(plan);
    const double gs = 100.0;
    // On the inbound leg, 3 NM before B.
    LatLon p = pointAtNm(kLat, kLon, 270.0, 3.0);
    nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 90, gs), 0.1);
    CHECK_FALSE(nav.state().turnActive);
    CHECK(nav.state().turnRadiusNm == doctest::Approx(flyByRadiusNm(gs, 15)).epsilon(1e-4));
    // Fly the arc the navigator will use (radius at 100 kt, 15 degrees).
    const FlyByTurn t = flyByTurn(c.from, c.fix, c.to, flyByRadiusNm(gs, 15));
    LatLon pts[16];
    const int n = arcPoints(t, pts, 16);
    // Just before T1 (inside the roll-in lead) it sequences and the turn begins.
    p = pointAtNm(t.start.latDeg, t.start.lonDeg, 270.0, 0.03);
    nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 90, gs), 0.1);
    REQUIRE(nav.plan().activeLeg == 2);
    CHECK(nav.state().turnActive);
    CHECK(nav.state().turnBankDeg == doctest::Approx(15).epsilon(0.01));  // right
    CHECK(nav.state().legTrackTrueDeg == doctest::Approx(180).epsilon(0.002));
    double lastDtk = 89;
    for (int k = 1; k < n - 1; ++k) {
        const double hdg = arcGuide(t, pts[k]).dtkTrueDeg;
        nav.update(makeAircraft(pts[k].latDeg, pts[k].lonDeg, 5000, hdg, gs), 0.1);
        const NavState& s = nav.state();
        CAPTURE(k);
        CHECK(s.turnActive);
        CHECK(std::fabs(s.crossTrackNm) < 0.003);
        CHECK(std::fabs(s.cdiDeflection) < 0.01);
        CHECK(s.desiredTrackTrueDeg > lastDtk);  // rotating from 90 toward 180
        lastDtk = s.desiredTrackTrueDeg;
        CHECK(s.legTrackTrueDeg == doctest::Approx(180).epsilon(0.002));
    }
    // Past T2: the leg to C, XTK about 0.
    p = pointAtNm(t.end.latDeg, t.end.lonDeg, 180.0, 0.1);
    nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 180, gs), 0.1);
    CHECK_FALSE(nav.state().turnActive);
    CHECK(std::fabs(nav.state().crossTrackNm) < 0.003);
    CHECK(nav.state().desiredTrackTrueDeg == doctest::Approx(180).epsilon(0.002));
}

TEST_CASE("nav: a turn ends on direct-to, and far off the arc") {
    SyntheticNavDb db;
    const Corner c = corner(90.0, 90.0);
    FlightPlan plan;
    plan.legs = {tf("A", c.from, LegType::IF), tf("B", c.fix), tf("C", c.to)};
    plan.activeLeg = 1;
    const FlyByTurn t = flyByTurn(c.from, c.fix, c.to, flyByRadiusNm(100, 15));
    auto enterTurn = [&](Navigator& nav) {
        nav.setPlan(plan);
        const LatLon p = pointAtNm(t.start.latDeg, t.start.lonDeg, 270.0, 0.03);
        nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 90, 100), 0.1);
        REQUIRE(nav.state().turnActive);
    };
    {
        Navigator nav(db);
        enterTurn(nav);
        REQUIRE(nav.directToLeg(2));
        nav.update(makeAircraft(t.start.latDeg, t.start.lonDeg, 5000, 90, 100), 0.1);
        CHECK_FALSE(nav.state().turnActive);
    }
    {
        Navigator nav(db);
        enterTurn(nav);
        const LatLon far = pointAtNm(t.center.latDeg, t.center.lonDeg, 0.0, t.radiusNm + 1.5);
        nav.update(makeAircraft(far.latDeg, far.lonDeg, 5000, 90, 100), 0.1);
        CHECK_FALSE(nav.state().turnActive);  // the NAV intercept of the leg takes over
    }
}
