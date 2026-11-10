// N10: fly-by turn arcs. Geometry (radius, anticipation, tangency, arc guidance) and the navigator
// guiding along the arc between two TF legs. No simulation.
#include <doctest/doctest.h>

#include <algorithm>
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
        CHECK(std::fabs(wrap180(gm.dtkTrueDeg - (t.inCourseTrueDeg + t.turnDeg * gm.progress01))) <
              0.2);
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

TEST_CASE("nav: N10b arcs at an IF in mid-plan, DF, a straight CF and through a same-fix leg") {
    const Corner c = corner(90.0, 90.0);  // east, then south
    auto plan3 = [&](LegType nextType) {
        FlightPlan p;
        p.legs = {tf("A", c.from, LegType::IF), tf("B", c.fix), tf("C", c.to, nextType)};
        p.activeLeg = 1;
        return p;
    };
    // An IF in mid-plan (the approach's first leg after the enroute legs) and a DF next.
    CHECK(flyByEligible(plan3(LegType::IF), 1));
    CHECK(flyByEligible(plan3(LegType::DF), 1));
    CHECK(legSpan(plan3(LegType::IF), 2, 1.0).turnIn.valid);
    // Leg 0's IF never: it is the origin.
    CHECK_FALSE(flyByEligible(plan3(LegType::IF), 0));
    // A CF next: straight when its course is within 5 degrees of the bearing B -> C (180) and its
    // line passes within 0.1 NM of B (the legs here are 10 NM).
    FlightPlan cf = plan3(LegType::CF);
    cf.legs[2].courseTrueDeg = 180.4;  // 0.07 NM off at B
    CHECK(flyByEligible(cf, 1));
    cf.legs[2].courseTrueDeg = 183.0;  // within 5 degrees, but 0.52 NM off at B
    CHECK_FALSE(flyByEligible(cf, 1));
    cf.legs[2].courseTrueDeg = 190.0;  // 10 degrees off: flown over, then intercepted
    CHECK_FALSE(flyByEligible(cf, 1));
    const Corner shortLegs = corner(90.0, 90.0, 1.5);  // 3 degrees off on 1.5 NM: 0.08 NM
    FlightPlan cfShort;
    cfShort.legs = {tf("A", shortLegs.from, LegType::IF), tf("B", shortLegs.fix),
                    tf("C", shortLegs.to, LegType::CF)};
    cfShort.legs[2].courseTrueDeg = 183.0;
    CHECK(flyByEligible(cfShort, 1));
    // A straight CF into the fix, too.
    FlightPlan cfIn = plan3(LegType::TF);
    cfIn.legs[1].type = LegType::CF;
    cfIn.legs[1].courseTrueDeg = 89.6;
    CHECK(flyByEligible(cfIn, 1));
    cfIn.legs[1].courseTrueDeg = 70.0;
    CHECK_FALSE(flyByEligible(cfIn, 1));
    // The fix before skips a CA / VA leg, as the navigator does: A, climb to an altitude, B, C.
    FlightPlan va;
    va.legs = {tf("A", c.from, LegType::IF), tf("", c.from, LegType::VA), tf("B", c.fix),
               tf("C", c.to)};
    va.legs[1].to.latDeg += 0.01;  // the VA's computed end, off the line
    const FlyByTurn vt = planTurnAt(va, 2, 1.0);
    REQUIRE(vt.valid);
    CHECK(vt.inCourseTrueDeg == doctest::Approx(90).epsilon(0.002));  // from A, not the VA end
    // A Direct-To the destination airport with its approach kept (N11a rule 3): no turn there.
    FlightPlan dest = plan3(LegType::IF);
    dest.destIdent = "B";
    dest.legs[1].type = LegType::DF;
    dest.legs[1].to.kind = WaypointKind::Airport;
    CHECK_FALSE(flyByEligible(dest, 1));
    dest.destIdent = "OTHER";
    CHECK(flyByEligible(dest, 1));
    // Course legs, holds and fly-over fixes keep their sharp corner.
    CHECK_FALSE(flyByEligible(plan3(LegType::CA), 1));
    CHECK_FALSE(flyByEligible(plan3(LegType::HM), 1));
    FlightPlan over = plan3(LegType::TF);
    over.legs[1].flyOver = true;
    CHECK_FALSE(flyByEligible(over, 1));
    // More than 135 degrees: flown over, as before.
    const Corner sharp = corner(90.0, 150.0);
    FlightPlan big;
    big.legs = {tf("A", sharp.from, LegType::IF), tf("B", sharp.fix), tf("C", sharp.to)};
    CHECK_FALSE(planTurnAt(big, 1, 1.0).valid);

    // A same-fix leg: B, then an approach IF at B again, then C. The turn at B looks through it.
    FlightPlan dup;
    dup.legs = {tf("A", c.from, LegType::IF), tf("B", c.fix), tf("B", c.fix, LegType::IF),
                tf("C", c.to)};
    dup.activeLeg = 1;
    CHECK(sameFixLeg(dup, 2));
    CHECK_FALSE(sameFixLeg(dup, 1));
    CHECK(flyByOutboundLeg(dup, 1) == 3);
    CHECK(flyByEligible(dup, 1));
    CHECK_FALSE(flyByEligible(dup, 2));  // no turn of its own
    const FlyByTurn t = planTurnAt(dup, 1, 1.0);
    REQUIRE(t.valid);
    CHECK(t.turnDeg == doctest::Approx(90).epsilon(0.01));
    CHECK(legSpan(dup, 3, 1.0).turnIn.valid);  // drawn into C
    CHECK(distNm(legSpan(dup, 3, 1.0).start, c.fix) == doctest::Approx(1.0).epsilon(0.002));
    CHECK_FALSE(legSpan(dup, 2, 1.0).turnIn.valid);
}

TEST_CASE("nav: N10b the navigator flies the arc through a same-fix leg and onto a straight CF") {
    SyntheticNavDb db;
    const Corner c = corner(90.0, 90.0);  // east, then south
    const double gs = 100.0;
    const FlyByTurn t = flyByTurn(c.from, c.fix, c.to, flyByRadiusNm(gs, 15));
    LatLon pts[16];
    const int n = arcPoints(t, pts, 16);
    for (const bool sameFix : {true, false}) {
        CAPTURE(sameFix);
        FlightPlan plan;
        plan.legs = {tf("A", c.from, LegType::IF), tf("B", c.fix)};
        if (sameFix) plan.legs.push_back(tf("B", c.fix, LegType::IF));  // approach IF at B
        Leg cfLeg = tf("C", c.to, LegType::CF);  // straight CF (course 180 into C)
        cfLeg.courseTrueDeg = 180.0;
        plan.legs.push_back(cfLeg);
        plan.activeLeg = 1;
        Navigator nav(db);
        nav.setPlan(plan);
        LatLon p = pointAtNm(kLat, kLon, 270.0, 3.0);
        nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 90, gs), 0.1);
        CHECK_FALSE(nav.state().turnActive);
        // Just before T1: it sequences (passing the same-fix leg at once) and the arc begins.
        p = pointAtNm(t.start.latDeg, t.start.lonDeg, 270.0, 0.03);
        nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 90, gs), 0.1);
        const int cLeg = static_cast<int>(plan.legs.size()) - 1;
        CHECK(nav.plan().activeLeg == cLeg);
        CHECK(nav.state().turnActive);
        double maxXtk = 0;
        for (int k = 1; k < n - 1; ++k) {
            const double hdg = arcGuide(t, pts[k]).dtkTrueDeg;
            nav.update(makeAircraft(pts[k].latDeg, pts[k].lonDeg, 5000, hdg, gs), 0.1);
            CHECK(nav.state().turnActive);
            maxXtk = std::max(maxXtk, std::fabs(static_cast<double>(nav.state().crossTrackNm)));
        }
        CHECK(maxXtk < 0.01);
        // Past T2: on the CF, XTK about 0, no fly-over excursion past B.
        p = pointAtNm(t.end.latDeg, t.end.lonDeg, 180.0, 0.1);
        nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 180, gs), 0.1);
        CHECK_FALSE(nav.state().turnActive);
        CHECK(nav.plan().activeLeg == cLeg);
        CHECK(std::fabs(nav.state().crossTrackNm) < 0.01);
    }
}

TEST_CASE("nav: N10b a DF leg after its arc runs from the end of the arc") {
    SyntheticNavDb db;
    const Corner c = corner(90.0, 90.0);  // east, then south
    const double gs = 100.0;
    const FlyByTurn t = flyByTurn(c.from, c.fix, c.to, flyByRadiusNm(gs, 15));
    FlightPlan plan;
    plan.legs = {tf("A", c.from, LegType::IF), tf("B", c.fix), tf("C", c.to, LegType::DF)};
    plan.activeLeg = 1;
    Navigator nav(db);
    nav.setPlan(plan);
    // Sequence at T1 a little left of the inbound line, so the DF from there would not be the
    // line B -> C.
    LatLon p = pointAtNm(t.start.latDeg, t.start.lonDeg, 270.0, 0.03);
    p = pointAtNm(p.latDeg, p.lonDeg, 0.0, 0.05);
    nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 90, gs), 0.1);
    REQUIRE(nav.plan().activeLeg == 2);
    REQUIRE(nav.state().turnActive);
    LatLon pts[16];
    const int n = arcPoints(t, pts, 16);
    for (int k = 1; k < n; ++k)
        nav.update(
            makeAircraft(pts[k].latDeg, pts[k].lonDeg, 5000, arcGuide(t, pts[k]).dtkTrueDeg, gs),
            0.1);
    // Past T2 the turn ends; the DF now runs from there, so it is the line T2 -> C.
    p = pointAtNm(t.end.latDeg, t.end.lonDeg, 180.0, 0.05);
    nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 180, gs), 0.1);
    nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 180, gs), 0.1);
    CHECK_FALSE(nav.state().turnActive);
    CHECK(std::fabs(nav.state().crossTrackNm) < 0.005);
    CHECK(nav.state().desiredTrackTrueDeg == doctest::Approx(180).epsilon(0.003));
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
