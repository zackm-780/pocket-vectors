#include <doctest/doctest.h>

#include <cmath>
#include <cstring>

#include "core/Geo.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "nav/ApproachBuilder.h"
#include "nav/FlightPlan.h"
#include "nav/Navigator.h"
#include "navigatorTestUtil.h"

using namespace fsim;
using namespace fsim::test;

namespace {

Leg makeLeg(LegType type, const char* ident, double lat, double lon, float altFt = 0,
            AltConstraintKind kind = AltConstraintKind::None) {
    Leg l;
    l.type = type;
    l.to = Waypoint{ident, WaypointKind::Fix, lat, lon};
    l.alt.kind = altFt > 0 && kind == AltConstraintKind::None ? AltConstraintKind::At : kind;
    l.alt.altFt = altFt;
    return l;
}

FlightPlan makePlan(std::vector<Leg> legs) {
    FlightPlan p;
    p.legs = std::move(legs);
    p.activeLeg = 1;
    return p;
}

// Aircraft on the great circle toward `to` from `from`, `remNm` before `to`.
AircraftState onLeg(const Waypoint& from, const Waypoint& to, double remNm, double altFt = 5000,
                    double gsKt = 100, double rightNm = 0) {
    const double brgToRad = initialBearingTrueRad(to.latDeg, to.lonDeg, from.latDeg, from.lonDeg);
    LatLon p = pointAtNm(to.latDeg, to.lonDeg, radToDeg(brgToRad), remNm);
    const double hdg = radToDeg(initialBearingTrueRad(p.latDeg, p.lonDeg, to.latDeg, to.lonDeg));
    if (rightNm != 0) p = pointAtNm(p.latDeg, p.lonDeg, hdg + 90.0, rightNm);
    return makeAircraft(p.latDeg, p.lonDeg, altFt, hdg, gsKt);
}

// A long enroute plan (126 NM leg): the middle is > 30 NM from both ends.
FlightPlan longPlan() {
    return makePlan({makeLeg(LegType::IF, "WPA", 45.0, -100.0),
                     makeLeg(LegType::TF, "WPB", 45.0, -97.0, 8000),
                     makeLeg(LegType::TF, "WPC", 45.0, -94.0, 8000)});
}

struct Xtst {
    SyntheticNavDb db;
    const Airport& airport = db.airport();
    const Runway* r27 = findRunway(airport, "27");
    const Runway* r09 = findRunway(airport, "09");
};

// Point on the extended centerline of a runway, distNm before the threshold, rightNm to the
// right of the INBOUND course.
LatLon finalPoint(const Runway& r, double distNm, double rightNm = 0) {
    return offsetPoint(LatLon{r.thrLatDeg, r.thrLonDeg}, r.headingTrueDeg + 180.0, distNm, -rightNm);
}

AircraftState onFinal(const Runway& r, double distNm, double rightNm, double altMslFt,
                      double hdgOffsetDeg = 0, double gsKt = 100) {
    const LatLon p = finalPoint(r, distNm, rightNm);
    return makeAircraft(p.latDeg, p.lonDeg, altMslFt, r.headingTrueDeg + hdgOffsetDeg, gsKt,
                        r.thrElevFt);
}

// Altitude on a 3 degree path from the synthesized GS antenna (1000 ft past the threshold).
double gsPathAltFt(const Runway& r, double distNm) {
    return r.thrElevFt + std::tan(degToRad(3.0)) * (nmToFt(distNm) + 1000.0);
}

Navigator ilsNavigator(const Xtst& x, const Runway& r, const LatLon& present) {
    Navigator nav(x.db);
    const Approach a = buildIlsApproach(x.airport, r);
    REQUIRE(nav.startApproachOnly(a, present, 0));
    return nav;
}

}  // namespace

// ---- GPS: signs, XTK, CDI scaling --------------------------------------------------------------

TEST_CASE("nav: GPS XTK and CDI signs (right of course: xtk +, cdi -)") {
    SyntheticNavDb db;
    Navigator nav(db);
    nav.setPlan(longPlan());
    const FlightPlan& p = nav.plan();
    const Waypoint& a = p.legs[0].to;
    const Waypoint& b = p.legs[1].to;

    // Heading east, a point SOUTH of the course is to the RIGHT of it.
    NavState s = nav.update(onLeg(a, b, 60.0, 8000, 100, 0.5), 0.1);
    CHECK(s.source == NavSource::Gps);
    CHECK(s.crossTrackNm == doctest::Approx(0.5).epsilon(0.02));
    CHECK(s.cdiFullScaleNm == doctest::Approx(2.0f));
    CHECK(s.cdiDeflection == doctest::Approx(-0.25).epsilon(0.02));  // course is to the LEFT
    CHECK(std::strcmp(s.activeWptIdent, "WPB") == 0);
    CHECK(std::strcmp(s.nextWptIdent, "WPC") == 0);

    // Left of course: xtk -, cdi +.
    s = nav.update(onLeg(a, b, 60.0, 8000, 100, -1.0), 0.1);
    CHECK(s.crossTrackNm == doctest::Approx(-1.0).epsilon(0.02));
    CHECK(s.cdiDeflection == doctest::Approx(0.5).epsilon(0.02));

    // On course.
    s = nav.update(onLeg(a, b, 60.0, 8000, 100, 0.0), 0.1);
    CHECK(std::fabs(s.crossTrackNm) < 0.01f);
    CHECK(std::fabs(s.cdiDeflection) < 0.01f);
    CHECK(s.toFlag);
    CHECK(s.desiredTrackTrueDeg == doctest::Approx(90.0).epsilon(0.01));
    CHECK(s.distToWptNm == doctest::Approx(60.0).epsilon(0.01));
    CHECK(s.eteToWptS == doctest::Approx(60.0 / 100.0 * 3600.0).epsilon(0.01));
    CHECK(s.distToDestNm == doctest::Approx(60.0 + 126.0).epsilon(0.02));
    CHECK(s.bearingToWptTrueDeg == doctest::Approx(90.0).epsilon(0.01));
    CHECK(s.activeLegIndex == 1);

    // Pegged display clamp.
    s = nav.update(onLeg(a, b, 60.0, 8000, 100, 9.0), 0.1);
    CHECK(s.cdiDeflection == doctest::Approx(-NavConstants::kCdiClamp));
}

TEST_CASE("nav: CDI scale is ENR 2.0, TERM 1.0 near the ends, and ramps over one minute") {
    SyntheticNavDb db;
    Navigator nav(db);
    nav.setPlan(longPlan());
    const Waypoint a = nav.plan().legs[0].to;
    const Waypoint b = nav.plan().legs[1].to;

    NavState s = nav.update(onLeg(a, b, 60.0), 0.1);
    CHECK(s.phase == NavPhase::Enroute);
    CHECK(s.cdiFullScaleNm == doctest::Approx(2.0f));

    // 20 NM from the destination (the fix after the active leg is far, but the destination is
    // the last leg's fix): the plan's end is WPC, 126 NM further, so use the second leg.
    nav.activateLeg(2);
    const Waypoint c = nav.plan().legs[2].to;
    AircraftState near = onLeg(b, c, 20.0);
    s = nav.update(near, 0.0);
    CHECK(s.phase == NavPhase::Terminal);
    CHECK(s.cdiFullScaleNm == doctest::Approx(2.0f));  // ramp has just started
    s = nav.update(near, 15.0);
    CHECK(s.cdiFullScaleNm == doctest::Approx(1.75f).epsilon(0.001));
    s = nav.update(near, 15.0);
    CHECK(s.cdiFullScaleNm == doctest::Approx(1.5f).epsilon(0.001));
    s = nav.update(near, 30.0);
    CHECK(s.cdiFullScaleNm == doctest::Approx(1.0f).epsilon(0.001));
    s = nav.update(near, 30.0);
    CHECK(s.cdiFullScaleNm == doctest::Approx(1.0f));
    // And back up to 2.0 when leaving the terminal area (30 NM from the plan end, 126 NM leg).
    AircraftState far = onLeg(b, c, 80.0);
    nav.update(far, 0.0);
    s = nav.update(far, 30.0);
    CHECK(s.cdiFullScaleNm == doctest::Approx(1.5f).epsilon(0.001));
    s = nav.update(far, 30.0);
    CHECK(s.cdiFullScaleNm == doctest::Approx(2.0f));
}

TEST_CASE("nav: APR scaling 0.3 NM starts 2 NM before the FAF") {
    Xtst x;
    // RNAV (no LOC) so the lateral source stays GPS.
    Navigator nav(x.db);
    const Approach a = buildRnavApproach(x.airport, *x.r27);
    REQUIRE(nav.startApproachOnly(a, finalPoint(*x.r27, 14.0), 0));
    // Between the IF and FAF, 4 NM from the threshold... first still 1.0 NM (TERM) at the IF leg.
    NavState s = nav.update(onFinal(*x.r27, 9.0, 0.0, 5200), 0.0);
    CHECK(s.phase == NavPhase::Approach);
    CHECK(s.cdiFullScaleNm == doctest::Approx(1.0f));
    CHECK(nav.fafLegIndex() == 2);
    // 6.9 NM out: 1.9 NM before the FAF -> APR target, ramping from 1.0 toward 0.3.
    s = nav.update(onFinal(*x.r27, 6.9, 0.0, 4300), 0.0);
    s = nav.update(onFinal(*x.r27, 6.9, 0.0, 4300), 30.0);
    CHECK(s.cdiFullScaleNm == doctest::Approx(0.65f).epsilon(0.001));
    s = nav.update(onFinal(*x.r27, 6.9, 0.0, 4300), 30.0);
    CHECK(s.cdiFullScaleNm == doctest::Approx(0.3f).epsilon(0.001));
    CHECK(s.cdiDeflection == doctest::Approx(0.0f).epsilon(0.1));
    // 0.15 NM right of the final course = half scale, to the right means cdi negative.
    s = nav.update(onFinal(*x.r27, 4.0, 0.15, 3500), 0.1);
    CHECK(s.cdiFullScaleNm == doctest::Approx(0.3f));
    CHECK(s.crossTrackNm == doctest::Approx(0.15).epsilon(0.03));
    CHECK(s.cdiDeflection == doctest::Approx(-0.5).epsilon(0.03));
}

// ---- LOC / GS --------------------------------------------------------------------------------

TEST_CASE("nav: LOC deviation signs and scale (right of course: cdi -)") {
    Xtst x;
    Navigator nav = ilsNavigator(x, *x.r27, finalPoint(*x.r27, 8.0));
    nav.setSourceSelect(NavSourceSelect::Loc);
    // XTST's LOC is 5 degrees wide in total (half-width 2.5): full scale at 2.5 degrees.
    NavState s = nav.update(onFinal(*x.r27, 5.0, 0.0, gsPathAltFt(*x.r27, 5.0)), 0.1);
    CHECK(s.source == NavSource::Loc);
    CHECK(s.locValid);
    CHECK(s.cdiFullScaleNm == 0.0f);
    CHECK(std::fabs(s.cdiDeflection) < 0.01f);
    CHECK(s.desiredTrackTrueDeg == doctest::Approx(270.0f));
    CHECK(s.toFlag);

    // 0.2 NM north of the centerline, flying west: aircraft is to the RIGHT of the course.
    const LatLon ant{x.r27->ils->locLatDeg, x.r27->ils->locLonDeg};
    const double dAnt = mToNm(greatCircleDistM(finalPoint(*x.r27, 5.0, 0.2).latDeg,
                                               finalPoint(*x.r27, 5.0, 0.2).lonDeg, ant.latDeg,
                                               ant.lonDeg));
    const double expectedDeg = radToDeg(std::asin(0.2 / dAnt));
    s = nav.update(onFinal(*x.r27, 5.0, 0.2, gsPathAltFt(*x.r27, 5.0)), 0.1);
    CHECK(s.cdiDeflection < 0.0f);
    CHECK(s.cdiDeflection == doctest::Approx(-expectedDeg / 2.5).epsilon(0.03));
    CHECK(s.crossTrackNm > 0.0f);  // right of course
    // Left of course.
    s = nav.update(onFinal(*x.r27, 5.0, -0.2, gsPathAltFt(*x.r27, 5.0)), 0.1);
    CHECK(s.cdiDeflection > 0.0f);
    CHECK(s.crossTrackNm < 0.0f);
    // Display clamp at +-1.25.
    s = nav.update(onFinal(*x.r27, 5.0, 2.0, gsPathAltFt(*x.r27, 5.0)), 0.1);
    CHECK(s.cdiDeflection == doctest::Approx(-NavConstants::kCdiClamp));
    s = nav.update(onFinal(*x.r27, 5.0, -2.0, gsPathAltFt(*x.r27, 5.0)), 0.1);
    CHECK(s.cdiDeflection == doctest::Approx(NavConstants::kCdiClamp));
}

TEST_CASE("nav: LOC sensitivity increases toward the threshold") {
    Xtst x;
    Navigator nav = ilsNavigator(x, *x.r27, finalPoint(*x.r27, 8.0));
    nav.setSourceSelect(NavSourceSelect::Loc);
    const double lateralNm = 0.05;
    double prev = 0;
    for (double d : {10.0, 8.0, 6.0, 4.0, 2.0, 1.0}) {
        NavState s = nav.update(onFinal(*x.r27, d, lateralNm, gsPathAltFt(*x.r27, d)), 0.1);
        const double dev = std::fabs(static_cast<double>(s.cdiDeflection));
        CHECK(dev > prev);
        prev = dev;
    }
    // Same lateral offset at 2 NM gives much more than 2x the deflection of 8 NM.
    const double d8 = std::fabs(nav.update(onFinal(*x.r27, 8.0, lateralNm, 4000), 0.1).cdiDeflection);
    const double d2 = std::fabs(nav.update(onFinal(*x.r27, 2.0, lateralNm, 2400), 0.1).cdiDeflection);
    CHECK(d2 > 2.0 * d8);
}

TEST_CASE("nav: GS deviation signs and scale (low: +, high: -)") {
    Xtst x;
    Navigator nav = ilsNavigator(x, *x.r27, finalPoint(*x.r27, 8.0));
    nav.setSourceSelect(NavSourceSelect::Loc);
    const double onPath = gsPathAltFt(*x.r27, 5.0);
    NavState s = nav.update(onFinal(*x.r27, 5.0, 0.0, onPath), 0.1);
    CHECK(s.gsValid);
    CHECK(std::fabs(s.gsDeflection) < 0.01f);

    s = nav.update(onFinal(*x.r27, 5.0, 0.0, onPath - 100.0), 0.1);  // low: glidepath above
    CHECK(s.gsDeflection > 0.0f);
    const double distFt = nmToFt(5.0) + 1000.0;
    const double lowDeg = radToDeg(std::atan2(onPath - 100.0 - 2000.0, distFt));
    CHECK(s.gsDeflection == doctest::Approx((3.0 - lowDeg) / 0.7).epsilon(0.02));

    s = nav.update(onFinal(*x.r27, 5.0, 0.0, onPath + 100.0), 0.1);  // high
    CHECK(s.gsDeflection < 0.0f);
    // Full scale is 0.7 degrees: 0.7 deg of path error at 5 NM is ~ 0.7 * 31380/57.3 = 383 ft.
    s = nav.update(onFinal(*x.r27, 5.0, 0.0, onPath - 383.0), 0.1);
    CHECK(s.gsDeflection == doctest::Approx(1.0).epsilon(0.03));
    // Pegged.
    s = nav.update(onFinal(*x.r27, 5.0, 0.0, onPath - 1500.0), 0.1);
    CHECK(s.gsDeflection == doctest::Approx(NavConstants::kCdiClamp));
}

TEST_CASE("nav: LOC and GS validity cones") {
    Xtst x;
    Navigator nav = ilsNavigator(x, *x.r27, finalPoint(*x.r27, 8.0));
    nav.setSourceSelect(NavSourceSelect::Loc);
    const Ils& ils = *x.r27->ils;
    // Distance: LOC valid within 18 NM of the antenna, GS within 10 NM.
    NavState s = nav.update(onFinal(*x.r27, 15.0, 0.0, 6000), 0.1);
    CHECK(s.locValid);
    CHECK_FALSE(s.gsValid);
    s = nav.update(onFinal(*x.r27, 9.0, 0.0, 4800), 0.1);
    CHECK(s.locValid);
    CHECK(s.gsValid);
    s = nav.update(onFinal(*x.r27, 19.0, 0.0, 6000), 0.1);
    CHECK_FALSE(s.locValid);
    CHECK_FALSE(s.gsValid);
    CHECK(s.cdiDeflection == 0.0f);
    // Angle: within 35 degrees of the course (seen from the antenna).
    const double back = ils.locCourseTrueDeg + 180.0;
    LatLon p30 = pointAtNm(ils.locLatDeg, ils.locLonDeg, back + 30.0, 5.0);
    LatLon p40 = pointAtNm(ils.locLatDeg, ils.locLonDeg, back + 40.0, 5.0);
    s = nav.update(makeAircraft(p30.latDeg, p30.lonDeg, 3500, 270, 100, 2000), 0.1);
    CHECK(s.locValid);
    CHECK_FALSE(s.gsValid);  // GS cone is +-8 degrees
    s = nav.update(makeAircraft(p40.latDeg, p40.lonDeg, 3500, 270, 100, 2000), 0.1);
    CHECK_FALSE(s.locValid);
    LatLon p6 = pointAtNm(ils.gsLatDeg, ils.gsLonDeg, back + 6.0, 5.0);
    s = nav.update(makeAircraft(p6.latDeg, p6.lonDeg, 3600, 270, 100, 2000), 0.1);
    CHECK(s.locValid);
    CHECK(s.gsValid);
    // In front of the antenna only: a point beyond the far end is invalid.
    LatLon behind = pointAtNm(ils.locLatDeg, ils.locLonDeg, ils.locCourseTrueDeg, 3.0);
    s = nav.update(makeAircraft(behind.latDeg, behind.lonDeg, 3000, 270, 100, 2000), 0.1);
    CHECK_FALSE(s.locValid);
    // GS needs the aircraft above the antenna elevation.
    s = nav.update(onFinal(*x.r27, 5.0, 0.0, 1990), 0.1);
    CHECK_FALSE(s.gsValid);
}

TEST_CASE("nav: automatic GPS -> LOC switch on the final course") {
    Xtst x;
    Navigator nav = ilsNavigator(x, *x.r27, finalPoint(*x.r27, 14.0));
    // Far out on the extended centerline and past the IF: on the FAF leg the LOC captures.
    NavState s = nav.update(onFinal(*x.r27, 7.0, 0.0, 4300, 0.0), 0.1);
    CHECK(s.onFinal);
    CHECK(s.source == NavSource::Loc);
    CHECK(s.locValid);
    CHECK(s.cdiFullScaleNm == 0.0f);
    CHECK(s.phase == NavPhase::Approach);
    CHECK(std::strcmp(s.approachName, "ILS 27") == 0);
    CHECK(s.minimumsFt == doctest::Approx(2200.0f));
    CHECK(s.minimumsIsDa);
    CHECK(s.distToThresholdNm == doctest::Approx(7.0).epsilon(0.01));
    // Once captured it stays on LOC even when the needle is pegged (still valid).
    s = nav.update(onFinal(*x.r27, 6.5, 0.8, 4200, 0.0), 0.1);
    CHECK(s.source == NavSource::Loc);
    CHECK(s.cdiDeflection == doctest::Approx(-NavConstants::kCdiClamp));
    // ... and falls back to GPS when the LOC is no longer valid (beyond the cone).
    s = nav.update(onFinal(*x.r27, 19.0, 0.0, 6000, 0.0), 0.1);
    CHECK(s.source == NavSource::Gps);
}

TEST_CASE("nav: no LOC capture with a large intercept angle, outside full scale or forced GPS") {
    Xtst x;
    {
        // Intercept 60 degrees (track 330 vs course 270): no switch.
        Navigator nav = ilsNavigator(x, *x.r27, finalPoint(*x.r27, 14.0));
        NavState s = nav.update(onFinal(*x.r27, 7.0, 0.0, 4300, 60.0), 0.1);
        CHECK(s.source == NavSource::Gps);
        CHECK(s.locValid);
        // Turn to a 40 degree intercept: capture.
        s = nav.update(onFinal(*x.r27, 7.0, 0.0, 4300, 40.0), 0.1);
        CHECK(s.source == NavSource::Loc);
    }
    {
        // Outside full scale (0.6 NM right at 7 NM is > 2.5 degrees): GPS until inside.
        Navigator nav = ilsNavigator(x, *x.r27, finalPoint(*x.r27, 14.0));
        NavState s = nav.update(onFinal(*x.r27, 7.0, 0.6, 4300, 20.0), 0.1);
        CHECK(s.source == NavSource::Gps);
        s = nav.update(onFinal(*x.r27, 7.0, 0.2, 4300, 20.0), 0.1);
        CHECK(s.source == NavSource::Loc);
    }
    {
        Navigator nav = ilsNavigator(x, *x.r27, finalPoint(*x.r27, 14.0));
        nav.setSourceSelect(NavSourceSelect::Gps);
        NavState s = nav.update(onFinal(*x.r27, 7.0, 0.0, 4300, 0.0), 0.1);
        CHECK(s.source == NavSource::Gps);
        CHECK_FALSE(s.gsValid);
        CHECK(s.cdiFullScaleNm > 0.0f);
    }
    {
        // An RNAV approach never switches to LOC.
        Navigator nav(x.db);
        REQUIRE(nav.startApproachOnly(buildRnavApproach(x.airport, *x.r27), finalPoint(*x.r27, 14.0), 0));
        NavState s = nav.update(onFinal(*x.r27, 7.0, 0.0, 4300), 0.1);
        CHECK(s.source == NavSource::Gps);
        CHECK_FALSE(s.locValid);
        CHECK(std::strcmp(s.approachName, "RNAV 27") == 0);
        CHECK(s.minimumsFt == doctest::Approx(2400.0f));
        CHECK_FALSE(s.minimumsIsDa);
    }
}

// ---- sequencing ------------------------------------------------------------------------------

TEST_CASE("nav: fly-by anticipation distance d = R tan(dPsi/2)") {
    SyntheticNavDb db;
    // A -> B east, B -> C north: a 90 degree turn at B.
    auto plan = makePlan({makeLeg(LegType::IF, "A", 45.0, -100.0), makeLeg(LegType::TF, "B", 45.0, -99.0),
                          makeLeg(LegType::TF, "C", 46.0, -99.0)});
    const Waypoint a = plan.legs[0].to, b = plan.legs[1].to;
    // Light aircraft: bank 20, V = 100 kt.
    const double v = ktToMps(100.0);
    const double r20 = v * v / (kStdGravityMps2 * std::tan(degToRad(20.0)));
    const double d20Nm = mToNm(r20 * std::tan(degToRad(45.0)));
    CHECK(d20Nm == doctest::Approx(0.40).epsilon(0.01));
    {
        Navigator nav(db);
        nav.setPlan(plan);
        nav.update(onLeg(a, b, 5.0), 0.1);
        nav.update(onLeg(a, b, d20Nm + 0.05), 0.1);
        CHECK(nav.plan().activeLeg == 1);
        nav.update(onLeg(a, b, d20Nm - 0.05), 0.1);
        CHECK(nav.plan().activeLeg == 2);
        CHECK(std::strcmp(nav.state().activeWptIdent, "C") == 0);
    }
    {  // Jets turn tighter at 25 degrees bank: the anticipation is shorter.
        const double r25 = v * v / (kStdGravityMps2 * std::tan(degToRad(25.0)));
        const double d25Nm = mToNm(r25 * std::tan(degToRad(45.0)));
        CHECK(d25Nm < d20Nm);
        Navigator nav(db);
        nav.setAircraftCategory(AircraftCategory::Jet);
        nav.setPlan(plan);
        nav.update(onLeg(a, b, d20Nm - 0.05), 0.1);
        CHECK(nav.plan().activeLeg == 1);  // would have sequenced with 20 degrees
        nav.update(onLeg(a, b, d25Nm - 0.03), 0.1);
        CHECK(nav.plan().activeLeg == 2);
    }
    {  // Anticipation grows with ground speed squared (200 kt: 4x).
        Navigator nav(db);
        nav.setPlan(plan);
        nav.update(onLeg(a, b, 2.0, 5000, 200), 0.1);
        nav.update(onLeg(a, b, 4.0 * d20Nm - 0.1, 5000, 200), 0.1);
        CHECK(nav.plan().activeLeg == 2);
    }
}

TEST_CASE("nav: turn anticipation caps the course change at 120 degrees and small turns barely lead") {
    SyntheticNavDb db;
    const double v = ktToMps(100.0);
    const double r = v * v / (kStdGravityMps2 * std::tan(degToRad(20.0)));
    {
        // ~160 degree reversal at B: anticipation = R tan(60 deg).
        auto plan = makePlan({makeLeg(LegType::IF, "A", 45.0, -100.0),
                              makeLeg(LegType::TF, "B", 45.0, -99.0),
                              makeLeg(LegType::TF, "C", 44.9, -100.3)});
        const double dNm = mToNm(r * std::tan(degToRad(60.0)));
        Navigator nav(db);
        nav.setPlan(plan);
        const Waypoint a = plan.legs[0].to, b = plan.legs[1].to;
        nav.update(onLeg(a, b, dNm + 0.04), 0.1);
        CHECK(nav.plan().activeLeg == 1);
        nav.update(onLeg(a, b, dNm - 0.04), 0.1);
        CHECK(nav.plan().activeLeg == 2);
    }
    {
        // Nearly straight: sequences only a few metres before the fix.
        auto plan = makePlan({makeLeg(LegType::IF, "A", 45.0, -100.0),
                              makeLeg(LegType::TF, "B", 45.0, -99.0),
                              makeLeg(LegType::TF, "C", 45.01, -98.0)});
        Navigator nav(db);
        nav.setPlan(plan);
        const Waypoint a = plan.legs[0].to, b = plan.legs[1].to;
        nav.update(onLeg(a, b, 0.1), 0.1);
        CHECK(nav.plan().activeLeg == 1);
    }
}

TEST_CASE("nav: fly-over legs sequence only when abeam; the last leg never sequences") {
    SyntheticNavDb db;
    auto plan = makePlan({makeLeg(LegType::IF, "A", 45.0, -100.0), makeLeg(LegType::TF, "B", 45.0, -99.0),
                          makeLeg(LegType::TF, "C", 46.0, -99.0)});
    plan.legs[1].flyOver = true;
    Navigator nav(db);
    nav.setPlan(plan);
    const Waypoint a = plan.legs[0].to, b = plan.legs[1].to;
    nav.update(onLeg(a, b, 0.4), 0.1);
    nav.update(onLeg(a, b, 0.05), 0.1);
    CHECK(nav.plan().activeLeg == 1);
    nav.update(onLeg(a, b, -0.05), 0.1);  // just past the fix
    CHECK(nav.plan().activeLeg == 2);
    // The last leg does not sequence off the end.
    const Waypoint c = plan.legs[2].to;
    nav.update(onLeg(b, c, -2.0), 0.1);
    CHECK(nav.plan().activeLeg == 2);
    CHECK_FALSE(nav.state().toFlag);  // FROM the last waypoint
}

TEST_CASE("nav: legs sequence in order when several fixes are crossed in one update") {
    SyntheticNavDb db;
    auto plan = makePlan({makeLeg(LegType::IF, "A", 45.0, -100.0), makeLeg(LegType::TF, "B", 45.0, -99.99),
                          makeLeg(LegType::TF, "C", 45.0, -99.98), makeLeg(LegType::TF, "D", 45.0, -90.0)});
    Navigator nav(db);
    nav.setPlan(plan);
    // The aircraft is already past B and C (and on the way to D).
    nav.update(makeAircraft(45.0, -99.5, 5000, 90), 0.1);
    CHECK(nav.plan().activeLeg == 3);
    CHECK(std::strcmp(nav.state().activeWptIdent, "D") == 0);
}

TEST_CASE("nav: CA legs sequence when climbing through the leg altitude") {
    SyntheticNavDb db;
    Leg ca = makeLeg(LegType::CA, "CA", 45.0, -99.9, 3000, AltConstraintKind::AtOrAbove);
    ca.courseTrueDeg = 90;
    auto plan = makePlan({makeLeg(LegType::IF, "A", 45.0, -100.0), ca,
                          makeLeg(LegType::TF, "B", 45.0, -98.0)});
    Navigator nav(db);
    nav.setPlan(plan);
    NavState s = nav.update(makeAircraft(45.0, -99.99, 2500, 90), 0.1);
    CHECK(nav.plan().activeLeg == 1);
    CHECK(std::strcmp(s.activeWptIdent, "3000FT") == 0);
    CHECK(s.desiredTrackTrueDeg == doctest::Approx(90.0f));
    CHECK(s.distToWptNm == 0.0f);
    s = nav.update(makeAircraft(45.0, -99.98, 3010, 90), 0.1);
    CHECK(nav.plan().activeLeg == 2);
    CHECK(std::strcmp(s.activeWptIdent, "B") == 0);
}

TEST_CASE("nav: direct-to makes a DF leg from the present position") {
    SyntheticNavDb db;
    Navigator nav(db);
    auto plan = longPlan();
    plan.legs.push_back(makeLeg(LegType::TF, "WPD", 44.0, -93.0));
    nav.setPlan(plan);
    const Waypoint a = nav.plan().legs[0].to, b = nav.plan().legs[1].to;
    // Well off course (10 NM south of the leg): direct to WPC.
    nav.update(onLeg(a, b, 60.0, 8000, 100, 10.0), 0.1);
    CHECK(nav.state().crossTrackNm == doctest::Approx(10.0).epsilon(0.02));
    REQUIRE(nav.directToIdent("wpc"));
    NavState s = nav.update(onLeg(a, b, 60.0, 8000, 100, 10.0), 0.1);
    CHECK(nav.plan().activeLeg == 1);
    CHECK(nav.plan().legs[1].type == LegType::DF);
    CHECK(std::strcmp(s.activeWptIdent, "WPC") == 0);
    CHECK(std::strcmp(s.nextWptIdent, "WPD") == 0);
    CHECK(std::fabs(s.crossTrackNm) < 0.05f);  // on the new direct course
    CHECK(s.desiredTrackTrueDeg == doctest::Approx(s.bearingToWptTrueDeg).epsilon(0.01));
    CHECK_FALSE(nav.directToIdent("NOPE123"));
}

// ---- phases ----------------------------------------------------------------------------------

TEST_CASE("nav: phases Departure -> Enroute/Terminal -> Approach -> Missed") {
    SyntheticNavDb db;
    auto parsed = loadPlanFromFmsFile("assets/plans/XTST-pattern.fms", db);
    REQUIRE(parsed.ok);
    Navigator nav(db);
    nav.setPlan(parsed.plan);
    REQUIRE(nav.plan().approachStartLeg >= 0);  // APP I09 was loaded by the navigator
    CHECK(nav.plan().approachId == "I09");
    CHECK(nav.plan().approachKind == ApproachKind::Ils);

    Xtst x;
    const Runway& r09 = *x.r09;
    // On the runway: departure.
    AircraftState ground = makeAircraft(r09.startLatDeg, r09.startLonDeg, 2000, 90, 5, 2000);
    ground.wowAny = true;
    CHECK(nav.update(ground, 0.1).phase == NavPhase::Departure);
    // Airborne 3 NM out, still on the first leg: departure; beyond 5 NM it is terminal.
    LatLon p3 = pointAtNm(r09.thrLatDeg, r09.thrLonDeg, 90, 3);
    CHECK(nav.update(makeAircraft(p3.latDeg, p3.lonDeg, 3000, 90, 120, 2000), 0.1).phase ==
          NavPhase::Departure);
    LatLon p8 = pointAtNm(r09.thrLatDeg, r09.thrLonDeg, 90, 8);
    CHECK(nav.update(makeAircraft(p8.latDeg, p8.lonDeg, 4000, 90, 120, 2000), 0.1).phase ==
          NavPhase::Terminal);

    // Enroute: a plan far from both ends.
    Navigator far(db);
    far.setPlan(longPlan());
    const Waypoint a = far.plan().legs[0].to, b = far.plan().legs[1].to;
    CHECK(far.update(onLeg(a, b, 60.0), 0.1).phase == NavPhase::Enroute);

    // Approach and Missed.
    Navigator app = ilsNavigator(x, *x.r27, finalPoint(*x.r27, 14.0));
    CHECK(app.update(onFinal(*x.r27, 8.0, 0.0, 4800), 0.1).phase == NavPhase::Approach);
    AircraftState pastMap = onFinal(*x.r27, -0.1, 0.0, 2400);  // 400 ft AGL just past the MAP
    pastMap.altAglFt = 400;
    CHECK(app.update(pastMap, 0.1).phase == NavPhase::Missed);
}

// ---- missed approach -------------------------------------------------------------------------

TEST_CASE("nav: missed approach sequences only after the MAP is passed airborne") {
    Xtst x;
    auto make = [&] {
        Navigator nav = ilsNavigator(x, *x.r27, finalPoint(*x.r27, 14.0));
        nav.update(onFinal(*x.r27, 1.0, 0.0, 2300), 0.1);
        CHECK(nav.plan().activeLeg == nav.mapLegIndex());
        return nav;
    };
    {  // Passed the threshold at 300 ft AGL: missed.
        Navigator nav = make();
        AircraftState ac = onFinal(*x.r27, -0.05, 0.0, 2300);
        NavState s = nav.update(ac, 0.1);
        CHECK(s.phase == NavPhase::Missed);
        CHECK(nav.plan().legs[static_cast<size_t>(nav.plan().activeLeg)].type == LegType::CA);
        CHECK(std::strcmp(s.activeWptIdent, "3500FT") == 0);
        CHECK(s.desiredTrackTrueDeg == doctest::Approx(270.0f).epsilon(0.001));
        CHECK(s.source == NavSource::Gps);  // LOC is dropped in the missed approach
    }
    {  // Landing: 40 ft AGL on the ground-effect flare, wheels down: stays on the MAP.
        Navigator nav = make();
        AircraftState ac = onFinal(*x.r27, -0.05, 0.0, 2040);
        NavState s = nav.update(ac, 0.1);
        CHECK(s.phase == NavPhase::Approach);
        ac.altAglFt = 0;
        ac.wowAny = true;
        ac.vsFpm = 0;
        CHECK(nav.update(ac, 0.1).phase == NavPhase::Approach);
        CHECK(nav.update(onFinal(*x.r27, -0.5, 0.0, 2000), 0.1).phase == NavPhase::Approach);
    }
    {  // Crossing at TCH (50 ft AGL) descending: a normal landing, not a missed approach.
        Navigator nav = make();
        AircraftState ac = onFinal(*x.r27, -0.02, 0.0, 2050);
        ac.vsFpm = -500;
        CHECK(nav.update(ac, 0.1).phase == NavPhase::Approach);
    }
    {  // Low and climbing: go-around.
        Navigator nav = make();
        AircraftState ac = onFinal(*x.r27, -0.05, 0.0, 2040);
        ac.vsFpm = 800;
        CHECK(nav.update(ac, 0.1).phase == NavPhase::Missed);
    }
    {  // Still airborne low over the runway more than 0.5 NM past the MAP: missed.
        Navigator nav = make();
        nav.update(onFinal(*x.r27, -0.1, 0.0, 2040), 0.1);
        CHECK(nav.state().phase == NavPhase::Approach);
        AircraftState ac = onFinal(*x.r27, -0.7, 0.0, 2080);
        ac.vsFpm = 0;
        CHECK(nav.update(ac, 0.1).phase == NavPhase::Missed);
    }
    {  // Before the MAP nothing happens, however high.
        Navigator nav = make();
        CHECK(nav.update(onFinal(*x.r27, 0.3, 0.0, 3500), 0.1).phase == NavPhase::Approach);
    }
}

TEST_CASE("nav: missed approach legs CA -> DF IF -> HM, hold never sequences, re-entry via direct-to IF") {
    Xtst x;
    Navigator nav = ilsNavigator(x, *x.r27, finalPoint(*x.r27, 14.0));
    nav.update(onFinal(*x.r27, 1.0, 0.0, 2300), 0.1);
    nav.update(onFinal(*x.r27, -0.05, 0.0, 2300), 0.1);
    REQUIRE(nav.state().phase == NavPhase::Missed);
    const FlightPlan& plan = nav.plan();
    int ca = plan.activeLeg;
    CHECK(plan.legs[static_cast<size_t>(ca)].type == LegType::CA);
    CHECK(plan.legs[static_cast<size_t>(ca) + 1].type == LegType::DF);
    CHECK(plan.legs[static_cast<size_t>(ca) + 2].type == LegType::HM);

    // Climb through 3500: the DF IF leg becomes active; it flies direct to the IF.
    AircraftState ac = onFinal(*x.r27, -1.0, 0.0, 3600);
    NavState s = nav.update(ac, 0.1);
    CHECK(plan.activeLeg == ca + 1);
    CHECK(std::strcmp(s.activeWptIdent, "IF") == 0);
    CHECK(s.phase == NavPhase::Missed);
    const Waypoint ifw = plan.legs[static_cast<size_t>(ca) + 1].to;
    CHECK(s.distToWptNm == doctest::Approx(10.0 + 1.0).epsilon(0.05));
    CHECK(s.bearingToWptTrueDeg == doctest::Approx(90.0).epsilon(0.01));

    // Reaching the IF sequences into the hold, which then never sequences.
    ac = makeAircraft(ifw.latDeg, ifw.lonDeg, 5000, 270, 100, 2000);
    nav.update(onFinal(*x.r27, 10.05, 0.0, 5000), 0.1);
    CHECK(plan.activeLeg == ca + 2);
    CHECK(plan.legs[static_cast<size_t>(plan.activeLeg)].type == LegType::HM);
    nav.update(onFinal(*x.r27, 16.0, 0.0, 5000), 0.1);
    CHECK(plan.activeLeg == ca + 2);

    // Re-entry: direct-to the IF and fly the approach again.
    REQUIRE(nav.reenterApproach());
    s = nav.update(onFinal(*x.r27, 12.0, 0.0, 5200), 0.1);
    CHECK(s.phase == NavPhase::Approach);
    CHECK(std::strcmp(s.activeWptIdent, "IF") == 0);
    CHECK(nav.plan().legs[1].type == LegType::DF);
    CHECK(nav.plan().approachStartLeg == 1);
}

// ---- VNAV ------------------------------------------------------------------------------------

TEST_CASE("nav: advisory VNAV 3 degree path, required VS and TOD distance") {
    SyntheticNavDb db;
    auto plan = makePlan({makeLeg(LegType::IF, "A", 45.0, -100.0),
                          makeLeg(LegType::TF, "B", 45.0, -98.0, 2000)});
    Navigator nav(db);
    nav.setPlan(plan);
    const Waypoint a = plan.legs[0].to, b = plan.legs[1].to;
    const double tan3 = std::tan(degToRad(3.0));
    const double ftPerNm = kFtPerNm;

    // 20 NM out at 6000 ft: the path is above the aircraft (TOD not reached): level.
    NavState s = nav.update(onLeg(a, b, 20.0, 6000, 100), 0.1);
    CHECK(s.targetAltFt == doctest::Approx(2000.0f));
    CHECK(s.vnavValid);
    CHECK(s.vnavDevFt == doctest::Approx(2000.0 + 20.0 * ftPerNm * tan3 - 6000.0).epsilon(0.01));
    CHECK(s.vnavDevFt > 0.0f);
    CHECK(s.requiredVsFpm == 0.0f);
    const double todExpected = 20.0 - ftToNm(4000.0) / tan3;
    CHECK(s.todDistNm == doctest::Approx(todExpected).epsilon(0.01));
    CHECK(todExpected == doctest::Approx(7.44).epsilon(0.01));

    // 10 NM out at 6000 ft: past the TOD, above the path by ~816 ft: descend at the path VS.
    s = nav.update(onLeg(a, b, 10.0, 6000, 100), 0.1);
    CHECK(s.vnavValid);
    CHECK(s.vnavDevFt == doctest::Approx(2000.0 + 10.0 * ftPerNm * tan3 - 6000.0).epsilon(0.01));
    CHECK(s.vnavDevFt < 0.0f);
    CHECK(s.requiredVsFpm == doctest::Approx(-100.0 * 101.27 * tan3).epsilon(0.003));
    CHECK(s.requiredVsFpm == doctest::Approx(-530.7).epsilon(0.005));
    CHECK(s.todDistNm == 0.0f);

    // Required VS scales with ground speed.
    s = nav.update(onLeg(a, b, 10.0, 6000, 200), 0.1);
    CHECK(s.requiredVsFpm == doctest::Approx(-1061.4).epsilon(0.005));

    // At or below the target: no descent advisory (target still reported).
    s = nav.update(onLeg(a, b, 10.0, 2050, 100), 0.1);
    CHECK_FALSE(s.vnavValid);
    CHECK(s.targetAltFt == doctest::Approx(2000.0f));
}

TEST_CASE("nav: VNAV targets the destination threshold + 50 ft when no constraint remains") {
    Xtst x;
    // Plan A -> XTST RW27, no constraints.
    Leg a = makeLeg(LegType::IF, "A", 40.0, -98.0);
    Leg rw;
    rw.type = LegType::TF;
    rw.to = Waypoint{"RW27", WaypointKind::Runway, x.r27->thrLatDeg, x.r27->thrLonDeg};
    FlightPlan plan = makePlan({a, rw});
    plan.destIdent = "XTST";
    plan.arrRunway = "27";
    Navigator nav(x.db);
    nav.setPlan(plan);
    AircraftState ac = makeAircraft(40.0, -98.5, 6000, 270, 120);
    NavState s = nav.update(ac, 0.1);
    CHECK(s.targetAltFt == doctest::Approx(2050.0f));
    CHECK(s.vnavValid);
    CHECK(s.distToThresholdNm > 0.0f);
}

TEST_CASE("nav: VNAV on a synthesized approach follows the leg altitude constraints") {
    Xtst x;
    Navigator nav(x.db);
    REQUIRE(nav.startApproachOnly(buildRnavApproach(x.airport, *x.r27), finalPoint(*x.r27, 14.0), 0));
    // Between the IF and the FAF at 5200 ft: next constraint is the FAF at 3600 ft.
    NavState s = nav.update(onFinal(*x.r27, 9.5, 0.0, 5200), 0.1);
    CHECK(std::strcmp(s.activeWptIdent, "FAF") == 0);
    CHECK(s.targetAltFt == doctest::Approx(3600.0f));
    CHECK(s.vnavValid);
    // The 3 degree path from the FAF at 5 NM thr+1600: at 7.5 NM out the path is ~ 3600 + 2.5 NM x 318 ft.
    s = nav.update(onFinal(*x.r27, 7.5, 0.0, 4300), 0.1);
    CHECK(s.vnavDevFt == doctest::Approx(3600.0 + 2.5 * kFtPerNm * std::tan(degToRad(3.0)) - 4300.0)
                             .epsilon(0.02));
}

// ---- VOR (optional source) -------------------------------------------------------------------

TEST_CASE("nav: VOR source TO/FROM flags and deflection signs") {
    SyntheticNavDb db;
    Navigator nav(db);
    Navaid vor;
    vor.ident = "TST";
    vor.latDeg = 45.0;
    vor.lonDeg = -100.0;
    nav.tuneVor(vor, 0.0f);  // OBS 360
    nav.setSourceSelect(NavSourceSelect::Vor);

    // 10 NM south of the station, 1 NM east of the radial: heading north the course is to the left.
    LatLon p = offsetPoint(LatLon{45.0, -100.0}, 180.0, 10.0, -1.0);  // right of 180 = west; -1 = east
    NavState s = nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 0), 0.1);
    CHECK(s.source == NavSource::Vor);
    CHECK(s.toFlag);
    CHECK(s.cdiDeflection < 0.0f);
    CHECK(s.cdiDeflection == doctest::Approx(-0.57).epsilon(0.05));
    CHECK(s.distToWptNm == doctest::Approx(10.05).epsilon(0.01));
    CHECK(std::strcmp(s.activeWptIdent, "TST") == 0);

    // West of the radial: positive.
    p = offsetPoint(LatLon{45.0, -100.0}, 180.0, 10.0, 1.0);
    s = nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 0), 0.1);
    CHECK(s.cdiDeflection > 0.0f);

    // North of the station, outbound: FROM, east of the radial = right of course = negative.
    p = offsetPoint(LatLon{45.0, -100.0}, 0.0, 10.0, 1.0);  // right of north = east
    s = nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 0), 0.1);
    CHECK_FALSE(s.toFlag);
    CHECK(s.cdiDeflection < 0.0f);
    // Full scale is 10 degrees; pegged beyond.
    p = offsetPoint(LatLon{45.0, -100.0}, 180.0, 5.0, -3.0);
    s = nav.update(makeAircraft(p.latDeg, p.lonDeg, 5000, 0), 0.1);
    CHECK(s.cdiDeflection == doctest::Approx(-NavConstants::kCdiClamp));
}

// ---- misc ------------------------------------------------------------------------------------

TEST_CASE("nav: empty navigator and invalid aircraft give an empty NavState") {
    SyntheticNavDb db;
    Navigator nav(db);
    NavState s = nav.update(makeAircraft(45, -100, 5000, 90), 0.1);
    CHECK(s.source == NavSource::None);
    CHECK(s.phase == NavPhase::None);
    AircraftState invalid;
    CHECK(nav.update(invalid, 0.1).source == NavSource::None);
    nav.setPlan(longPlan());
    CHECK(nav.update(makeAircraft(45, -99, 8000, 90), 0.1).source == NavSource::Gps);
    CHECK(nav.update(invalid, 0.1).source == NavSource::Gps);  // keeps the last state
    nav.clearPlan();
    CHECK(nav.state().source == NavSource::None);
}

TEST_CASE("nav: magnetic variation comes from the database and activateNearestLeg picks the leg ahead") {
    SyntheticNavDb db;
    Navigator nav(db);
    auto plan = makePlan({makeLeg(LegType::IF, "A", 45.0, -100.0), makeLeg(LegType::TF, "B", 45.0, -99.0),
                          makeLeg(LegType::TF, "C", 45.0, -98.0), makeLeg(LegType::TF, "D", 45.0, -97.0)});
    nav.setPlan(plan);
    // Over the middle of leg C -> D, heading east.
    AircraftState ac = makeAircraft(45.0, -97.5, 5000, 90);
    nav.activateNearestLeg(ac);
    CHECK(nav.plan().activeLeg == 3);
    NavState s = nav.update(ac, 0.1);
    CHECK(std::strcmp(s.activeWptIdent, "D") == 0);
    CHECK(s.magVarDeg == 0.0f);  // SyntheticNavDb reports zero
    // Before the start of the route, heading east: the first leg.
    nav.activateNearestLeg(makeAircraft(45.0, -100.4, 5000, 90));
    CHECK(nav.plan().activeLeg == 1);
}
