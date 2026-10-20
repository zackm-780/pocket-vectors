// "Perfect pilot" tests: a kinematic point-mass follower flies the NavState guidance (GPS legs,
// LOC / GS on the final) through complete plans with the real Navigator. No JSBSim involved.
#include <doctest/doctest.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "core/Geo.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "nav/ApproachBuilder.h"
#include "nav/FlightPlan.h"
#include "nav/NavDb.h"
#include "nav/Navigator.h"
#include "navigatorTestUtil.h"

using namespace fsim;
using namespace fsim::test;

namespace {

struct Sample {
    double tS = 0;
    NavState nav;
    double latDeg = 0, lonDeg = 0, altMslFt = 0, aglFt = 0;
};

struct FlightResult {
    bool reachedThreshold = false;
    double timeS = 0;
    std::vector<Sample> samples;   // every 2 s
    std::vector<int> legSequence;  // active leg index each time it changes
    std::vector<std::string> identSequence;
    std::set<NavPhase> phases;
    double firstLocS = -1;
    Sample atLocCapture, atEnd;
    // Worst deviations on final.
    double maxCdiWithin6Nm = 0, maxGsWithin45Nm = 0, maxXtkGpsSecondHalf = 0;
    bool anyMissed = false, legsOutOfOrder = false;
};

// Flies until `distToThresholdNm` < endNm, with the aircraft crossing the threshold, or until
// maxS. dt = 0.25 s.
FlightResult fly(Navigator& nav, Follower& pilot, double maxS, double endNm = 0.05) {
    FlightResult r;
    const double dt = 0.25;
    int lastLeg = -1;
    double nextSampleS = 0;
    std::vector<double> legLen;
    for (int i = 0; i < static_cast<int>(nav.plan().legs.size()); ++i)
        legLen.push_back(legDistNm(nav.plan(), i));
    NavState s = nav.update(pilot.ac, dt);
    for (double t = 0; t < maxS; t += dt) {
        pilot.step(s, dt);
        s = nav.update(pilot.ac, dt);
        r.phases.insert(s.phase);
        if (s.phase == NavPhase::Missed) r.anyMissed = true;
        const int leg = s.activeLegIndex;
        if (leg != lastLeg) {
            if (lastLeg >= 0 && (leg < lastLeg || leg > lastLeg + 1)) r.legsOutOfOrder = true;
            r.legSequence.push_back(leg);
            r.identSequence.emplace_back(s.activeWptIdent);
            lastLeg = leg;
            legLen.clear();
            for (int i = 0; i < static_cast<int>(nav.plan().legs.size()); ++i)
                legLen.push_back(legDistNm(nav.plan(), i));
        }
        Sample smp;
        smp.tS = t;
        smp.nav = s;
        smp.latDeg = pilot.ac.latDeg;
        smp.lonDeg = pilot.ac.lonDeg;
        smp.altMslFt = pilot.ac.altMslFt;
        smp.aglFt = pilot.ac.altAglFt;
        if (t >= nextSampleS) {
            r.samples.push_back(smp);
            nextSampleS += 2.0;
        }
        if (s.source == NavSource::Loc && r.firstLocS < 0) {
            r.firstLocS = t;
            r.atLocCapture = smp;
        }
        // Worst deviations on final.
        if (s.source == NavSource::Loc && s.distToThresholdNm <= 6.0f && s.distToThresholdNm > 0.3f)
            r.maxCdiWithin6Nm =
                std::max(r.maxCdiWithin6Nm, std::fabs(static_cast<double>(s.cdiDeflection)));
        if (s.source == NavSource::Loc && s.gsValid && s.distToThresholdNm <= 4.5f &&
            s.distToThresholdNm > 0.3f)
            r.maxGsWithin45Nm =
                std::max(r.maxGsWithin45Nm, std::fabs(static_cast<double>(s.gsDeflection)));
        // GPS legs: cross-track in the second half of long legs.
        if (s.source == NavSource::Gps && !s.onFinal && leg >= 1 &&
            leg < static_cast<int>(legLen.size()) && legLen[static_cast<size_t>(leg)] > 8.0 &&
            s.distToWptNm < 0.4 * legLen[static_cast<size_t>(leg)] && s.distToWptNm > 1.5f &&
            s.phase != NavPhase::Missed)
            r.maxXtkGpsSecondHalf =
                std::max(r.maxXtkGpsSecondHalf, std::fabs(static_cast<double>(s.crossTrackNm)));
        if (s.distToThresholdNm > 0 && s.distToThresholdNm < endNm &&
            s.activeLegIndex == nav.mapLegIndex()) {
            r.reachedThreshold = true;
            r.timeS = t;
            r.atEnd = smp;
            break;
        }
    }
    return r;
}

void printHighlights(const char* title, const FlightResult& r) {
    std::printf("---- %s ----\n", title);
    std::printf("reached threshold: %s after %.0f s\n", r.reachedThreshold ? "yes" : "NO", r.timeS);
    std::printf("legs: ");
    for (size_t i = 0; i < r.legSequence.size(); ++i)
        std::printf("%s%d(%s)", i ? " > " : "", r.legSequence[i], r.identSequence[i].c_str());
    std::printf("\n");
    if (r.firstLocS >= 0)
        std::printf("LOC capture at t=%.0f s: %.2f NM to threshold, cdi %.2f, gs %.2f (valid %d), "
                    "alt %.0f ft\n",
                    r.firstLocS, r.atLocCapture.nav.distToThresholdNm,
                    r.atLocCapture.nav.cdiDeflection, r.atLocCapture.nav.gsDeflection,
                    r.atLocCapture.nav.gsValid ? 1 : 0, r.atLocCapture.altMslFt);
    std::printf("max |cdi| within 6 NM: %.3f   max |gs| within 4.5 NM: %.3f   max GPS xtk (2nd "
                "half): %.3f NM\n",
                r.maxCdiWithin6Nm, r.maxGsWithin45Nm, r.maxXtkGpsSecondHalf);
    std::printf(
        "at threshold: src %d phase %d cdi %.3f gs %.3f (valid %d) agl %.0f ft dist %.3f NM\n",
        static_cast<int>(r.atEnd.nav.source), static_cast<int>(r.atEnd.nav.phase),
        r.atEnd.nav.cdiDeflection, r.atEnd.nav.gsDeflection, r.atEnd.nav.gsValid ? 1 : 0,
        r.atEnd.aglFt, r.atEnd.nav.distToThresholdNm);
    std::printf("trace (every ~60 s): t src phase leg wpt dist xtk cdi gs alt\n");
    for (size_t i = 0; i < r.samples.size(); i += 30) {
        const Sample& s = r.samples[i];
        std::printf("  %5.0f %d %d %2d %-7s %6.2f %6.3f %6.2f %6.2f %6.0f\n", s.tS,
                    static_cast<int>(s.nav.source), static_cast<int>(s.nav.phase),
                    s.nav.activeLegIndex, s.nav.activeWptIdent, s.nav.distToWptNm,
                    s.nav.crossTrackNm, s.nav.cdiDeflection, s.nav.gsDeflection, s.altMslFt);
    }
}

void checkPerfectApproach(const FlightResult& r) {
    CHECK(r.reachedThreshold);
    CHECK_FALSE(r.legsOutOfOrder);
    CHECK_FALSE(r.anyMissed);
    CHECK(r.phases.count(NavPhase::Approach) == 1);
    // LOC captured and tracked closely on the final.
    REQUIRE(r.firstLocS >= 0);
    CHECK(r.atLocCapture.nav.distToThresholdNm > 4.0f);
    CHECK(r.maxCdiWithin6Nm < 0.35);
    CHECK(r.maxGsWithin45Nm < 0.35);
    CHECK(r.maxXtkGpsSecondHalf < 0.25);
    CHECK(r.atEnd.nav.source == NavSource::Loc);
    CHECK(std::fabs(r.atEnd.nav.cdiDeflection) < 0.35f);
    CHECK(std::fabs(r.atEnd.nav.gsDeflection) < 0.35f);
    CHECK(r.atEnd.nav.gsValid);
    CHECK(r.atEnd.aglFt == doctest::Approx(50.0).epsilon(0.6));
    // The legs ended on the MAP (the threshold fix) and no missed approach started.
    CHECK(r.identSequence.back() == "RW09");
}

LatLon finalPointForTest(const Runway& r, double distNm) {
    return pointAtNm(r.thrLatDeg, r.thrLonDeg, r.headingTrueDeg + 180.0, distNm);
}

}  // namespace

TEST_CASE(
    "nav: perfect pilot flies the XTST pattern plan and the synthesized ILS 09 to the threshold") {
    SyntheticNavDb db;
    auto parsed = loadPlanFromFmsFile("assets/plans/XTST-pattern.fms", db);
    REQUIRE(parsed.ok);
    Navigator nav(db);
    nav.setAircraftCategory(AircraftCategory::Piston);
    nav.setPlan(parsed.plan);
    REQUIRE(nav.plan().approachStartLeg > 0);
    CHECK(nav.plan().approachId == "I09");
    // Straight-in entry from wp3: no IAF inserted (course change from wp3 to the IF is 71 degrees).
    for (const Leg& l : nav.plan().legs) CHECK(l.to.ident != "IAF");

    const Runway* r09 = findRunway(db.airport(), "09");
    REQUIRE(r09);
    Follower pilot;
    const LatLon start = pointAtNm(r09->thrLatDeg, r09->thrLonDeg, 90.0, 0.5);
    pilot.start(start.latDeg, start.lonDeg, 2400.0, 90.0, 120.0, 2000.0);
    nav.activateNearestLeg(pilot.ac);
    FlightResult r = fly(nav, pilot, 7200.0);
    printHighlights("XTST pattern + ILS 09", r);
    checkPerfectApproach(r);
    // All the plan's legs were flown in order: origin leg 0 is not flown, then wp1..wp3, IF, FAF, MAP.
    REQUIRE(r.legSequence.size() >= 6);
    CHECK(r.legSequence.front() == 1);
    CHECK(r.phases.count(NavPhase::Departure) + r.phases.count(NavPhase::Terminal) >= 1);
    CHECK(r.identSequence[r.identSequence.size() - 3] == "IF");
    CHECK(r.identSequence[r.identSequence.size() - 2] == "FAF");
}

TEST_CASE("nav: perfect pilot flies the pattern plan and the synthesized ILS 27 through an IAF") {
    SyntheticNavDb db;
    auto parsed = loadPlanFromFmsFile("assets/plans/XTST-pattern.fms", db);
    REQUIRE(parsed.ok);
    FlightPlan plan = parsed.plan;
    // Land on 27 instead: the plan ends at the 27 threshold and asks for I27. The pattern ends west
    // of the field, so the course to the IF (10 NM east) is ~180 degrees off the final: IAF entry.
    const Runway* r27 = findRunway(db.airport(), "27");
    REQUIRE(r27);
    plan.arrRunway = "27";
    plan.approachId = "I27";
    plan.approachStartLeg = -1;
    plan.legs.back().to = Waypoint{"RW27", WaypointKind::Runway, r27->thrLatDeg, r27->thrLonDeg};
    Navigator nav(db);
    nav.setPlan(plan);
    REQUIRE(nav.plan().approachStartLeg > 0);
    int iafLegs = 0;
    for (const Leg& l : nav.plan().legs) iafLegs += l.to.ident == "IAF" ? 1 : 0;
    CHECK(iafLegs == 1);

    Follower pilot;
    const Runway* r09 = findRunway(db.airport(), "09");
    const LatLon start = pointAtNm(r09->thrLatDeg, r09->thrLonDeg, 90.0, 0.5);
    pilot.start(start.latDeg, start.lonDeg, 2400.0, 90.0, 120.0, 2000.0);
    nav.activateNearestLeg(pilot.ac);
    FlightResult r = fly(nav, pilot, 7200.0);
    printHighlights("XTST pattern + ILS 27 via IAF", r);
    CHECK(r.reachedThreshold);
    CHECK_FALSE(r.legsOutOfOrder);
    CHECK_FALSE(r.anyMissed);
    REQUIRE(r.firstLocS >= 0);
    CHECK(r.maxCdiWithin6Nm < 0.35);
    CHECK(r.maxGsWithin45Nm < 0.35);
    CHECK(r.maxXtkGpsSecondHalf < 0.25);
    CHECK(r.atEnd.nav.source == NavSource::Loc);
    CHECK(r.identSequence.back() == "RW27");
    bool sawIaf = false, sawIf = false;
    for (const std::string& id : r.identSequence) {
        if (id == "IAF") sawIaf = true;
        if (id == "IF") {
            CHECK(sawIaf);  // the IAF is flown before the IF
            sawIf = true;
        }
    }
    CHECK(sawIaf);
    CHECK(sawIf);
}

TEST_CASE(
    "nav: perfect pilot on a synthesized RNAV approach stays on GPS and follows VNAV constraints") {
    SyntheticNavDb db;
    const Airport& ap = db.airport();
    const Runway* r27 = findRunway(ap, "27");
    Navigator nav(db);
    const LatLon start = offsetPoint(LatLon{r27->thrLatDeg, r27->thrLonDeg}, 90.0, 16.0, 3.0);
    REQUIRE(nav.startApproachOnly(buildRnavApproach(ap, *r27), start, 0));
    Follower pilot;
    pilot.start(start.latDeg, start.lonDeg, 5300.0, 250.0, 100.0, 2000.0);
    pilot.vsLimitFpm = 700;
    nav.activateNearestLeg(pilot.ac);
    FlightResult r = fly(nav, pilot, 3600.0);
    printHighlights("XTST RNAV 27", r);
    CHECK(r.reachedThreshold);
    CHECK(r.firstLocS < 0);  // never LOC on RNAV
    CHECK_FALSE(r.anyMissed);
    CHECK_FALSE(r.legsOutOfOrder);
    CHECK(r.atEnd.nav.source == NavSource::Gps);
    CHECK(std::fabs(r.atEnd.nav.crossTrackNm) < 0.05f);
    CHECK(r.atEnd.nav.cdiFullScaleNm == doctest::Approx(0.3f));
}

TEST_CASE("nav: landing past the threshold does not turn into a missed approach") {
    SyntheticNavDb db;
    const Airport& ap = db.airport();
    const Runway* r27 = findRunway(ap, "27");
    Navigator nav(db);
    const LatLon start = finalPointForTest(*r27, 9.0);
    REQUIRE(nav.startApproachOnly(buildIlsApproach(ap, *r27), start, 0));
    Follower pilot;
    pilot.start(start.latDeg, start.lonDeg, 5240.0, 270.0, 100.0, 2000.0);
    nav.activateNearestLeg(pilot.ac);
    FlightResult r = fly(nav, pilot, 3600.0, 0.05);
    REQUIRE(r.reachedThreshold);
    // Keep descending to touchdown on the glidepath (flare is not modelled), 1 NM past the threshold.
    NavState s;
    for (int i = 0; i < 80; ++i) {
        pilot.step(nav.state(), 0.25);
        pilot.ac.altMslFt -= 3.0;  // flare / touchdown is not modelled: sink onto the runway
        pilot.ac.altAglFt = static_cast<float>(pilot.ac.altMslFt - 2000.0);
        pilot.ac.wowAny = pilot.ac.altAglFt <= 0.0f;
        if (pilot.ac.wowAny) pilot.ac.altAglFt = 0.0f;
        s = nav.update(pilot.ac, 0.25);
        CHECK(s.phase != NavPhase::Missed);
    }
    CHECK(pilot.ac.wowAny);
    CHECK(s.phase == NavPhase::Approach);
}

TEST_CASE("nav: real data: KPDX-KOTH plan and approach I05 (manual, set FSIM_NAV_DATA_DIR)") {
    const char* dir = std::getenv("FSIM_NAV_DATA_DIR");
    if (!dir) return;  // manual check only; CI has no data pack
    CsvNavDatabase db;
    std::string err;
    REQUIRE(db.load(dir, err));
    auto parsed = loadPlanFromFmsFile("assets/plans/KPDX-KOTH.fms", db);
    REQUIRE(parsed.ok);
    Navigator nav(db);
    nav.setPlan(parsed.plan);
    REQUIRE(nav.plan().approachStartLeg > 0);
    const Airport* koth = db.findAirport("KOTH");
    const Airport* kpdx = db.findAirport("KPDX");
    REQUIRE(koth);
    REQUIRE(kpdx);
    const Runway* r05 = findRunway(*koth, "05");
    const Runway* r28l = findRunway(*kpdx, "28L");
    REQUIRE(r05);
    REQUIRE(r28l);
    REQUIRE(r05->ils.has_value());
    std::printf("KOTH 05 ILS: ident %s freq %.2f synthesized %d course %.1f width %.2f\n",
                r05->ils->ident, r05->ils->freqMhz, r05->ils->synthesized ? 1 : 0,
                r05->ils->locCourseTrueDeg, r05->ils->locWidthDeg);
    std::printf("plan legs: ");
    for (const Leg& l : nav.plan().legs)
        std::printf("%s%s ", l.to.ident.c_str(), l.missedApproach ? "(m)" : "");
    std::printf("\n");

    Follower pilot;
    const LatLon start = pointAtNm(r28l->startLatDeg, r28l->startLonDeg, r28l->headingTrueDeg, 2.0);
    // Terrain = the arrival field elevation (both fields are near sea level).
    pilot.start(start.latDeg, start.lonDeg, 3000.0, r28l->headingTrueDeg, 120.0, r05->thrElevFt);
    nav.activateNearestLeg(pilot.ac);
    FlightResult r = fly(nav, pilot, 14400.0);
    printHighlights("KPDX-KOTH + I05 (real NASR ILS)", r);
    CHECK(r.reachedThreshold);
    CHECK_FALSE(r.legsOutOfOrder);
    CHECK_FALSE(r.anyMissed);
    CHECK(r.firstLocS >= 0);
    CHECK(r.maxCdiWithin6Nm < 0.5);
    CHECK(r.maxGsWithin45Nm < 0.5);
}
