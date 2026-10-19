#include <doctest/doctest.h>

#include <cmath>

#include "core/Geo.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "nav/ApproachBuilder.h"
#include "navigatorTestUtil.h"

using namespace fsim;
using namespace fsim::test;

namespace {

double distNmTo(const Waypoint& w, double lat, double lon) {
    return mToNm(greatCircleDistM(w.latDeg, w.lonDeg, lat, lon));
}

struct Xtst {
    SyntheticNavDb db;
    const Airport& airport = db.airport();
    const Runway* r27 = findRunway(airport, "27");
    const Runway* r09 = findRunway(airport, "09");
};

}  // namespace

TEST_CASE("nav: ApproachBuilder ILS 27 geometry, altitudes and minimums") {
    Xtst x;
    const Approach a = buildIlsApproach(x.airport, *x.r27);
    CHECK(a.id == "I27");
    CHECK(a.kind == ApproachKind::Ils);
    CHECK(a.airportIdent == "XTST");
    CHECK(a.runwayIdent == "27");
    CHECK(a.synthesized);
    REQUIRE(a.finalLegs.size() == 3);
    const Leg& ifLeg = a.finalLegs[0];
    const Leg& faf = a.finalLegs[1];
    const Leg& map = a.finalLegs[2];
    CHECK(ifLeg.type == LegType::IF);
    CHECK(ifLeg.to.ident == "IF");
    CHECK(faf.to.ident == "FAF");
    CHECK(map.to.ident == "RW27");
    CHECK(map.to.kind == WaypointKind::Runway);
    CHECK(map.flyOver);
    // 10 / 5 NM out on the extended centerline (landing 270, so the fixes are EAST of the threshold).
    CHECK(distNmTo(ifLeg.to, x.r27->thrLatDeg, x.r27->thrLonDeg) == doctest::Approx(10.0).epsilon(0.002));
    CHECK(distNmTo(faf.to, x.r27->thrLatDeg, x.r27->thrLonDeg) == doctest::Approx(5.0).epsilon(0.002));
    CHECK(distNmTo(map.to, x.r27->thrLatDeg, x.r27->thrLonDeg) < 0.001);
    CHECK(ifLeg.to.lonDeg > x.r27->thrLonDeg);
    CHECK(inboundCourseTrueDeg(a) == doctest::Approx(270.0).epsilon(0.001));
    // Altitudes: IF = ceil100(thr + 3200) = 5200, FAF = thr + 1600, MAP = thr + 50.
    CHECK(ifLeg.alt.altFt == doctest::Approx(5200.0f));
    CHECK(faf.alt.altFt == doctest::Approx(3600.0f));
    CHECK(map.alt.altFt == doctest::Approx(2050.0f));
    // ILS DA = threshold + 200 ft (MSL).
    CHECK(a.minimumsFt == doctest::Approx(2200.0f));
    CHECK(a.minimumsIsDa);
}

TEST_CASE("nav: ApproachBuilder missed approach and IAF transitions") {
    Xtst x;
    const Approach a = buildIlsApproach(x.airport, *x.r27);
    REQUIRE(a.missedLegs.size() == 3);
    CHECK(a.missedLegs[0].type == LegType::CA);
    CHECK(a.missedLegs[0].courseTrueDeg == doctest::Approx(270.0f));
    CHECK(a.missedLegs[0].alt.altFt == doctest::Approx(3500.0f));  // threshold + 1500
    CHECK(a.missedLegs[1].type == LegType::DF);
    CHECK(a.missedLegs[1].to.ident == "IF");
    CHECK(a.missedLegs[2].type == LegType::HM);
    CHECK(a.missedLegs[2].to.ident == "IF");
    CHECK(a.missedLegs[2].courseTrueDeg == doctest::Approx(270.0f));  // inbound on final course
    CHECK(a.missedLegs[2].arcRight);                                   // right turns
    for (const Leg& l : a.missedLegs) CHECK(l.missedApproach);
    for (const Leg& l : a.finalLegs) CHECK_FALSE(l.missedApproach);

    REQUIRE(a.transitions.size() == 3);
    CHECK(a.transitions[0].legs.empty());  // vectors
    const Waypoint& ifw = a.finalLegs[0].to;
    const Waypoint& left = a.transitions[1].legs[0].to;
    const Waypoint& right = a.transitions[2].legs[0].to;
    CHECK(a.transitions[1].name == "IAF-L");
    CHECK(a.transitions[2].name == "IAF-R");
    // 4 NM abeam the IF; facing west the left side is south and the right side is north.
    CHECK(distNmTo(left, ifw.latDeg, ifw.lonDeg) == doctest::Approx(4.0).epsilon(0.002));
    CHECK(distNmTo(right, ifw.latDeg, ifw.lonDeg) == doctest::Approx(4.0).epsilon(0.002));
    CHECK(left.latDeg < ifw.latDeg);
    CHECK(right.latDeg > ifw.latDeg);
    CHECK(a.transitions[1].legs[0].alt.altFt == doctest::Approx(5200.0f));
}

TEST_CASE("nav: ApproachBuilder RNAV, visual and localizer-only variants") {
    Xtst x;
    const Approach r = buildRnavApproach(x.airport, *x.r09);
    CHECK(r.id == "R09");
    CHECK(r.kind == ApproachKind::Rnav);
    CHECK(r.minimumsFt == doctest::Approx(2400.0f));  // thr + 400, rounded up to 10
    CHECK_FALSE(r.minimumsIsDa);
    CHECK(inboundCourseTrueDeg(r) == doctest::Approx(90.0).epsilon(0.001));
    CHECK(r.finalLegs[0].alt.altFt == doctest::Approx(5200.0f));

    const Approach v = buildVisualApproach(x.airport, *x.r09);
    CHECK(v.id == "VIS09");
    CHECK(v.kind == ApproachKind::Visual);
    REQUIRE(v.finalLegs.size() == 2);
    CHECK(v.finalLegs[0].to.ident == "FAF");
    CHECK(distNmTo(v.finalLegs[0].to, x.r09->thrLatDeg, x.r09->thrLonDeg) ==
          doctest::Approx(3.0).epsilon(0.002));
    CHECK(v.finalLegs[0].alt.altFt == doctest::Approx(3000.0f));  // thr + 1000
    CHECK(v.missedLegs.size() == 1);                              // CA only: no hold
    CHECK(v.transitions.size() == 1);
    CHECK(v.minimumsFt == 0.0f);

    Runway locOnly = *x.r27;
    locOnly.ils->hasGs = false;
    const Approach l = buildIlsApproach(x.airport, locOnly);
    CHECK(l.id == "L27");
    CHECK(l.kind == ApproachKind::Loc);
    CHECK(l.minimumsFt == doctest::Approx(2400.0f));

    Runway noIls = *x.r27;
    noIls.ils.reset();
    CHECK(buildIlsApproach(x.airport, noIls).kind == ApproachKind::None);
}

TEST_CASE("nav: availableApproaches lists real first, then synthesized, no duplicate ids") {
    Xtst x;
    // Synthetic ILS + RNAV + visual, in that order.
    auto all = availableApproaches(x.airport, *x.r27);
    REQUIRE(all.size() == 3);
    CHECK(all[0].id == "I27");
    CHECK(all[1].id == "R27");
    CHECK(all[2].id == "VIS27");

    // A real provider: an ILS (same id as the synthesized one) and a VOR approach.
    RealApproachProvider provider = [](const Airport& ap, const Runway& rw) {
        std::vector<Approach> v;
        Approach ils;
        ils.id = "I27";
        ils.airportIdent = ap.ident;
        ils.runwayIdent = rw.ident;
        ils.kind = ApproachKind::Ils;
        ils.finalLegs.push_back(Leg{});
        v.push_back(ils);
        Approach vor = ils;
        vor.id = "V27";
        vor.kind = ApproachKind::Vor;
        v.push_back(vor);
        return v;
    };
    auto list = availableApproaches(x.airport, *x.r27, provider);
    REQUIRE(list.size() == 4);
    CHECK(list[0].id == "I27");
    CHECK_FALSE(list[0].synthesized);  // the real one wins the id
    CHECK(list[1].id == "V27");
    CHECK(list[2].id == "R27");
    CHECK(list[3].id == "VIS27");

    // By ident through the database, case-insensitive; unknown things give nothing.
    CHECK(availableApproaches(x.db, "XTST", "09").size() == 3);
    CHECK(availableApproaches(x.db, "XTST", "99").empty());
    CHECK(availableApproaches(x.db, "NOPE", "09").empty());
    auto found = findApproach(x.db, "XTST", "09", "r09");
    REQUIRE(found.has_value());
    CHECK(found->kind == ApproachKind::Rnav);
    CHECK_FALSE(findApproach(x.db, "XTST", "09", "R27").has_value());

    // A runway without an Ils offers only RNAV and visual.
    Runway noIls = *x.r27;
    noIls.ils.reset();
    auto two = availableApproaches(x.airport, noIls);
    REQUIRE(two.size() == 2);
    CHECK(two[0].id == "R27");
    CHECK(two[1].id == "VIS27");
}

TEST_CASE("nav: runwayFromApproachId") {
    CHECK(runwayFromApproachId("I05") == "05");
    CHECK(runwayFromApproachId("R28L") == "28L");
    CHECK(runwayFromApproachId("R29-Y") == "29");
    CHECK(runwayFromApproachId("VIS09") == "09");
    CHECK(runwayFromApproachId("i10r") == "10R");
    CHECK(runwayFromApproachId("V5") == "05");
    CHECK(runwayFromApproachId("").empty());
    CHECK(runwayFromApproachId("VDM").empty());
}

TEST_CASE("nav: IAF entry rule applies for course differences above 90 degrees") {
    Xtst x;
    const Approach a = buildIlsApproach(x.airport, *x.r27);
    const Waypoint& ifw = a.finalLegs[0].to;
    const LatLon ifPt{ifw.latDeg, ifw.lonDeg};
    // Final course 270, so inbound traffic comes from the east: straight in.
    CHECK(selectTransition(a, offsetPoint(ifPt, 90.0, 15.0, 3.0)) == 0);
    // Arriving from the west (course to the IF is ~090, 180 degrees off): IAF.
    LatLon westNorth = offsetPoint(ifPt, 270.0, 20.0, 8.0);  // right of bearing 270 = north
    CHECK(westNorth.latDeg > ifPt.latDeg);
    const int idxN = selectTransition(a, westNorth);
    CHECK(idxN == 2);  // IAF-R, the north side
    CHECK(a.transitions[static_cast<size_t>(idxN)].name == "IAF-R");
    LatLon westSouth = offsetPoint(ifPt, 270.0, 20.0, -8.0);
    CHECK(selectTransition(a, westSouth) == 1);  // IAF-L, south
    // Exactly around 90 degrees: prev 10 NM north of the IF and 1 NM east -> 84 deg -> vectors,
    // 1 NM west -> 96 deg -> IAF.
    CHECK(selectTransition(a, offsetPoint(ifPt, 0.0, 10.0, 1.0)) == 0);   // 1 NM east of north
    CHECK(selectTransition(a, offsetPoint(ifPt, 0.0, 10.0, -1.0)) == 2);  // 1 NM west of north
    // Visual approaches have no IAFs.
    const Approach v = buildVisualApproach(x.airport, *x.r27);
    CHECK(selectTransition(v, westNorth) == 0);
}

TEST_CASE("nav: approachPlanLegs flags missed legs and honours the transition index") {
    Xtst x;
    const Approach a = buildIlsApproach(x.airport, *x.r27);
    auto vec = approachPlanLegs(a, 0);
    CHECK(vec.size() == 6);  // IF FAF MAP + CA DF HM
    auto iaf = approachPlanLegs(a, 2);
    REQUIRE(iaf.size() == 7);
    CHECK(iaf[0].to.ident == "IAF");
    CHECK(iaf[1].to.ident == "IF");
    CHECK_FALSE(iaf[3].missedApproach);
    CHECK(iaf[4].missedApproach);
    CHECK(approachPlanLegs(a, 99).size() == 6);
    CHECK(approachPlanLegs(a, -1).size() == 6);
}
