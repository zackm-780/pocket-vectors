#include <doctest/doctest.h>

#include <cmath>
#include <utility>

#include "core/Geo.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "nav/FlightPlan.h"
#include "planFakeDb.h"

using namespace fsim;
using fsim::test::PlanFakeDb;

namespace {

Waypoint wp(const char* id, double lat, double lon, WaypointKind k = WaypointKind::Fix) {
    return Waypoint{id, k, lat, lon};
}

// KAAA RW28L -> A -> B -> C -> KBBB RW05 along a rough line, 5 legs counting the origin IF.
FlightPlan basePlan(const PlanFakeDb& db) {
    QuickPlanRequest q;
    q.originIdent = "KAAA";
    q.originRunway = "28L";
    q.destIdent = "KBBB";
    q.destRunway = "05";
    q.cruiseAltFt = 8000;
    auto r = buildQuickPlan(q, db);
    REQUIRE(r.ok);
    FlightPlan p = r.plan;
    REQUIRE(insertWaypoint(p, 1, wp("AAA", 44.5, -122.5)));
    REQUIRE(insertWaypoint(p, 2, wp("BBB", 44.0, -123.0)));
    REQUIRE(insertWaypoint(p, 3, wp("CCC", 43.5, -123.5)));
    return p;
}

Leg mk(LegType t, Waypoint w) {
    Leg l;
    l.type = t;
    l.to = std::move(w);
    return l;
}

Leg tf(const char* id, double lat, double lon, bool missed = false) {
    Leg l;
    l.type = LegType::TF;
    l.to = wp(id, lat, lon);
    l.missedApproach = missed;
    return l;
}

}  // namespace

TEST_CASE("nav: plan quick plan direct with runways") {
    PlanFakeDb db;
    QuickPlanRequest q;
    q.originIdent = "KAAA";
    q.originRunway = "28L";
    q.destIdent = "KBBB";
    q.destRunway = "05";
    q.cruiseAltFt = 7500;
    auto r = buildQuickPlan(q, db);
    REQUIRE(r.ok);
    const FlightPlan& p = r.plan;
    REQUIRE(p.legs.size() == 2);
    CHECK(p.legs[0].type == LegType::IF);
    CHECK(p.legs[0].to.ident == "RW28L");
    CHECK(p.legs[1].to.ident == "RW05");
    CHECK(p.depRunway == "28L");
    CHECK(p.arrRunway == "05");
    CHECK(p.cruiseAltFt == doctest::Approx(7500.0f));
    double expectNm = mToNm(greatCircleDistM(45.0, -121.98, 43.0, -124.02));
    CHECK(planDistNm(p) == doctest::Approx(expectNm));
    CHECK(enrouteEndIndex(p) == 1);

    SUBCASE("no runway: airports") {
        q.originRunway.clear();
        q.destRunway.clear();
        auto r2 = buildQuickPlan(q, db);
        REQUIRE(r2.ok);
        CHECK(r2.plan.legs[0].to.kind == WaypointKind::Airport);
        CHECK(r2.plan.depRunway.empty());
        // 2 degrees of latitude plus 2 of longitude: well over 120 NM.
        CHECK(planDistNm(r2.plan) > 120.0);
    }
    SUBCASE("unknown runway warns") {
        q.originRunway = "09";
        auto r2 = buildQuickPlan(q, db);
        REQUIRE(r2.ok);
        CHECK(r2.warnings.size() == 1);
        CHECK(r2.plan.legs[0].to.kind == WaypointKind::Airport);
    }
    SUBCASE("unknown destination fails") {
        q.destIdent = "KZZZ";
        auto r2 = buildQuickPlan(q, db);
        CHECK_FALSE(r2.ok);
        CHECK(r2.error.find("KZZZ") != std::string::npos);
    }
    SUBCASE("present position origin") {
        q.originIdent.clear();
        q.presentPos = LatLon{44.0, -122.0};
        auto r2 = buildQuickPlan(q, db);
        REQUIRE(r2.ok);
        CHECK(r2.plan.legs[0].to.ident == "PPOS");
        CHECK(r2.plan.legs[0].to.kind == WaypointKind::Synthetic);
        q.presentPos.reset();
        CHECK_FALSE(buildQuickPlan(q, db).ok);
    }
}

TEST_CASE("nav: plan quick plan on the synthetic database") {
    SyntheticNavDb db;
    QuickPlanRequest q;
    q.originIdent = "XTST";
    q.originRunway = "27";
    q.destIdent = "XTST";
    q.destRunway = "09";
    auto r = buildQuickPlan(q, db);
    REQUIRE(r.ok);
    CHECK(r.plan.legs[0].to.ident == "RW27");
    CHECK(r.plan.legs[1].to.ident == "RW09");
}

TEST_CASE("nav: plan quick plan via navaids") {
    PlanFakeDb db;
    // Origin KAAA (45N 122W) to a far destination 8 degrees south: ~480 NM on a meridian, VORs at
    // chosen places near the line (within 15 NM) and one far off.
    db.airports_[1].latDeg = 37.0;
    db.airports_[1].lonDeg = -122.0;
    db.airports_[1].runways.clear();
    db.navaids_.clear();
    auto vor = [&](const char* id, double lat, double lon) {
        Navaid n;
        n.ident = id;
        n.name = id;
        n.type = NavaidType::Vor;
        n.latDeg = lat;
        n.lonDeg = lon;
        db.navaids_.push_back(n);
    };
    vor("V1", 43.05, -122.1);  // 114 NM south, 4.4 NM off the line
    vor("V2", 41.2, -121.9);   // 228 NM
    vor("V3", 39.5, -122.0);   // 330 NM
    vor("V6", 38.2, -122.0);   // 408 NM
    vor("V4", 40.0, -125.0);   // 128 NM west of the line: rejected
    vor("V5", 44.95, -122.0);  // 3 NM from the origin: rejected (too close)
    Navaid ndb;
    ndb.ident = "N1";
    ndb.type = NavaidType::Ndb;
    ndb.latDeg = 42.0;
    ndb.lonDeg = -122.0;
    db.navaids_.push_back(ndb);  // NDBs are never used

    QuickPlanRequest q;
    q.originIdent = "KAAA";
    q.destIdent = "KBBB";
    q.cruiseAltFt = 9000;
    q.viaNavaids = true;
    q.navaidsNear = [&](double lat, double lon, float rNm) {
        return db.navaidsNear(lat, lon, rNm);
    };
    auto r = buildQuickPlan(q, db);
    REQUIRE(r.ok);
    std::string route;
    for (const Leg& l : r.plan.legs) route += l.to.ident + " ";
    INFO(route);
    REQUIRE(r.plan.legs.size() >= 4);
    CHECK(r.plan.legs[1].to.ident == "V1");
    // No leg is longer than 120 NM plus the slack of the VOR spacing.
    for (int i = 1; i < static_cast<int>(r.plan.legs.size()); ++i)
        CHECK(legDistNm(r.plan, i) <= 121.0);
    for (size_t i = 1; i + 1 < r.plan.legs.size(); ++i) {
        CHECK(r.plan.legs[i].to.kind == WaypointKind::Vor);
        CHECK(r.plan.legs[i].alt.altFt == doctest::Approx(9000.0f));
        CHECK(r.plan.legs[i].to.ident != "V4");
        CHECK(r.plan.legs[i].to.ident != "V5");
    }
    // Without a provider it falls back to direct and says so.
    q.navaidsNear = nullptr;
    auto d = buildQuickPlan(q, db);
    REQUIRE(d.ok);
    CHECK(d.plan.legs.size() == 2);
    CHECK(d.warnings.size() == 1);
}

TEST_CASE("nav: plan insert, delete and altitude constraints") {
    PlanFakeDb db;
    FlightPlan p = basePlan(db);
    REQUIRE(p.legs.size() == 5);
    CHECK(p.legs[1].to.ident == "AAA");
    CHECK(p.legs[3].to.ident == "CCC");
    CHECK(enrouteEndIndex(p) == 4);

    CHECK_FALSE(insertWaypoint(p, 0, wp("X", 1, 1)));      // before the origin
    CHECK_FALSE(insertWaypoint(p, 5, wp("X", 1, 1)));      // after the destination
    CHECK(insertWaypoint(p, 4, wp("DDD", 43.2, -123.8)));  // last enroute slot
    CHECK(p.legs[4].to.ident == "DDD");
    CHECK(p.legs[5].to.ident == "RW05");

    p.activeLeg = 3;
    CHECK(insertWaypoint(p, 2, wp("EEE", 44.2, -122.8)));
    CHECK(p.activeLeg == 4);  // still pointing at CCC
    CHECK(p.legs[p.activeLeg].to.ident == "CCC");
    CHECK(deleteLeg(p, 2));
    CHECK(p.activeLeg == 3);
    CHECK(p.legs[p.activeLeg].to.ident == "CCC");

    CHECK_FALSE(deleteLeg(p, 0));
    CHECK_FALSE(deleteLeg(p, static_cast<int>(p.legs.size()) - 1));  // destination
    CHECK_FALSE(deleteLeg(p, 99));

    AltConstraint c;
    c.kind = AltConstraintKind::AtOrAbove;
    c.altFt = 5500;
    CHECK(setAltConstraint(p, 1, c));
    CHECK(p.legs[1].alt.kind == AltConstraintKind::AtOrAbove);
    CHECK_FALSE(setAltConstraint(p, 42, c));
    // The constraint is exported.
    std::string txt = writePlanFmsText(p, db);
    CHECK(txt.find("11 AAA DRCT 5500.000000") != std::string::npos);
}

TEST_CASE("nav: plan direct-to a plan waypoint and to an ident") {
    PlanFakeDb db;
    FlightPlan p = basePlan(db);
    LatLon here{44.6, -122.4};
    SUBCASE("plan waypoint skips the legs before it") {
        REQUIRE(directToLeg(p, 2, here));  // BBB
        REQUIRE(p.legs.size() == 4);       // PPOS, DF BBB, CCC, destination
        CHECK(p.legs[0].to.ident == "PPOS");
        CHECK(p.legs[0].to.kind == WaypointKind::Synthetic);
        CHECK(p.legs[0].to.latDeg == doctest::Approx(here.latDeg));
        CHECK(p.legs[1].type == LegType::DF);
        CHECK(p.legs[1].to.ident == "BBB");
        CHECK(p.legs[2].to.ident == "CCC");
        CHECK(p.activeLeg == 1);
        CHECK(legDistNm(p, 1) ==
              doctest::Approx(mToNm(greatCircleDistM(here.latDeg, here.lonDeg, 44.0, -123.0))));
        // Origin entry is not written for a plan that no longer starts at the airport.
        CHECK_FALSE(directToLeg(p, 0, here));
    }
    SUBCASE("direct-to by ident finds plan waypoints first") {
        REQUIRE(directToIdent(p, "ccc", db, here));
        CHECK(p.legs[1].to.ident == "CCC");
        CHECK(p.legs.size() == 3);
    }
    SUBCASE("direct-to a DB ident that is not in the plan") {
        REQUIRE(directToIdent(p, "MID", db, here));
        REQUIRE(p.legs.size() == 3);
        CHECK(p.legs[1].type == LegType::DF);
        CHECK(p.legs[1].to.ident == "MID");
        CHECK(p.legs[1].to.kind == WaypointKind::Vor);
        CHECK(p.legs[2].to.ident == "RW05");  // straight on to the destination
        CHECK_FALSE(directToIdent(p, "NOPE", db, here));
    }
    SUBCASE("ambiguous ident picks the nearest to the present position") {
        REQUIRE(directToIdent(p, "CVO", db, here));
        CHECK(p.legs[1].to.latDeg == doctest::Approx(44.5));
    }
    SUBCASE("direct-to with an approach inserted keeps the approach") {
        std::vector<Leg> app = {tf("IAF", 43.1, -124.5), tf("FAF", 43.0, -124.2),
                                tf("MAP", 43.0, -124.02), tf("MISS", 43.3, -124.0, true)};
        REQUIRE(insertApproach(p, "I05", ApproachKind::Ils, "05", app));
        directToWaypoint(p, wp("DDD", 43.6, -123.4), here);
        CHECK(p.legs.size() == 6);
        CHECK(p.approachStartLeg == 2);
        CHECK(p.legs[p.approachStartLeg].to.ident == "IAF");
        CHECK(p.legs.back().missedApproach);
        CHECK(p.approachId == "I05");
    }
    SUBCASE("direct-to a waypoint inside the approach") {
        std::vector<Leg> app = {tf("IAF", 43.1, -124.5), tf("FAF", 43.0, -124.2),
                                tf("MAP", 43.0, -124.02)};
        REQUIRE(insertApproach(p, "I05", ApproachKind::Ils, "05", app));
        int faf = p.approachStartLeg + 1;
        REQUIRE(directToLeg(p, faf, here));
        CHECK(p.legs.size() == 3);  // PPOS, DF FAF, MAP
        CHECK(p.approachStartLeg == 1);
        CHECK(p.legs[1].to.ident == "FAF");
    }
}

TEST_CASE("nav: plan approach insertion replaces the tail") {
    PlanFakeDb db;
    FlightPlan p = basePlan(db);
    const size_t before = p.legs.size();
    std::vector<Leg> app = {tf("IAF", 43.1, -124.5),         tf("IF", 43.05, -124.35),
                            tf("FAF", 43.0, -124.2),         tf("RW05", 43.0, -124.02),
                            tf("MISS1", 43.2, -124.0, true), tf("HOLD", 43.4, -124.0, true)};
    app[3].flyOver = true;
    p.activeLeg = 2;
    std::string err;
    REQUIRE(insertApproach(p, "I05", ApproachKind::Ils, "05", app, &err));
    CHECK(p.legs.size() == before - 1 + app.size());
    CHECK(p.approachStartLeg == 4);
    CHECK(p.legs[3].to.ident == "CCC");  // last enroute waypoint is untouched
    CHECK(p.legs[4].to.ident == "IAF");
    CHECK(p.legs[7].flyOver);
    CHECK_FALSE(p.legs[7].missedApproach);
    CHECK(p.legs[8].missedApproach);
    CHECK(p.legs[9].missedApproach);
    CHECK(p.approachId == "I05");
    CHECK(p.approachKind == ApproachKind::Ils);
    CHECK(p.activeLeg == 2);  // active leg was enroute, untouched
    CHECK(enrouteEndIndex(p) == 4);

    // Distance counts the flown legs only unless missed legs are asked for.
    double flown = planDistNm(p);
    double all = planDistNm(p, 0, true);
    CHECK(all > flown);
    CHECK(all - flown == doctest::Approx(legDistNm(p, 8) + legDistNm(p, 9)));
    CHECK(legDistNm(p, 8) > 0);

    // Edits stay in the enroute part and keep the approach marker right.
    CHECK(insertWaypoint(p, 4, wp("NEW", 43.4, -123.7)));
    CHECK(p.approachStartLeg == 5);
    CHECK(p.legs[5].to.ident == "IAF");
    CHECK_FALSE(insertWaypoint(p, 6, wp("BAD", 1, 1)));  // inside the approach
    CHECK_FALSE(deleteLeg(p, 6));
    CHECK(deleteLeg(p, 4));
    CHECK(p.approachStartLeg == 4);

    // Writing the plan keeps the APP name but not the legs.
    std::string txt = writePlanFmsText(p, db);
    CHECK(txt.find("APP I05\n") != std::string::npos);
    CHECK(txt.find("IAF") == std::string::npos);
    CHECK(txt.find("ADES KBBB") != std::string::npos);
    auto back = parseFms(txt);
    REQUIRE(back.ok);
    CHECK(back.file.entries.back().ident == "KBBB");
    CHECK(back.file.entries.size() == 5);  // KAAA, AAA, BBB, CCC, KBBB

    // Replace by a second approach: no duplicates.
    std::vector<Leg> app2 = {tf("X1", 43.1, -124.4), tf("X2", 43.0, -124.1)};
    REQUIRE(insertApproach(p, "R05", ApproachKind::Rnav, "05", app2));
    CHECK(p.approachStartLeg == 4);
    CHECK(p.legs.size() == 6);
    CHECK(p.approachKind == ApproachKind::Rnav);

    // An active leg inside the replaced tail moves to the first approach leg.
    p.activeLeg = 5;
    REQUIRE(insertApproach(p, "I05", ApproachKind::Ils, "05", app));
    CHECK(p.activeLeg == 4);

    // Remove: the destination returns.
    removeApproach(p, db);
    CHECK(p.approachStartLeg == -1);
    CHECK(p.approachId.empty());
    CHECK(p.approachKind == ApproachKind::None);
    CHECK(p.legs.back().to.ident == "RW05");
    CHECK(p.legs.size() == 5);
    CHECK(p.activeLeg == 4);
}

TEST_CASE("nav: plan approach insertion rejects bad leg lists") {
    PlanFakeDb db;
    FlightPlan p = basePlan(db);
    std::string err;
    CHECK_FALSE(insertApproach(p, "I05", ApproachKind::Ils, "05", {}, &err));
    CHECK(err.find("no legs") != std::string::npos);
    CHECK_FALSE(insertApproach(p, "I05", ApproachKind::Ils, "05",
                               {tf("A", 43, -124, true), tf("B", 43, -124, false)}, &err));
    CHECK(err.find("missed") != std::string::npos);
    CHECK_FALSE(insertApproach(p, "I05", ApproachKind::Ils, "05", {tf("A", 43, -124, true)}, &err));
    CHECK(p.approachStartLeg == -1);  // untouched on failure
    CHECK(p.legs.size() == 5);
}

TEST_CASE("nav: plan distances and ETE match great-circle values") {
    FlightPlan p;
    p.legs.push_back(mk(LegType::IF, wp("A", 0.0, 0.0)));
    p.legs.push_back(mk(LegType::TF, wp("B", 1.0, 0.0)));  // 1 degree of latitude
    p.legs.push_back(mk(LegType::TF, wp("C", 1.0, 1.0)));  // 1 degree of longitude at 1 N
    const double degNm = mToNm(kEarthRadiusM * degToRad(1.0));
    CHECK(degNm == doctest::Approx(60.04).epsilon(0.001));
    CHECK(legDistNm(p, 0) == 0.0);
    CHECK(legDistNm(p, 1) == doctest::Approx(degNm).epsilon(1e-6));
    CHECK(legDistNm(p, 2) == doctest::Approx(degNm * std::cos(degToRad(1.0))).epsilon(1e-3));
    double total = planDistNm(p);
    CHECK(total == doctest::Approx(legDistNm(p, 1) + legDistNm(p, 2)));
    CHECK(eteS(total, 120.0) == doctest::Approx(total / 120.0 * 3600.0));
    CHECK(eteS(60.0, 120.0) == doctest::Approx(1800.0));
    CHECK(eteS(60.0, 0.0) == 0.0);
    CHECK(planDistNm(p, 2) == doctest::Approx(legDistNm(p, 2)));

    // Independently known: one degree of longitude on the equator.
    FlightPlan e;
    e.legs.push_back(mk(LegType::IF, wp("A", 0.0, 10.0)));
    e.legs.push_back(mk(LegType::TF, wp("B", 0.0, 11.0)));
    CHECK(legDistNm(e, 1) == doctest::Approx(60.04).epsilon(0.001));

    // Distance remaining from the present position.
    p.activeLeg = 1;
    CHECK(distRemainingNm(p, LatLon{0.5, 0.0}) ==
          doctest::Approx(degNm * 0.5 + legDistNm(p, 2)).epsilon(1e-4));
    p.activeLeg = 2;
    CHECK(distRemainingNm(p, LatLon{1.0, 0.5}) ==
          doctest::Approx(degNm * 0.5 * std::cos(degToRad(1.0))).epsilon(1e-3));
}

TEST_CASE("nav: plan distance of the KPDX to KOTH sample route") {
    // Great-circle legs of assets/plans/KPDX-KOTH.fms computed independently.
    FlightPlan p;
    p.legs.push_back(mk(LegType::IF, wp("KPDX", 45.588699, -122.598, WaypointKind::Airport)));
    const struct {
        const char* id;
        double lat, lon;
    } pts[] = {{"UBG", 45.353199, -122.977997},
               {"CVO", 44.499599, -123.293999},
               {"EUG", 44.120899, -123.223},
               {"OTH", 43.415501, -124.167999},
               {"KOTH", 43.417099, -124.246002}};
    for (auto& q : pts) p.legs.push_back(mk(LegType::TF, wp(q.id, q.lat, q.lon)));
    CHECK(planDistNm(p) == doctest::Approx(159.61).epsilon(0.002));
    CHECK(eteS(planDistNm(p), 110.0) / 60.0 == doctest::Approx(87.06).epsilon(0.005));
}

TEST_CASE("nav: plan DTK true and magnetic") {
    PlanFakeDb db;
    FlightPlan p;
    p.legs.push_back(mk(LegType::IF, wp("A", 0.0, 0.0)));
    p.legs.push_back(mk(LegType::TF, wp("B", 0.0, 1.0)));   // due east
    p.legs.push_back(mk(LegType::TF, wp("C", -1.0, 1.0)));  // due south
    CHECK_FALSE(legDtkTrueDeg(p, 0).has_value());
    CHECK(*legDtkTrueDeg(p, 1) == doctest::Approx(90.0f).epsilon(1e-4));
    CHECK(*legDtkTrueDeg(p, 2) == doctest::Approx(180.0f).epsilon(1e-4));
    MagVarFn east15 = [](double, double) { return 15.0f; };
    CHECK(*legDtkMagDeg(p, 1, east15) ==
          doctest::Approx(75.0f).epsilon(1e-4));  // east var: mag < true
    MagVarFn west20 = [](double, double) { return -20.0f; };
    CHECK(*legDtkMagDeg(p, 1, west20) == doctest::Approx(110.0f).epsilon(1e-4));
    MagVarFn viaDb = [&](double lat, double lon) { return db.magVarDeg(lat, lon); };
    CHECK(*legDtkMagDeg(p, 2, viaDb) == doctest::Approx(165.0f).epsilon(1e-4));
    // Wraps through north.
    p.legs[2].to = wp("C", 1.0, 1.0);  // due north: 360 -> 0
    CHECK(*legDtkMagDeg(p, 2, [](double, double) { return -5.0f; }) ==
          doctest::Approx(5.0f).epsilon(1e-3));

    SUBCASE("CF and CA legs use their own course") {
        p.legs[2].type = LegType::CF;
        p.legs[2].courseTrueDeg = 270.0f;
        CHECK(*legDtkTrueDeg(p, 2) == doctest::Approx(270.0f));
        p.legs[2].type = LegType::CA;
        CHECK(legDistNm(p, 2) == 0.0);
    }
    SUBCASE("a CA leg does not move the leg start") {
        p.legs.insert(p.legs.begin() + 2, mk(LegType::CA, wp("", 0, 0)));
        p.legs[2].courseTrueDeg = 90.0f;
        // The leg after the CA starts from B again.
        CHECK(legDistNm(p, 3) > 0);
        CHECK(legDistNm(p, 3) ==
              doctest::Approx(mToNm(greatCircleDistM(0.0, 1.0, 1.0, 1.0))).epsilon(1e-6));
    }
}

TEST_CASE("nav: plan RF leg length is the arc length") {
    FlightPlan p;
    p.legs.push_back(mk(LegType::IF, wp("S", 0.0, 0.0)));
    Leg rf;
    rf.type = LegType::RF;
    // Centre 2 NM north of S; quarter circle to the right (clockwise): ends 2 NM north, 2 NM... of centre.
    const double twoNmDeg = 2.0 / 60.0;
    rf.arcCenterLatDeg = twoNmDeg;
    rf.arcCenterLonDeg = 0.0;
    rf.arcRadiusNm = 2.0f;
    rf.arcRight = false;  // from the south of the centre going counter-clockwise = to the east
    rf.to = wp("E", twoNmDeg, twoNmDeg);
    p.legs.push_back(rf);
    CHECK(legDistNm(p, 1) == doctest::Approx(2.0 * kPi / 2.0).epsilon(0.01));
    p.legs[1].arcRight = true;  // the long way round
    CHECK(legDistNm(p, 1) == doctest::Approx(2.0 * 1.5 * kPi).epsilon(0.01));
}

TEST_CASE("nav: plan summary rows") {
    PlanFakeDb db;
    FlightPlan p = basePlan(db);
    std::vector<Leg> app = {tf("IAF", 43.1, -124.5), tf("MISS", 43.3, -124.0, true)};
    REQUIRE(insertApproach(p, "I05", ApproachKind::Ils, "05", app));
    MagVarFn mv = [](double, double) { return 15.0f; };
    auto rows = planSummary(p, mv);
    REQUIRE(rows.size() == p.legs.size());
    CHECK_FALSE(rows[0].dtkTrueDeg.has_value());
    CHECK(rows[0].distNm == 0.0);
    REQUIRE(rows[1].dtkTrueDeg.has_value());
    REQUIRE(rows[1].dtkMagDeg.has_value());
    CHECK(wrap360(*rows[1].dtkTrueDeg - *rows[1].dtkMagDeg) == doctest::Approx(15.0));
    CHECK(rows[1].ident == "AAA");
    CHECK(rows.back().missedApproach);
    double sum = 0;
    for (const auto& r : rows)
        if (!r.missedApproach) sum += r.distNm;
    CHECK(sum == doctest::Approx(planDistNm(p)));
    CHECK_FALSE(planSummary(p)[1].dtkMagDeg.has_value());
}
