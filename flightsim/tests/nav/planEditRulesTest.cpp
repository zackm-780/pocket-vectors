// N11a: the plan-edit rules of NAVIGATION.md 2.4, table-driven. Each row is a starting plan, an
// edit and where the aircraft is, then the legs, the active leg, the procedures left and the one
// message the pilot sees.
#include <doctest/doctest.h>

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "SimFixture.h"
#include "autopilot/Autopilot.h"
#include "core/Geo.h"
#include "nav/FlightPlan.h"
#include "nav/FlyBy.h"
#include "nav/Navigator.h"
#include "nav/PlanEdit.h"
#include "navigatorTestUtil.h"
#include "planFakeDb.h"

using namespace fsim;
using fsim::test::PlanFakeDb;

namespace {

Waypoint fix(const char* id, double lat, double lon, WaypointKind k = WaypointKind::Fix) {
    return Waypoint{id, k, lat, lon};
}

Leg tf(const char* id, double lat, double lon, bool missed = false) {
    Leg l;
    l.type = LegType::TF;
    l.to = fix(id, lat, lon);
    l.missedApproach = missed;
    return l;
}

// A third airport for "another airport" and "new destination": KCCC, 44N 121W, runway 09/27.
PlanFakeDb makeDb() {
    PlanFakeDb db;
    Airport c;
    c.ident = "KCCC";
    c.name = "Charlie Field";
    c.latDeg = 44.0;
    c.lonDeg = -121.0;
    c.elevFt = 3000;
    Runway r;
    r.ident = "09";
    r.oppositeIdent = "27";
    r.thrLatDeg = r.startLatDeg = 44.0;
    r.thrLonDeg = r.startLonDeg = -121.02;
    r.endLatDeg = 44.0;
    r.endLonDeg = -120.98;
    c.runways.push_back(r);
    db.airports_.push_back(c);
    return db;
}

enum class Start { Enroute, WithSid, WithStarApproach, InMissed };

// KAAA RW28L -> AAA -> BBB -> CCC -> KBBB RW05, a south-westerly route.
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
    REQUIRE(insertWaypoint(p, 1, fix("AAA", 44.5, -122.5)));
    REQUIRE(insertWaypoint(p, 2, fix("BBB", 44.0, -123.0)));
    REQUIRE(insertWaypoint(p, 3, fix("CCC", 43.5, -123.5)));
    return p;
}

FlightPlan startPlan(const PlanFakeDb& db, Start start) {
    FlightPlan p = basePlan(db);
    switch (start) {
    case Start::Enroute: p.activeLeg = 2; break;  // flying AAA -> BBB
    case Start::WithSid:
        REQUIRE(insertSid(p, "MINNE5", "", {tf("SID1", 44.9, -122.2), tf("SID2", 44.7, -122.4)}));
        p.activeLeg = 1;
        break;
    case Start::WithStarApproach:
    case Start::InMissed:
        REQUIRE(insertStar(p, "STAR1", "", {tf("ST1", 43.4, -123.6), tf("ST2", 43.2, -123.8)}));
        REQUIRE(insertApproach(p, "I05", ApproachKind::Ils, "05",
                               {tf("IAF", 43.1, -124.5), tf("FAF", 43.0, -124.2),
                                tf("MAP", 43.0, -124.02), tf("MISS", 43.3, -124.0, true),
                                tf("HOLDX", 43.5, -124.2, true)}));
        p.activeLeg = start == Start::InMissed ? 9 : 3;  // MISS, or flying BBB -> CCC
        break;
    }
    return p;
}

std::string legIdents(const FlightPlan& p) {
    std::string s;
    for (const Leg& l : p.legs) s += (s.empty() ? "" : " ") + l.to.ident;
    return s;
}

PlanEditContext ctxAt(double lat, double lon, float trackDeg, bool airborne = true) {
    PlanEditContext c;
    c.present = LatLon{lat, lon};
    c.trackTrueDeg = trackDeg;
    c.gsKt = 120;
    c.airborne = airborne;
    return c;
}

struct Row {
    const char* name;
    Start start;
    std::function<PlanEditResult(const FlightPlan&, const PlanFakeDb&)> edit;
    bool ok;
    const char* legs;  // expected leg idents
    int active;
    int approachStart;
    const char* sid;
    const char* star;
    const char* message;
    bool destructive;
};

}  // namespace

TEST_CASE("nav: plan edit rules (N11a, NAVIGATION 2.4)") {
    const PlanFakeDb db = makeDb();
    // Between AAA and BBB, tracking south-west; past BBB toward CCC; near CCC.
    const PlanEditContext midLeg = ctxAt(44.4, -122.6, 225);
    const PlanEditContext pastBbb = ctxAt(43.8, -123.2, 225);
    const PlanEditContext nearCcc = ctxAt(43.7, -123.3, 225);
    const PlanEditContext missed = ctxAt(43.2, -124.0, 0);
    const Waypoint kbbb = fix("KBBB", 43.0, -124.0, WaypointKind::Airport);
    const Waypoint kccc = fix("KCCC", 44.0, -121.0, WaypointKind::Airport);

    const std::vector<Row> rows = {
        // ---- rule 1: Direct-To a waypoint in the plan
        {"1: direct to a plan waypoint ahead", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             return directTo(p, fix("CCC", 43.5, -123.5), midLeg);
         },
         true, "PPOS CCC RW05", 1, -1, "", "", "DIRECT TO CCC", false},
        {"1: direct to an enroute fix past the SID drops the SID", Start::WithSid,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             return directTo(p, fix("AAA", 44.5, -122.5), ctxAt(44.8, -122.3, 225));
         },
         true, "PPOS AAA BBB CCC RW05", 1, -1, "", "", "DIRECT TO AAA", false},
        {"1: direct to a SID fix keeps the SID", Start::WithSid,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             return directTo(p, fix("SID2", 44.7, -122.4), ctxAt(44.95, -122.1, 225));
         },
         true, "PPOS SID2 AAA BBB CCC RW05", 1, -1, "MINNE5", "", "DIRECT TO SID2", false},
        {"1: direct to a STAR fix keeps the STAR and the approach", Start::WithStarApproach,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             return directTo(p, fix("ST2", 43.2, -123.8), nearCcc);
         },
         true, "PPOS ST2 IAF FAF MAP MISS HOLDX", 1, 2, "", "STAR1", "DIRECT TO ST2", false},
        // ---- rule 2: Direct-To a fix that is not in the plan
        {"2: off to the side, rejoin at the active waypoint", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             return directTo(p, fix("SIDE", 44.3, -122.5), midLeg);
         },
         true, "PPOS SIDE BBB CCC RW05", 1, -1, "", "", "DIRECT TO SIDE, THEN BBB", false},
        {"2: beyond the active waypoint, it is dropped", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             return directTo(p, fix("MID", 44.0, -123.5, WaypointKind::Vor), midLeg);
         },
         true, "PPOS MID CCC RW05", 1, -1, "", "", "DIRECT TO MID, THEN CCC", false},
        {"2: nothing sensible ahead, rejoin at the destination", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             return directTo(p, fix("FAR", 40.0, -118.0, WaypointKind::Vor), midLeg);
         },
         true, "PPOS FAR RW05", 1, -1, "", "", "DIRECT TO FAR, THEN RW05", false},
        {"2: behind, with a STAR and an approach: rejoin at the approach", Start::WithStarApproach,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             return directTo(p, fix("MID", 44.0, -123.5, WaypointKind::Vor), nearCcc);
         },
         true, "PPOS MID IAF FAF MAP MISS HOLDX", 1, 2, "", "", "DIRECT TO MID, THEN IAF", false},
        {"2: a fix near the STAR rejoins the STAR", Start::WithStarApproach,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             return directTo(p, fix("NEAR", 43.45, -123.45), nearCcc);
         },
         true, "PPOS NEAR ST1 ST2 IAF FAF MAP MISS HOLDX", 1, 4, "", "STAR1",
         "DIRECT TO NEAR, THEN ST1", false},
        // ---- rule 3: Direct-To the destination
        {"3: the destination keeps the STAR and the approach", Start::WithStarApproach,
         [&](const FlightPlan& p, const PlanFakeDb&) { return directTo(p, kbbb, nearCcc); }, true,
         "PPOS KBBB ST1 ST2 IAF FAF MAP MISS HOLDX", 1, 4, "", "STAR1", "DIRECT TO KBBB", false},
        {"3: the destination without a STAR or approach flies the destination leg", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) { return directTo(p, kbbb, midLeg); }, true,
         "PPOS RW05", 1, -1, "", "", "DIRECT TO KBBB", false},
        // ---- rule 4: Direct-To another airport
        {"4: another airport replaces the plan", Start::WithStarApproach,
         [&](const FlightPlan& p, const PlanFakeDb&) { return directTo(p, kccc, nearCcc); }, true,
         "PPOS KCCC", 1, -1, "", "", "DIRECT TO KCCC, ROUTE REPLACED", true},
        // ---- rule 5: a new destination
        {"5: new destination, clear the route (the default)", Start::WithStarApproach,
         [&](const FlightPlan& p, const PlanFakeDb& d) {
             return changeDestination(p, d, "KCCC", "09", false, nearCcc);
         },
         true, "PPOS RW09", 1, -1, "", "", "NEW DEST KCCC, ROUTE CLEARED", true},
        {"5: new destination, keep the route: STAR and approach go", Start::WithStarApproach,
         [&](const FlightPlan& p, const PlanFakeDb& d) {
             return changeDestination(p, d, "kccc", "", true, nearCcc);
         },
         true, "RW28L AAA BBB CCC KCCC", 3, -1, "", "", "NEW DEST KCCC, ROUTE KEPT", true},
        {"5: keep the route with the active leg in the removed tail: direct to the new one",
         Start::InMissed,
         [&](const FlightPlan& p, const PlanFakeDb& d) {
             return changeDestination(p, d, "KCCC", "09", true, missed);
         },
         true, "PPOS RW09", 1, -1, "", "", "NEW DEST KCCC, ROUTE KEPT", true},
        {"5: on the ground, clear the route from the departure runway", Start::WithSid,
         [&](const FlightPlan& p, const PlanFakeDb& d) {
             return changeDestination(p, d, "KCCC", "", false, ctxAt(45.0, -121.98, 280, false));
         },
         true, "RW28L KCCC", 1, -1, "", "", "NEW DEST KCCC, ROUTE CLEARED", true},
        {"5: an unknown airport changes nothing", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb& d) {
             return changeDestination(p, d, "KZZZ", "", false, midLeg);
         },
         false, "RW28L AAA BBB CCC RW05", 2, -1, "", "", "UNKNOWN AIRPORT KZZZ", false},
        // ---- rule 6: departure runway, SID, STAR, approach replace only their part
        {"6: a new departure runway removes a SID that does not serve it", Start::WithSid,
         [&](const FlightPlan& p, const PlanFakeDb& d) {
             return changeDepartureRunway(p, d, "10R", std::nullopt);
         },
         true, "RW10R AAA BBB CCC RW05", 1, -1, "", "", "DEPARTURE RWY 10R, SID MINNE5 REMOVED",
         true},
        {"6: a new departure runway re-assembles the SID that serves it", Start::WithSid,
         [&](const FlightPlan& p, const PlanFakeDb& d) {
             return changeDepartureRunway(
                 p, d, "10R",
                 std::vector<Leg>{tf("SIDE1", 45.0, -121.9), tf("SID2", 44.7, -122.4)});
         },
         true, "RW10R SIDE1 SID2 AAA BBB CCC RW05", 1, -1, "MINNE5", "", "DEPARTURE RWY 10R",
         false},
        {"6: a new STAR replaces only the STAR legs", Start::WithStarApproach,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             PlanEditResult r;
             r.plan = p;
             r.ok = insertStar(r.plan, "STAR2", "", {tf("NS1", 43.3, -123.7)});
             return r;
         },
         true, "RW28L AAA BBB CCC NS1 IAF FAF MAP MISS HOLDX", 3, 5, "", "STAR2", "", false},
        {"6: a new approach replaces only the approach legs", Start::WithStarApproach,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             PlanEditResult r;
             r.plan = p;
             r.ok = insertApproach(r.plan, "R05", ApproachKind::Rnav, "05",
                                   {tf("RIF", 43.05, -124.3), tf("RMAP", 43.0, -124.02)});
             return r;
         },
         true, "RW28L AAA BBB CCC ST1 ST2 RIF RMAP", 3, 6, "", "STAR1", "", false},
        // ---- rule 7: delete or skip a waypoint
        {"7: delete the active waypoint: direct to the next", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) { return deleteWaypoint(p, 2, midLeg); }, true,
         "PPOS CCC RW05", 1, -1, "", "", "BBB DELETED, DIRECT TO CCC", false},
        {"7: delete a later waypoint", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) { return deleteWaypoint(p, 3, midLeg); }, true,
         "RW28L AAA BBB RW05", 2, -1, "", "", "CCC DELETED", false},
        {"7: approach legs are not deleted one by one", Start::WithStarApproach,
         [&](const FlightPlan& p, const PlanFakeDb&) { return deleteWaypoint(p, 7, nearCcc); },
         false, "RW28L AAA BBB CCC ST1 ST2 IAF FAF MAP MISS HOLDX", 3, 6, "", "STAR1",
         "USE CHANGE APPROACH", false},
        {"7: the destination is changed, not deleted", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) { return deleteWaypoint(p, 4, midLeg); },
         false, "RW28L AAA BBB CCC RW05", 2, -1, "", "", "USE NEW DESTINATION", false},
        // ---- rule 8: back onto the route after vectors
        {"8: direct to next, the active waypoint still ahead", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) { return directToNext(p, midLeg); }, true,
         "PPOS BBB CCC RW05", 1, -1, "", "", "DIRECT TO BBB", false},
        {"8: direct to next after vectors past the active waypoint", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) { return directToNext(p, pastBbb); }, true,
         "PPOS CCC RW05", 1, -1, "", "", "DIRECT TO CCC", false},
        {"8: direct to next, vectored back toward the passed active waypoint", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             return directToNext(p, ctxAt(43.8, -123.2, 45));
         },
         true, "PPOS BBB CCC RW05", 1, -1, "", "", "DIRECT TO BBB", false},
        {"8: activate a leg (intercept its course, no direct-to)", Start::Enroute,
         [&](const FlightPlan& p, const PlanFakeDb&) { return activateLeg(p, 3, midLeg); }, true,
         "RW28L AAA BBB CCC RW05", 3, -1, "", "", "ACTIVATE LEG CCC", false},
        // ---- rule 9: the missed approach
        {"9: direct to a missed-approach fix", Start::InMissed,
         [&](const FlightPlan& p, const PlanFakeDb&) {
             return directTo(p, fix("HOLDX", 43.5, -124.2), missed);
         },
         true, "PPOS HOLDX", 1, 1, "", "", "DIRECT TO HOLDX", false},
        {"9: direct to next in the missed approach stays in it", Start::InMissed,
         [&](const FlightPlan& p, const PlanFakeDb&) { return directToNext(p, missed); }, true,
         "PPOS MISS HOLDX", 1, 1, "", "", "DIRECT TO MISS", false},
    };

    for (const Row& row : rows) {
        CAPTURE(row.name);
        const FlightPlan start = startPlan(db, row.start);
        const PlanEditResult r = row.edit(start, db);
        CHECK(r.ok == row.ok);
        const FlightPlan& p = r.ok ? r.plan : start;
        CHECK(legIdents(p) == row.legs);
        CHECK(p.activeLeg == row.active);
        CHECK(p.approachStartLeg == row.approachStart);
        CHECK(p.sidId == row.sid);
        CHECK(p.starId == row.star);
        if (row.message[0]) CHECK(r.message == row.message);
        CHECK(r.destructive == row.destructive);
        if (r.ok && p.legs.size() > 1) {
            // A direct-to always starts at the present position with a DF leg.
            if (p.legs[0].to.ident == "PPOS") CHECK(p.legs[1].type == LegType::DF);
            // Missed-approach legs stay at the end.
            bool seenMissed = false;
            for (const Leg& l : p.legs) {
                if (seenMissed) CHECK(l.missedApproach);
                seenMissed = seenMissed || l.missedApproach;
            }
        }
    }
}

TEST_CASE("nav: plan edits keep the later constraints and the destination fields") {
    const PlanFakeDb db = makeDb();
    FlightPlan p = startPlan(db, Start::Enroute);
    AltConstraint c;
    c.kind = AltConstraintKind::At;
    c.altFt = 6000;
    REQUIRE(setAltConstraint(p, 3, c));  // CCC
    const PlanEditResult r = directTo(p, fix("SIDE", 44.3, -122.5), ctxAt(44.4, -122.6, 225));
    REQUIRE(r.ok);
    REQUIRE(r.plan.legs.size() == 5);
    CHECK(r.plan.legs[3].to.ident == "CCC");
    CHECK(r.plan.legs[3].alt.kind == AltConstraintKind::At);
    CHECK(r.plan.legs[3].alt.altFt == doctest::Approx(6000));
    CHECK(r.plan.destIdent == "KBBB");
    CHECK(r.plan.arrRunway == "05");
    // Another airport: the old destination and procedures go.
    const PlanEditResult other =
        directTo(startPlan(db, Start::WithStarApproach),
                 fix("KCCC", 44.0, -121.0, WaypointKind::Airport), ctxAt(43.7, -123.3, 225));
    REQUIRE(other.ok);
    CHECK(other.plan.destIdent == "KCCC");
    CHECK(other.plan.arrRunway.empty());
    CHECK(other.plan.approachId.empty());
    CHECK(other.plan.originIdent == "KAAA");
    // New destination with a runway: arrRunway follows.
    const PlanEditResult nd = changeDestination(startPlan(db, Start::Enroute), db, "KCCC", "09",
                                                true, ctxAt(44.4, -122.6, 225));
    REQUIRE(nd.ok);
    CHECK(nd.plan.arrRunway == "09");
    CHECK(nd.plan.legs.back().to.ident == "RW09");
    CHECK(nd.plan.activeLeg == 2);  // AAA -> BBB is still flown
}

TEST_CASE("nav: plan edit undo keeps one level for 10 s") {
    const PlanFakeDb db = makeDb();
    Navigator nav(db);
    nav.setPlan(startPlan(db, Start::WithStarApproach));
    const FlightPlan before = nav.plan();
    const PlanEditContext ctx = ctxAt(43.7, -123.3, 225);
    CHECK_FALSE(nav.undoAvailable(0.0));
    CHECK(nav.undo(0.0).empty());

    PlanEditResult edit =
        directTo(nav.plan(), fix("KCCC", 44.0, -121.0, WaypointKind::Airport), ctx);
    REQUIRE(nav.applyEdit(edit, 100.0));
    CHECK(legIdents(nav.plan()) == "PPOS KCCC");
    CHECK(nav.undoAvailable(109.9));
    CHECK(nav.undoLabel() == "ROUTE REPLACED");
    const std::string label = nav.undo(105.0);
    CHECK(label == "ROUTE REPLACED");
    CHECK(undoneMessage(label) == "ROUTE REPLACED UNDONE");
    CHECK(legIdents(nav.plan()) == legIdents(before));
    CHECK(nav.plan().activeLeg == before.activeLeg);
    CHECK(nav.plan().approachId == "I05");
    CHECK(nav.plan().starId == "STAR1");
    CHECK_FALSE(nav.undoAvailable(105.0));  // one level: it is gone once used

    // Outside the window: refused, the edit stays.
    REQUIRE(nav.applyEdit(
        directTo(nav.plan(), fix("KCCC", 44.0, -121.0, WaypointKind::Airport), ctx), 200.0));
    CHECK(nav.undo(210.5).empty());
    CHECK(legIdents(nav.plan()) == "PPOS KCCC");

    // A newer destructive edit replaces the stash: undo goes back one step only.
    nav.setPlan(startPlan(db, Start::Enroute));
    REQUIRE(nav.applyEdit(changeDestination(nav.plan(), db, "KCCC", "", true, ctx), 300.0));
    const std::string afterNewDest = legIdents(nav.plan());
    CHECK(afterNewDest == "RW28L AAA BBB CCC KCCC");
    REQUIRE(nav.applyEdit(
        directTo(nav.plan(), fix("KBBB", 43.0, -124.0, WaypointKind::Airport), ctx), 301.0));
    CHECK(nav.undo(302.0) == "ROUTE REPLACED");
    CHECK(legIdents(nav.plan()) == afterNewDest);
    CHECK(nav.undo(302.5).empty());

    // A non-destructive edit keeps the undo of the destructive one before it.
    nav.setPlan(startPlan(db, Start::Enroute));
    REQUIRE(nav.applyEdit(
        directTo(nav.plan(), fix("KCCC", 44.0, -121.0, WaypointKind::Airport), ctx), 500.0));
    REQUIRE(nav.applyEdit(directTo(nav.plan(), fix("SIDE", 44.3, -122.5), ctx), 501.0));
    CHECK(nav.undoLabel() == "ROUTE REPLACED");
    CHECK(nav.undo(503.0) == "ROUTE REPLACED");
    CHECK(legIdents(nav.plan()) == "RW28L AAA BBB CCC RW05");

    // A failed edit changes nothing and keeps no undo.
    nav.setPlan(startPlan(db, Start::Enroute));
    CHECK_FALSE(nav.applyEdit(deleteWaypoint(nav.plan(), 4, ctx), 400.0));
    CHECK_FALSE(nav.undoAvailable(400.0));
}

TEST_CASE("nav: direct to the destination flies to the airport, then the approach is re-entered") {
    const PlanFakeDb db = makeDb();
    Navigator nav(db);
    nav.setPlan(startPlan(db, Start::WithStarApproach));
    const PlanEditResult r = directTo(nav.plan(), fix("KBBB", 43.0, -124.0, WaypointKind::Airport),
                                      ctxAt(43.7, -123.3, 225));
    REQUIRE(nav.applyEdit(r, 0.0));
    // Fly over the airport and past it: the DF leg to the destination is not sequenced.
    for (double lon = -123.6; lon >= -124.3; lon -= 0.05) {
        const AircraftState a = test::makeAircraft(43.0 + (lon + 123.6) * 0.3, lon, 3000, 240);
        nav.update(a, 1.0);
        CHECK(nav.plan().activeLeg == 1);
    }
    // ACTIVATE APPROACH picks up the kept approach.
    REQUIRE(nav.reenterApproach());
    CHECK(nav.plan().legs[nav.plan().activeLeg].to.ident == "IAF");
    CHECK(nav.plan().approachId == "I05");
}

TEST_CASE("nav: the VNAV target after a direct-to is the next constrained leg") {
    const PlanFakeDb db = makeDb();
    Navigator nav(db);
    FlightPlan p = startPlan(db, Start::Enroute);
    AltConstraint c;
    c.kind = AltConstraintKind::At;
    c.altFt = 6000;
    REQUIRE(setAltConstraint(p, 3, c));  // CCC
    nav.setPlan(p);
    const AircraftState a = test::makeAircraft(44.4, -122.6, 9000, 225);
    nav.update(a, 0.1);
    REQUIRE(
        nav.applyEdit(directTo(nav.plan(), fix("SIDE", 44.3, -122.5), planEditContext(a)), 1.0));
    const NavState& s = nav.update(a, 0.1);
    CHECK(s.activeLegIndex == 1);
    CHECK(std::string(s.activeWptIdent) == "SIDE");
    CHECK(s.targetAltFt == doctest::Approx(6000.0f));
    // The fly-by arcs (N10) follow the new plan: a turn at SIDE onto BBB; AAA's is gone.
    CHECK(s.turnRadiusNm > 0.0f);
    const FlyByTurn atSide = planTurnAt(nav.plan(), 1, s.turnRadiusNm);
    REQUIRE(atSide.valid);
    CHECK(atSide.fix.latDeg == doctest::Approx(44.3));
    CHECK(atSide.fix.lonDeg == doctest::Approx(-122.5));
    for (size_t i = 0; i < nav.plan().legs.size(); ++i) CHECK(nav.plan().legs[i].to.ident != "AAA");
    const FlyByTurn atAaaBefore = planTurnAt(p, 1, s.turnRadiusNm);
    CHECK(atAaaBefore.valid);
    CHECK(atAaaBefore.fix.latDeg == doctest::Approx(44.5));
}

TEST_CASE("nav: direct to the destination matches its ident in any case") {
    const PlanFakeDb db = makeDb();
    Navigator nav(db);
    nav.setPlan(startPlan(db, Start::WithStarApproach));
    const PlanEditResult r = directTo(nav.plan(), fix("kbbb", 43.0, -124.0, WaypointKind::Airport),
                                      ctxAt(43.7, -123.3, 225));
    REQUIRE(r.ok);
    CHECK(r.plan.legs[1].to.ident == "KBBB");  // matched against destIdent by the navigator
    REQUIRE(nav.applyEdit(r, 0.0));
    const AircraftState over = test::makeAircraft(42.95, -124.05, 3000, 240);
    nav.update(over, 1.0);
    CHECK(nav.plan().activeLeg == 1);
}

TEST_CASE("nav: an engaged NAV follows every plan edit without dropping to ROL") {
    using fsim::test::SimFixture;
    SimFixture f;
    REQUIRE_MESSAGE(f.initFile("ap_lnav_c172.ini"), f.error);
    f.run(20.0);
    REQUIRE(f.sim.autopilotState().latActive == LatMode::Nav);
    Navigator& nav = f.sim.navigator();
    const NavDatabase& db = f.sim.navDb();
    auto ctx = [&] { return planEditContext(f.sim.aircraft()); };
    auto check = [&](const char* what, const PlanEditResult& r, const char* expectWpt) {
        CAPTURE(what);
        REQUIRE(r.ok);
        REQUIRE(nav.applyEdit(r, f.sim.aircraft().simTimeS));
        for (int i = 0; i < 2 * kSimHz; ++i) {
            f.sim.stepOnce();
            REQUIRE(f.sim.autopilotState().latActive == LatMode::Nav);
        }
        CHECK(std::string(f.sim.navState().activeWptIdent) == expectWpt);
        CHECK(f.sim.navState().activeLegIndex >= 1);
        CHECK(f.sim.autopilotState().apEngaged);
    };
    // Off the plan, then on; the active waypoint deleted; direct to next; a new destination.
    check("direct to an off-plan fix", directTo(nav.plan(), fix("OFF1", 40.05, -100.05), ctx()),
          "OFF1");
    check("direct to next", directToNext(nav.plan(), ctx()), "OFF1");
    const FlightPlan& p = nav.plan();
    REQUIRE(p.legs.size() > 3);
    const std::string next = p.legs[2].to.ident;
    check("delete the active waypoint", deleteWaypoint(p, 1, ctx()), next.substr(0, 7).c_str());
    check("another airport",
          directTo(nav.plan(), fix("KZZZ", 40.2, -100.3, WaypointKind::Airport), ctx()), "KZZZ");
    check("new destination, clear the route",
          changeDestination(nav.plan(), db, "XTST", "09", false, ctx()), "RW09");
    // Undo brings the previous plan back, still in NAV.
    REQUIRE(nav.undo(f.sim.aircraft().simTimeS) == "PLAN CLEARED");
    f.run(2.0);
    CHECK(f.sim.autopilotState().latActive == LatMode::Nav);
    CHECK(std::string(f.sim.navState().activeWptIdent) == "KZZZ");
}
