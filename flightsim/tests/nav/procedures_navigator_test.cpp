// N4 + N5 together: real CIFP approaches (tests/fixtures/nav/procedures) offered by
// ApproachBuilder, loaded by the Navigator, and flown by the kinematic perfect pilot.
#include <doctest/doctest.h>

#include <string>
#include <vector>

#include "core/Geo.h"
#include "core/Units.h"
#include "nav/ApproachBuilder.h"
#include "nav/FlightPlan.h"
#include "nav/NavDb.h"
#include "nav/Navigator.h"
#include "nav/Procedures.h"
#include "navigatorTestUtil.h"

using namespace fsim;
using namespace fsim::test;

namespace {

std::string fixtureNavDir() {
    std::string f = __FILE__;  // .../tests/nav/procedures_navigator_test.cpp
    f = f.substr(0, f.find_last_of('/'));
    f = f.substr(0, f.find_last_of('/'));
    return f + "/fixtures/nav";
}

const CsvNavDatabase& fixtureDb() {
    static CsvNavDatabase db;
    static bool loaded = false;
    if (!loaded) {
        std::string err;
        REQUIRE_MESSAGE(db.load(fixtureNavDir(), err), err);
        loaded = true;
    }
    return db;
}

RealApproachProvider provider() {
    return makeCifpApproachProvider(fixtureNavDir());
}

std::vector<std::string> ids(const std::vector<Approach>& v) {
    std::vector<std::string> out;
    for (const auto& a : v) out.push_back(a.id);
    return out;
}

struct Flown {
    bool reachedMap = false;
    bool legsOutOfOrder = false;
    bool anyMissed = false;
    double timeS = 0;
    std::vector<std::string> idents;  // active waypoint each time the active leg changes
    NavState atEnd;
};

// dt 0.25 s; stops when the MAP leg is active and the threshold is < 0.1 NM away.
Flown fly(Navigator& nav, Follower& pilot, double maxS) {
    Flown r;
    const double dt = 0.25;
    int lastLeg = -1;
    NavState s = nav.update(pilot.ac, dt);
    for (double t = 0; t < maxS; t += dt) {
        pilot.step(s, dt);
        s = nav.update(pilot.ac, dt);
        if (s.phase == NavPhase::Missed) r.anyMissed = true;
        if (s.activeLegIndex != lastLeg) {
            if (lastLeg >= 0 && (s.activeLegIndex < lastLeg || s.activeLegIndex > lastLeg + 1))
                r.legsOutOfOrder = true;
            r.idents.emplace_back(s.activeWptIdent);
            lastLeg = s.activeLegIndex;
        }
        if (s.distToThresholdNm > 0 && s.distToThresholdNm < 0.1f &&
            s.activeLegIndex == nav.mapLegIndex()) {
            r.reachedMap = true;
            r.timeS = t;
            r.atEnd = s;
            break;
        }
    }
    return r;
}

}  // namespace

TEST_CASE("nav: real approaches are listed before the synthesized ones") {
    const CsvNavDatabase& db = fixtureDb();
    auto list = availableApproaches(db, "KOTH", "05", provider());
    CHECK(ids(list) == std::vector<std::string>{"H05-Z", "I05-Y", "I05-Z", "L05-Y", "L05-Z",
                                                "R05-Y", "I05", "R05", "VIS05"});
    for (size_t i = 0; i < list.size(); ++i) CHECK(list[i].synthesized == (i >= 6));
    CHECK(list[2].transitions.size() == 4);  // vectors + LUPCI, OTH, RARES

    // KPDX 10R: the real I10R replaces the synthesized ILS of the same id.
    auto pdx = availableApproaches(db, "KPDX", "10R", provider());
    CHECK(ids(pdx) == std::vector<std::string>{"I10R", "R10R", "VIS10R"});
    CHECK_FALSE(pdx[0].synthesized);

    // Runways and airports without procedures keep the synthesized list.
    CHECK(ids(availableApproaches(db, "KOTH", "13", provider())).front() != "");
    CHECK(availableApproaches(db, "KOTH", "23", provider()).front().synthesized);
    CHECK(availableApproaches(db, "KEUG", "16R", provider()).front().synthesized);

    auto real = findApproach(db, "KOTH", "05", "i05-z", provider());
    REQUIRE(real.has_value());
    CHECK_FALSE(real->synthesized);
    CHECK(real->kind == ApproachKind::Ils);
    CHECK_FALSE(findApproach(db, "KOTH", "05", "I05-Z").has_value());  // no provider, no real
}

TEST_CASE("nav: Navigator inserts a real approach with the IAF nearest to the route") {
    const CsvNavDatabase& db = fixtureDb();
    QuickPlanRequest req;
    req.originIdent = "KPDX";
    req.originRunway = "10R";
    req.destIdent = "KOTH";
    req.destRunway = "05";
    req.cruiseAltFt = 8000;
    QuickPlanResult q = buildQuickPlan(req, db);
    REQUIRE_MESSAGE(q.ok, q.error);

    Navigator nav(db);
    nav.setRealApproachProvider(provider());
    nav.setPlan(q.plan);
    REQUIRE(nav.loadApproachById("I05-Z"));
    const FlightPlan& p = nav.plan();
    CHECK(p.approachId == "I05-Z");
    CHECK(p.approachKind == ApproachKind::Ils);
    CHECK(p.arrRunway == "05");
    REQUIRE(p.approachStartLeg > 0);
    // Arriving from the north-east (course to MITUE is > 90 degrees off the final): the IAF
    // transition whose first fix is nearest the route start is used, RARES.
    CHECK(p.legs[static_cast<size_t>(p.approachStartLeg)].to.ident == "RARES");
    CHECK(p.legs.back().missedApproach);
    CHECK(p.legs.back().type == LegType::HM);
    const Approach* active = nav.activeApproach();
    REQUIRE(active != nullptr);
    CHECK_FALSE(active->synthesized);
    CHECK(active->minimumsFt == doctest::Approx(216.0f));  // N5 default: threshold 16 + 200
    CHECK(active->minimumsIsDa);

    // Reloading the plan (approach already inserted) finds the real approach again.
    Navigator again(db);
    again.setRealApproachProvider(provider());
    again.setPlan(p);
    REQUIRE(again.activeApproach() != nullptr);
    CHECK(again.activeApproach()->id == "I05-Z");
}

TEST_CASE("nav: perfect pilot flies real KOTH approaches from the IAF to the threshold") {
    const CsvNavDatabase& db = fixtureDb();
    struct Case {
        const char* id;
        const char* transition;
        double startAltFt;
    };
    const Case cases[] = {{"I05-Z", "RARES", 5000},  // 16 DME arc, localizer capture, GS
                          {"R05-Y", "GAMLL", 4000},  // RNAV, LUPCI -> GAMLL -> MITUE
                          {"H05-Z", "DEROY", 4000},  // RNP with RF legs
                          {"I05-Y", "OTH3", 4000}};
    for (const Case& c : cases) {
        CAPTURE(c.id);
        auto approach = findApproach(db, "KOTH", "05", c.id, provider());
        REQUIRE(approach.has_value());
        int tIdx = -1;
        for (size_t i = 0; i < approach->transitions.size(); ++i)
            if (approach->transitions[i].name == c.transition ||
                (approach->transitions[i].name == "LUPCI" && std::string(c.id) == "R05-Y"))
                tIdx = static_cast<int>(i);
        REQUIRE(tIdx > 0);
        const Leg& iaf = approach->transitions[static_cast<size_t>(tIdx)].legs.front();

        Navigator nav(db);
        nav.setAircraftCategory(AircraftCategory::Piston);
        nav.setRealApproachProvider(provider());
        // start 3 NM before the IAF, heading for it
        const Waypoint& next = approach->transitions[static_cast<size_t>(tIdx)].legs[1].to;
        const double brg =
            radToDeg(initialBearingTrueRad(next.latDeg, next.lonDeg, iaf.to.latDeg, iaf.to.lonDeg));
        const LatLon start = pointAtNm(iaf.to.latDeg, iaf.to.lonDeg, brg, 3.0);
        REQUIRE(nav.startApproachOnly(*approach, start, tIdx));
        Follower pilot;
        pilot.start(start.latDeg, start.lonDeg, c.startAltFt, wrap360(brg + 180.0), 110.0, 16.0);
        nav.activateNearestLeg(pilot.ac);
        Flown r = fly(nav, pilot, 3600.0);
        std::string seq;
        for (const auto& i : r.idents) seq += i + " ";
        MESSAGE(std::string(c.id) << ": reached MAP " << r.reachedMap << " after " << r.timeS
                     << " s, active fixes: " << seq);
        CHECK(r.reachedMap);
        CHECK_FALSE(r.legsOutOfOrder);
        CHECK_FALSE(r.anyMissed);
        if (r.reachedMap) CHECK(r.idents.back() == "RW05");
    }
}
