// N8: SIDs and STARs from the FAA CIFP (fixtures: tests/fixtures/nav/procedures, cycle 2610):
// parsing with the published fixes and constraints, runway transitions, leg assembly, insertion
// into a flight plan, the .fms round trip and a perfect-pilot flight from KPDX into the plan.
#include <doctest/doctest.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "core/Geo.h"
#include "core/Units.h"
#include "nav/FlightPlan.h"
#include "nav/NavDb.h"
#include "nav/Navigator.h"
#include "nav/Procedures.h"
#include "navigatorTestUtil.h"

using namespace fsim;
using namespace fsim::test;

namespace {

std::string fixtureNavDir() {
    std::string f = __FILE__;  // .../tests/nav/procedures_sidstar_test.cpp
    f = f.substr(0, f.find_last_of('/'));
    f = f.substr(0, f.find_last_of('/'));
    return f + "/fixtures/nav";
}

std::string repoFile(const std::string& rel) {
    std::string f = __FILE__;
    f = f.substr(0, f.find_last_of('/'));
    f = f.substr(0, f.find_last_of('/'));
    f = f.substr(0, f.find_last_of('/'));
    std::ifstream in(f + "/" + rel);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
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

std::shared_ptr<const ProcedureSet> kpdx() {
    auto set = loadProcedureSet(fixtureNavDir(), "KPDX");
    REQUIRE(set != nullptr);
    return set;
}

std::vector<std::string> idents(const std::vector<Leg>& legs) {
    std::vector<std::string> out;
    for (const Leg& l : legs) out.push_back(l.to.ident.empty() ? "-" : l.to.ident);
    return out;
}

std::vector<std::string> ids(const std::vector<const TerminalProcedure*>& v) {
    std::vector<std::string> out;
    for (const TerminalProcedure* p : v) out.push_back(p->id);
    return out;
}

using Strs = std::vector<std::string>;

}  // namespace

TEST_CASE("procedures: KPDX and KOTH SIDs and STARs load with the approaches unchanged") {
    const auto set = kpdx();
    CHECK(ids({&set->sids[0], &set->sids[1], &set->sids[2], &set->sids[3], &set->sids[4],
               &set->sids[5]}) == Strs{"CASCD4", "HRMNS6", "LAVAA7", "MINNE5", "PTLD2", "WHAMY5"});
    REQUIRE(set->stars.size() == 5);
    CHECK(set->stars[0].id == "HELNS6");
    CHECK(set->stars[4].id == "TMBRS4");
    CHECK(set->approaches.size() == 2);  // the fixture's ILS 10R / 28L, as before
    const auto koth = loadProcedureSet(fixtureNavDir(), "KOTH");
    REQUIRE(koth != nullptr);
    REQUIRE(koth->sids.size() == 1);
    CHECK(koth->sids[0].id == "OTH7");
    CHECK(koth->stars.empty());
    CHECK(koth->approaches.size() == 7);
    // Every SID / STAR leg is tagged.
    for (const TerminalProcedure& p : set->sids)
        for (const ProcRoute& r : p.runwayTransitions)
            for (const Leg& l : r.legs) CHECK(l.source == LegSource::Sid);
}

TEST_CASE("procedures: MINNE5 runway 28L has the published fixes, climb and transitions") {
    const auto set = kpdx();
    const TerminalProcedure* sid = findSid(*set, "minne5");
    REQUIRE(sid != nullptr);
    CHECK(sid->isSid);
    REQUIRE(sid->runwayTransitions.size() == 3);  // 10B, 28L, 28R
    const ProcRoute& r28l = sid->runwayTransitions[1];
    CHECK(r28l.name == "28L");
    CHECK(idents(r28l.legs) == Strs{"-", "PEGTY", "KELYY", "DUDRE", "MINNE"});
    CHECK(r28l.legs[0].type == LegType::VA);  // climb heading 299 to 540 ft
    CHECK(r28l.legs[0].courseTrueDeg == doctest::Approx(299.0f));
    CHECK(r28l.legs[0].alt.kind == AltConstraintKind::AtOrAbove);
    CHECK(r28l.legs[0].alt.altFt == doctest::Approx(540.0f));
    CHECK(r28l.legs[1].type == LegType::DF);
    REQUIRE(sid->enrouteTransitions.size() == 3);
    CHECK(sid->enrouteTransitions[1].name == "FAMUK");
}

TEST_CASE("procedures: HHOOD6 STAR keeps its speed and altitude windows") {
    const auto set = kpdx();
    const TerminalProcedure* star = findStar(*set, "HHOOD6");
    REQUIRE(star != nullptr);
    CHECK_FALSE(star->isSid);
    const Leg& hhood = star->runwayTransitions[0].legs[0];  // 10B
    CHECK(hhood.to.ident == "HHOOD");
    CHECK(hhood.alt.kind == AltConstraintKind::Between);
    CHECK(hhood.alt.altFt == doctest::Approx(9000.0f));
    CHECK(hhood.alt.alt2Ft == doctest::Approx(15000.0f));
    CHECK(hhood.speedLimitKt == doctest::Approx(250.0f));
}

TEST_CASE("procedures: runway transitions match their runways, 'B' means both parallels") {
    CHECK(runwayTransitionMatches("28L", "28L"));
    CHECK(runwayTransitionMatches("10B", "10L"));
    CHECK(runwayTransitionMatches("10B", "10R"));
    CHECK(runwayTransitionMatches("10B", "10C"));
    CHECK_FALSE(runwayTransitionMatches("10B", "28L"));
    CHECK_FALSE(runwayTransitionMatches("10B", "1"));
    CHECK(runwayTransitionMatches("ALL", "21"));
    CHECK_FALSE(runwayTransitionMatches("28L", "28R"));
    const auto set = kpdx();
    // MINNE5 has no runway 03/21 transition; HHOOD6 serves 10L through 10B.
    const Strs from03 = ids(sidsForRunway(*set, "03"));
    CHECK(std::find(from03.begin(), from03.end(), "MINNE5") == from03.end());
    const Strs to10l = ids(starsForRunway(*set, "10L"));
    CHECK(std::find(to10l.begin(), to10l.end(), "HHOOD6") != to10l.end());
}

TEST_CASE("procedures: SID and STAR legs assemble without repeated fixes") {
    const auto set = kpdx();
    std::string missing;
    const std::vector<Leg> sid =
        assembleSidLegs(*findSid(*set, "MINNE5"), "28L", "FAMUK", &missing);
    CHECK(missing.empty());
    CHECK(idents(sid) == Strs{"-", "PEGTY", "KELYY", "DUDRE", "MINNE", "FAMUK"});
    for (const Leg& l : sid) CHECK(l.source == LegSource::Sid);
    // STAR: enroute transition, then the runway transition; HHOOD appears once.
    const std::vector<Leg> star =
        assembleStarLegs(*findStar(*set, "HHOOD6"), "BAYYR", "28L", &missing);
    CHECK(idents(star) == Strs{"BAYYR", "HHOOD", "BEEVS", "SHAFR"});
    CHECK(star[1].alt.kind == AltConstraintKind::Between);
    // Unknown names are reported, the rest still comes back.
    const std::vector<Leg> partial =
        assembleSidLegs(*findSid(*set, "MINNE5"), "03", "NOPE", &missing);
    CHECK_FALSE(missing.empty());
    CHECK(partial.empty());  // MINNE5 has no common route
}

TEST_CASE("procedures: a SID and a STAR go into the KPDX-KOTH plan and come out again") {
    const CsvNavDatabase& db = fixtureDb();
    PlanBuildResult r = loadPlanFromFmsText(repoFile("assets/plans/KPDX-KOTH.fms"), db);
    REQUIRE(r.ok);
    FlightPlan plan = r.plan;
    plan.activeLeg = 1;
    const size_t base = plan.legs.size();
    const auto set = kpdx();
    REQUIRE(insertSid(plan, "MINNE5", "FAMUK",
                      assembleSidLegs(*findSid(*set, "MINNE5"), "28L", "FAMUK")));
    CHECK(plan.sidId == "MINNE5");
    CHECK(plan.sidTransition == "FAMUK");
    CHECK(plan.legs.size() == base + 6);
    CHECK(plan.legs[1].source == LegSource::Sid);
    CHECK(plan.legs[6].to.ident == "FAMUK");
    CHECK(plan.legs[7].to.ident == "UBG");  // the plan goes on
    CHECK(plan.activeLeg == 1);             // the SID's first leg
    // Replacing it removes the old legs first.
    REQUIRE(insertSid(plan, "MINNE5", "", assembleSidLegs(*findSid(*set, "MINNE5"), "28L", "")));
    CHECK(plan.legs.size() == base + 5);
    CHECK(plan.legs[5].to.ident == "MINNE");
    removeSid(plan);
    CHECK(plan.legs.size() == base);
    CHECK(plan.sidId.empty());
    // A STAR whose first fix is the last enroute waypoint replaces that waypoint.
    FlightPlan p2 = r.plan;
    std::vector<Leg> star;
    Leg a;
    a.type = LegType::IF;
    a.to = p2.legs[p2.legs.size() - 2].to;  // OTH, the last enroute fix
    a.alt.kind = AltConstraintKind::AtOrAbove;
    a.alt.altFt = 4000;
    Leg b;
    b.to = Waypoint{"XYZZY", WaypointKind::Fix, 43.40, -124.20};
    star = {a, b};
    REQUIRE(insertStar(p2, "TEST1", "", star));
    const size_t n = p2.legs.size();
    CHECK(n == r.plan.legs.size() + 1);
    CHECK(p2.legs[n - 3].to.ident == "OTH");
    CHECK(p2.legs[n - 3].source == LegSource::Star);
    CHECK(p2.legs[n - 3].type == LegType::TF);  // mid-route IF becomes TF
    CHECK(p2.legs[n - 2].to.ident == "XYZZY");
    CHECK(p2.legs[n - 1].to.ident == "RW05");  // destination (runway) leg still last
    removeStar(p2);
    CHECK(p2.legs.size() == r.plan.legs.size() - 1);  // the replaced OTH waypoint stays gone
}

TEST_CASE("procedures: .fms SID / STAR / APPTRANS lines insert and write back by name") {
    const CsvNavDatabase& db = fixtureDb();
    std::string text = repoFile("assets/plans/KPDX-KOTH.fms");
    const std::string anchor = "DEPRWY RW28L\n";
    text.insert(text.find(anchor) + anchor.size(), "SID MINNE5\nSIDTRANS FAMUK\n");
    text.insert(text.find("APP I05\n") + 8, "APPTRANS RARES\n");
    FmsFile file;
    PlanBuildResult r = loadPlanFromFmsText(text, db, &file);
    REQUIRE(r.ok);
    CHECK(file.sid == "MINNE5");
    const std::vector<std::string> notes = applyFmsProcedures(r.plan, file, fixtureNavDir());
    CHECK(notes.empty());
    CHECK(r.plan.sidId == "MINNE5");
    CHECK(r.plan.approachTransition == "RARES");
    CHECK(r.plan.legs[6].to.ident == "FAMUK");
    // Written back: names, no SID legs among the waypoints.
    const std::string out = writePlanFmsText(r.plan, db);
    CHECK(out.find("SID MINNE5") != std::string::npos);
    CHECK(out.find("SIDTRANS FAMUK") != std::string::npos);
    CHECK(out.find("APPTRANS RARES") != std::string::npos);
    CHECK(out.find("PEGTY") == std::string::npos);
    // A name the database does not know is a note, not a failure.
    file.star = "NOSUCH1";
    FlightPlan p = r.plan;
    const auto notes2 = applyFmsProcedures(p, file, fixtureNavDir());
    REQUIRE(notes2.size() == 1);
    CHECK(notes2[0].find("NOSUCH1") != std::string::npos);
}

TEST_CASE("procedures: a reader that predates N8 still loads the file's approaches") {
    // Simulates the 1.0 / 1.1 reader rule: unknown directives between approaches are skipped. The
    // SID / STAR blocks only use directives that reader does not know (SID, RWYTRANS, PLEG ...).
    const std::string text =
        "FORMAT flightsim-proc 1\nAIRPORT KXYZ\nAPPROACH I05 ILS 05 t\nFINAL\n"
        "LEG IF ABC 1 2\nEND\nSID TEST1 TEST1\nRWYTRANS 05\nPLEG VA - CRS 50.0 ALT ATORABOVE 500\n"
        "PLEG DF ABC 1 2\nPEND\n";
    for (const char* forbidden : {"\nLEG ", "\nEND\n", "\nTRANSITION", "\nFINAL", "\nMISSED"}) {
        const size_t sidAt = text.find("SID TEST1");
        CHECK(text.find(forbidden, sidAt) == std::string::npos);
    }
    ProcedureSet set;
    std::string err;
    REQUIRE_MESSAGE(parseProcedures(text, set, err), err);
    CHECK(set.approaches.size() == 1);
    REQUIRE(set.sids.size() == 1);
    CHECK(set.sids[0].runwayTransitions[0].legs.size() == 2);
    CHECK_FALSE(parseProcedures("FORMAT flightsim-proc 1\nPLEG TF ABC 1 2\n", set, err));
}

// N8 acceptance: the perfect pilot flies KPDX 28L MINNE5.FAMUK into the KPDX-KOTH plan (UBG
// dropped: FAMUK lies south of it) and on to the KOTH approach.
TEST_CASE("procedures: perfect pilot flies the MINNE5 departure into the KPDX-KOTH plan") {
    const CsvNavDatabase& db = fixtureDb();
    std::string text = repoFile("assets/plans/KPDX-KOTH.fms");
    text.insert(text.find("DEPRWY RW28L\n") + 13, "SID MINNE5\nSIDTRANS FAMUK\n");
    FmsFile file;
    PlanBuildResult r = loadPlanFromFmsText(text, db, &file);
    REQUIRE(r.ok);
    REQUIRE(applyFmsProcedures(r.plan, file, fixtureNavDir()).empty());
    for (int i = 0; i < static_cast<int>(r.plan.legs.size()); ++i)
        if (r.plan.legs[static_cast<size_t>(i)].to.ident == "UBG") REQUIRE(deleteLeg(r.plan, i));
    const Waypoint origin = r.plan.legs[0].to;

    Navigator nav(db);
    nav.setAircraftCategory(AircraftCategory::Piston);
    nav.setRealApproachProvider(makeCifpApproachProvider(fixtureNavDir()));
    nav.setPlan(r.plan);
    REQUIRE(nav.plan().approachStartLeg > 0);
    Follower pilot;
    // Just after liftoff from 28L: 600 ft, runway heading (the VA leg's 540 ft is behind us).
    pilot.start(origin.latDeg, origin.lonDeg, 600.0, 284.0, 120.0, 30.0);
    pilot.vsLimitFpm = 800;
    std::vector<std::string> seen;
    int lastLeg = -1;
    bool outOfOrder = false, reachedOth = false;
    NavState s = nav.update(pilot.ac, 0.25);
    for (double t = 0; t < 8000 && !reachedOth; t += 0.25) {
        pilot.step(s, 0.25);
        s = nav.update(pilot.ac, 0.25);
        if (s.activeLegIndex != lastLeg) {
            if (lastLeg >= 0 && s.activeLegIndex != lastLeg + 1) outOfOrder = true;
            seen.emplace_back(s.activeWptIdent);
            lastLeg = s.activeLegIndex;
        }
        if (std::string(s.activeWptIdent) != "OTH" && !seen.empty() && seen.back() != "OTH" &&
            std::find(seen.begin(), seen.end(), "OTH") != seen.end())
            reachedOth = true;
    }
    std::string seq;
    for (const std::string& id : seen) seq += id + " ";
    MESSAGE("MINNE5.FAMUK into KPDX-KOTH: " << seq);
    CHECK_FALSE(outOfOrder);
    const Strs expect{"PEGTY", "KELYY", "DUDRE", "MINNE", "FAMUK", "CVO", "EUG", "OTH"};
    size_t k = 0;
    for (const std::string& id : seen)
        if (k < expect.size() && id == expect[k]) ++k;
    CHECK(k == expect.size());  // every fix, in order
    CHECK(reachedOth);
}
