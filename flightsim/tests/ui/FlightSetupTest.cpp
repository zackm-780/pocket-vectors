#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>

#include "SimFixture.h"
#include "ui/FakeNavDb.h"
#include "ui/FlightSetup.h"

using namespace fsim;
using fsim::test::FakeNavDb;

namespace {

std::string tempDir(const char* leaf) {
    const std::filesystem::path p = std::filesystem::temp_directory_path() / "fsim_h5_tests" / leaf;
    std::filesystem::remove_all(p);
    std::filesystem::create_directories(p);
    return p.string();
}

void writeText(const std::string& path, const std::string& text) {
    std::ofstream(path, std::ios::binary) << text;
}

}  // namespace

TEST_CASE("ui: airport search puts exact idents first and hides airports without runways") {
    FakeNavDb db;
    auto hits = searchAirportsOrdered(db, "kbbb", 8, 45.5, -122.6);
    REQUIRE_FALSE(hits.empty());
    CHECK(hits[0].airport->ident == "KBBB");
    CHECK(hits[0].distNm > 5);

    // "OTX": the database lists the name match "Kotx Field" (OTHR) first; the FAA-code rule
    // promotes KOTX.
    hits = searchAirportsOrdered(db, "OTX", 8, 43.4, -124.2);
    REQUIRE(hits.size() >= 2);
    CHECK(hits[0].airport->ident == "KOTX");
    CHECK(hits[0].distNm < 1.0);
    CHECK(hits[1].airport->ident == "OTHR");

    // No duplicates, no airports without runways, and the cap is honoured.
    hits = searchAirportsOrdered(db, "NORW", 8, 0, 0);
    CHECK(hits.empty());
    hits = searchAirportsOrdered(db, "K", 2, 45, -122);
    CHECK(hits.size() <= 2);
    CHECK(searchAirportsOrdered(db, "", 8, 0, 0).empty());
}

TEST_CASE("ui: nearest airports are sorted by distance and have runways") {
    FakeNavDb db;
    const auto hits = nearestAirportHits(db, 45.5, -122.6, 10, 100.0f);
    REQUIRE(hits.size() == 2);  // KAAA and KBBB; NORW has no runways; the coast is > 100 NM
    CHECK(hits[0].airport->ident == "KAAA");
    CHECK(hits[1].airport->ident == "KBBB");
    CHECK(hits[0].distNm < hits[1].distNm);
    CHECK(nearestAirportHits(db, 45.5, -122.6, 1, 100.0f).size() == 1);
}

TEST_CASE("ui: runways sort by number and side, default is the longest hard one") {
    FakeNavDb db;
    const Airport& a = *db.findAirport("KAAA");
    const auto sorted = sortedRunways(a);
    REQUIRE(sorted.size() == 5);
    CHECK(sorted[0]->ident == "03");
    CHECK(sorted[1]->ident == "10L");
    CHECK(sorted[2]->ident == "10R");
    CHECK(sorted[3]->ident == "28L");
    CHECK(sorted[4]->ident == "28R");
    // 10L and 28R are both 11000 ft: the first in sorted order wins.
    CHECK(defaultRunway(a)->ident == "10L");
    CHECK(defaultRunway(*db.findAirport("KOTX"))->ident == "05");

    const std::string line = runwayDescription(*sorted[1], 15.0f);  // 100 true, 15 E -> 085 mag
    CHECK(line.find("10L") == 0);
    CHECK(line.find("085M") != std::string::npos);
    CHECK(line.find("11000X150") != std::string::npos);
    CHECK(line.find("ILS") != std::string::npos);
    CHECK(runwayDescription(*sorted[0], 0.0f).find("SOFT") != std::string::npos);
    CHECK(runwayHeadingMagDeg(*sorted[1], -10.0f) == doctest::Approx(110.0f));
}

TEST_CASE("ui: in-air altitude stepping") {
    CHECK(stepInAirAltitude(0, +1, 100.0f) == 3000);
    CHECK(stepInAirAltitude(3000, +1, 100.0f) == 3500);
    CHECK(stepInAirAltitude(3500, -1, 100.0f) == 3000);
    CHECK(stepInAirAltitude(1500, -1, 100.0f) == 0);  // below the floor = AUTO
    CHECK(stepInAirAltitude(0, -1, 100.0f) == 0);
    CHECK(stepInAirAltitude(45000, +1, 100.0f) == 45000);
    CHECK(stepInAirAltitude(0, +1, 6000.0f) == 7000);  // high field: at least 1000 ft above it
}

TEST_CASE("ui: aircraft and plan discovery read files at runtime") {
    const std::string dir = tempDir("discover");
    writeText(dir + "/zeta.ini",
              "[aircraft]\nid = zeta\ndisplay_name = Zeta Jet\ncategory = jet\n");
    writeText(dir + "/alpha.ini", "[aircraft]\nid = alpha\ndisplay_name = Alpha Piston\n");
    writeText(dir + "/broken.ini", "this is not an aircraft");
    writeText(dir + "/readme.txt", "ignored");
    const auto aircraft = discoverAircraft(dir);
    // The loader may reject files without a jsbsim model; whatever it accepts is sorted by name.
    for (size_t i = 1; i < aircraft.size(); ++i)
        CHECK(aircraft[i - 1].displayName <= aircraft[i].displayName);
    for (const AircraftEntry& e : aircraft) CHECK(e.id != "broken");

    const std::string d1 = tempDir("plans1"), d2 = tempDir("plans2");
    writeText(d1 + "/B.fms", "x");
    writeText(d1 + "/A.fms", "x");
    writeText(d2 + "/A.fms", "y");
    writeText(d2 + "/C.FMS", "y");
    writeText(d2 + "/notes.txt", "y");
    const auto plans = discoverPlanFiles({d1, d2, d1 + "/missing"});
    REQUIRE(plans.size() == 3);
    CHECK(plans[0].name == "A.fms");
    CHECK(plans[0].path.find("plans1") != std::string::npos);  // the earlier directory wins
    CHECK(plans[1].name == "B.fms");
    CHECK(plans[2].name == "C.FMS");
}

TEST_CASE("ui: aircraft discovery lists the shipped profiles") {
    const Paths paths = Paths::resolve({});
    const auto list = discoverAircraft(paths.asset("assets/aircraft"));
    REQUIRE(list.size() >= 2);
    bool c172 = false, b737 = false;
    for (const AircraftEntry& e : list) {
        c172 |= e.id == "c172p";
        b737 |= e.id == "737";
        CHECK_FALSE(e.displayName.empty());
    }
    CHECK(c172);
    CHECK(b737);
}

TEST_CASE("ui: approach choices list synthesized approaches with titles") {
    SyntheticNavDb db;
    const auto list = approachChoices(db, "XTST", "27", "", {});
    REQUIRE(list.size() == 3);
    CHECK(list[0].id == "I27");
    CHECK(list[0].title == "ILS RWY 27");
    CHECK(list[0].synthesized);
    CHECK(list[0].minimumsIsDa);
    CHECK(list[0].minimumsFt == doctest::Approx(2200.0f));  // threshold 2000 + 200
    CHECK(list[1].id == "R27");
    CHECK(list[1].title == "RNAV (GPS) RWY 27");
    CHECK_FALSE(list[1].minimumsIsDa);
    CHECK(list[2].id == "VIS27");
    CHECK(list[2].title == "VISUAL RWY 27");
    CHECK(approachChoices(db, "XTST", "99", "", {}).empty());

    // A real procedure's title comes from ApproachInfo.
    Approach real;
    real.kind = ApproachKind::Ils;
    real.runwayIdent = "05";
    ApproachInfo info;
    info.title = "ILS Z RWY 05";
    CHECK(approachTitle(real, &info) == "ILS Z RWY 05");
    CHECK(approachTitle(real, nullptr) == "ILS RWY 05");
}

TEST_CASE("ui: selections build the scenario fields") {
    FlightSetup s;
    s.aircraftId = "737";
    s.airportIdent = "KPDX";
    s.runwayIdent = "28R";

    s.startType = StartType::Runway;
    Scenario sc = s.buildScenario("");
    CHECK(sc.aircraftId == "737");
    CHECK(sc.airportIdent == "KPDX");
    CHECK(sc.runwayIdent == "28R");
    CHECK(sc.startKind == "runway");
    CHECK(sc.planPath.empty());
    CHECK(sc.approachId.empty());

    s.startType = StartType::Final2Nm;
    sc = s.buildScenario("");
    CHECK(sc.startKind == "final");
    CHECK(sc.distanceNm == doctest::Approx(2));
    s.startType = StartType::Final6Nm;
    CHECK(s.buildScenario("").distanceNm == doctest::Approx(6));
    s.startType = StartType::Final10Nm;
    CHECK(s.buildScenario("").distanceNm == doctest::Approx(10));

    s.startType = StartType::InAir;
    s.inAirAltFt = 6500;
    sc = s.buildScenario("");
    CHECK(sc.startKind == "cruise");
    CHECK(sc.altitudeFt == doctest::Approx(6500));
    s.inAirAltFt = 0;
    CHECK(s.buildScenario("").altitudeFt == doctest::Approx(0));  // profile cruise altitude

    // A practice approach without a plan travels in the scenario.
    s.setApproach("I28R");
    CHECK(s.buildScenario("").approachId == "I28R");
    CHECK(s.destAirportIdent() == "KPDX");
    CHECK(s.destRunwayIdent() == "28R");
    // ... and is dropped when the runway changes.
    s.setRunway("10L");
    CHECK(s.approachId.empty());
}

TEST_CASE("ui: quick plan and plan files feed the scenario") {
    FakeNavDb db;
    FlightSetup s;
    REQUIRE(s.setAirport(db, "KAAA"));
    CHECK(s.runwayIdent == "10L");
    CHECK_FALSE(s.setAirport(db, "NORW"));  // no runways
    CHECK_FALSE(s.setAirport(db, "ZZZZ"));
    CHECK(s.airportIdent == "KAAA");

    std::string err;
    REQUIRE(s.makeQuickPlan(db, "KOTX", "05", 5500.0f, err));
    INFO(err);
    CHECK(s.planKind == PlanKind::Quick);
    CHECK(s.planName == "QUICK KAAA-KOTX");
    CHECK(s.plan.destIdent == "KOTX");
    CHECK(s.destAirportIdent() == "KOTX");
    CHECK(s.destRunwayIdent() == "05");
    CHECK(s.planDistNm() > 100);
    CHECK(s.routeText().find("KOTX") != std::string::npos);

    s.setApproach("R05");
    const std::string dir = tempDir("quickplan");
    const std::string planPath = dir + "/menu_plan.fms";
    REQUIRE(s.writePlanFile(planPath, err));
    const Scenario sc = s.buildScenario(planPath);
    CHECK(sc.planPath == planPath);
    CHECK(sc.approachId == "R05");
    CHECK(sc.airportIdent == "KAAA");
    // The written file reads back to the same plan and approach name.
    std::ifstream in(planPath);
    std::string text((std::istreambuf_iterator<char>(in)), {});
    CHECK(text.find("ADES KOTX") != std::string::npos);
    CHECK(text.find("APP R05") != std::string::npos);
    CHECK(text.find("DESRWY RW05") != std::string::npos);

    // Changing the destination runway rebuilds the plan and drops the approach.
    REQUIRE(s.setPlanDestRunway(db, "23"));
    CHECK(s.plan.arrRunway == "23");
    CHECK(s.approachId.empty());

    // Changing the start airport throws away a quick plan.
    REQUIRE(s.setAirport(db, "KBBB"));
    CHECK(s.planKind == PlanKind::None);
    CHECK_FALSE(s.hasPlan());
    CHECK(s.destAirportIdent() == "KBBB");
    CHECK_FALSE(s.writePlanFile(planPath, err));
}

TEST_CASE("ui: loading an .fms moves the start to the plan origin") {
    FakeNavDb db;
    const std::string dir = tempDir("fmsload");
    writeText(dir + "/T.fms", "I\n1100 Version\nADEP KBBB\nDEPRWY RW34\nADES KOTX\nDESRWY RW05\n"
                              "APP I05\nNUMENR 2\n"
                              "1 KBBB ADEP 60.000000 45.300000 -122.900000\n"
                              "1 KOTX ADES 17.000000 43.400000 -124.200000\n");
    FlightSetup s;
    std::string err;
    REQUIRE(s.loadPlanFile(db, dir + "/T.fms", "T.fms", err));
    INFO(err);
    CHECK(s.planKind == PlanKind::File);
    CHECK(s.airportIdent == "KBBB");
    CHECK(s.runwayIdent == "34");
    CHECK(s.approachId == "I05");
    CHECK(s.destAirportIdent() == "KOTX");
    CHECK(s.plan.arrRunway == "05");

    s.setApproach("");  // "no approach" really removes the APP from the written file
    REQUIRE(s.writePlanFile(dir + "/out.fms", err));
    std::ifstream in(dir + "/out.fms");
    std::string text((std::istreambuf_iterator<char>(in)), {});
    CHECK(text.find("APP ") == std::string::npos);

    CHECK_FALSE(s.loadPlanFile(db, dir + "/missing.fms", "missing.fms", err));
    CHECK_FALSE(err.empty());
    CHECK(s.planKind == PlanKind::File);  // a failed load leaves the previous plan alone
    s.clearPlan();
    CHECK(s.planKind == PlanKind::None);
}

TEST_CASE("sim: a menu-built scenario initialises the simulation") {
    // XTST synthetic data only. Every start type and a practice approach.
    fsim::test::SimFixture f;
    FlightSetup setup;
    setup.aircraftId = "c172p";
    setup.airportIdent = "XTST";
    setup.runwayIdent = "27";

    for (StartType t : {StartType::Runway, StartType::Final2Nm, StartType::Final6Nm,
                        StartType::Final10Nm, StartType::InAir}) {
        setup.startType = t;
        setup.inAirAltFt = t == StartType::InAir ? 5500 : 0;
        const Scenario sc = setup.buildScenario("");
        INFO(startTypeLabel(t));
        REQUIRE(f.sim.init(sc, "", f.error));
        f.run(1.0);
        const AircraftState& st = f.sim.aircraft();
        CHECK(st.valid);
        CHECK_FALSE(st.crashed);
        if (t == StartType::InAir) CHECK(st.altMslFt == doctest::Approx(5500).epsilon(0.05));
        if (t == StartType::Final6Nm) CHECK(st.altMslFt > 2000 + 1500);
    }

    setup.startType = StartType::Final6Nm;
    setup.setApproach("I27");
    REQUIRE(f.sim.init(setup.buildScenario(""), "", f.error));
    CHECK(f.sim.navigator().hasPlan());
    CHECK(f.sim.navigator().plan().approachId == "I27");

    // The shipped sample plan through the plan path.
    FlightSetup withPlan;
    std::string err;
    REQUIRE(withPlan.loadPlanFile(f.nav, f.paths.asset("assets/plans/XTST-pattern.fms"),
                                  "XTST-pattern.fms", err));
    INFO(err);
    const std::string planPath = tempDir("simplan") + "/menu_plan.fms";
    REQUIRE(withPlan.writePlanFile(planPath, err));
    REQUIRE(f.sim.init(withPlan.buildScenario(planPath), "", f.error));
    CHECK(f.sim.navigator().hasPlan());
}
