#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>

#include "SimFixture.h"
#include "hud/Renderer2D.h"
#include "ui/FakeNavDb.h"
#include "ui/MenuScreens.h"
#include "ui/MenuSystem.h"

using namespace fsim;

namespace {

std::string tempDir(const char* leaf) {
    const std::filesystem::path p = std::filesystem::temp_directory_path() / "fsim_h5_tests" / leaf;
    std::filesystem::remove_all(p);
    std::filesystem::create_directories(p);
    return p.string();
}

// Presses a button `times` frames in a row; returns the last frame that produced a result.
MenuResult tap(MenuSystem& m, bool MenuInput::*button, int times = 1) {
    MenuResult last;
    for (int i = 0; i < times; ++i) {
        MenuInput in;
        in.*button = true;
        MenuResult r = m.update(in);
        if (r.action != MenuAction::None || r.settingsChanged || !r.message.empty()) last = r;
    }
    return last;
}

// A menu over the synthetic world (XTST only, or `navOverride`) with a simulation as the flight.
struct MenuFixture {
    test::SimFixture f;
    Settings settings;
    std::string dir;
    MenuEnv env;
    std::unique_ptr<MenuSystem> menu;

    explicit MenuFixture(const char* leaf, const NavDatabase* navOverride = nullptr) {
        dir = tempDir(leaf);
        env.nav = navOverride ? navOverride : &f.nav;
        env.settings = &settings;
        env.settingsPath = dir + "/settings.ini";
        env.aircraftDir = f.paths.asset("assets/aircraft");
        env.planDirs = {dir + "/plans", f.paths.asset("assets/plans")};
        env.lastFlightPath = dir + "/last_flight.ini";
        env.menuPlanPath = dir + "/menu_plan.fms";
        env.controlsDefaultsPath = f.paths.asset("assets/controls_default.ini");
        env.controlsUserPath = dir + "/controls.ini";
        env.helpDir = f.paths.asset("assets/help");
        env.attributionPath = dir + "/ATTRIBUTION.txt";
        menu = std::make_unique<MenuSystem>(env);
    }
    // Starts a cruise flight over XTST with the sample plan and its ILS 09 approach loaded.
    void startFlying() {
        REQUIRE(f.initText("[scenario]\naircraft = c172p\nairport = XTST\nrunway = 27\n"
                           "start = cruise\naltitude_ft = 5000\n"
                           "plan = assets/plans/XTST-pattern.fms\napproach = I09\n"));
        f.run(0.5);
        menu->setFlight(
            {&f.sim.navigator(), &f.sim.aircraft(), &f.sim.profile(), &f.sim.scenario()});
    }
};

}  // namespace

TEST_CASE("ui: main menu without a last flight starts on New flight") {
    MenuFixture mf("main_nolast");
    MenuSystem& m = *mf.menu;
    m.openMain();
    CHECK(m.active());
    CHECK(m.top() == ScreenId::Main);
    CHECK(m.fullScreen());
    tap(m, &MenuInput::accept);  // "Resume last flight" is disabled: the cursor is on New flight
    CHECK(m.top() == ScreenId::Setup);
    tap(m, &MenuInput::back);
    CHECK(m.top() == ScreenId::Main);
    tap(m, &MenuInput::down);  // Settings
    tap(m, &MenuInput::accept);
    CHECK(m.top() == ScreenId::Settings);
    tap(m, &MenuInput::back);
    CHECK(m.top() == ScreenId::Main);
}

TEST_CASE("ui: resume last flight returns the saved scenario") {
    MenuFixture mf("main_last");
    Scenario last;
    last.aircraftId = "c172p";
    last.airportIdent = "XTST";
    last.runwayIdent = "09";
    last.startKind = "final";
    last.distanceNm = 6;
    REQUIRE(writeLastFlight(mf.env.lastFlightPath, last));
    MenuSystem m(mf.env);  // reads the last flight on construction
    CHECK(m.lastFlightAvailable());
    m.openMain();
    const MenuResult r = tap(m, &MenuInput::accept);  // first row = resume
    CHECK(r.action == MenuAction::StartFlight);
    CHECK(r.scenario.runwayIdent == "09");
    CHECK(r.scenario.startKind == "final");
    CHECK(r.scenario.distanceNm == doctest::Approx(6));
    // The New flight screen starts from the last flight's selections.
    CHECK(m.setup().runwayIdent == "09");
    CHECK(m.setup().startType == StartType::Final6Nm);
    CHECK(lastFlightSummary(last) == "C172P  XTST 09  6 NM FINAL");
}

TEST_CASE("ui: quit asks for confirmation") {
    MenuFixture mf("quit");
    MenuSystem& m = *mf.menu;
    m.openMain();
    tap(m, &MenuInput::down, 3);  // Quit (row 4; the cursor starts on row 1)
    CHECK(tap(m, &MenuInput::accept).action == MenuAction::None);
    CHECK(m.top() == ScreenId::Confirm);
    CHECK(tap(m, &MenuInput::accept).action == MenuAction::None);  // the default answer is NO
    CHECK(m.top() == ScreenId::Main);
    tap(m, &MenuInput::accept);
    tap(m, &MenuInput::back);  // B = no
    CHECK(m.top() == ScreenId::Main);
    tap(m, &MenuInput::accept);
    tap(m, &MenuInput::left);  // YES
    CHECK(tap(m, &MenuInput::accept).action == MenuAction::Quit);
}

TEST_CASE("ui: new flight setup by typing an airport and starting") {
    test::FakeNavDb fake;
    MenuFixture mf("setup_flow", &fake);
    MenuSystem& m = *mf.menu;
    m.openMain();
    tap(m, &MenuInput::accept);  // New flight
    REQUIRE(m.top() == ScreenId::Setup);

    // Aircraft: the list comes from assets/aircraft; pick another entry.
    REQUIRE(m.aircraft().size() >= 2);
    tap(m, &MenuInput::accept);
    CHECK(m.top() == ScreenId::Aircraft);
    tap(m, &MenuInput::down);
    tap(m, &MenuInput::accept);
    CHECK(m.top() == ScreenId::Setup);
    const std::string picked = m.setup().aircraftId;
    CHECK(m.aircraftEntry(picked) != nullptr);

    // Airport: type "BBB" on the keyboard (A is under the cursor, B is one key to the right).
    tap(m, &MenuInput::down);
    tap(m, &MenuInput::accept);
    REQUIRE(m.top() == ScreenId::AirportSearch);
    tap(m, &MenuInput::right);
    tap(m, &MenuInput::accept, 3);
    tap(m, &MenuInput::start);   // done: focus moves to the results
    tap(m, &MenuInput::accept);  // first result: the exact ident KBBB
    CHECK(m.setup().airportIdent == "KBBB");
    CHECK(m.top() == ScreenId::Runway);
    tap(m, &MenuInput::down);  // 34
    tap(m, &MenuInput::accept);
    CHECK(m.top() == ScreenId::Setup);
    CHECK(m.setup().runwayIdent == "34");

    // Start type: Left/Right on the row cycles, A opens the list.
    tap(m, &MenuInput::down, 2);  // START row
    tap(m, &MenuInput::right, 2);
    CHECK(m.setup().startType == StartType::Final6Nm);
    tap(m, &MenuInput::accept);
    CHECK(m.top() == ScreenId::StartType);
    tap(m, &MenuInput::down, 2);   // IN THE AIR
    tap(m, &MenuInput::right, 3);  // AUTO -> 3,000 -> 3,500 -> 4,000
    tap(m, &MenuInput::accept);
    CHECK(m.top() == ScreenId::Setup);
    CHECK(m.setup().startType == StartType::InAir);
    CHECK(m.setup().inAirAltFt == 4000);

    // Start button.
    const MenuResult r = tap(m, &MenuInput::start);
    REQUIRE(r.action == MenuAction::StartFlight);
    CHECK(r.scenario.aircraftId == picked);
    CHECK(r.scenario.airportIdent == "KBBB");
    CHECK(r.scenario.runwayIdent == "34");
    CHECK(r.scenario.startKind == "cruise");
    CHECK(r.scenario.altitudeFt == doctest::Approx(4000));
    CHECK(m.top() == ScreenId::Setup);  // the app closes the menu once the flight has started
    m.close();
    CHECK_FALSE(m.active());
}

TEST_CASE("ui: airport search keyboard: delete, and B on an empty text leaves") {
    test::FakeNavDb fake;
    MenuFixture mf("search_flow", &fake);
    MenuSystem& m = *mf.menu;
    m.push(makeSearchScreen(SearchMode::StartAirport, m, "KAA"));
    m.update(MenuInput{});  // applies the push
    REQUIRE(m.top() == ScreenId::AirportSearch);
    tap(m, &MenuInput::back, 3);  // delete A, A, K
    CHECK(m.top() == ScreenId::AirportSearch);
    tap(m, &MenuInput::back);  // empty text: leaves the screen
    CHECK_FALSE(m.active());
}

TEST_CASE("ui: plan file and approach selection feed the scenario") {
    MenuFixture mf("plan_flow");
    MenuSystem& m = *mf.menu;
    std::string err;
    REQUIRE(m.openCanned("plan:XTST-pattern", err));
    INFO(err);
    CHECK(m.top() == ScreenId::Plan);
    CHECK(m.setup().planKind == PlanKind::File);
    CHECK(m.setup().approachId == "I09");
    CHECK(m.setup().airportIdent == "XTST");
    CHECK(m.setup().runwayIdent == "09");

    // Approach list: "no approach", I09, R09, VIS09. It opens on the current one.
    tap(m, &MenuInput::down, 3);  // APPROACH row
    tap(m, &MenuInput::accept);
    REQUIRE(m.top() == ScreenId::Approach);
    tap(m, &MenuInput::down);  // R09
    tap(m, &MenuInput::accept);
    CHECK(m.top() == ScreenId::Plan);
    CHECK(m.setup().approachId == "R09");

    // Back on the hub, the START FLIGHT row writes the plan to menu_plan.fms and the scenario
    // references it.
    tap(m, &MenuInput::back);
    REQUIRE(m.top() == ScreenId::Setup);
    tap(m, &MenuInput::down, 6);
    const MenuResult r = tap(m, &MenuInput::accept);
    REQUIRE(r.action == MenuAction::StartFlight);
    CHECK(r.scenario.planPath == mf.env.menuPlanPath);
    CHECK(r.scenario.approachId == "R09");
    REQUIRE(std::filesystem::exists(mf.env.menuPlanPath));
    std::ifstream in(mf.env.menuPlanPath);
    const std::string text((std::istreambuf_iterator<char>(in)), {});
    CHECK(text.find("APP R09") != std::string::npos);

    // ... and the simulation takes it.
    REQUIRE(mf.f.sim.init(r.scenario, "", mf.f.error));
    CHECK(mf.f.sim.navigator().plan().approachId == "R09");
    CHECK(mf.f.sim.navigator().plan().approachStartLeg > 0);
}

TEST_CASE("ui: pause menu resumes, restarts and leaves") {
    MenuFixture mf("pause");
    MenuSystem& m = *mf.menu;
    mf.startFlying();
    m.openPause();
    CHECK(m.top() == ScreenId::Pause);
    CHECK_FALSE(m.fullScreen());
    CHECK(tap(m, &MenuInput::accept).action == MenuAction::ResumeFlight);  // RESUME
    m.openPause();
    CHECK(tap(m, &MenuInput::start).action == MenuAction::ResumeFlight);  // Start resumes
    m.openPause();
    CHECK(tap(m, &MenuInput::back).action == MenuAction::ResumeFlight);  // B resumes

    // Restart needs a yes.
    m.openPause();
    tap(m, &MenuInput::down, 2);
    CHECK(tap(m, &MenuInput::accept).action == MenuAction::None);
    CHECK(m.top() == ScreenId::Confirm);
    CHECK(tap(m, &MenuInput::accept).action == MenuAction::None);  // default NO
    CHECK(m.top() == ScreenId::Pause);
    tap(m, &MenuInput::accept);
    tap(m, &MenuInput::left);
    CHECK(tap(m, &MenuInput::accept).action == MenuAction::RestartFlight);

    // New flight is seeded from the running flight.
    m.openPause();
    tap(m, &MenuInput::down, 3);
    tap(m, &MenuInput::accept);
    CHECK(m.top() == ScreenId::Setup);
    CHECK(m.setup().airportIdent == "XTST");
    CHECK(m.setup().startType == StartType::InAir);
    CHECK(m.setup().inAirAltFt == 5000);

    // Quit to menu: confirm, then the main menu is the bottom of the stack.
    m.openPause();
    tap(m, &MenuInput::down, 5);
    tap(m, &MenuInput::accept);
    tap(m, &MenuInput::left);
    CHECK(tap(m, &MenuInput::accept).action == MenuAction::None);
    CHECK(m.top() == ScreenId::Main);
    CHECK(m.fullScreen());

    // Quit the game.
    m.openPause();
    tap(m, &MenuInput::down, 6);
    tap(m, &MenuInput::accept);
    tap(m, &MenuInput::left);
    CHECK(tap(m, &MenuInput::accept).action == MenuAction::Quit);
}

TEST_CASE("ui: in-flight plan screen: direct-to a leg, re-enter the approach, direct-to an ident") {
    MenuFixture mf("flightplan");
    MenuSystem& m = *mf.menu;
    mf.startFlying();
    Navigator& nav = mf.f.sim.navigator();
    REQUIRE(nav.hasPlan());
    REQUIRE(nav.activeApproach() != nullptr);
    const int active = nav.plan().activeLeg;
    REQUIRE(active >= 1);
    REQUIRE(active + 1 < static_cast<int>(nav.plan().legs.size()));
    const std::string fix = nav.plan().legs[static_cast<size_t>(active) + 1].to.ident;

    m.openPause();
    tap(m, &MenuInput::down);
    tap(m, &MenuInput::accept);  // FLIGHT PLAN
    REQUIRE(m.top() == ScreenId::FlightPlan);
    // Rows: 3 actions, then the legs. The list opens on the active leg: go to the next one.
    tap(m, &MenuInput::down);
    MenuResult r = tap(m, &MenuInput::accept);
    CHECK(r.action == MenuAction::ResumeFlight);
    CHECK(r.message.find("DIRECT TO") == 0);
    CHECK(nav.plan().legs[static_cast<size_t>(nav.plan().activeLeg)].to.ident == fix);

    // Re-enter the approach: the third action row.
    m.openPause();
    tap(m, &MenuInput::down);
    tap(m, &MenuInput::accept);
    tap(m, &MenuInput::up);  // the active leg is now leg 1 (row 4); row 3 (leg 0) is disabled
    r = tap(m, &MenuInput::accept);
    CHECK(r.action == MenuAction::ResumeFlight);
    CHECK(r.message == "DIRECT TO APPROACH");

    // Direct-to an ident through the search screen; "XTST" is the only airport in this database.
    m.push(makeSearchScreen(SearchMode::DirectTo, m, "XTST"));
    m.update(MenuInput{});
    REQUIRE(m.top() == ScreenId::AirportSearch);
    tap(m, &MenuInput::start);  // done: focus on the results
    r = tap(m, &MenuInput::accept);
    CHECK(r.action == MenuAction::ResumeFlight);
    CHECK(r.message == "DIRECT TO XTST");
    CHECK(nav.plan().legs[static_cast<size_t>(nav.plan().activeLeg)].to.ident == "XTST");
}

TEST_CASE("ui: in-flight approach change and removal") {
    MenuFixture mf("approach_flight");
    MenuSystem& m = *mf.menu;
    mf.startFlying();
    Navigator& nav = mf.f.sim.navigator();
    REQUIRE(nav.plan().approachId == "I09");

    m.push(makeApproachScreen(ApproachMode::Flight, m));
    m.update(MenuInput{});
    REQUIRE(m.top() == ScreenId::Approach);
    tap(m, &MenuInput::down);  // I09 -> R09
    MenuResult r = tap(m, &MenuInput::accept);
    CHECK(r.action == MenuAction::ResumeFlight);
    CHECK(r.message == "APPROACH R09");
    CHECK(nav.plan().approachId == "R09");

    m.push(makeApproachScreen(ApproachMode::Flight, m));
    m.update(MenuInput{});
    tap(m, &MenuInput::up, 2);  // R09 (row 2) -> "remove approach" (row 0)
    r = tap(m, &MenuInput::accept);
    CHECK(r.message == "APPROACH REMOVED");
    CHECK(nav.plan().approachId.empty());
    CHECK(nav.plan().approachStartLeg < 0);
}

TEST_CASE("ui: settings are edited and saved when leaving the screen") {
    MenuFixture mf("settings");
    MenuSystem& m = *mf.menu;
    std::string err;
    REQUIRE(m.openCanned("settings", err));
    CHECK(m.top() == ScreenId::Settings);
    CHECK(mf.settings.hudColor == HudColor::Green);
    tap(m, &MenuInput::right);  // HUD COLOR -> white
    CHECK(mf.settings.hudColor == HudColor::White);
    tap(m, &MenuInput::down, 3);  // SHOW FPS
    tap(m, &MenuInput::accept);   // A toggles
    CHECK(mf.settings.showFps);
    tap(m, &MenuInput::down);  // FIELD OF VIEW: numbers change with Left/Right only
    tap(m, &MenuInput::accept);
    CHECK(mf.settings.fovVerticalDeg == doctest::Approx(55.0f));
    tap(m, &MenuInput::right, 100);  // clamps at 110
    CHECK(mf.settings.fovVerticalDeg == doctest::Approx(110.0f));
    CHECK_FALSE(std::filesystem::exists(mf.env.settingsPath));  // nothing written yet

    const MenuResult r = tap(m, &MenuInput::back);
    CHECK(r.settingsChanged);
    CHECK(m.top() == ScreenId::Main);
    REQUIRE(std::filesystem::exists(mf.env.settingsPath));
    Settings reloaded;
    reloaded.load("", mf.env.settingsPath);
    CHECK(reloaded.hudColor == HudColor::White);
    CHECK(reloaded.showFps);
    CHECK(reloaded.fovVerticalDeg == doctest::Approx(110.0f));

    // Leaving without a change reports nothing.
    REQUIRE(m.openCanned("settings", err));
    CHECK_FALSE(tap(m, &MenuInput::back).settingsChanged);
}

TEST_CASE("ui: an error dialog is dismissed with A") {
    MenuFixture mf("error");
    MenuSystem& m = *mf.menu;
    m.openMain();
    m.showError("COULD NOT START");
    CHECK(m.top() == ScreenId::Confirm);
    tap(m, &MenuInput::accept);
    CHECK(m.top() == ScreenId::Main);
}

TEST_CASE("ui: canned screens open and draw inside the canvas; bad specs are rejected") {
    MenuFixture mf("canned");
    MenuSystem& m = *mf.menu;
    mf.startFlying();
    std::string err;
    static const gfx::GlApi noGl{};
    for (const char* spec : {"main",
                             "setup",
                             "aircraft",
                             "airport",
                             "airport:XT",
                             "runway",
                             "runway:XTST",
                             "start",
                             "plan:XTST-pattern",
                             "approach:XTST-pattern",
                             "settings",
                             "pause",
                             "flightplan",
                             "confirm",
                             "help",
                             "help:controls",
                             "help:controls:l1",
                             "help:controls:r1",
                             "help:controls:l2",
                             "help:controls:map",
                             "help:controls:menus",
                             "help:buttons",
                             "help:howto",
                             "help:howto:ils",
                             "help:about"}) {
        INFO(spec);
        REQUIRE(m.openCanned(spec, err));
        CHECK(m.active());
        for (int width : {640, 853}) {  // 4:3 and 16:9 canvases, 480 high
            Renderer2D r(noGl, gfx::GlFlavor::Es2);
            r.begin(width, 480);
            m.draw(r);
            r.end();
            CHECK(r.batchVertexCount() > 100);
            const float vw = r.virtualWidth();
            int outside = 0;
            for (int i = 0; i < r.batchVertexCount(); ++i) {
                const Renderer2D::Vertex& v = r.batchData()[i];
                if (v.x < -2 || v.x > vw + 2 || v.y < -2 || v.y > 482) ++outside;
            }
            CHECK(outside == 0);
        }
    }
    CHECK_FALSE(m.openCanned("nonsense", err));
    CHECK(err.find("nonsense") != std::string::npos);
    CHECK_FALSE(m.openCanned("plan:NoSuchPlan", err));
    CHECK_FALSE(m.openCanned("runway:ZZZZ", err));
    MenuFixture noFlight("canned_noflight");
    CHECK_FALSE(noFlight.menu->openCanned("pause", err));
}

TEST_CASE("ui: help opens from the main menu and from a canned page") {
    MenuFixture mf("help_nav");
    MenuSystem& m = *mf.menu;
    m.openMain();
    tap(m, &MenuInput::down, 2);  // Help (cursor starts on New flight)
    tap(m, &MenuInput::accept);
    CHECK(m.top() == ScreenId::Help);
    tap(m, &MenuInput::accept);  // Controls
    tap(m, &MenuInput::back);    // back to the help index
    CHECK(m.top() == ScreenId::Help);
    tap(m, &MenuInput::back);
    CHECK(m.top() == ScreenId::Main);

    std::string err;
    REQUIRE(m.openCanned("help:controls:map", err));
    CHECK(m.top() == ScreenId::Help);
    tap(m, &MenuInput::back);
    CHECK(m.top() == ScreenId::Help);
    tap(m, &MenuInput::back);
    CHECK(m.top() == ScreenId::Main);
}
