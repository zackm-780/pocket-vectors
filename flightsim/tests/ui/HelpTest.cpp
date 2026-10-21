#include <doctest/doctest.h>

#include <fstream>

#include "input/InputTestUtil.h"
#include "ui/FakeNavDb.h"
#include "ui/HelpPages.h"
#include "ui/MenuScreens.h"
#include "ui/MenuSystem.h"

using namespace fsim;
using namespace fsim::testutil;

namespace {

bool hasLine(const HelpLayerPage& page, const std::string& input, const std::string& action) {
    for (const HelpLine& line : page.lines)
        if (line.input == input && line.action == action) return true;
    return false;
}

const HelpLayerPage* findPage(const std::vector<HelpLayerPage>& pages, const std::string& id) {
    for (const HelpLayerPage& page : pages)
        if (page.id == id) return &page;
    return nullptr;
}

std::string tempFile(const std::string& name) {
    return std::string("/tmp/fsim_h6_") + name;
}

void writeText(const std::string& path, const std::string& body) {
    std::ofstream out(path);
    out << body;
}

}  // namespace

TEST_CASE("ui: help controls list every pad binding of the default config") {
    const ControlsConfig cfg = defaultConfig();
    const std::vector<HelpLayerPage> pages = helpControlPages(cfg);
    REQUIRE(pages.size() == 6);
    std::string all;
    for (const HelpLayerPage& page : pages) all += page.plain();
    const char* ids[] = {"base", "l1", "r1", "l2", "map", "menus"};
    for (const char* id : ids) CHECK(findPage(pages, id) != nullptr);

    for (int layer = 0; layer < kPadLayerCount; ++layer) {
        for (const PadButtonBinding& b : cfg.padButtons[layer]) {
            INFO(actionName(b.action));
            CHECK(all.find(actionName(b.action)) != std::string::npos);
        }
        for (const PadAxisBinding& a : cfg.padAxes[layer]) {
            INFO(axisName(a.axis));
            CHECK(all.find(axisName(a.axis)) != std::string::npos);
        }
    }
    for (const PadChord& c : cfg.chords) CHECK(all.find(actionName(c.action)) != std::string::npos);

    const HelpLayerPage* base = findPage(pages, "base");
    REQUIRE(base != nullptr);
    CHECK(hasLine(*base, "A", "GearToggle"));
    CHECK(hasLine(*base, "B", "Brakes"));
    CHECK(hasLine(*base, "X", "SpeedbrakeCycle"));
    CHECK(hasLine(*base, "Y", "HudDeclutter"));
    CHECK(hasLine(*base, "SELECT", "MapToggle"));
    CHECK(hasLine(*base, "SELECT+START", "Quit"));
    CHECK(hasLine(*base, "R2", "Brakes"));
    CHECK(hasLine(*base, "L3", "OverlayPreset"));

    const HelpLayerPage* map = findPage(pages, "map");
    REQUIRE(map != nullptr);
    CHECK(hasLine(*map, "SELECT", "MapToggle"));
    CHECK(map->plain().find("zoom in") != std::string::npos);
    const HelpLayerPage* menus = findPage(pages, "menus");
    REQUIRE(menus != nullptr);
    CHECK(menus->plain().find("MenuAccept") != std::string::npos);
}

TEST_CASE("ui: a remapped controls.ini changes the help controls page") {
    const std::string user = tempFile("remap.ini");
    writeText(user, "[pad.base]\nGearToggle = x\n");
    ControlsConfig cfg;
    REQUIRE(cfg.loadFiles("assets/controls_default.ini", user));
    const std::vector<HelpLayerPage> pages = helpControlPages(cfg);
    const HelpLayerPage* base = findPage(pages, "base");
    REQUIRE(base != nullptr);
    CHECK(hasLine(*base, "X", "GearToggle"));
    CHECK_FALSE(hasLine(*base, "A", "GearToggle"));
    CHECK(hasLine(*base, "A", "MenuAccept"));
    const HelpLayerPage* map = findPage(pages, "map");
    REQUIRE(map != nullptr);
    CHECK(hasLine(*map, "X", "GearToggle") == false);
    CHECK(map->plain().find("zoom in") != std::string::npos);
}

TEST_CASE("ui: help button test names guide and back as Select") {
    const ControlsConfig cfg = defaultConfig();
    const HelpPadReading guide = describeHelpPad(cfg, false, SDL_CONTROLLER_BUTTON_GUIDE);
    const HelpPadReading back = describeHelpPad(cfg, false, SDL_CONTROLLER_BUTTON_BACK);
    CHECK(guide.label == "SELECT");
    CHECK(back.label == "SELECT");
    CHECK(guide.sdlName == "guide");
    CHECK(back.sdlName == "back");
    CHECK(guide.baseAction.find("MapToggle") != std::string::npos);
    CHECK(guide.baseAction.find("Quit") != std::string::npos);
    CHECK(back.baseAction.find("MapToggle") != std::string::npos);

    const HelpPadReading gear = describeHelpPad(cfg, false, SDL_CONTROLLER_BUTTON_A);
    CHECK(gear.label == "A");
    CHECK(gear.sdlName == "a");
    CHECK(gear.baseAction.find("GearToggle") != std::string::npos);

    const HelpPadReading roll = describeHelpPad(cfg, true, SDL_CONTROLLER_AXIS_LEFTX);
    CHECK(roll.label == "L STICK X");
    CHECK(roll.baseAction.find("Roll") != std::string::npos);

    const HelpPadReading brakes = describeHelpPad(cfg, true, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
    CHECK(brakes.label == "R2");
    CHECK(brakes.baseAction.find("Brakes") != std::string::npos);
}

TEST_CASE("ui: swap A/B writes device_labels and keeps every other line") {
    const std::string user = tempFile("swap.ini");
    writeText(
        user,
        "; keep this comment\n[axes]\nleft_deadzone = 0.2\n\n[device_labels]\nselect = guide\n");
    const ControlsConfig before = defaultConfig();
    std::string error;
    REQUIRE(swapDeviceLabels(user, before, true, error));
    std::ifstream in(user);
    std::string body((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    CHECK(body.find("; keep this comment") != std::string::npos);
    CHECK(body.find("left_deadzone = 0.2") != std::string::npos);
    CHECK(body.find("select = guide") != std::string::npos);
    CHECK(body.find("a = b") != std::string::npos);
    CHECK(body.find("b = a") != std::string::npos);

    ControlsConfig after;
    REQUIRE(after.loadFiles("assets/controls_default.ini", user));
    CHECK(after.padInputForSdlButton(SDL_CONTROLLER_BUTTON_B) == PadInput::A);
    CHECK(after.padInputForSdlButton(SDL_CONTROLLER_BUTTON_A) == PadInput::B);
    CHECK(after.padInputForSdlButton(SDL_CONTROLLER_BUTTON_GUIDE) == PadInput::Select);
    const auto pages = helpControlPages(after);
    const HelpLayerPage* base = findPage(pages, "base");
    REQUIRE(base != nullptr);
    CHECK(hasLine(*base, "A", "GearToggle"));
    CHECK(describeHelpPad(after, false, SDL_CONTROLLER_BUTTON_B).label == "A");

    REQUIRE(swapDeviceLabels(user, after, false, error));
    ControlsConfig xy;
    REQUIRE(xy.loadFiles("assets/controls_default.ini", user));
    CHECK(xy.padInputForSdlButton(SDL_CONTROLLER_BUTTON_Y) == PadInput::X);
    CHECK(xy.padInputForSdlButton(SDL_CONTROLLER_BUTTON_X) == PadInput::Y);
    CHECK(xy.padInputForSdlButton(SDL_CONTROLLER_BUTTON_B) == PadInput::A);
}

TEST_CASE("ui: how-to pages load from assets/help and about shows credits") {
    const std::vector<HelpTextPage> pages = loadHelpTexts("assets/help");
    REQUIRE(pages.size() == 8);
    CHECK(pages[0].id == "first-flight");
    CHECK(pages[3].id == "ils");
    CHECK(pages[5].id == "fms-plans");
    CHECK(pages[7].id == "troubleshooting");
    for (const HelpTextPage& page : pages) {
        INFO(page.id);
        CHECK_FALSE(page.title.empty());
        CHECK_FALSE(page.lines.empty());
    }
    std::string plans;
    for (const std::string& line : pages[5].lines) plans += line;
    CHECK(plans.find("flightsim/data/plans") != std::string::npos);

    const std::string attr = tempFile("ATTRIBUTION.txt");
    writeText(attr, "Copernicus WorldDEM-30 notice for the pack.\n");
    const std::vector<std::string> about = helpAboutLines(attr);
    std::string joined;
    for (const std::string& line : about) joined += line + "\n";
    CHECK(joined.find("ZACK MARTIN") != std::string::npos);
    CHECK(joined.find("github.com/zackm-780/pocket-vectors") != std::string::npos);
    CHECK(joined.find("buymeacoffee.com/zackm") != std::string::npos);
    CHECK(joined.find("JSBSIM") != std::string::npos);
    CHECK(joined.find("LGPL-2.1") != std::string::npos);
    CHECK(joined.find("Copernicus") != std::string::npos);
    CHECK(joined.find("VERSION ") != std::string::npos);
    const std::vector<std::string> bare = helpAboutLines("");
    std::string bareJoined;
    for (const std::string& line : bare) bareJoined += line;
    CHECK(bareJoined.find("NO DATA PACK") != std::string::npos);
}
