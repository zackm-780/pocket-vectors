#include <doctest/doctest.h>

#include "core/Paths.h"
#include "input/InputTestUtil.h"

using namespace fsim;
using namespace fsim::testutil;

namespace {

bool hasPadBinding(const ControlsConfig& c, PadLayer layer, PadInput in, Action a) {
    for (const PadButtonBinding& b : c.padButtons[static_cast<int>(layer)])
        if (b.input == in && b.action == a) return true;
    return false;
}

bool hasKeyBinding(const ControlsConfig& c, int layer, int key, unsigned mods, Action a) {
    for (const KeyBinding& b : c.keys[layer])
        if (b.keycode == key && b.mods == mods && b.action == a) return true;
    return false;
}

}  // namespace

TEST_CASE("input: name tables round trip") {
    for (int i = 0; i < kActionCount; ++i) {
        Action out;
        REQUIRE(actionFromName(actionName(static_cast<Action>(i)), out));
        CHECK(static_cast<int>(out) == i);
    }
    for (int i = 0; i < kAxisCount; ++i) {
        Axis out;
        REQUIRE(axisFromName(axisName(static_cast<Axis>(i)), out));
        CHECK(static_cast<int>(out) == i);
    }
    Action a;
    CHECK(actionFromName("gEaRtOgGlE", a));  // case-insensitive
    CHECK_FALSE(actionFromName("Nope", a));
}

TEST_CASE("input: default ini parses without warnings and matches CONTROLS.md") {
    const ControlsConfig c = defaultConfig();
    CHECK(c.warningCount == 0);
    CHECK(c.leftDeadzone == doctest::Approx(0.08f));
    CHECK(c.leftExpo == doctest::Approx(0.35f));
    CHECK(c.rightDeadzone == doctest::Approx(0.12f));

    // Base layer
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::DpadUp, Action::TrimNoseDown));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::DpadDown, Action::TrimNoseUp));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::DpadLeft, Action::FlapsUp));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::DpadRight, Action::FlapsDown));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::A, Action::GearToggle));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::B, Action::Brakes));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::R2, Action::Brakes));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::X, Action::SpeedbrakeCycle));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::Y, Action::HudDeclutter));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::Select, Action::MapToggle));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::Start, Action::Pause));
    // L1 layer
    CHECK(hasPadBinding(c, PadLayer::L1, PadInput::A, Action::ApToggle));
    CHECK(hasPadBinding(c, PadLayer::L1, PadInput::B, Action::AtToggle));
    CHECK(hasPadBinding(c, PadLayer::L1, PadInput::X, Action::HdgMode));
    CHECK(hasPadBinding(c, PadLayer::L1, PadInput::Y, Action::NavMode));
    CHECK(hasPadBinding(c, PadLayer::L1, PadInput::DpadLeft, Action::HdgBugDec));
    CHECK(hasPadBinding(c, PadLayer::L1, PadInput::DpadRight, Action::HdgBugInc));
    CHECK(hasPadBinding(c, PadLayer::L1, PadInput::DpadUp, Action::AltBugInc));
    CHECK(hasPadBinding(c, PadLayer::L1, PadInput::DpadDown, Action::AltBugDec));
    CHECK(hasPadBinding(c, PadLayer::L1, PadInput::R2, Action::AltMode));
    CHECK(hasPadBinding(c, PadLayer::L1, PadInput::Select, Action::DirectTo));
    // R1 layer
    CHECK(hasPadBinding(c, PadLayer::R1, PadInput::DpadLeft, Action::RudderTrimLeft));
    CHECK(hasPadBinding(c, PadLayer::R1, PadInput::DpadRight, Action::RudderTrimRight));
    CHECK(hasPadBinding(c, PadLayer::R1, PadInput::A, Action::ReverseToggle));
    CHECK(hasPadBinding(c, PadLayer::R1, PadInput::B, Action::FdToggle));
    CHECK(hasPadBinding(c, PadLayer::R1, PadInput::X, Action::SyncBugs));
    CHECK(hasPadBinding(c, PadLayer::R1, PadInput::Y, Action::NavSourceCycle));
    CHECK(hasPadBinding(c, PadLayer::R1, PadInput::DpadUp, Action::BaroInc));
    CHECK(hasPadBinding(c, PadLayer::R1, PadInput::DpadDown, Action::BaroDec));
    CHECK(hasPadBinding(c, PadLayer::R1, PadInput::Select, Action::BaroStd));
    // L2 layer
    CHECK(hasPadBinding(c, PadLayer::L2, PadInput::DpadLeft, Action::LookLeft));
    CHECK(hasPadBinding(c, PadLayer::L2, PadInput::DpadRight, Action::LookRight));
    CHECK(hasPadBinding(c, PadLayer::L2, PadInput::DpadDown, Action::LookBack));
    CHECK(hasPadBinding(c, PadLayer::L2, PadInput::DpadUp, Action::LookDown));
    CHECK(c.chords.size() == 1);

    // Keyboard (CONTROLS.md section 4)
    CHECK(hasKeyBinding(c, 0, SDLK_g, 0, Action::GearToggle));
    CHECK(hasKeyBinding(c, 0, SDLK_b, 0, Action::Brakes));
    CHECK(hasKeyBinding(c, 0, SDLK_b, kModShift, Action::ParkingBrake));
    CHECK(hasKeyBinding(c, 0, SDLK_SLASH, 0, Action::SpeedbrakeCycle));
    CHECK(hasKeyBinding(c, 0, SDLK_LEFTBRACKET, 0, Action::FlapsUp));
    CHECK(hasKeyBinding(c, 0, SDLK_RIGHTBRACKET, 0, Action::FlapsDown));
    CHECK(hasKeyBinding(c, 0, SDLK_HOME, 0, Action::TrimNoseDown));
    CHECK(hasKeyBinding(c, 0, SDLK_END, 0, Action::TrimNoseUp));
    CHECK(hasKeyBinding(c, 0, SDLK_r, 0, Action::ReverseToggle));
    CHECK(hasKeyBinding(c, 0, SDLK_TAB, 0, Action::HudDeclutter));
    CHECK(hasKeyBinding(c, 0, SDLK_m, 0, Action::MapToggle));
    CHECK(hasKeyBinding(c, 0, SDLK_ESCAPE, 0, Action::Pause));
    CHECK(hasKeyBinding(c, 0, SDLK_p, 0, Action::Pause));
    CHECK(hasKeyBinding(c, 0, SDLK_F12, 0, Action::Screenshot));
    CHECK(hasKeyBinding(c, 0, SDLK_F1, 0, Action::ApToggle));
    CHECK(hasKeyBinding(c, 0, SDLK_F2, 0, Action::AtToggle));
    CHECK(hasKeyBinding(c, 0, SDLK_F3, 0, Action::HdgMode));
    CHECK(hasKeyBinding(c, 0, SDLK_F4, 0, Action::NavMode));
    CHECK(hasKeyBinding(c, 0, SDLK_F5, 0, Action::FdToggle));
    CHECK(hasKeyBinding(c, 0, SDLK_MINUS, 0, Action::HdgBugDec));
    CHECK(hasKeyBinding(c, 0, SDLK_EQUALS, 0, Action::HdgBugInc));
    CHECK(hasKeyBinding(c, 0, SDLK_9, 0, Action::AltBugDec));
    CHECK(hasKeyBinding(c, 0, SDLK_0, 0, Action::AltBugInc));
    CHECK(hasKeyBinding(c, 0, SDLK_q, kModCtrl, Action::Quit));
    CHECK(c.keyLookModifier == SDLK_v);

    // Repeat set: the bug buttons
    CHECK(c.repeats[idx(Action::HdgBugInc)]);
    CHECK(c.repeats[idx(Action::AltBugDec)]);
    CHECK_FALSE(c.repeats[idx(Action::GearToggle)]);
    CHECK(c.doubleTapSource == Action::Brakes);
    CHECK(c.doubleTapFires == Action::ParkingBrake);
}

TEST_CASE("input: device label table maps printed labels to SDL buttons in one place") {
    const ControlsConfig c = defaultConfig();
    CHECK(c.warningCount == 0);
    // RG40XXH stock OS: printed labels equal SDL names.
    CHECK(c.sdlButtonForLabel[static_cast<int>(PadInput::A)] == SDL_CONTROLLER_BUTTON_A);
    CHECK(c.sdlButtonForLabel[static_cast<int>(PadInput::B)] == SDL_CONTROLLER_BUTTON_B);
    CHECK(c.sdlButtonForLabel[static_cast<int>(PadInput::X)] == SDL_CONTROLLER_BUTTON_X);
    CHECK(c.sdlButtonForLabel[static_cast<int>(PadInput::Y)] == SDL_CONTROLLER_BUTTON_Y);
    CHECK(c.sdlButtonForLabel[static_cast<int>(PadInput::L1)] ==
          SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
    const int sel = static_cast<int>(PadInput::Select);
    CHECK(c.sdlButtonForLabel[sel] == SDL_CONTROLLER_BUTTON_BACK);
    CHECK(c.sdlButtonCountForLabel[sel] == 2);
    CHECK(c.sdlButtonsForLabel[sel][0] == SDL_CONTROLLER_BUTTON_BACK);
    CHECK(c.sdlButtonsForLabel[sel][1] == SDL_CONTROLLER_BUTTON_GUIDE);
    CHECK(c.sdlButtonForLabel[static_cast<int>(PadInput::L2)] == -1);  // trigger, not a button
    CHECK(c.padInputForSdlButton(SDL_CONTROLLER_BUTTON_A) == PadInput::A);
    CHECK(c.padInputForSdlButton(SDL_CONTROLLER_BUTTON_B) == PadInput::B);
    CHECK(c.padInputForSdlButton(SDL_CONTROLLER_BUTTON_BACK) == PadInput::Select);
    CHECK(c.padInputForSdlButton(SDL_CONTROLLER_BUTTON_GUIDE) == PadInput::Select);

    // Another device can still swap the four face buttons. Bindings stay on printed labels.
    Ini flip;
    flip.loadText("[device_labels]\na = b\nb = a\nx = y\ny = x\n");
    Ini base;
    base.loadFile("assets/controls_default.ini");
    ControlsConfig f;
    f.load(base, &flip);
    CHECK(f.padInputForSdlButton(SDL_CONTROLLER_BUTTON_B) == PadInput::A);
    CHECK(f.padInputForSdlButton(SDL_CONTROLLER_BUTTON_A) == PadInput::B);
    CHECK(hasPadBinding(f, PadLayer::Base, PadInput::A, Action::GearToggle));
    // Select is not in the override, so both stock-OS buttons still count.
    CHECK(f.padInputForSdlButton(SDL_CONTROLLER_BUTTON_GUIDE) == PadInput::Select);
}

TEST_CASE("input: a label may list several SDL buttons") {
    const ControlsConfig c = configFromText("[device_labels]\nselect = back, guide\na = a, b\n"
                                            "l2 = lefttrigger\n");
    CHECK(c.warningCount == 0);
    CHECK(c.sdlButtonCountForLabel[static_cast<int>(PadInput::Select)] == 2);
    CHECK(c.padInputForSdlButton(SDL_CONTROLLER_BUTTON_GUIDE) == PadInput::Select);
    CHECK(c.padInputForSdlButton(SDL_CONTROLLER_BUTTON_BACK) == PadInput::Select);
    CHECK(c.sdlButtonForLabel[static_cast<int>(PadInput::A)] == SDL_CONTROLLER_BUTTON_A);
    CHECK(c.sdlButtonsForLabel[static_cast<int>(PadInput::A)][1] == SDL_CONTROLLER_BUTTON_B);
    CHECK(c.padInputForSdlButton(SDL_CONTROLLER_BUTTON_B) == PadInput::A);
    CHECK(c.sdlButtonForLabel[static_cast<int>(PadInput::L2)] == -1);

    const ControlsConfig bad = configFromText("[device_labels]\nselect = back, notabutton\n");
    CHECK(bad.warningCount == 1);
    CHECK(bad.padInputForSdlButton(SDL_CONTROLLER_BUTTON_BACK) == PadInput::Select);
    CHECK(bad.sdlButtonCountForLabel[static_cast<int>(PadInput::Select)] == 1);
}

TEST_CASE("input: no in-flight layer binds R3") {
    const ControlsConfig c = defaultConfig();
    for (int layer = 0; layer < kPadLayerCount; ++layer) {
        for (const PadButtonBinding& b : c.padButtons[layer]) {
            CHECK(b.input != PadInput::R3);
        }
    }
    for (const PadChord& chord : c.chords) {
        CHECK(chord.a != PadInput::R3);
        CHECK(chord.b != PadInput::R3);
    }
}

TEST_CASE("input: the label table decides which SDL button is gear") {
    InputSystem in;
    in.setConfig(defaultConfig());
    // SDL button A is the printed A: gear.
    SDL_Event e{};
    e.type = SDL_CONTROLLERBUTTONDOWN;
    e.cbutton.button = SDL_CONTROLLER_BUTTON_A;
    in.handleEvent(e);
    CHECK(in.update(kFrameS).pressed[idx(Action::GearToggle)]);
    // SDL button B is the printed B: brakes.
    e.cbutton.button = SDL_CONTROLLER_BUTTON_B;
    in.handleEvent(e);
    CHECK(in.update(kFrameS).down[idx(Action::Brakes)]);
}

TEST_CASE("input: user ini replaces the bindings of the actions it names") {
    const char* defaults =
        "[pad.base]\nGearToggle = a\nFlapsUp = dpad_left\nFlapsDown = dpad_right\n"
        "[key.base]\nGearToggle = g\n[repeat]\nactions = FlapsUp\n";
    const char* user = "[pad.base]\nGearToggle = y, x\n[key.base]\nGearToggle = ctrl+g\n";
    const ControlsConfig c = configFromText(defaults, user);
    CHECK(c.warningCount == 0);
    CHECK_FALSE(hasPadBinding(c, PadLayer::Base, PadInput::A, Action::GearToggle));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::Y, Action::GearToggle));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::X, Action::GearToggle));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::DpadLeft, Action::FlapsUp));  // untouched
    CHECK_FALSE(hasKeyBinding(c, 0, SDLK_g, 0, Action::GearToggle));
    CHECK(hasKeyBinding(c, 0, SDLK_g, kModCtrl, Action::GearToggle));
    CHECK(c.repeats[idx(Action::FlapsUp)]);
}

TEST_CASE("input: user ini remap takes effect in the input system") {
    InputSystem in;
    in.setConfig(configFromText("[device_labels]\na = a\nb = b\nx = x\ny = y\nl1 = leftshoulder\n"
                                "[pad.base]\nGearToggle = a\nSpeedbrakeCycle = x\n",
                                "[pad.base]\nGearToggle = x\nSpeedbrakeCycle = a\n"));
    padButton(in, PadInput::X, true);
    CHECK(in.update(kFrameS).pressed[idx(Action::GearToggle)]);
    padButton(in, PadInput::A, true);
    CHECK(in.update(kFrameS).pressed[idx(Action::SpeedbrakeCycle)]);
}

TEST_CASE("input: bad names are skipped with a warning, the rest still loads") {
    const char* text =
        "[pad.base]\nGearToggle = a, bogus_button\nNotAnAction = b\naxis.Nope = lefty\n"
        "axis.Pitch = sideways\nFlapsUp = dpad_left\n"
        "[key.base]\nGearToggle = nosuchkey\naxis.Pitch = up\n"
        "[pad.chords]\nQuit = select\n[repeat]\nactions = FlapsUp, Bogus\n";
    const ControlsConfig c = configFromText(text);
    CHECK(c.warningCount == 8);
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::A, Action::GearToggle));
    CHECK(hasPadBinding(c, PadLayer::Base, PadInput::DpadLeft, Action::FlapsUp));
    CHECK(c.chords.empty());
    CHECK(c.repeats[idx(Action::FlapsUp)]);
}

TEST_CASE("input: missing default file leaves no bindings and does not crash") {
    InputSystem in;
    CHECK_FALSE(in.loadFiles("does/not/exist.ini", ""));
    padButton(in, PadInput::A, true);
    key(in, SDLK_g, true);
    CHECK_FALSE(in.update(kFrameS).pressed[idx(Action::GearToggle)]);
}

TEST_CASE("input: ini scalars can be tuned") {
    const ControlsConfig c =
        configFromText("[axes]\nleft_deadzone = 0.2\nleft_expo = 0\ndominance_ratio = 3\n"
                       "[repeat]\ndelay_s = 1.0\nactions = HdgBugInc\n"
                       "[double_tap]\naction = Brakes\nfires = ParkingBrake\nwindow_s = 0.5\n");
    CHECK(c.leftDeadzone == doctest::Approx(0.2f));
    CHECK(c.leftExpo == 0.0f);
    CHECK(c.dominanceRatio == doctest::Approx(3.0f));
    CHECK(c.repeatDelayS == doctest::Approx(1.0f));
    CHECK(c.doubleTapWindowS == doctest::Approx(0.5f));
}

TEST_CASE("input: default ini file has no inline comments") {
    std::string text;
    REQUIRE(readTextFile("assets/controls_default.ini", text));
    size_t pos = 0;
    int lineNo = 0;
    while (pos < text.size()) {
        size_t eol = text.find('\n', pos);
        if (eol == std::string::npos) eol = text.size();
        const std::string line = text.substr(pos, eol - pos);
        pos = eol + 1;
        ++lineNo;
        const size_t first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == ';' || line[first] == '#') continue;
        INFO("line " << lineNo << ": " << line);
        CHECK(line.find(';') == std::string::npos);
        CHECK(line.find('#') == std::string::npos);
    }
}
