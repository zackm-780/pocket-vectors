#include <doctest/doctest.h>

#include <cstdlib>

#include "input/InputSystem.h"
#include "input/InputTestUtil.h"

using namespace fsim;
using namespace fsim::testutil;

namespace {

struct Fixture {
    InputSystem in;
    Fixture() { in.setConfig(defaultConfig()); }
    const ActionState& frame(double dtS = kFrameS) { return in.update(dtS); }
    // Runs frames until `seconds` have elapsed; returns how many frames had `a` pressed.
    int countPressed(Action a, double seconds) {
        int n = 0;
        for (double t = 0; t < seconds; t += kFrameS)
            if (frame().pressed[idx(a)]) ++n;
        return n;
    }
};

void pressSdlButton(InputSystem& in, Uint8 button, bool down) {
    SDL_Event e{};
    e.type = down ? SDL_CONTROLLERBUTTONDOWN : SDL_CONTROLLERBUTTONUP;
    e.cbutton.button = button;
    e.cbutton.state = down ? SDL_PRESSED : SDL_RELEASED;
    in.handleEvent(e);
}

}  // namespace

TEST_CASE("input: left stick gives pitch and roll through deadzone and expo") {
    Fixture f;
    padAxis(f.in, SDL_CONTROLLER_AXIS_LEFTX, 0.05f);
    padAxis(f.in, SDL_CONTROLLER_AXIS_LEFTY, 0.05f);
    CHECK(f.frame().axis[idx(Axis::Roll)] == 0.0f);
    CHECK(f.frame().axis[idx(Axis::Pitch)] == 0.0f);

    padAxis(f.in, SDL_CONTROLLER_AXIS_LEFTX, 1.0f);  // right
    padAxis(f.in, SDL_CONTROLLER_AXIS_LEFTY, 0.0f);
    CHECK(f.frame().axis[idx(Axis::Roll)] == doctest::Approx(1.0f));

    // Stick pulled back (SDL +Y) = nose up = +Pitch.
    padAxis(f.in, SDL_CONTROLLER_AXIS_LEFTX, 0.0f);
    padAxis(f.in, SDL_CONTROLLER_AXIS_LEFTY, 1.0f);
    CHECK(f.frame().axis[idx(Axis::Pitch)] == doctest::Approx(1.0f));
    padAxis(f.in, SDL_CONTROLLER_AXIS_LEFTY, -1.0f);
    CHECK(f.frame().axis[idx(Axis::Pitch)] == doctest::Approx(-1.0f));

    // Half deflection is softened by the expo (0.35): below linear.
    padAxis(f.in, SDL_CONTROLLER_AXIS_LEFTY, 0.5f);
    const float half = f.frame().axis[idx(Axis::Pitch)];
    CHECK(half > 0.0f);
    CHECK(half < 0.45f);
}

TEST_CASE("input: right stick Y is throttle rate, X is rudder, dominance filters") {
    Fixture f;
    padAxis(f.in, SDL_CONTROLLER_AXIS_RIGHTY, -1.0f);  // push up = increase
    const ActionState& up = f.frame();
    CHECK(up.axis[idx(Axis::ThrottleRate)] == doctest::Approx(1.0f));
    CHECK(up.axis[idx(Axis::Rudder)] == 0.0f);

    // Mostly vertical with a little sideways wobble: rudder suppressed.
    padAxis(f.in, SDL_CONTROLLER_AXIS_RIGHTX, 0.3f);
    padAxis(f.in, SDL_CONTROLLER_AXIS_RIGHTY, 0.9f);
    const ActionState& vertical = f.frame();
    CHECK(vertical.axis[idx(Axis::Rudder)] == 0.0f);
    CHECK(vertical.axis[idx(Axis::ThrottleRate)] == doctest::Approx(-0.88f).epsilon(0.1));

    // Mostly horizontal: throttle change suppressed.
    padAxis(f.in, SDL_CONTROLLER_AXIS_RIGHTX, 0.9f);
    padAxis(f.in, SDL_CONTROLLER_AXIS_RIGHTY, 0.3f);
    const ActionState& horizontal = f.frame();
    CHECK(horizontal.axis[idx(Axis::ThrottleRate)] == 0.0f);
    CHECK(horizontal.axis[idx(Axis::Rudder)] > 0.8f);

    // Inside the 0.12 deadzone nothing happens at all.
    padAxis(f.in, SDL_CONTROLLER_AXIS_RIGHTX, 0.1f);
    padAxis(f.in, SDL_CONTROLLER_AXIS_RIGHTY, -0.1f);
    CHECK(f.frame().axis[idx(Axis::Rudder)] == 0.0f);
    CHECK(f.frame().axis[idx(Axis::ThrottleRate)] == 0.0f);
}

TEST_CASE("input: base layer buttons produce edges and held state") {
    Fixture f;
    padButton(f.in, PadInput::A, true);
    const ActionState& s = f.frame();
    CHECK(s.pressed[idx(Action::GearToggle)]);
    CHECK(s.down[idx(Action::GearToggle)]);
    CHECK(s.pressed[idx(Action::MenuAccept)]);  // same physical button, menu meaning
    const ActionState& s2 = f.frame();
    CHECK_FALSE(s2.pressed[idx(Action::GearToggle)]);  // edge lives for one frame only
    CHECK(s2.down[idx(Action::GearToggle)]);
    padButton(f.in, PadInput::A, false);
    CHECK_FALSE(f.frame().down[idx(Action::GearToggle)]);
}

TEST_CASE("input: a tap shorter than a frame still pressed") {
    Fixture f;
    padButton(f.in, PadInput::X, true);
    padButton(f.in, PadInput::X, false);
    const ActionState& s = f.frame();
    CHECK(s.pressed[idx(Action::SpeedbrakeCycle)]);
    CHECK_FALSE(f.frame().down[idx(Action::SpeedbrakeCycle)]);
}

TEST_CASE("input: B and R2 both brake, hold semantics") {
    Fixture f;
    padButton(f.in, PadInput::B, true);
    CHECK(f.frame().down[idx(Action::Brakes)]);
    padAxis(f.in, SDL_CONTROLLER_AXIS_TRIGGERRIGHT, 1.0f);  // R2 too
    padButton(f.in, PadInput::B, false);
    CHECK(f.frame().down[idx(Action::Brakes)]);  // R2 still holds them
    padAxis(f.in, SDL_CONTROLLER_AXIS_TRIGGERRIGHT, 0.0f);
    CHECK_FALSE(f.frame().down[idx(Action::Brakes)]);
}

TEST_CASE("input: L1 layer replaces base bindings") {
    Fixture f;
    padButton(f.in, PadInput::L1, true);
    padButton(f.in, PadInput::A, true);
    const ActionState& s = f.frame();
    CHECK(s.pressed[idx(Action::ApToggle)]);
    CHECK_FALSE(s.pressed[idx(Action::GearToggle)]);
    padButton(f.in, PadInput::A, false);
    padButton(f.in, PadInput::DpadRight, true);
    CHECK(f.frame().pressed[idx(Action::HdgBugInc)]);
    padButton(f.in, PadInput::DpadRight, false);
    padButton(f.in, PadInput::DpadUp, true);
    const ActionState& up = f.frame();
    CHECK(up.pressed[idx(Action::AltBugInc)]);
    CHECK_FALSE(up.pressed[idx(Action::TrimNoseDown)]);
    padButton(f.in, PadInput::DpadUp, false);
    // L1 + R2 = ALT hold, and R2 does not brake in this layer.
    padAxis(f.in, SDL_CONTROLLER_AXIS_TRIGGERRIGHT, 1.0f);
    const ActionState& alt = f.frame();
    CHECK(alt.pressed[idx(Action::AltMode)]);
    CHECK_FALSE(alt.down[idx(Action::Brakes)]);
}

TEST_CASE("input: R1 layer is the systems layer") {
    Fixture f;
    padButton(f.in, PadInput::R1, true);
    padButton(f.in, PadInput::A, true);
    CHECK(f.frame().pressed[idx(Action::ReverseToggle)]);
    padButton(f.in, PadInput::A, false);
    padButton(f.in, PadInput::DpadLeft, true);
    const ActionState& s = f.frame();
    CHECK(s.pressed[idx(Action::RudderTrimLeft)]);
    CHECK_FALSE(s.pressed[idx(Action::FlapsUp)]);
}

TEST_CASE("input: layer is decided at press time, release always matches") {
    Fixture f;
    padButton(f.in, PadInput::B, true);   // brakes (base)
    padButton(f.in, PadInput::L1, true);  // layer changes while B is held
    CHECK(f.frame().down[idx(Action::Brakes)]);
    padButton(f.in, PadInput::B, false);
    CHECK_FALSE(f.frame().down[idx(Action::Brakes)]);  // not stuck
}

TEST_CASE("input: L2 look layer, right stick looks and does not fly") {
    Fixture f;
    padAxis(f.in, SDL_CONTROLLER_AXIS_TRIGGERLEFT, 1.0f);
    padAxis(f.in, SDL_CONTROLLER_AXIS_RIGHTX, 1.0f);
    padAxis(f.in, SDL_CONTROLLER_AXIS_RIGHTY, -1.0f);  // up = look up
    const ActionState& s = f.frame();
    CHECK(s.down[idx(Action::LookHold)]);
    CHECK(s.axis[idx(Axis::LookYaw)] == doctest::Approx(1.0f));
    CHECK(s.axis[idx(Axis::LookPitch)] == doctest::Approx(1.0f));
    CHECK(s.axis[idx(Axis::Rudder)] == 0.0f);
    CHECK(s.axis[idx(Axis::ThrottleRate)] == 0.0f);

    padButton(f.in, PadInput::DpadLeft, true);
    const ActionState& snap = f.frame();
    CHECK(snap.pressed[idx(Action::LookLeft)]);
    CHECK_FALSE(snap.pressed[idx(Action::FlapsUp)]);
    padButton(f.in, PadInput::DpadLeft, false);
    padButton(f.in, PadInput::DpadDown, true);
    CHECK(f.frame().pressed[idx(Action::LookBack)]);
    padButton(f.in, PadInput::DpadDown, false);
    padButton(f.in, PadInput::DpadUp, true);
    CHECK(f.frame().pressed[idx(Action::LookDown)]);

    // Released: look axes go back to zero and the sticks fly again.
    padAxis(f.in, SDL_CONTROLLER_AXIS_TRIGGERLEFT, 0.0f);
    const ActionState& back = f.frame();
    CHECK_FALSE(back.down[idx(Action::LookHold)]);
    CHECK(back.axis[idx(Axis::LookYaw)] == 0.0f);
    CHECK(back.axis[idx(Axis::Rudder)] > 0.0f);
}

TEST_CASE("input: L1 right stick Y drives the bug rate axis, not the throttle") {
    Fixture f;
    padButton(f.in, PadInput::L1, true);
    padAxis(f.in, SDL_CONTROLLER_AXIS_RIGHTY, -1.0f);
    const ActionState& s = f.frame();
    CHECK(s.axis[idx(Axis::BugRate)] == doctest::Approx(1.0f));
    CHECK(s.axis[idx(Axis::ThrottleRate)] == 0.0f);
}

TEST_CASE("input: analog trigger hysteresis") {
    Fixture f;
    padAxis(f.in, SDL_CONTROLLER_AXIS_TRIGGERLEFT, 0.4f);  // below on-threshold
    CHECK_FALSE(f.frame().down[idx(Action::LookHold)]);
    padAxis(f.in, SDL_CONTROLLER_AXIS_TRIGGERLEFT, 0.6f);
    CHECK(f.frame().down[idx(Action::LookHold)]);
    padAxis(f.in, SDL_CONTROLLER_AXIS_TRIGGERLEFT, 0.4f);  // between thresholds: stays held
    CHECK(f.frame().down[idx(Action::LookHold)]);
    padAxis(f.in, SDL_CONTROLLER_AXIS_TRIGGERLEFT, 0.1f);
    CHECK_FALSE(f.frame().down[idx(Action::LookHold)]);
}

TEST_CASE("input: auto-repeat for held bug buttons") {
    Fixture f;
    padButton(f.in, PadInput::L1, true);
    padButton(f.in, PadInput::DpadRight, true);
    const ActionState& first = f.frame();
    CHECK(first.pressed[idx(Action::HdgBugInc)]);
    CHECK(first.repeatCount[idx(Action::HdgBugInc)] == 0);

    // Nothing before the initial delay (0.4 s).
    int early = 0;
    double t = 0;
    for (; t < 0.38; t += kFrameS)
        if (f.frame().pressed[idx(Action::HdgBugInc)]) ++early;
    CHECK(early == 0);

    // Then repeats arrive, faster and faster, and repeatCount counts them.
    double lastT = t, firstGap = 0, lastGap = 0;
    int repeats = 0;
    int lastCount = 0;
    for (; t < 3.0; t += kFrameS) {
        const ActionState& s = f.frame();
        if (!s.pressed[idx(Action::HdgBugInc)]) continue;
        ++repeats;
        if (repeats == 2) firstGap = t - lastT;
        lastGap = t - lastT;
        lastT = t;
        CHECK(s.repeatCount[idx(Action::HdgBugInc)] == lastCount + 1);
        lastCount = s.repeatCount[idx(Action::HdgBugInc)];
    }
    CHECK(repeats > 15);
    CHECK(firstGap > lastGap);
    CHECK(lastGap == doctest::Approx(0.05).epsilon(0.4));

    padButton(f.in, PadInput::DpadRight, false);
    const ActionState& rel = f.frame();
    CHECK_FALSE(rel.pressed[idx(Action::HdgBugInc)]);
    CHECK(rel.repeatCount[idx(Action::HdgBugInc)] == 0);
    CHECK(f.countPressed(Action::HdgBugInc, 1.0) == 0);
}

TEST_CASE("input: non-repeat actions do not auto-repeat") {
    Fixture f;
    padButton(f.in, PadInput::A, true);
    f.frame();
    CHECK(f.countPressed(Action::GearToggle, 2.0) == 0);
}

TEST_CASE("input: double-tap B toggles the parking brake") {
    Fixture f;
    padButton(f.in, PadInput::B, true);
    CHECK_FALSE(f.frame().pressed[idx(Action::ParkingBrake)]);
    padButton(f.in, PadInput::B, false);
    for (int i = 0; i < 5; ++i) f.frame();  // ~83 ms
    padButton(f.in, PadInput::B, true);
    const ActionState& s = f.frame();
    CHECK(s.pressed[idx(Action::ParkingBrake)]);
    CHECK(s.down[idx(Action::Brakes)]);
    padButton(f.in, PadInput::B, false);
    f.frame();

    // A third quick tap starts a new pair, it is not a second double-tap.
    padButton(f.in, PadInput::B, true);
    CHECK_FALSE(f.frame().pressed[idx(Action::ParkingBrake)]);
    padButton(f.in, PadInput::B, false);
}

TEST_CASE("input: two slow taps of B are not a double-tap") {
    Fixture f;
    padButton(f.in, PadInput::B, true);
    f.frame();
    padButton(f.in, PadInput::B, false);
    for (int i = 0; i < 30; ++i) f.frame();  // 0.5 s
    padButton(f.in, PadInput::B, true);
    CHECK_FALSE(f.frame().pressed[idx(Action::ParkingBrake)]);
}

TEST_CASE("input: Select+Start chord quits") {
    Fixture f;
    padButton(f.in, PadInput::Select, true);
    f.frame();
    CHECK_FALSE(f.in.quitRequested());
    padButton(f.in, PadInput::Start, true);
    const ActionState& s = f.frame();
    CHECK(s.pressed[idx(Action::Quit)]);
    CHECK(f.in.quitRequested());
}

TEST_CASE("input: Select+Start quits from guide as well as back") {
    for (Uint8 selectButton : {static_cast<Uint8>(SDL_CONTROLLER_BUTTON_GUIDE),
                               static_cast<Uint8>(SDL_CONTROLLER_BUTTON_BACK)}) {
        Fixture f;
        pressSdlButton(f.in, selectButton, true);
        f.frame();
        CHECK_FALSE(f.in.quitRequested());
        pressSdlButton(f.in, static_cast<Uint8>(SDL_CONTROLLER_BUTTON_START), true);
        CHECK(f.frame().pressed[idx(Action::Quit)]);
        CHECK(f.in.quitRequested());
    }
}

TEST_CASE("input: SDL_QUIT requests quit") {
    Fixture f;
    SDL_Event e{};
    e.type = SDL_QUIT;
    f.in.handleEvent(e);
    CHECK(f.in.quitRequested());
}

TEST_CASE("input: hot-plug removal releases held buttons and centres sticks") {
    Fixture f;
    padButton(f.in, PadInput::B, true);
    padAxis(f.in, SDL_CONTROLLER_AXIS_LEFTX, 1.0f);
    CHECK(f.frame().down[idx(Action::Brakes)]);
    SDL_Event removed{};
    removed.type = SDL_CONTROLLERDEVICEREMOVED;
    removed.cdevice.which = 7;
    f.in.handleEvent(removed);
    const ActionState& s = f.frame();
    CHECK_FALSE(s.down[idx(Action::Brakes)]);
    CHECK(s.axis[idx(Axis::Roll)] == 0.0f);
    CHECK(f.in.controllerCount() == 0);
}

TEST_CASE("input: hot-plug add of a device that cannot be opened is harmless") {
    Fixture f;
    SDL_Event added{};
    added.type = SDL_CONTROLLERDEVICEADDED;
    added.cdevice.which = 5;
    f.in.handleEvent(added);
    CHECK(f.in.controllerCount() == 0);
    f.frame();
}

TEST_CASE("input: keyboard ramps stick axes and springs back") {
    Fixture f;
    key(f.in, SDLK_DOWN, true);  // pull back = nose up
    const float a = f.frame().axis[idx(Axis::Pitch)];
    CHECK(a > 0.0f);
    CHECK(a < 0.1f);  // ramped, not instant
    for (int i = 0; i < 60; ++i) f.frame();
    CHECK(f.frame().axis[idx(Axis::Pitch)] == doctest::Approx(1.0f));
    key(f.in, SDLK_DOWN, false);
    for (int i = 0; i < 30; ++i) f.frame();
    CHECK(f.frame().axis[idx(Axis::Pitch)] == doctest::Approx(0.0f));

    key(f.in, SDLK_UP, true);
    for (int i = 0; i < 60; ++i) f.frame();
    CHECK(f.frame().axis[idx(Axis::Pitch)] == doctest::Approx(-1.0f));
    key(f.in, SDLK_UP, false);
    key(f.in, SDLK_q, true);
    for (int i = 0; i < 60; ++i) f.frame();
    CHECK(f.frame().axis[idx(Axis::Rudder)] == doctest::Approx(-1.0f));
}

TEST_CASE("input: keyboard throttle keys are instant rate commands") {
    Fixture f;
    key(f.in, SDLK_w, true);
    CHECK(f.frame().axis[idx(Axis::ThrottleRate)] == doctest::Approx(1.0f));
    key(f.in, SDLK_w, false);
    key(f.in, SDLK_PAGEDOWN, true);
    CHECK(f.frame().axis[idx(Axis::ThrottleRate)] == doctest::Approx(-1.0f));
}

TEST_CASE("input: keyboard actions, modifiers and key repeat") {
    Fixture f;
    key(f.in, SDLK_g, true);
    CHECK(f.frame().pressed[idx(Action::GearToggle)]);
    key(f.in, SDLK_g, true, 0, true);  // OS key repeat is ignored
    CHECK_FALSE(f.frame().pressed[idx(Action::GearToggle)]);
    key(f.in, SDLK_g, false);

    key(f.in, SDLK_b, true);
    const ActionState& plain = f.frame();
    CHECK(plain.down[idx(Action::Brakes)]);
    CHECK_FALSE(plain.pressed[idx(Action::ParkingBrake)]);
    key(f.in, SDLK_b, false);
    CHECK_FALSE(f.frame().down[idx(Action::Brakes)]);

    key(f.in, SDLK_b, true, KMOD_LSHIFT);  // Shift+B = parking brake, not wheel brakes
    const ActionState& shifted = f.frame();
    CHECK(shifted.pressed[idx(Action::ParkingBrake)]);
    CHECK_FALSE(shifted.down[idx(Action::Brakes)]);
    key(f.in, SDLK_b, false, KMOD_LSHIFT);

    key(f.in, SDLK_LEFTBRACKET, true);
    CHECK(f.frame().pressed[idx(Action::FlapsUp)]);
    key(f.in, SDLK_RIGHTBRACKET, true);
    CHECK(f.frame().pressed[idx(Action::FlapsDown)]);
    key(f.in, SDLK_ESCAPE, true);
    CHECK(f.frame().pressed[idx(Action::Pause)]);
    key(f.in, SDLK_p, true);
    CHECK(f.frame().pressed[idx(Action::Pause)]);
    key(f.in, SDLK_F12, true);
    CHECK(f.frame().pressed[idx(Action::Screenshot)]);
}

TEST_CASE("input: Ctrl+Q quits, plain Q is rudder") {
    Fixture f;
    key(f.in, SDLK_q, true);
    f.frame();
    CHECK_FALSE(f.in.quitRequested());
    key(f.in, SDLK_q, false);
    key(f.in, SDLK_q, true, KMOD_LCTRL);
    f.frame();
    CHECK(f.in.quitRequested());
}

TEST_CASE("input: keyboard look layer (V + arrows)") {
    Fixture f;
    key(f.in, SDLK_v, true);
    key(f.in, SDLK_RIGHT, true);
    for (int i = 0; i < 60; ++i) f.frame();
    const ActionState& s = f.frame();
    CHECK(s.down[idx(Action::LookHold)]);
    CHECK(s.axis[idx(Axis::LookYaw)] == doctest::Approx(1.0f));
    CHECK(s.axis[idx(Axis::Roll)] == 0.0f);  // arrows do not fly while looking
    key(f.in, SDLK_RIGHT, false);
    key(f.in, SDLK_v, false);
    for (int i = 0; i < 60; ++i) f.frame();
    CHECK_FALSE(f.frame().down[idx(Action::LookHold)]);
    CHECK(f.frame().axis[idx(Axis::LookYaw)] == doctest::Approx(0.0f));
}

TEST_CASE("input: keyboard ignored on the portmaster platform") {
    setenv("FLIGHTSIM_PLATFORM", "portmaster", 1);
    InputSystem in;
    unsetenv("FLIGHTSIM_PLATFORM");
    in.setConfig(defaultConfig());
    CHECK_FALSE(in.keyboardEnabled());
    key(in, SDLK_g, true);
    key(in, SDLK_DOWN, true);
    key(in, SDLK_q, true, KMOD_LCTRL);
    const ActionState& s = in.update(kFrameS);
    CHECK_FALSE(s.pressed[idx(Action::GearToggle)]);
    CHECK(s.axis[idx(Axis::Pitch)] == 0.0f);
    CHECK_FALSE(in.quitRequested());
    // The gamepad still works.
    padButton(in, PadInput::A, true);
    CHECK(in.update(kFrameS).pressed[idx(Action::GearToggle)]);
}

TEST_CASE("input: keyboard enabled by default and for other platforms") {
    setenv("FLIGHTSIM_PLATFORM", "desktop", 1);
    InputSystem in;
    unsetenv("FLIGHTSIM_PLATFORM");
    CHECK(in.keyboardEnabled());
}
