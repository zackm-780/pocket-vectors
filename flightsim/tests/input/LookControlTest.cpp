// L2 look layer (CONTROLS.md section 2): snaps, free look, limits and recentering, through the
// real InputSystem and default bindings where it matters.
#include <doctest/doctest.h>

#include "input/InputTestUtil.h"
#include "input/LookControl.h"

using namespace fsim;
using namespace fsim::testutil;

namespace {

struct Rig {
    InputSystem in;
    LookControl look;
    Rig() { in.setConfig(defaultConfig()); }
    void run(double seconds) {
        for (double t = 0; t < seconds; t += kFrameS) look.update(in.update(kFrameS), kFrameS);
    }
    void l2(bool down) { padAxis(in, SDL_CONTROLLER_AXIS_TRIGGERLEFT, down ? 1.0f : 0.0f); }
    void tap(PadInput label) {
        padButton(in, label, true);
        run(kFrameS);
        padButton(in, label, false);
    }
};

}  // namespace

TEST_CASE("look: straight ahead without L2") {
    Rig r;
    r.tap(PadInput::DpadLeft);  // flaps up in the base layer, not a look
    padAxis(r.in, SDL_CONTROLLER_AXIS_RIGHTX, 1.0f);
    r.run(1.0);
    CHECK(r.look.yawDeg() == 0.0f);
    CHECK(r.look.pitchDeg() == 0.0f);
    CHECK_FALSE(r.look.active());
}

TEST_CASE("look: L2 + D-pad snaps left, right, back and down, release recenters") {
    Rig r;
    r.l2(true);
    r.tap(PadInput::DpadLeft);
    r.run(0.5);
    CHECK(r.look.yawDeg() == doctest::Approx(-90.0f));
    CHECK(r.look.yawRad() < 0.0f);

    r.tap(PadInput::DpadDown);  // back, turning the short way from the left look
    r.run(0.5);
    CHECK(r.look.yawDeg() == doctest::Approx(-180.0f));

    r.tap(PadInput::DpadRight);
    r.run(0.5);
    CHECK(r.look.yawDeg() == doctest::Approx(90.0f));

    r.tap(PadInput::DpadUp);  // over the nose
    r.run(0.5);
    CHECK(r.look.yawDeg() == doctest::Approx(0.0f));
    CHECK(r.look.pitchDeg() == doctest::Approx(-25.0f));

    r.tap(PadInput::DpadLeft);
    r.run(0.5);
    r.l2(false);
    r.run(0.15);
    CHECK(r.look.yawDeg() > -30.0f);  // most of the way back already
    r.run(0.35);
    CHECK(r.look.yawDeg() == 0.0f);
    CHECK(r.look.pitchDeg() == 0.0f);
    CHECK_FALSE(r.look.active());
}

TEST_CASE("look: the D-pad under L2 does not trim or move flaps") {
    Rig r;
    r.l2(true);
    padButton(r.in, PadInput::DpadLeft, true);
    const ActionState& a = r.in.update(kFrameS);
    CHECK(a.down[idx(Action::LookHold)]);
    CHECK(a.pressed[idx(Action::LookLeft)]);
    CHECK_FALSE(a.pressed[idx(Action::FlapsUp)]);
}

TEST_CASE("look: R2 is brakes, not the look layer") {
    Rig r;
    padAxis(r.in, SDL_CONTROLLER_AXIS_TRIGGERRIGHT, 1.0f);
    padButton(r.in, PadInput::DpadLeft, true);
    const ActionState& a = r.in.update(kFrameS);
    CHECK(a.down[idx(Action::Brakes)]);
    CHECK_FALSE(a.down[idx(Action::LookHold)]);
    CHECK(a.pressed[idx(Action::FlapsUp)]);
}

TEST_CASE("look: free look with the right stick holds where it is left and stays in limits") {
    Rig r;
    r.l2(true);
    padAxis(r.in, SDL_CONTROLLER_AXIS_RIGHTX, 1.0f);  // right
    r.run(0.4);
    padAxis(r.in, SDL_CONTROLLER_AXIS_RIGHTX, 0.0f);
    r.run(0.5);
    const float held = r.look.yawDeg();
    CHECK(held > 30.0f);
    CHECK(held < 90.0f);
    r.run(0.5);
    CHECK(r.look.yawDeg() == doctest::Approx(held));

    padAxis(r.in, SDL_CONTROLLER_AXIS_RIGHTX, 1.0f);
    r.run(3.0);
    CHECK(r.look.yawDeg() == doctest::Approx(150.0f));
    padAxis(r.in, SDL_CONTROLLER_AXIS_RIGHTX, 0.0f);
    padAxis(r.in, SDL_CONTROLLER_AXIS_RIGHTY, -1.0f);  // push up = look up
    r.run(3.0);
    CHECK(r.look.pitchDeg() == doctest::Approx(45.0f));
    padAxis(r.in, SDL_CONTROLLER_AXIS_RIGHTY, 1.0f);
    r.run(3.0);
    CHECK(r.look.pitchDeg() == doctest::Approx(-60.0f));
}

TEST_CASE("look: the stick moves a back look toward the front, never past 180") {
    Rig r;
    r.l2(true);
    r.tap(PadInput::DpadDown);
    r.run(0.5);
    CHECK(r.look.yawDeg() == doctest::Approx(180.0f));
    padAxis(r.in, SDL_CONTROLLER_AXIS_RIGHTX, 1.0f);
    r.run(0.5);
    CHECK(r.look.yawDeg() == doctest::Approx(180.0f));
    padAxis(r.in, SDL_CONTROLLER_AXIS_RIGHTX, -1.0f);
    r.run(0.3);
    CHECK(r.look.yawDeg() < 170.0f);
}

TEST_CASE("look: reset goes straight ahead at once") {
    Rig r;
    r.l2(true);
    r.tap(PadInput::DpadRight);
    r.run(0.5);
    r.look.reset();
    CHECK(r.look.yawDeg() == 0.0f);
    CHECK_FALSE(r.look.active());
}
