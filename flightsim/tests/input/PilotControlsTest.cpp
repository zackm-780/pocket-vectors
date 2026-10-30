#include <doctest/doctest.h>

#include <cstring>

#include "input/InputTestUtil.h"
#include "input/PilotControls.h"

using namespace fsim;
using namespace fsim::testutil;

namespace {

AircraftProfile makeProfile() {
    AircraftProfile p;
    p.engineCount = 1;
    const char* labels[] = {"UP", "10", "20", "30"};
    const float cmd[] = {0.0f, 0.33f, 0.66f, 1.0f};
    for (int i = 0; i < 4; ++i) {
        FlapDetent d;
        d.cmdNorm = cmd[i];
        std::strncpy(d.label, labels[i], sizeof d.label - 1);
        p.flapDetents.push_back(d);
    }
    p.hasRetractableGear = true;
    p.hasSpeedbrake = true;
    p.hasReversers = true;
    return p;
}

struct Rig {
    AircraftProfile profile = makeProfile();
    AircraftState state;
    ControlInputs controls;
    MessageQueue messages;
    PilotControls pilot;
    ActionState actions;

    Rig() { state.valid = true; }

    // One frame with the current `actions`; edges are cleared afterwards like the real input does.
    void step(double dtS = kFrameS) {
        pilot.update(actions, profile, state, dtS, controls, &messages);
        for (bool& p : actions.pressed) p = false;
    }
    void press(Action a) {
        actions.pressed[idx(a)] = true;
        actions.down[idx(a)] = true;
    }
    void hold(Action a, bool on) { actions.down[idx(a)] = on; }
    bool saidLast(const char* text) const {
        return messages.size() > 0 && std::strcmp(messages.newest().text, text) == 0;
    }
};

}  // namespace

TEST_CASE("input: pilot controls leave start controls untouched with no input") {
    Rig r;
    r.controls.throttleNorm[0] = 0.62f;
    r.controls.elevTrimNorm = -0.07f;
    r.controls.flapDetent = 1;
    r.controls.gearDown = false;
    r.controls.parkingBrake = true;
    r.controls.brakeLeftNorm = r.controls.brakeRightNorm = 0.4f;  // scripted
    const ControlInputs before = r.controls;
    for (int i = 0; i < 120; ++i) r.step();
    CHECK(r.controls.throttleNorm[0] == before.throttleNorm[0]);
    CHECK(r.controls.elevTrimNorm == before.elevTrimNorm);
    CHECK(r.controls.flapDetent == before.flapDetent);
    CHECK(r.controls.gearDown == before.gearDown);
    CHECK(r.controls.parkingBrake == before.parkingBrake);
    CHECK(r.controls.brakeLeftNorm == before.brakeLeftNorm);
    CHECK(r.messages.size() == 0);
}

TEST_CASE("input: stick axes map to surfaces and steering follows the rudder") {
    Rig r;
    r.actions.axis[idx(Axis::Pitch)] = 0.5f;
    r.actions.axis[idx(Axis::Roll)] = -0.25f;
    r.actions.axis[idx(Axis::Rudder)] = 0.75f;
    r.step();
    CHECK(r.controls.elevatorNorm == doctest::Approx(0.5f));
    CHECK(r.controls.aileronNorm == doctest::Approx(-0.25f));
    CHECK(r.controls.rudderNorm == doctest::Approx(0.75f));
    CHECK(r.controls.steeringNorm == doctest::Approx(0.75f));
}

TEST_CASE("input: throttle rate integration, full stick = 0 to 100 percent in 2.5 s") {
    Rig r;
    r.actions.axis[idx(Axis::ThrottleRate)] = 1.0f;
    for (int i = 0; i < 75; ++i) r.step(1.0 / 30.0);  // 2.5 s
    CHECK(r.controls.throttleNorm[0] == doctest::Approx(1.0f).epsilon(0.001));
    for (int i = 0; i < 60; ++i) r.step();
    CHECK(r.controls.throttleNorm[0] == 1.0f);  // clamped

    // Half deflection down is half the rate, and released holds the value.
    r.actions.axis[idx(Axis::ThrottleRate)] = -0.5f;
    for (int i = 0; i < 60; ++i) r.step();  // 1 s at 0.2/s
    CHECK(r.controls.throttleNorm[0] == doctest::Approx(0.8f).epsilon(0.001));
    r.actions.axis[idx(Axis::ThrottleRate)] = 0.0f;
    for (int i = 0; i < 120; ++i) r.step();
    CHECK(r.controls.throttleNorm[0] == doctest::Approx(0.8f).epsilon(0.001));

    r.actions.axis[idx(Axis::ThrottleRate)] = -1.0f;
    for (int i = 0; i < 600; ++i) r.step();
    CHECK(r.controls.throttleNorm[0] == 0.0f);
}

TEST_CASE("input: throttle moves every engine of the profile") {
    Rig r;
    r.profile.engineCount = 2;
    r.controls.throttleNorm[0] = 0.3f;
    r.controls.throttleNorm[1] = 0.5f;
    r.controls.throttleNorm[2] = 0.9f;  // not an engine of this aircraft
    r.actions.axis[idx(Axis::ThrottleRate)] = 1.0f;
    for (int i = 0; i < 60; ++i) r.step();
    CHECK(r.controls.throttleNorm[0] == doctest::Approx(0.7f).epsilon(0.001));
    CHECK(r.controls.throttleNorm[1] == doctest::Approx(0.9f).epsilon(0.001));
    CHECK(r.controls.throttleNorm[2] == 0.9f);
}

TEST_CASE("input: elevator trim integrates while held and clamps") {
    Rig r;
    r.hold(Action::TrimNoseUp, true);
    r.actions.pressed[idx(Action::TrimNoseUp)] = true;
    for (int i = 0; i < 60; ++i) r.step();  // 1 s at 0.1/s
    CHECK(r.controls.elevTrimNorm == doctest::Approx(0.1f).epsilon(0.001));
    CHECK(r.saidLast("TRIM NOSE UP"));
    CHECK(r.messages.size() == 1);  // one message per press, not per frame
    r.hold(Action::TrimNoseUp, false);
    for (int i = 0; i < 60; ++i) r.step();
    CHECK(r.controls.elevTrimNorm == doctest::Approx(0.1f).epsilon(0.001));

    r.press(Action::TrimNoseDown);
    for (int i = 0; i < 6000; ++i) r.step();
    CHECK(r.controls.elevTrimNorm == -1.0f);
    CHECK(r.saidLast("TRIM NOSE DOWN"));
}

TEST_CASE("input: rudder trim integrates") {
    Rig r;
    r.hold(Action::RudderTrimRight, true);
    for (int i = 0; i < 60; ++i) r.step();
    CHECK(r.controls.rudderTrimNorm == doctest::Approx(0.1f).epsilon(0.001));
    r.hold(Action::RudderTrimRight, false);
    r.hold(Action::RudderTrimLeft, true);
    for (int i = 0; i < 120; ++i) r.step();
    CHECK(r.controls.rudderTrimNorm == doctest::Approx(-0.1f).epsilon(0.001));
}

TEST_CASE("input: flap detents step one at a time and stay inside the profile") {
    Rig r;
    r.press(Action::FlapsDown);
    r.step();
    CHECK(r.controls.flapDetent == 1);
    CHECK(r.saidLast("FLAPS 10"));
    r.press(Action::FlapsDown);
    r.step();
    r.press(Action::FlapsDown);
    r.step();
    CHECK(r.controls.flapDetent == 3);
    CHECK(r.saidLast("FLAPS 30"));
    r.press(Action::FlapsDown);  // already at the last detent
    r.step();
    CHECK(r.controls.flapDetent == 3);
    r.press(Action::FlapsUp);
    r.step();
    CHECK(r.controls.flapDetent == 2);
    CHECK(r.saidLast("FLAPS 20"));
    for (int i = 0; i < 5; ++i) {
        r.press(Action::FlapsUp);
        r.step();
    }
    CHECK(r.controls.flapDetent == 0);
    CHECK(r.saidLast("FLAPS UP"));
}

TEST_CASE("input: flaps on an aircraft without flaps") {
    Rig r;
    r.profile.flapDetents.clear();
    r.press(Action::FlapsDown);
    r.step();
    CHECK(r.controls.flapDetent == 0);
    CHECK(r.saidLast("NO FLAPS"));
}

TEST_CASE("input: gear toggles in the air and is locked on the ground") {
    Rig r;
    r.state.wowAny = true;
    r.press(Action::GearToggle);
    r.step();
    CHECK(r.controls.gearDown);
    CHECK(r.saidLast("GEAR LOCKED ON GROUND"));

    r.state.wowAny = false;
    r.press(Action::GearToggle);
    r.step();
    CHECK_FALSE(r.controls.gearDown);
    CHECK(r.saidLast("GEAR UP"));
    r.press(Action::GearToggle);
    r.step();
    CHECK(r.controls.gearDown);
    CHECK(r.saidLast("GEAR DOWN"));
}

TEST_CASE("input: fixed gear never moves") {
    Rig r;
    r.profile.hasRetractableGear = false;
    r.press(Action::GearToggle);
    r.step();
    CHECK(r.controls.gearDown);
    CHECK(r.saidLast("GEAR FIXED"));
}

TEST_CASE("input: speedbrake cycles retracted, 50, 100, retracted") {
    Rig r;
    r.press(Action::SpeedbrakeCycle);
    r.step();
    CHECK(r.controls.speedbrakeNorm == doctest::Approx(0.5f));
    CHECK(r.saidLast("SPEEDBRAKE 50%"));
    r.press(Action::SpeedbrakeCycle);
    r.step();
    CHECK(r.controls.speedbrakeNorm == doctest::Approx(1.0f));
    r.press(Action::SpeedbrakeCycle);
    r.step();
    CHECK(r.controls.speedbrakeNorm == 0.0f);

    r.profile.hasSpeedbrake = false;
    r.press(Action::SpeedbrakeCycle);
    r.step();
    CHECK(r.controls.speedbrakeNorm == 0.0f);
    CHECK(r.saidLast("NO SPEEDBRAKE"));
}

TEST_CASE("input: wheel brakes follow the button, parking brake toggles") {
    Rig r;
    r.hold(Action::Brakes, true);
    r.step();
    CHECK(r.controls.brakeLeftNorm == 1.0f);
    CHECK(r.controls.brakeRightNorm == 1.0f);
    r.hold(Action::Brakes, false);
    r.step();
    CHECK(r.controls.brakeLeftNorm == 0.0f);

    r.press(Action::ParkingBrake);
    r.step();
    CHECK(r.controls.parkingBrake);
    CHECK(r.saidLast("PARKING BRAKE ON"));
    r.press(Action::ParkingBrake);
    r.step();
    CHECK_FALSE(r.controls.parkingBrake);
    CHECK(r.saidLast("PARKING BRAKE OFF"));
}

TEST_CASE("input: reversers only on the ground with the throttle at idle") {
    Rig r;
    r.controls.throttleNorm[0] = 0.5f;
    r.state.wowAny = true;
    r.press(Action::ReverseToggle);
    r.step();
    CHECK_FALSE(r.controls.reverseThrust);  // throttle not at idle
    CHECK(r.messages.size() == 1);

    r.controls.throttleNorm[0] = 0.0f;
    r.state.wowAny = false;
    r.press(Action::ReverseToggle);
    r.step();
    CHECK_FALSE(r.controls.reverseThrust);  // airborne

    r.state.wowAny = true;
    r.press(Action::ReverseToggle);
    r.step();
    CHECK(r.controls.reverseThrust);
    CHECK(r.saidLast("REVERSERS DEPLOYED"));

    r.state.wowAny = false;  // lifted off with them out: stowed automatically
    r.step();
    CHECK_FALSE(r.controls.reverseThrust);

    r.profile.hasReversers = false;
    r.state.wowAny = true;
    r.press(Action::ReverseToggle);
    r.step();
    CHECK_FALSE(r.controls.reverseThrust);
    CHECK(r.saidLast("NO REVERSERS"));
}

TEST_CASE("input: end to end, synthetic pad events drive the pilot controls") {
    InputSystem in;
    in.setConfig(defaultConfig());
    AircraftProfile profile = makeProfile();
    AircraftState state;
    state.valid = true;
    ControlInputs controls;
    controls.throttleNorm[0] = 0.2f;
    MessageQueue messages;
    PilotControls pilot;

    padAxis(in, SDL_CONTROLLER_AXIS_RIGHTY, -1.0f);  // throttle up
    padButton(in, PadInput::DpadRight, true);        // flaps down
    for (int i = 0; i < 60; ++i) {
        pilot.update(in.update(kFrameS), profile, state, kFrameS, controls, &messages);
        if (i == 0) padButton(in, PadInput::DpadRight, false);
    }
    CHECK(controls.throttleNorm[0] == doctest::Approx(0.6f).epsilon(0.01));
    CHECK(controls.flapDetent == 1);
    CHECK(messages.size() == 1);
    CHECK(std::strcmp(messages.newest().text, "FLAPS 10") == 0);
}

TEST_CASE("input: message queue ring keeps the newest eight") {
    MessageQueue q;
    for (int i = 0; i < 11; ++i) {
        q.tick(1.0);
        q.postf("MSG %d", i);
    }
    CHECK(q.size() == MessageQueue::kCapacity);
    CHECK(std::strcmp(q.newest(0).text, "MSG 10") == 0);
    CHECK(std::strcmp(q.newest(7).text, "MSG 3") == 0);
    CHECK(q.ageS(q.newest(0)) == doctest::Approx(0.0));
    CHECK(q.ageS(q.newest(7)) == doctest::Approx(7.0));
    q.post("A VERY LONG MESSAGE THAT DOES NOT FIT IN THE BUFFER AT ALL");
    CHECK(std::strlen(q.newest().text) == sizeof(Message::text) - 1);
}

TEST_CASE("input: the A that closes a menu does not move the gear once its buttons are dropped") {
    // On the pad A is both MenuAccept and GearToggle (CONTROLS.md). The app clears the buttons of
    // a frame the menu consumed, so resuming in the air with A left the gear where it was.
    InputSystem in;
    in.setConfig(defaultConfig());
    padButton(in, PadInput::A, true);
    ActionState frame = in.update(kFrameS);
    CHECK(frame.pressed[idx(Action::MenuAccept)]);
    CHECK(frame.pressed[idx(Action::GearToggle)]);

    Rig r;
    r.state.wowAny = false;  // airborne
    r.controls.gearDown = true;
    r.actions = frame;
    clearButtons(r.actions);
    r.step();
    CHECK(r.controls.gearDown);
    CHECK(r.messages.size() == 0);

    // A real press in flight still works.
    padButton(in, PadInput::A, false);
    in.update(kFrameS);
    padButton(in, PadInput::A, true);
    r.actions = in.update(kFrameS);
    r.step();
    CHECK_FALSE(r.controls.gearDown);
    CHECK(r.saidLast("GEAR UP"));
}

TEST_CASE("input: clearButtons keeps the sticks") {
    InputSystem in;
    in.setConfig(defaultConfig());
    padAxis(in, SDL_CONTROLLER_AXIS_LEFTX, 1.0f);
    padButton(in, PadInput::B, true);
    ActionState a = in.update(kFrameS);
    clearButtons(a);
    CHECK(a.axis[idx(Axis::Roll)] == doctest::Approx(1.0f));
    CHECK_FALSE(a.down[idx(Action::Brakes)]);
    CHECK_FALSE(a.pressed[idx(Action::MenuBack)]);
}
