#include "input/PilotControls.h"

#include <algorithm>

#include "input/InputCurves.h"

namespace fsim {
namespace {

constexpr int idx(Action a) {
    return static_cast<int>(a);
}
constexpr int idx(Axis a) {
    return static_cast<int>(a);
}

}  // namespace

void PilotControls::update(const ActionState& in, const AircraftProfile& profile,
                           const AircraftState& state, double dtS, ControlInputs& c,
                           MessageQueue* msg) {
    const float dt = static_cast<float>(dtS);
    auto pressed = [&in](Action a) { return in.pressed[idx(a)]; };
    auto down = [&in](Action a) { return in.down[idx(a)]; };
    auto say = [msg](const char* text) {
        if (msg) msg->post(text);
    };

    // ---- sticks and steering ----
    // P3: per-axis user expo then sensitivity (defaults are the identity).
    auto tune = [](float x, float sens, float expo) {
        x = std::clamp(x, -1.0f, 1.0f);
        return std::clamp(input::applyExpo(x, expo) * sens, -1.0f, 1.0f);
    };
    c.elevatorNorm = tune(in.axis[idx(Axis::Pitch)], stick_.pitchSens, stick_.pitchExpo);
    c.aileronNorm = tune(in.axis[idx(Axis::Roll)], stick_.rollSens, stick_.rollExpo);
    c.rudderNorm = tune(in.axis[idx(Axis::Rudder)], stick_.rudderSens, stick_.rudderExpo);
    c.steeringNorm = c.rudderNorm;  // nosewheel steering follows the rudder

    // ---- throttle rate ----
    const float throttleDelta =
        in.axis[idx(Axis::ThrottleRate)] * tuning_.throttleFullRatePerS * dt;
    if (stick_.throttleAbsolute) {  // P3: stick position = throttle
        const float pos = 0.5f * (std::clamp(in.axis[idx(Axis::ThrottleRate)], -1.0f, 1.0f) + 1.0f);
        const int engines = std::min(profile.engineCount, kMaxEngines);
        for (int i = 0; i < engines; ++i) c.throttleNorm[i] = pos;
    } else if (throttleDelta != 0.0f) {
        const int engines = std::min(profile.engineCount, kMaxEngines);
        for (int i = 0; i < engines; ++i)
            c.throttleNorm[i] = std::clamp(c.throttleNorm[i] + throttleDelta, 0.0f, 1.0f);
    }

    // ---- trims (held) ----
    if (down(Action::TrimNoseUp) != down(Action::TrimNoseDown)) {
        const float d = (down(Action::TrimNoseUp) ? 1.0f : -1.0f) * tuning_.trimRatePerS * dt;
        c.elevTrimNorm = std::clamp(c.elevTrimNorm + d, -1.0f, 1.0f);
    }
    if (pressed(Action::TrimNoseUp)) say("TRIM NOSE UP");
    if (pressed(Action::TrimNoseDown)) say("TRIM NOSE DOWN");
    if (down(Action::RudderTrimLeft) != down(Action::RudderTrimRight)) {
        const float d = (down(Action::RudderTrimRight) ? 1.0f : -1.0f) * tuning_.trimRatePerS * dt;
        c.rudderTrimNorm = std::clamp(c.rudderTrimNorm + d, -1.0f, 1.0f);
    }
    if (pressed(Action::RudderTrimLeft)) say("RUDDER TRIM LEFT");
    if (pressed(Action::RudderTrimRight)) say("RUDDER TRIM RIGHT");

    // ---- flaps: one detent per press, bounded by the profile ----
    const int detents = static_cast<int>(profile.flapDetents.size());
    if (pressed(Action::FlapsUp) || pressed(Action::FlapsDown)) {
        if (detents <= 1) {
            say("NO FLAPS");
        } else {
            int step = 0;
            if (pressed(Action::FlapsDown)) ++step;
            if (pressed(Action::FlapsUp)) --step;
            c.flapDetent = std::clamp(c.flapDetent + step, 0, detents - 1);
            if (msg)
                msg->postf("FLAPS %s",
                           profile.flapDetents[static_cast<size_t>(c.flapDetent)].label);
        }
    }
    if (detents > 0) c.flapDetent = std::clamp(c.flapDetent, 0, detents - 1);

    // ---- gear: locked while any wheel carries weight ----
    if (pressed(Action::GearToggle)) {
        if (!profile.hasRetractableGear) {
            say("GEAR FIXED");
        } else if (state.wowAny) {
            say("GEAR LOCKED ON GROUND");
        } else {
            c.gearDown = !c.gearDown;
            say(c.gearDown ? "GEAR DOWN" : "GEAR UP");
        }
    }

    // ---- speedbrake: retracted -> 50 % -> 100 % -> retracted ----
    if (pressed(Action::SpeedbrakeCycle)) {
        if (!profile.hasSpeedbrake) {
            say("NO SPEEDBRAKE");
        } else if (c.speedbrakeNorm < 0.25f) {
            c.speedbrakeNorm = 0.5f;
            say("SPEEDBRAKE 50%");
        } else if (c.speedbrakeNorm < 0.75f) {
            c.speedbrakeNorm = 1.0f;
            say("SPEEDBRAKE FULL");
        } else {
            c.speedbrakeNorm = 0.0f;
            say("SPEEDBRAKE RETRACTED");
        }
    }

    // ---- wheel brakes (edges only, so scripted brakes are not overwritten) ----
    const bool brakesDown = down(Action::Brakes);
    if (brakesDown != brakesWereDown_) {
        c.brakeLeftNorm = c.brakeRightNorm = brakesDown ? 1.0f : 0.0f;
        brakesWereDown_ = brakesDown;
    }
    if (pressed(Action::ParkingBrake)) {
        c.parkingBrake = !c.parkingBrake;
        say(c.parkingBrake ? "PARKING BRAKE ON" : "PARKING BRAKE OFF");
    }

    // ---- thrust reversers: only on the ground with the throttles at idle ----
    float maxThrottle = 0.0f;
    for (int i = 0; i < std::min(profile.engineCount, kMaxEngines); ++i)
        maxThrottle = std::max(maxThrottle, c.throttleNorm[i]);
    if (pressed(Action::ReverseToggle)) {
        if (!profile.hasReversers) {
            say("NO REVERSERS");
        } else if (c.reverseThrust) {
            c.reverseThrust = false;
            say("REVERSERS STOWED");
        } else if (state.wowAny && maxThrottle <= tuning_.reverserMaxThrottleNorm) {
            c.reverseThrust = true;
            say("REVERSERS DEPLOYED");
        } else {
            say("REVERSERS: GROUND+IDLE ONLY");
        }
    }
    if (c.reverseThrust && state.valid && !state.wowAny) {  // never fly with them out
        c.reverseThrust = false;
        say("REVERSERS STOWED");
    }
}

}  // namespace fsim
