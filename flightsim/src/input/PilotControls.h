#pragma once
// Integrates the per-frame ActionState into the persistent pilot ControlInputs (CONTROLS.md
// section 2): stick axes, throttle rate, trims, flap detents, gear, speedbrake, brakes, parking
// brake, steering and thrust reversers. The ControlInputs object is owned by the caller and
// should start as FlightModel::startControls() so nothing jumps at the first step; fields that
// no action touches this frame (and scripted edits) are left alone.

#include "core/Actions.h"
#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/ControlInputs.h"
#include "core/MessageQueue.h"

namespace fsim {

struct PilotTuning {
    float throttleFullRatePerS = 0.4f;      // full right-stick deflection: 0 -> 100 % in 2.5 s
    float trimRatePerS = 0.1f;              // trim travel per second while held (-1..1 range)
    float reverserMaxThrottleNorm = 0.05f;  // "idle" for the reverser rule
};

class PilotControls {
public:
    explicit PilotControls(PilotTuning tuning = {}) : tuning_(tuning) {}

    // `state` supplies weight-on-wheels (gear lockout, reversers). `messages` may be null.
    void update(const ActionState& actions, const AircraftProfile& profile,
                const AircraftState& state, double dtS, ControlInputs& controls,
                MessageQueue* messages = nullptr);

private:
    PilotTuning tuning_;
    bool brakesWereDown_ = false;
};

}  // namespace fsim
