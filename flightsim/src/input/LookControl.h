#pragma once
// L2 look layer (CONTROLS.md section 2) -> camera look angles relative to the aircraft
// (yaw + = right, pitch + = up), as gfx::Camera::setPoseFromAircraft takes them.
//
// While LookHold is down the right stick slews the view (rate-based, so a released stick keeps
// the view where it is) and the D-pad snaps it: left 90, right 90, back, and down over the nose.
// When LookHold is released the view eases back to straight ahead.

#include "core/Actions.h"

namespace fsim {

struct LookTuning {
    float slewRateDegps = 150.0f;  // full right-stick deflection
    float maxYawDeg = 150.0f;      // free-look limit (the back snap goes to 180)
    float minPitchDeg = -60.0f;
    float maxPitchDeg = 45.0f;
    float snapSideDeg = 90.0f;
    float overNoseDeg = -25.0f;  // look down over the nose (landing view)
    // Ease toward the target: exponential with this time constant, but never slower than the
    // minimum rate, so a 90 degree snap or a recenter settles in about 0.3 s.
    float easeTauS = 0.08f;
    float minEaseRateDegps = 60.0f;
};

class LookControl {
public:
    explicit LookControl(LookTuning tuning = {}) : tuning_(tuning) {}

    void update(const ActionState& actions, double dtS);
    // Straight ahead at once (a new flight, a reset).
    void reset();

    float yawRad() const;
    float pitchRad() const;
    float yawDeg() const { return yawDeg_; }
    float pitchDeg() const { return pitchDeg_; }
    // True while the view is away from straight ahead (held, or still easing back).
    bool active() const;

private:
    LookTuning tuning_;
    float yawDeg_ = 0, pitchDeg_ = 0;
    float targetYawDeg_ = 0, targetPitchDeg_ = 0;
};

}  // namespace fsim
