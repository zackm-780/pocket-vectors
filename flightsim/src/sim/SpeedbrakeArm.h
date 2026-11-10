#pragma once
// F10: armed speed brakes. Armed (ControlInputs::speedbrakeArmed, the X cycle's ARMED position),
// they deploy fully when the main gear touches down with every thrust lever at idle, after the
// aircraft has been airborne since it was armed (arming on the ground does not deploy them on the
// spot). With the thrust above idle at touchdown they deploy the moment it comes back to idle.
// Deployed by the arm, any thrust lever above half stows them and disarms (a go-around). Pure
// logic, no allocation; Simulation runs it once per step on the pilot's controls.

#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/ControlInputs.h"

namespace fsim {

class SpeedbrakeArm {
public:
    static constexpr float kIdleThrottleNorm = 0.05f;
    static constexpr float kStowThrottleNorm = 0.5f;

    // Updates `controls` (speedbrakeNorm, speedbrakeArmed); returns the HUD message for what it
    // did this step ("SPEEDBRAKE DEPLOYED", "SPEEDBRAKE STOWED") or nullptr.
    const char* update(const AircraftState& state, const AircraftProfile& profile,
                       ControlInputs& controls);
    void reset() { *this = SpeedbrakeArm{}; }
    bool deployedByArm() const { return deployed_; }
    bool airborneSinceArmed() const { return airborne_; }

private:
    bool airborne_ = false;  // airborne at some point since the arm
    bool deployed_ = false;  // speedbrakes out because of the arm
};

}  // namespace fsim
