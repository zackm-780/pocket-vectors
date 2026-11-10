#include "sim/SpeedbrakeArm.h"

#include <algorithm>

namespace fsim {

const char* SpeedbrakeArm::update(const AircraftState& s, const AircraftProfile& p,
                                  ControlInputs& c) {
    if (!p.hasSpeedbrake || !c.speedbrakeArmed) {
        airborne_ = deployed_ = false;  // disarmed (X, a manual position, a new flight)
        return nullptr;
    }
    if (!s.valid) return nullptr;
    if (!s.wowAny) airborne_ = true;
    float maxThrottle = 0.0f;
    for (int i = 0; i < std::min(p.engineCount, kMaxEngines); ++i)
        maxThrottle = std::max(maxThrottle, c.throttleNorm[i]);
    if (deployed_) {
        if (maxThrottle > kStowThrottleNorm) {  // go-around: stow and disarm
            c.speedbrakeNorm = 0.0f;
            c.speedbrakeArmed = false;
            airborne_ = deployed_ = false;
            return "SPEEDBRAKE STOWED";
        }
        c.speedbrakeNorm = 1.0f;
        return nullptr;
    }
    if (airborne_ && s.wowAny && maxThrottle <= kIdleThrottleNorm) {
        c.speedbrakeNorm = 1.0f;  // still armed
        deployed_ = true;
        return "SPEEDBRAKE DEPLOYED";
    }
    return nullptr;
}

}  // namespace fsim
