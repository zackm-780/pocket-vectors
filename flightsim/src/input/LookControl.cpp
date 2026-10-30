#include "input/LookControl.h"

#include <algorithm>
#include <cmath>

#include "core/Units.h"

namespace fsim {
namespace {

constexpr int idx(Action a) {
    return static_cast<int>(a);
}
constexpr int idx(Axis a) {
    return static_cast<int>(a);
}

}  // namespace

void LookControl::reset() {
    yawDeg_ = pitchDeg_ = 0;
    targetYawDeg_ = targetPitchDeg_ = 0;
}

void LookControl::update(const ActionState& in, double dtS) {
    const float dt = static_cast<float>(std::max(dtS, 0.0));
    if (in.down[idx(Action::LookHold)]) {
        if (in.pressed[idx(Action::LookLeft)]) {
            targetYawDeg_ = -tuning_.snapSideDeg;
            targetPitchDeg_ = 0;
        }
        if (in.pressed[idx(Action::LookRight)]) {
            targetYawDeg_ = tuning_.snapSideDeg;
            targetPitchDeg_ = 0;
        }
        if (in.pressed[idx(Action::LookBack)]) {
            // Turn the short way: from a left look go round to the left.
            targetYawDeg_ = yawDeg_ < 0 ? -180.0f : 180.0f;
            targetPitchDeg_ = 0;
        }
        if (in.pressed[idx(Action::LookDown)]) {
            targetYawDeg_ = 0;
            targetPitchDeg_ = tuning_.overNoseDeg;
        }
        // The stick moves the target. A back look may slew toward the front, never further back.
        const float yawLimitDeg =
            std::min(180.0f, std::max(tuning_.maxYawDeg, std::fabs(targetYawDeg_)));
        targetYawDeg_ =
            std::clamp(targetYawDeg_ + in.axis[idx(Axis::LookYaw)] * tuning_.slewRateDegps * dt,
                       -yawLimitDeg, yawLimitDeg);
        targetPitchDeg_ =
            std::clamp(targetPitchDeg_ + in.axis[idx(Axis::LookPitch)] * tuning_.slewRateDegps * dt,
                       tuning_.minPitchDeg, tuning_.maxPitchDeg);
    } else {
        targetYawDeg_ = targetPitchDeg_ = 0;
    }

    // Exponential ease (frame-rate independent) with a minimum rate, so it lands exactly.
    const float k = tuning_.easeTauS > 0 ? 1.0f - std::exp(-dt / tuning_.easeTauS) : 1.0f;
    auto ease = [&](float& valueDeg, float targetDeg) {
        const float diff = targetDeg - valueDeg;
        const float step = std::max(std::fabs(diff) * k, tuning_.minEaseRateDegps * dt);
        valueDeg = std::fabs(diff) <= step ? targetDeg : valueDeg + std::copysign(step, diff);
    };
    ease(yawDeg_, targetYawDeg_);
    ease(pitchDeg_, targetPitchDeg_);
}

float LookControl::yawRad() const {
    return static_cast<float>(degToRad(yawDeg_));
}

float LookControl::pitchRad() const {
    return static_cast<float>(degToRad(pitchDeg_));
}

bool LookControl::active() const {
    return yawDeg_ != 0.0f || pitchDeg_ != 0.0f || targetYawDeg_ != 0.0f || targetPitchDeg_ != 0.0f;
}

}  // namespace fsim
