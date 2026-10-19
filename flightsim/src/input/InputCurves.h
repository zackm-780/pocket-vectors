#pragma once
// Pure axis math for the input system (no SDL): normalisation, deadzones, expo, the right-stick
// dominance filter and the held-button step acceleration. Unit-tested in tests/input.

#include <algorithm>
#include <cmath>

namespace fsim {
namespace input {

// SDL axis value (-32768..32767) to -1..1.
inline float normalizeSdlAxis(int raw) {
    return std::clamp(static_cast<float>(raw) / 32767.0f, -1.0f, 1.0f);
}

// Expo blend: 0 = linear, 1 = pure cubic. Keeps +-1 and 0 fixed and the sign.
inline float applyExpo(float x, float expo) {
    return (1.0f - expo) * x + expo * x * x * x;
}

// Single-axis deadzone with rescale, so the output is continuous from 0 at the edge to 1.
inline float applyDeadzone(float x, float deadzone) {
    const float a = std::fabs(x);
    if (a <= deadzone) return 0.0f;
    const float scaled = std::min((a - deadzone) / (1.0f - deadzone), 1.0f);
    return x < 0 ? -scaled : scaled;
}

// Radial deadzone for a two-axis stick (x, y modified in place) followed by expo on the radius.
inline void applyRadialStick(float& x, float& y, float deadzone, float expo) {
    const float mag = std::sqrt(x * x + y * y);
    if (mag <= deadzone) {
        x = y = 0.0f;
        return;
    }
    const float clamped = std::min(mag, 1.0f);
    const float scaled = applyExpo((clamped - deadzone) / (1.0f - deadzone), expo);
    const float k = scaled / mag;
    x *= k;
    y *= k;
}

// Right-stick axis-dominance filter: when one axis is `ratio` times larger than the other the
// weaker one is zeroed (mostly vertical -> no rudder, mostly horizontal -> no throttle change).
inline void applyDominance(float& x, float& y, float ratio) {
    const float ax = std::fabs(x), ay = std::fabs(y);
    if (ay > ratio * ax) {
        x = 0.0f;
    } else if (ax > ratio * ay) {
        y = 0.0f;
    }
}

// Auto-repeat interval for the n-th repeat (n = 0 is the first repeat after the initial delay):
// intervalS * decay^n, never below minIntervalS.
inline float repeatIntervalS(int n, float intervalS, float minIntervalS, float decay) {
    return std::max(minIntervalS, intervalS * std::pow(decay, static_cast<float>(n)));
}

// Step size for a held bug button: baseStep for the first `rampStart` repeats, then growing
// linearly to maxStep at `rampEnd` repeats (e.g. heading bug 1 deg -> 10 deg).
inline float acceleratedStep(int repeatCount, float baseStep, float maxStep, int rampStart = 5,
                             int rampEnd = 25) {
    if (repeatCount <= rampStart) return baseStep;
    if (repeatCount >= rampEnd) return maxStep;
    const float t = static_cast<float>(repeatCount - rampStart) / (rampEnd - rampStart);
    return baseStep + t * (maxStep - baseStep);
}

}  // namespace input
}  // namespace fsim
