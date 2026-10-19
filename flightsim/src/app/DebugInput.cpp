#include "app/DebugInput.h"

#include <SDL.h>

#include <algorithm>
#include <cmath>

namespace fsim {
namespace {

constexpr float kStickRatePerS = 2.0f;    // deflection rate while a key is held
constexpr float kStickReturnPerS = 4.0f;  // spring-back rate on release
constexpr float kThrottleRatePerS = 0.5f;
constexpr float kTrimRatePerS = 0.05f;

// Moves `value` toward target at `rate`, never overshooting.
float approach(float value, float target, float rate, float dtS) {
    const float step = rate * dtS;
    if (value < target) return std::min(value + step, target);
    return std::max(value - step, target);
}

float axisFromKeys(float value, bool negative, bool positive, float dtS) {
    float target = 0;
    float rate = kStickReturnPerS;
    if (negative != positive) {
        target = positive ? 1.0f : -1.0f;
        rate = kStickRatePerS;
    }
    return approach(value, target, rate, dtS);
}

}  // namespace

void DebugInput::handleEvent(const SDL_Event& e) {
    if (e.type == SDL_QUIT) quit_ = true;
    if (e.type != SDL_KEYDOWN || e.key.repeat) return;
    switch (e.key.keysym.sym) {
    case SDLK_ESCAPE: quit_ = true; break;
    case SDLK_p: pauseToggle_ = !pauseToggle_; break;
    case SDLK_g: gearToggle_ = !gearToggle_; break;
    case SDLK_SPACE: parkingToggle_ = !parkingToggle_; break;
    case SDLK_LEFTBRACKET: --flapStep_; break;
    case SDLK_RIGHTBRACKET: ++flapStep_; break;
    default: break;
    }
}

bool DebugInput::consumePauseToggle() {
    const bool t = pauseToggle_;
    pauseToggle_ = false;
    return t;
}

void DebugInput::update(double dtS, int engineCount, int flapDetentCount, ControlInputs& c) {
    const float dt = static_cast<float>(dtS);
    const Uint8* k = SDL_GetKeyboardState(nullptr);

    // Down arrow = pull back = nose up (+ elevator).
    c.elevatorNorm = axisFromKeys(c.elevatorNorm, k[SDL_SCANCODE_UP], k[SDL_SCANCODE_DOWN], dt);
    c.aileronNorm = axisFromKeys(c.aileronNorm, k[SDL_SCANCODE_LEFT], k[SDL_SCANCODE_RIGHT], dt);
    c.rudderNorm = axisFromKeys(c.rudderNorm, k[SDL_SCANCODE_Q], k[SDL_SCANCODE_E], dt);
    c.steeringNorm = 0.5f * c.rudderNorm;  // gentle nosewheel steering

    float throttleDelta = 0;
    if (k[SDL_SCANCODE_PAGEUP] || k[SDL_SCANCODE_EQUALS] || k[SDL_SCANCODE_KP_PLUS])
        throttleDelta += kThrottleRatePerS * dt;
    if (k[SDL_SCANCODE_PAGEDOWN] || k[SDL_SCANCODE_MINUS] || k[SDL_SCANCODE_KP_MINUS])
        throttleDelta -= kThrottleRatePerS * dt;
    if (throttleDelta != 0.0f) {
        for (int i = 0; i < engineCount && i < kMaxEngines; ++i)
            c.throttleNorm[i] = std::clamp(c.throttleNorm[i] + throttleDelta, 0.0f, 1.0f);
    }

    if (k[SDL_SCANCODE_HOME]) c.elevTrimNorm = std::min(1.0f, c.elevTrimNorm + kTrimRatePerS * dt);
    if (k[SDL_SCANCODE_END]) c.elevTrimNorm = std::max(-1.0f, c.elevTrimNorm - kTrimRatePerS * dt);

    const bool brakeKey = k[SDL_SCANCODE_B] != 0;
    if (brakeKey != brakeKeyPrev_) {  // only on changes, so scripted brakes are not overwritten
        c.brakeLeftNorm = c.brakeRightNorm = brakeKey ? 1.0f : 0.0f;
        brakeKeyPrev_ = brakeKey;
    }
    if (parkingToggle_) {
        c.parkingBrake = !c.parkingBrake;
        parkingToggle_ = false;
    }

    if (flapStep_ != 0) {
        c.flapDetent = std::clamp(c.flapDetent + flapStep_, 0, std::max(0, flapDetentCount - 1));
        flapStep_ = 0;
    }
    if (gearToggle_) {
        c.gearDown = !c.gearDown;
        gearToggle_ = false;
    }
}

}  // namespace fsim
