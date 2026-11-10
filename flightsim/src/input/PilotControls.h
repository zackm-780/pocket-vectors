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
    float trimRatePerS = 0.1f;              // rudder trim travel per second while held (-1..1)
    float reverserMaxThrottleNorm = 0.05f;  // "idle" for the reverser rule
    // F9: elevator trim. A press moves it one step at once; held past trimHoldDelayS it moves
    // at the rate. Both shrink by (Vref / IAS)^2 above Vref (at least 1/4), so one press does
    // about the same to the flight path at any speed (airliners slow their trim with the flaps
    // up the same way). AircraftProfile::trimStepNorm / trimRateNormPerS override the defaults.
    float elevTrimStepNorm = 0.004f;
    float elevTrimRateNormPerS = 0.04f;
    float trimHoldDelayS = 0.3f;
    float trimSpeedScaleMin = 0.25f;
    // F9 aids (StickTuning::pitchDamper / autoTrim).
    // The input system has applied the deadzone and expo by now, so these are shaped values: the
    // damper fades out linearly from the centre to damperStickMax (about raw 0.3 in the C172,
    // 0.2 at the 0.6 default expo), and "centred" is inside the deadzone.
    float damperStickMax = 0.05f;
    float damperPerDegPsPiston = 0.010f;  // elevator per deg/s of pitch rate
    float damperPerDegPsJet = 0.008f;
    float autoTrimCentredS = 1.0f;  // stick centred this long, then the path is held
    float autoTrimStickMax = 0.002f;
    float autoTrimPerDegS = 0.002f;      // trim per second per degree of flight-path error
    float autoTrimDampPerDegPs = 0.06f;  // trim per second per deg/s of flight-path change
    float autoTrimMaxRateNormPerS = 0.01f;
};

// P3: user stick tuning from settings.ini, applied on top of the input system's deadzone/expo.
// sens scales the deflection (result clamped to -1..1); expo 0 = linear, 1 = pure cubic.
struct StickTuning {
    float pitchSens = 1.0f, rollSens = 1.0f, rudderSens = 1.0f;
    float pitchExpo = 0.0f, rollExpo = 0.0f, rudderExpo = 0.0f;
    // true: the throttle-rate axis is read as a position (stick up = full), not a rate.
    bool throttleAbsolute = false;
    // F9 aids, not realistic, off by default (settings PITCH DAMPER and AUTO TRIM):
    // pitchDamper adds -k * pitch rate to the elevator while the stick is near the centre;
    // autoTrim, with the stick centred for 1 s, trims to hold the flight-path angle it was left
    // at (off on the ground and while the AP is engaged; a trim press takes over).
    bool pitchDamper = false, autoTrim = false;
};

class PilotControls {
public:
    void setStickTuning(const StickTuning& tuning) { stick_ = tuning; }  // P3
    const StickTuning& stickTuning() const { return stick_; }

    explicit PilotControls(PilotTuning tuning = {}) : tuning_(tuning) {}
    PilotTuning& tuning() { return tuning_; }  // F9: tests tune the aids

    // `state` supplies weight-on-wheels (gear lockout, reversers). `messages` may be null.
    void update(const ActionState& actions, const AircraftProfile& profile,
                const AircraftState& state, double dtS, ControlInputs& controls,
                MessageQueue* messages = nullptr);
    // F9: the autopilot flies (and trims): auto trim stays out of its way. Set each frame.
    void setAutopilotEngaged(bool engaged) { apEngaged_ = engaged; }
    // F9: the elevator trim step and rate for this profile and speed (the speed factor applied).
    float elevTrimStepNorm(const AircraftProfile& profile, const AircraftState& state) const;
    float elevTrimRateNormPerS(const AircraftProfile& profile, const AircraftState& state) const;
    bool autoTrimHolding() const { return autoTrimHolding_; }
    // F9: a new flight: the trim press, the stick-centred timer and the auto-trim hold start over.
    void resetFlight();

private:
    float trimSpeedFactor(const AircraftProfile& profile, const AircraftState& state) const;

    PilotTuning tuning_;
    StickTuning stick_;  // P3
    bool brakesWereDown_ = false;
    // F9: elevator trim press and hold, and the aids.
    int trimDir_ = 0;
    float trimHeldS_ = 0.0f;
    bool apEngaged_ = false;
    float stickCentredS_ = 0.0f;
    bool autoTrimHolding_ = false;
    float autoTrimPathDeg_ = 0.0f;
    float lastPathDeg_ = 0.0f;
    bool havePath_ = false;
};

}  // namespace fsim
