#pragma once
// H9: V-speed and flap bugs on the speed tape (HUD.md section 3). Pure logic, no allocation:
// the speeds for the present weight, which bugs belong to the phase of flight, and the next bug
// ahead for the chase view. The HUD draws them in drawSpeedTape.
//
// Speeds (kt IAS), all scaled with k = sqrt(gross weight / the weight the profile's speeds were
// measured at), 1 when either is unknown, clamped to 0.8 .. 1.25:
//   VR = vr k. Jets and turboprops: V1 = VR - 5, V2 = max(1.2 Vs(takeoff detent) k, VR + 3).
//   Pistons show VR only on takeoff.
//   REF = vref k, APP = vapp k.
//   Flap bug of detent d = 1.3 Vs(d) k (its minimum manoeuvring speed), never above the next
//   detent's Vfe (that is where the next notch goes in), labelled with the detent's label.
// Vs(d) is the profile's vs_by_flap (F5) or, without it, vs1 at the first detent to vs0 at the last,
// interpolated by the detent's command.

#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/ControlInputs.h"

namespace fsim {

float speedBugWeightScale(float grossWeightLbs, float refWeightLbs);
float stallSpeedForDetentKt(const AircraftProfile& p, int detent);  // 0 = unknown
float flapBugKt(const AircraftProfile& p, int detent, float k);     // 0 = none

struct VSpeeds {
    float k = 1;
    float v1Kt = 0, vrKt = 0, v2Kt = 0;  // v1 / v2 = 0 for pistons
    float vrefKt = 0, vappKt = 0;
};
VSpeeds computeVSpeeds(const AircraftProfile& p, float grossWeightLbs);

enum class SpeedBugKind { V1, Vr, V2, Ref, App, Flap };
struct SpeedBug {
    SpeedBugKind kind = SpeedBugKind::Flap;
    float kt = 0;
    char label[8] = {0};  // "V1", "VR", "V2", "REF", "APP", or the flap detent's label
};
constexpr int kMaxSpeedBugs = 8;
struct SpeedBugSet {
    SpeedBug bugs[kMaxSpeedBugs];
    int count = 0;
};

// The phase that picks the bugs (a small latched state machine).
//   Takeoff: on the ground with the throttle at 50 % or more, and after liftoff until the flaps
//            are up or 3,000 ft AGL. V1 and VR until liftoff, V2 until the flaps are up, and the
//            flap bug of the next lower detent (when to retract).
//   Approach: airborne, not Takeoff, and the flaps out, an approach loaded or below 1.6 x Vref.
//            REF, APP, the current detent's flap bug and the next one down.
//   Descent: airborne, neither of those, VS below -300 fpm: the current detent's flap bug (UP =
//            the minimum clean speed) and the next one down (owner, 2026-11-09).
//   Other (cruise, climb, taxi, after landing): the current detent's flap bug while flaps are out.
enum class SpeedBugPhase { Other, Takeoff, Descent, Approach };
const char* speedBugPhaseName(SpeedBugPhase phase);

class SpeedBugTracker {
public:
    // Updates the phase from this frame's state; returns the bugs to draw (none when hide).
    const SpeedBugSet& update(const AircraftState& s, const ControlInputs& c,
                              const AircraftProfile& p, bool approachLoaded, bool hide);
    SpeedBugPhase phase() const { return phase_; }
    const SpeedBugSet& bugs() const { return set_; }
    void reset();

private:
    SpeedBugPhase phase_ = SpeedBugPhase::Other;
    SpeedBugSet set_;
};

// The bugs for a phase and state (what the tracker returns; exposed for tests).
SpeedBugSet speedBugsFor(SpeedBugPhase phase, const AircraftState& s, const AircraftProfile& p);

// The next bug ahead in the direction the speed is moving (trend > +1 kt per 10 s: the lowest bug
// above the speed; < -1: the highest below), nullptr when none.
const SpeedBug* nextSpeedBug(const SpeedBugSet& set, float iasKt, float trend10sKt);

}  // namespace fsim
