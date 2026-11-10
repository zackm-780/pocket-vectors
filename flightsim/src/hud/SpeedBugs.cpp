#include "hud/SpeedBugs.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fsim {
namespace {

constexpr float kTakeoffThrottleNorm = 0.5f;
constexpr float kTakeoffEndAglFt = 3000.0f;
constexpr float kApproachBelowVrefFactor = 1.6f;
constexpr float kDescentVsFpm = -300.0f;

int detentCount(const AircraftProfile& p) {
    return static_cast<int>(p.flapDetents.size());
}

void add(SpeedBugSet& set, SpeedBugKind kind, float kt, const char* label) {
    if (kt <= 0.0f || set.count >= kMaxSpeedBugs) return;
    SpeedBug& b = set.bugs[set.count++];
    b.kind = kind;
    b.kt = kt;
    std::snprintf(b.label, sizeof b.label, "%s", label);
}

void addFlap(SpeedBugSet& set, const AircraftProfile& p, int detent, float k) {
    if (detent < 0 || detent >= detentCount(p)) return;
    add(set, SpeedBugKind::Flap, flapBugKt(p, detent, k),
        p.flapDetents[static_cast<size_t>(detent)].label);
}

}  // namespace

float speedBugWeightScale(float grossWeightLbs, float refWeightLbs) {
    if (grossWeightLbs <= 0.0f || refWeightLbs <= 0.0f) return 1.0f;
    return std::clamp(std::sqrt(grossWeightLbs / refWeightLbs), 0.8f, 1.25f);
}

float stallSpeedForDetentKt(const AircraftProfile& p, int detent) {
    const int n = detentCount(p);
    if (n == 0) return p.vs1Kt > 0 ? p.vs1Kt : p.vs0Kt;
    const int d = std::clamp(detent, 0, n - 1);
    if (static_cast<int>(p.vsByFlapKt.size()) == n) return p.vsByFlapKt[static_cast<size_t>(d)];
    const float vs1 = p.vs1Kt > 0 ? p.vs1Kt : p.vs0Kt;
    const float vs0 = p.vs0Kt > 0 ? p.vs0Kt : p.vs1Kt;
    if (vs1 <= 0.0f) return 0.0f;
    const float c0 = p.flapDetents.front().cmdNorm, cl = p.flapDetents.back().cmdNorm;
    const float t = cl > c0 ? (p.flapDetents[static_cast<size_t>(d)].cmdNorm - c0) / (cl - c0) : 0;
    return vs1 + (vs0 - vs1) * std::clamp(t, 0.0f, 1.0f);
}

float flapBugKt(const AircraftProfile& p, int detent, float k) {
    const float vs = stallSpeedForDetentKt(p, detent);
    if (vs <= 0.0f) return 0.0f;
    float kt = 1.3f * vs * k;
    const int next = detent + 1;
    if (next < detentCount(p)) {
        const float vfeNext = p.flapDetents[static_cast<size_t>(next)].vfeKt > 0
                                  ? p.flapDetents[static_cast<size_t>(next)].vfeKt
                                  : p.vfeKt;
        if (vfeNext > 0.0f) kt = std::min(kt, vfeNext);
    }
    return kt;
}

VSpeeds computeVSpeeds(const AircraftProfile& p, float grossWeightLbs) {
    VSpeeds v;
    v.k = speedBugWeightScale(grossWeightLbs, p.refWeightLbs);
    v.vrKt = p.vrKt * v.k;
    if (p.category != AircraftCategory::Piston && v.vrKt > 0.0f) {
        v.v1Kt = v.vrKt - 5.0f;
        v.v2Kt =
            std::max(1.2f * stallSpeedForDetentKt(p, p.takeoffFlapDetent) * v.k, v.vrKt + 3.0f);
    }
    v.vrefKt = p.vrefKt * v.k;
    v.vappKt = p.vappKt * v.k;
    return v;
}

const char* speedBugPhaseName(SpeedBugPhase phase) {
    switch (phase) {
    case SpeedBugPhase::Takeoff: return "TAKEOFF";
    case SpeedBugPhase::Descent: return "DESCENT";
    case SpeedBugPhase::Approach: return "APPROACH";
    default: return "OTHER";
    }
}

SpeedBugSet speedBugsFor(SpeedBugPhase phase, const AircraftState& s, const AircraftProfile& p) {
    SpeedBugSet set;
    const VSpeeds v = computeVSpeeds(p, s.grossWeightLbs);
    const int n = detentCount(p);
    const int d = n > 0 ? std::clamp(s.flapDetent, 0, n - 1) : 0;
    switch (phase) {
    case SpeedBugPhase::Takeoff:
        if (s.wowAny) {
            add(set, SpeedBugKind::V1, v.v1Kt, "V1");
            add(set, SpeedBugKind::Vr, v.vrKt, "VR");
        }
        add(set, SpeedBugKind::V2, v.v2Kt, "V2");
        if (d >= 1) addFlap(set, p, d - 1, v.k);  // where to retract to the next lower detent
        break;
    case SpeedBugPhase::Approach:
        add(set, SpeedBugKind::Ref, v.vrefKt, "REF");
        add(set, SpeedBugKind::App, v.vappKt, "APP");
        addFlap(set, p, d, v.k);
        addFlap(set, p, d + 1, v.k);
        break;
    case SpeedBugPhase::Descent:
        addFlap(set, p, d, v.k);
        addFlap(set, p, d + 1, v.k);
        break;
    case SpeedBugPhase::Other:
        if (d >= 1) addFlap(set, p, d, v.k);
        break;
    }
    return set;
}

const SpeedBugSet& SpeedBugTracker::update(const AircraftState& s, const ControlInputs& c,
                                           const AircraftProfile& p, bool approachLoaded,
                                           bool hide) {
    float maxThrottle = 0.0f;
    for (int i = 0; i < std::min(p.engineCount, kMaxEngines); ++i)
        maxThrottle = std::max(maxThrottle, c.throttleNorm[i]);
    if (s.wowAny) {
        // On the runway with takeoff power; anything else on the ground (taxi, after landing, a
        // rejected takeoff) shows only a flap bug.
        phase_ =
            maxThrottle >= kTakeoffThrottleNorm ? SpeedBugPhase::Takeoff : SpeedBugPhase::Other;
    } else {
        if (phase_ == SpeedBugPhase::Takeoff) {
            const bool flapsUp = s.flapDetent == 0 && s.flapPosNorm < 0.02f;
            if (flapsUp || s.altAglFt > kTakeoffEndAglFt) phase_ = SpeedBugPhase::Other;
        }
        if (phase_ != SpeedBugPhase::Takeoff) {
            const float k = speedBugWeightScale(s.grossWeightLbs, p.refWeightLbs);
            const float vref = p.vrefKt * k;
            // The speed alone means an approach only while descending (a slow climb is not one).
            const bool slowDescending =
                vref > 0 && s.iasKt < kApproachBelowVrefFactor * vref && s.vsFpm < 0.0f;
            if (s.flapDetent > 0 || approachLoaded || slowDescending)
                phase_ = SpeedBugPhase::Approach;
            else if (s.vsFpm < kDescentVsFpm)
                phase_ = SpeedBugPhase::Descent;
            else
                phase_ = SpeedBugPhase::Other;
        }
    }
    if (hide)
        set_ = SpeedBugSet{};
    else
        set_ = speedBugsFor(phase_, s, p);
    return set_;
}

void SpeedBugTracker::reset() {
    phase_ = SpeedBugPhase::Other;
    set_ = SpeedBugSet{};
}

const SpeedBug* nextSpeedBug(const SpeedBugSet& set, float iasKt, float trend10sKt) {
    const SpeedBug* best = nullptr;
    for (int i = 0; i < set.count; ++i) {
        const SpeedBug& b = set.bugs[i];
        if (trend10sKt > 1.0f && b.kt > iasKt + 0.5f && (!best || b.kt < best->kt)) best = &b;
        if (trend10sKt < -1.0f && b.kt < iasKt - 0.5f && (!best || b.kt > best->kt)) best = &b;
    }
    return best;
}

}  // namespace fsim
