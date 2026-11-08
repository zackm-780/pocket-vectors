#include "autopilot/Autopilot.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/Geo.h"
#include "core/Units.h"

namespace fsim {

namespace {

// X1: altitude bug, capture, hold and minimums are all on the indicated (baro) altitude, the
// one the HUD tape shows. With the altimeter on QNH it equals the true altitude.
inline float altIndFt(const AircraftState& ac) {
    return static_cast<float>(ac.altMslFt + ac.altBaroOffsetFt);
}

constexpr float kG = static_cast<float>(kStdGravityMps2);
constexpr float kFpmPerKtPerRad = 101.2686f;  // fpm of VS per kt of speed per unit of sin(gamma)
constexpr float kFpmPerKtPerDeg = 1.76748f;   // = kFpmPerKtPerRad * pi / 180
constexpr float kGsFullScaleDeg = 0.7f;       // NAVIGATION 3.2
constexpr float kPathDeg = 3.0f;              // GS / VNAV path angle
constexpr float kWarnDurationS = 5.0f;        // HUD.md 5: flashing AP OFF
constexpr float kGsCaptureBand = 0.5f;        // 1 dot = half of full-scale deflection
// N7 speed law: VS change (fpm) per kt of TAS per kt/s of IAS acceleration, from the energy
// balance dh/dt + (V / g) dV/dt = const: 0.5144^2 / 9.80665 * 196.85 = 5.311.
constexpr float kFpmPerKtPerKtps = 5.311f;
constexpr float kSpdLawMaxVsFpm = 8000.f;  // FLC / GA VS target clamp (pitch limits still apply)
constexpr float kGaBugMinAboveFt = 500.f, kGaBugAboveFt = 1000.f;
constexpr float kGaSinkArrestKt = 5.f;

float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}
float wrap180f(float deg) {
    return static_cast<float>(wrap180(deg));
}
float slew(float cur, float target, float ratePerS, double dtS) {
    const float step = ratePerS * static_cast<float>(dtS);
    return cur + clampf(target - cur, -step, step);
}
float roundTo(float v, float step) {
    return std::round(v / step) * step;
}
float rollDegOf(const AircraftState& ac) {
    return static_cast<float>(radToDeg(ac.rollRad));
}
float pitchDegOf(const AircraftState& ac) {
    return static_cast<float>(radToDeg(ac.pitchRad));
}
float headingDegOf(const AircraftState& ac) {
    return static_cast<float>(wrap360(radToDeg(ac.headingTrueRad)));
}
// Step size for a held bug button: baseStep for the first repeats, growing to maxStep.
float acceleratedStep(int repeatCount, float baseStep, float maxStep) {
    constexpr int kRampStart = 5, kRampEnd = 25;
    if (repeatCount <= kRampStart) return baseStep;
    if (repeatCount >= kRampEnd) return maxStep;
    const float t = static_cast<float>(repeatCount - kRampStart) / (kRampEnd - kRampStart);
    return baseStep + t * (maxStep - baseStep);
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// configuration and state

void Autopilot::configure(const AircraftProfile& profile) {
    cfg_ = ApConfig::fromProfile(profile);
    hasAt_ = profile.hasAutothrottle || cfg_.atAvailable > 0.5f;
    engineCount_ = std::max(1, profile.engineCount);
    approachFlapDetent_ = profile.approachFlapDetent;
    gaFlapsMinKt_ = profile.vrefKt > 0 ? profile.vrefKt : 1.3f * profile.vs0Kt;
    st_ = AutopilotState{};
    rollHoldDeg_ = pitchHoldDeg_ = holdAltFt_ = captureAltFt_ = 0;
    vsBugRawFpm_ = spdBugRaw_ = 0;
    bankCmdDeg_ = pitchCmdDeg_ = 0;
    vsIntegPitchDeg_ = 0;
    loopsInit_ = false;
    elevInteg_ = ailInteg_ = 0;
    apTrimNorm_ = lastSyncTrim_ = 0;
    stickOverS_ = 0;
    atInit_ = atSynced_ = trimSynced_ = false;
    atInteg_ = atThrottle_ = lastSyncThrottle_ = lastIasKt_ = iasAccelKtps_ = 0;
    warnTimerS_ = 0;
    vsFresh_ = true;
    bugAccumVs_ = bugAccumSpd_ = 0;
    retardLatched_ = false;
    lastAc_ = AircraftState{};
    gaSpeedKt_ = 0;
    iasAccelInit_ = thrustWas_ = false;
    apLastIasKt_ = apIasAccelKtps_ = 0;
}

void Autopilot::initModes(const AircraftState& ac) {
    st_.latActive = LatMode::Roll;
    st_.latArmed = LatMode::Off;
    st_.vertActive = VertMode::Pitch;
    st_.vertArmed = VertMode::Off;
    selectRoll(ac);
    selectPitch(ac);
}

void Autopilot::clearModes() {
    st_.latActive = st_.latArmed = LatMode::Off;
    st_.vertActive = st_.vertArmed = VertMode::Off;
    st_.fdValid = false;
    st_.fdPitchCmdDeg = st_.fdRollCmdDeg = 0;
}

bool Autopilot::engageAp(const AircraftState& ac, const NavState&) {
    if (st_.apEngaged) return true;
    if (!ac.valid || ac.wowAny || ac.crashed || ac.stallWarning) return false;
    st_.apEngaged = true;
    st_.disconnectWarning = false;
    loopsInit_ = false;
    if (!st_.fdOn || st_.latActive == LatMode::Off) initModes(ac);
    return true;
}

void Autopilot::disengageAp(bool warn) {
    if (!st_.apEngaged) return;
    st_.apEngaged = false;
    loopsInit_ = false;
    trimSynced_ = false;
    stickOverS_ = 0;
    if (warn) {
        st_.disconnectWarning = true;
        warnTimerS_ = 0;
    }
    if (!st_.fdOn) clearModes();
}

bool Autopilot::toggleAp(const AircraftState& ac, const NavState& nav) {
    if (st_.apEngaged) {
        disengageAp(true);
        return false;
    }
    return engageAp(ac, nav);
}

void Autopilot::setFd(bool on, const AircraftState& ac) {
    if (on == st_.fdOn) return;
    st_.fdOn = on;
    if (on && st_.latActive == LatMode::Off) initModes(ac);
    if (!on && !st_.apEngaged) clearModes();
}

bool Autopilot::setAt(bool on, const AircraftState& ac, const AircraftProfile*) {
    if (!on) {
        st_.atEngaged = false;
        st_.spdActive = SpdMode::Off;
        st_.thrustTargetNorm = 0;
        atInit_ = atSynced_ = retardLatched_ = thrustWas_ = false;
        return true;
    }
    if (!hasAt_ || ac.wowAny || !ac.valid) return false;
    st_.atEngaged = true;
    st_.spdActive = SpdMode::Speed;
    atInit_ = atSynced_ = retardLatched_ = false;
    if (st_.spdBugKt <= 0) setSpdBugKt(std::round(ac.iasKt));
    return true;
}

void Autopilot::selectRoll(const AircraftState& ac) {
    if (!modesLive()) return;
    st_.latActive = LatMode::Roll;
    const float bank = rollDegOf(ac);
    rollHoldDeg_ = std::fabs(bank) <= cfg_.rollHoldWingsLevelDeg
                       ? 0.f
                       : clampf(bank, -cfg_.bankLimitDeg, cfg_.bankLimitDeg);
}

void Autopilot::selectPitch(const AircraftState& ac) {
    if (!modesLive()) return;
    st_.vertActive = VertMode::Pitch;
    pitchHoldDeg_ = clampf(pitchDegOf(ac), cfg_.pitchMinDeg, cfg_.pitchMaxDeg);
    vsFresh_ = true;
}

void Autopilot::selectHdg(const AircraftState& ac) {
    if (!modesLive()) return;
    if (st_.latActive == LatMode::Hdg) {
        selectRoll(ac);
    } else {
        st_.latActive = LatMode::Hdg;
    }
    if (st_.latArmed == LatMode::Loc || st_.latArmed == LatMode::Nav) st_.latArmed = LatMode::Off;
    if (st_.vertArmed == VertMode::Gs) st_.vertArmed = VertMode::Off;
}

void Autopilot::selectNav(const NavState& nav) {
    if (!modesLive()) return;
    if (st_.latActive == LatMode::Nav || st_.latArmed == LatMode::Nav) {
        st_.latArmed = LatMode::Off;
        if (st_.latActive == LatMode::Nav) selectRoll(lastAc_);
        return;
    }
    if (!navAvailable(nav)) return;
    st_.latArmed = LatMode::Nav;
    if (st_.vertArmed == VertMode::Gs) st_.vertArmed = VertMode::Off;
}

void Autopilot::selectApr(const NavState& nav) {
    if (!modesLive()) return;
    if (st_.latArmed == LatMode::Loc || st_.vertArmed == VertMode::Gs ||
        st_.latActive == LatMode::Loc) {
        if (st_.latArmed == LatMode::Loc) st_.latArmed = LatMode::Off;
        if (st_.vertArmed == VertMode::Gs) st_.vertArmed = VertMode::Off;
        if (st_.latActive == LatMode::Loc) selectRoll(lastAc_);
        return;
    }
    const bool ils = nav.locValid || nav.source == NavSource::Loc ||
                     std::strncmp(nav.approachName, "ILS", 3) == 0 ||
                     std::strncmp(nav.approachName, "LOC", 3) == 0;
    if (ils) {
        st_.latArmed = LatMode::Loc;
        const bool hasGs = nav.gsValid || std::strncmp(nav.approachName, "ILS", 3) == 0 ||
                           nav.source == NavSource::Loc;
        st_.vertArmed = hasGs ? VertMode::Gs : VertMode::Off;
    } else if (nav.approachName[0] != 0 && navAvailable(nav)) {
        st_.latArmed = LatMode::Nav;  // RNAV / VOR approach: GPS course, vertical stays manual
    }
}

void Autopilot::selectAltHold(const AircraftState& ac) {
    if (!modesLive()) return;
    st_.vertActive = VertMode::Alt;
    holdAltFt_ = altIndFt(ac);
    if (st_.vertArmed == VertMode::Alt) st_.vertArmed = VertMode::Off;
}

void Autopilot::selectAltButton(const AircraftState& ac) {
    if (!modesLive()) return;
    const bool holding = st_.vertActive == VertMode::Alt || st_.vertActive == VertMode::AltCapture;
    const float toBugFt = st_.altBugFt - altIndFt(ac);
    if (holding && std::fabs(toBugFt) > cfg_.altCaptureMinFt) {
        setVsBugFpm((toBugFt > 0.f ? 1.f : -1.f) * cfg_.vsDefaultFpm);
        st_.vertActive = VertMode::Vs;
        st_.vertArmed = VertMode::Alt;
        vsFresh_ = true;
        return;
    }
    selectAltHold(ac);
}

void Autopilot::selectVs(const AircraftState& ac) {
    if (!modesLive()) return;
    float vs = roundTo(ac.vsFpm, 100.f);
    if (std::fabs(vs) < 200.f) {
        const float toBug = st_.altBugFt - altIndFt(ac);
        vs = (toBug < -100.f ? -1.f : 1.f) * cfg_.vsDefaultFpm;
    }
    setVsBugFpm(vs);
    st_.vertActive = VertMode::Vs;
    vsFresh_ = true;
}

bool Autopilot::selectGoAround(const AircraftState& ac, const NavState& nav) {
    if (!ac.valid || ac.wowAny || ac.crashed) return false;
    magVarDeg_ = nav.magVarDeg;
    if (!modesLive()) setFd(true, ac);
    if (st_.vertActive == VertMode::Ga) return true;  // a second press changes nothing
    // Lateral: hold the present heading; NAV arms on the plan (the navigator sequences into the
    // missed approach once the MAP is passed airborne).
    const float magDeg = static_cast<float>(wrap360(radToDeg(ac.headingTrueRad) - nav.magVarDeg));
    setHdgBugMagDeg(std::round(magDeg));
    st_.latActive = LatMode::Hdg;
    st_.latArmed = nav.activeLegIndex >= 0 && navAvailable(nav) ? LatMode::Nav : LatMode::Off;
    // Vertical: pitch on speed, no GS arm left over.
    st_.vertActive = VertMode::Ga;
    st_.vertArmed = VertMode::Off;
    // Clean, 1.25 Vs1 (minClimbIasKt); with flaps out that is far too fast (190 kt in the 737),
    // so Vref is the floor there.
    gaSpeedKt_ = std::max(ac.iasKt, ac.flapDetent > 0 ? gaFlapsMinKt_ : cfg_.minClimbIasKt);
    vsFresh_ = true;
    const float altFt = altIndFt(ac);
    if (st_.altBugFt < altFt + kGaBugMinAboveFt) setAltBugFt(roundTo(altFt + kGaBugAboveFt, 100.f));
    if (hasAt_ && !st_.atEngaged) setAt(true, ac);  // TOGA thrust (updateAutothrottle)
    return true;
}

void Autopilot::selectFlc(const AircraftState& ac) {
    if (!modesLive()) return;
    if (st_.vertActive == VertMode::Flc) {
        selectPitch(ac);
        return;
    }
    if (std::fabs(st_.altBugFt - altIndFt(ac)) <= cfg_.altCaptureMinFt) {
        st_.vertActive = VertMode::Alt;  // already there: hold the bug
        holdAltFt_ = st_.altBugFt;
        if (st_.vertArmed == VertMode::Alt) st_.vertArmed = VertMode::Off;
        return;
    }
    if (st_.spdBugKt <= 0) setSpdBugKt(std::round(ac.iasKt));
    st_.vertActive = VertMode::Flc;
    vsFresh_ = true;
}

void Autopilot::setHdgBugMagDeg(float deg) {
    float d = std::fmod(deg, 360.f);
    if (d <= 0.f) d += 360.f;
    st_.hdgBugMagDeg = d;
}
void Autopilot::setAltBugFt(float ft) {
    st_.altBugFt = clampf(ft, 0.f, 60000.f);
}
void Autopilot::setVsBugFpm(float fpm) {
    st_.vsBugFpm = clampf(fpm, -cfg_.vsMaxDescentFpm, cfg_.vsMaxClimbFpm);
}
void Autopilot::setSpdBugKt(float kt) {
    st_.spdBugKt = clampf(kt, cfg_.spdBugMinKt, cfg_.spdBugMaxKt);
}

void Autopilot::syncBugs(const AircraftState& ac, const NavState& nav) {
    const float magDeg = static_cast<float>(wrap360(radToDeg(ac.headingTrueRad) - nav.magVarDeg));
    setHdgBugMagDeg(std::round(magDeg));
    setAltBugFt(roundTo(altIndFt(ac), 100.f));
}

// ---------------------------------------------------------------------------------------------
// actions and scenario

void Autopilot::handleActions(const ActionState& a, const AircraftState& ac, const NavState& nav,
                              double dtS) {
    auto pressed = [&](Action act) { return a.pressed[static_cast<int>(act)]; };
    auto repeats = [&](Action act) { return a.repeatCount[static_cast<int>(act)]; };
    magVarDeg_ = nav.magVarDeg;

    if (pressed(Action::ApToggle)) {
        if (!st_.apEngaged && st_.disconnectWarning)
            st_.disconnectWarning = false;  // acknowledge (HUD.md 5), does not re-engage
        else
            toggleAp(ac, nav);
    }
    if (pressed(Action::AtToggle)) setAt(!st_.atEngaged, ac);
    if (pressed(Action::FdToggle)) setFd(!st_.fdOn, ac);
    // A mode button with the AP and the FD both off turns the FD on first (ROL / PIT), like
    // TO/GA, so the button selects its mode instead of doing nothing. 1.2 device note: L1 + R2
    // (ALT) only worked once the AP was on.
    const bool modeButton = pressed(Action::HdgMode) || pressed(Action::NavMode) ||
                            pressed(Action::AprMode) || pressed(Action::AltMode) ||
                            pressed(Action::VsMode) || pressed(Action::FlcMode);
    if (modeButton && !modesLive() && ac.valid && !ac.crashed) setFd(true, ac);
    if (pressed(Action::HdgMode)) selectHdg(ac);
    if (pressed(Action::NavMode)) selectNav(nav);
    if (pressed(Action::AprMode)) selectApr(nav);
    if (pressed(Action::AltMode)) selectAltButton(ac);
    if (pressed(Action::VsMode)) selectVs(ac);
    if (pressed(Action::SyncBugs)) syncBugs(ac, nav);
    if (pressed(Action::GoAround)) selectGoAround(ac, nav);
    if (pressed(Action::FlcMode)) selectFlc(ac);

    if (pressed(Action::HdgBugInc))
        setHdgBugMagDeg(st_.hdgBugMagDeg + acceleratedStep(repeats(Action::HdgBugInc), 1, 10));
    if (pressed(Action::HdgBugDec))
        setHdgBugMagDeg(st_.hdgBugMagDeg - acceleratedStep(repeats(Action::HdgBugDec), 1, 10));
    if (pressed(Action::AltBugInc))
        setAltBugFt(roundTo(st_.altBugFt, 100.f) +
                    acceleratedStep(repeats(Action::AltBugInc), 100, 1000));
    if (pressed(Action::AltBugDec))
        setAltBugFt(roundTo(st_.altBugFt, 100.f) -
                    acceleratedStep(repeats(Action::AltBugDec), 100, 1000));

    // L1 + right stick Y: VS bug in VS mode, else speed bug with the A/T on (rate based).
    const float bugAxis = a.axis[static_cast<int>(Axis::BugRate)];
    if (std::fabs(bugAxis) > 0.1f) {
        const float rate = bugAxis * std::fabs(bugAxis);
        if (st_.vertActive == VertMode::Vs) {
            bugAccumVs_ += rate * 5.f * static_cast<float>(dtS);  // up to 5 steps (500 fpm) per s
            const float whole = std::trunc(bugAccumVs_);
            if (whole != 0.f) {
                setVsBugFpm(roundTo(st_.vsBugFpm, 100.f) + whole * 100.f);
                bugAccumVs_ -= whole;
            }
        } else if (st_.atEngaged) {
            bugAccumSpd_ += rate * 10.f * static_cast<float>(dtS);  // up to 10 kt per s
            const float whole = std::trunc(bugAccumSpd_);
            if (whole != 0.f) {
                setSpdBugKt(st_.spdBugKt + whole);
                bugAccumSpd_ -= whole;
            }
        }
    } else {
        bugAccumVs_ = bugAccumSpd_ = 0;
    }
}

void Autopilot::applyScenario(const std::vector<std::string>& tokens, const AircraftState& ac,
                              const NavState& nav) {
    lastAc_ = ac;
    magVarDeg_ = nav.magVarDeg;
    for (const std::string& t : tokens) {
        const bool modeToken = t == "hdg" || t == "nav" || t == "apr" || t == "alt" || t == "vs" ||
                               t == "rol" || t == "pit" || t == "pitch" || t == "flc";
        if (t == "ap" || modeToken) engageAp(ac, nav);
        if (t == "fd") setFd(true, ac);
        if (t == "at" || t == "spd") setAt(true, ac);
        if (t == "hdg") {
            if (st_.latActive != LatMode::Hdg) selectHdg(ac);
        } else if (t == "nav") {
            selectNav(nav);
        } else if (t == "apr") {
            selectApr(nav);
        } else if (t == "alt") {
            // ALT: fly to the altitude bug (VS toward it, capture), or hold when already there.
            const float err = st_.altBugFt - altIndFt(ac);
            if (std::fabs(err) > cfg_.altCaptureMinFt) {
                setVsBugFpm((err > 0 ? 1.f : -1.f) * cfg_.vsDefaultFpm);
                st_.vertActive = VertMode::Vs;
                vsFresh_ = true;
            } else {
                selectAltHold(ac);
                holdAltFt_ = st_.altBugFt;
            }
        } else if (t == "vs") {
            if (std::fabs(st_.vsBugFpm) >= 100.f) {
                st_.vertActive = VertMode::Vs;
                vsFresh_ = true;
            } else {
                selectVs(ac);
            }
        } else if (t == "rol") {
            selectRoll(ac);
        } else if (t == "pit" || t == "pitch") {
            selectPitch(ac);
        } else if (t == "flc") {
            if (st_.vertActive != VertMode::Flc) selectFlc(ac);
        } else if (t == "toga") {
            selectGoAround(ac, nav);  // FD only unless "ap" is given too
        }
    }
}

void Autopilot::syncPilot(ControlInputs& pilot) {
    if (st_.atEngaged && atInit_) {
        const int n = std::min(kMaxEngines, std::max(lastAc_.engineCount, engineCount_));
        for (int i = 0; i < n; ++i) pilot.throttleNorm[i] = atThrottle_;
        lastSyncThrottle_ = atThrottle_;
        atSynced_ = true;
    }
    if (st_.apEngaged && loopsInit_) {
        pilot.elevTrimNorm = apTrimNorm_;
        lastSyncTrim_ = apTrimNorm_;
        trimSynced_ = true;
    }
}

// ---------------------------------------------------------------------------------------------
// the 120 Hz step

ControlInputs Autopilot::update(const ControlInputs& pilot, const AircraftState& ac,
                                const NavState& nav, double dtS) {
    lastAc_ = ac;
    magVarDeg_ = nav.magVarDeg;
    ControlInputs out = pilot;

    updateIasAccel(ac, dtS);
    checkDisconnects(pilot, ac, nav, dtS);

    if (modesLive()) {
        updateModes(ac, nav);
        const float bankTarget =
            clampf(lateralBankCmd(ac, nav), -cfg_.bankLimitDeg, cfg_.bankLimitDeg);
        float pitchTarget =
            clampf(verticalPitchCmd(ac, nav, dtS), cfg_.pitchMinDeg, cfg_.pitchMaxDeg);
        // Angle-of-attack protection: never ask for more pitch while close to the stall warning.
        if (cfg_.alphaLimitDeg > 0.f && ac.alphaDeg > cfg_.alphaLimitDeg) {
            pitchTarget =
                std::min(pitchTarget, pitchDegOf(ac) - 1.5f * (ac.alphaDeg - cfg_.alphaLimitDeg));
            vsIntegPitchDeg_ = std::min(vsIntegPitchDeg_, pitchTarget);  // no windup
        }
        st_.fdRollCmdDeg = bankTarget;
        st_.fdPitchCmdDeg = pitchTarget;
        st_.fdValid = st_.latActive != LatMode::Off && st_.vertActive != VertMode::Off;
        if (st_.apEngaged) {
            if (!loopsInit_) initLoops(pilot, ac);
            bankCmdDeg_ = slew(bankCmdDeg_, bankTarget, cfg_.bankRateLimitDegps, dtS);
            const float pitchRate = st_.vertActive == VertMode::Ga
                                        ? std::max(cfg_.pitchRateLimitDegps, cfg_.gaPitchRateDegps)
                                        : cfg_.pitchRateLimitDegps;
            pitchCmdDeg_ = slew(pitchCmdDeg_, pitchTarget, pitchRate, dtS);
            innerLoops(pilot, ac, dtS, out);
        }
    } else {
        st_.fdValid = false;
    }
    if (!st_.apEngaged) loopsInit_ = false;

    updateAutothrottle(pilot, ac, dtS, out);
    return out;
}

void Autopilot::initLoops(const ControlInputs& pilot, const AircraftState& ac) {
    loopsInit_ = true;
    bankCmdDeg_ = rollDegOf(ac);
    pitchCmdDeg_ = pitchDegOf(ac);
    // Bumpless start: whatever the pilot was holding becomes the integrator, trim carries over.
    elevInteg_ = clampf(pilot.elevatorNorm, -0.5f, 0.5f);
    ailInteg_ = clampf(pilot.aileronNorm, -0.3f, 0.3f);
    apTrimNorm_ = pilot.elevTrimNorm;
    lastSyncTrim_ = apTrimNorm_;
    trimSynced_ = false;
    stickOverS_ = 0;
}

void Autopilot::checkDisconnects(const ControlInputs& pilot, const AircraftState& ac,
                                 const NavState& nav, double dtS) {
    if (st_.disconnectWarning) {
        warnTimerS_ += static_cast<float>(dtS);
        if (warnTimerS_ >= kWarnDurationS) st_.disconnectWarning = false;
    }
    if (!st_.apEngaged) {
        stickOverS_ = 0;
        return;
    }
    if (ac.wowAny || ac.crashed) {
        disengageAp(false);
        return;
    }
    // Pilot override: stick beyond the threshold for long enough.
    if (std::fabs(pilot.aileronNorm) > cfg_.stickDisconnect ||
        std::fabs(pilot.elevatorNorm) > cfg_.stickDisconnect)
        stickOverS_ += static_cast<float>(dtS);
    else
        stickOverS_ = 0;
    if (stickOverS_ >= cfg_.stickDisconnectS) {
        disengageAp(true);
        return;
    }
    if (ac.stallWarning || std::fabs(rollDegOf(ac)) > cfg_.bankDisconnectDeg ||
        std::fabs(pitchDegOf(ac)) > cfg_.pitchDisconnectDeg) {
        disengageAp(true);
        return;
    }
    // No autoland in v1: disconnect below minimums - 50 ft while flying an approach. Not in GA:
    // TO/GA below minimums keeps the AP flying the go-around (N7).
    const bool coupled =
        st_.vertActive != VertMode::Ga &&
        (st_.latActive == LatMode::Loc || st_.vertActive == VertMode::Gs || nav.onFinal);
    if (nav.phase == NavPhase::Approach && coupled && nav.minimumsFt > 0 &&
        altIndFt(ac) < nav.minimumsFt - cfg_.minimumsDisconnectMarginFt)
        disengageAp(true);
}

// ---------------------------------------------------------------------------------------------
// mode logic (arming / capture / reversion)

bool Autopilot::navAvailable(const NavState& nav) const {
    return nav.source != NavSource::None && (nav.activeLegIndex >= 0 || nav.locValid);
}

float Autopilot::currentTrackDeg(const AircraftState& ac) const {
    return ac.gsKt > 25.f ? static_cast<float>(wrap360(radToDeg(ac.trackTrueRad)))
                          : headingDegOf(ac);
}

// True when the aircraft should turn onto the course now: close enough to it (turn-radius lead)
// and closing. loc additionally needs the needle inside full scale.
bool Autopilot::captureOk(const AircraftState& ac, const NavState& nav, bool loc) const {
    if (loc && std::fabs(nav.cdiDeflection) > 1.0f) return false;
    const float xtkNm = nav.crossTrackNm;
    const float relDeg = wrap180f(currentTrackDeg(ac) - nav.desiredTrackTrueDeg);
    if (std::fabs(relDeg) > 135.f) return false;
    if (std::fabs(xtkNm) < 0.1f) return std::fabs(relDeg) < 90.f;
    const float closing = -xtkNm * std::sin(static_cast<float>(degToRad(relDeg)));
    if (closing <= 0.f) return false;  // moving away from the course
    const float vMps = std::max(20.f, static_cast<float>(ktToMps(ac.gsKt)));
    const float tanBank = std::tan(static_cast<float>(degToRad(cfg_.bankLimitDeg)));
    const float radiusNm = static_cast<float>(mToNm(vMps * vMps / (kG * tanBank)));
    const float leadNm =
        radiusNm * (1.f - std::cos(static_cast<float>(degToRad(std::fabs(relDeg)))));
    return std::fabs(xtkNm) <= leadNm + 0.1f;
}

void Autopilot::updateModes(const AircraftState& ac, const NavState& nav) {
    // ---- lateral ----
    // 1.2.3: NAV armed over ROL captures at once (airborne): ROL has no heading to intercept on,
    // and the NAV law flies its own intercept (<= max_intercept_deg). Before, ROL (NAV) held
    // the wings level until a closing intercept that never came, so a pilot who followed the FD
    // (a mode button turns it on in ROL) flew away from the GPS track. Over HDG (the pilot's
    // intercept heading, or TO/GA) NAV stays armed until captureOk. No capture on the ground.
    const bool navCapture =
        !ac.wowAny && (st_.latActive == LatMode::Roll || captureOk(ac, nav, false));
    if (st_.latArmed == LatMode::Nav && navAvailable(nav) && navCapture) {
        st_.latActive = LatMode::Nav;
        st_.latArmed = LatMode::Off;
    } else if (st_.latArmed == LatMode::Loc && nav.source == NavSource::Loc && nav.locValid &&
               captureOk(ac, nav, true)) {
        st_.latActive = LatMode::Loc;
        st_.latArmed = LatMode::Off;
    }
    if (st_.latActive == LatMode::Nav && !navAvailable(nav)) selectRoll(ac);
    if (st_.latActive == LatMode::Loc && !(nav.source == NavSource::Loc && nav.locValid)) {
        if (nav.phase == NavPhase::Missed && navAvailable(nav))
            st_.latActive = LatMode::Nav;
        else
            selectRoll(ac);
    }

    // ---- vertical ----
    if (st_.vertArmed == VertMode::Gs && st_.latActive == LatMode::Loc && nav.gsValid &&
        std::fabs(nav.gsDeflection) <= kGsCaptureBand) {
        st_.vertActive = VertMode::Gs;
        st_.vertArmed = VertMode::Off;
        vsFresh_ = true;  // restart the vertical PI with the path feed-forward
    }
    if (st_.vertActive == VertMode::Gs && (!nav.gsValid || st_.latActive != LatMode::Loc))
        selectPitch(ac);

    const float altErr = st_.altBugFt - altIndFt(ac);
    // N7: FLC and GA capture the bug like PIT and VS (GA ends at ALT*; the A/T goes back to SPD).
    const bool capturing = st_.vertActive == VertMode::Pitch || st_.vertActive == VertMode::Vs ||
                           st_.vertActive == VertMode::Flc || st_.vertActive == VertMode::Ga;
    if (capturing) {
        const float vs = ac.vsFpm;
        const bool toward = vs * altErr > 0.f && std::fabs(vs) >= 100.f;
        const float captureFt =
            std::max(cfg_.altCaptureMinFt, std::fabs(vs) / cfg_.altCaptureGainFpmPerFt);
        const bool nearAlready = std::fabs(altErr) <= cfg_.altCaptureMinFt && std::fabs(vs) < 300.f;
        if ((toward && std::fabs(altErr) <= captureFt) || nearAlready) {
            st_.vertActive = VertMode::AltCapture;
            captureAltFt_ = st_.altBugFt;
        } else if (st_.vertArmed != VertMode::Gs) {
            const bool modeToward =
                (st_.vertActive == VertMode::Vs && st_.vsBugFpm * altErr > 0.f) ||
                st_.vertActive == VertMode::Flc || (st_.vertActive == VertMode::Ga && altErr > 0.f);
            st_.vertArmed = (toward || modeToward) ? VertMode::Alt : VertMode::Off;
        }
    } else if (st_.vertActive == VertMode::Alt || st_.vertActive == VertMode::AltCapture) {
        if (st_.vertArmed == VertMode::Alt) st_.vertArmed = VertMode::Off;
    }
    if (st_.vertActive == VertMode::AltCapture &&
        std::fabs(captureAltFt_ - altIndFt(ac)) <= cfg_.altHoldWithinFt &&
        std::fabs(ac.vsFpm) < 400.f) {
        st_.vertActive = VertMode::Alt;
        holdAltFt_ = captureAltFt_;
    }
    if (st_.vertActive == VertMode::Off || st_.vertActive == VertMode::Pitch) vsFresh_ = true;
    if (nav.phase == NavPhase::Missed && st_.vertActive == VertMode::Gs) selectPitch(ac);
}

// ---------------------------------------------------------------------------------------------
// outer loops

float Autopilot::hdgBugTrueDeg(const NavState& nav) const {
    return static_cast<float>(wrap360(st_.hdgBugMagDeg + nav.magVarDeg));
}

// Turn-rate law: heading error closes with time constant tauS; the bank that gives that turn rate
// at the present speed (coordinated turn), clamped to the bank limit.
float Autopilot::turnLawBankDeg(const AircraftState& ac, float errDeg, float tauS) const {
    const float vMps = std::max(20.f, static_cast<float>(ktToMps(ac.tasKt)));
    const float omegaRadps = static_cast<float>(degToRad(errDeg)) / std::max(0.5f, tauS);
    const float bankRad = std::atan(vMps * omegaRadps / kG);
    return clampf(static_cast<float>(radToDeg(bankRad)), -cfg_.bankLimitDeg, cfg_.bankLimitDeg);
}

float Autopilot::lateralBankCmd(const AircraftState& ac, const NavState& nav) {
    switch (st_.latActive) {
    case LatMode::Off: return 0.f;
    case LatMode::Roll: return rollHoldDeg_;
    case LatMode::Hdg: {
        const float err = wrap180f(hdgBugTrueDeg(nav) - headingDegOf(ac));
        return turnLawBankDeg(ac, err, cfg_.headingTauS);
    }
    case LatMode::Nav:
    case LatMode::Loc: {
        // Intercept course = desired track steered by the cross-track error (look-ahead law).
        const bool loc = st_.latActive == LatMode::Loc;
        const float maxAngle = loc ? cfg_.locMaxInterceptDeg : cfg_.maxInterceptDeg;
        const float lookNm = std::max(0.2f, std::max(ac.gsKt, 60.f) / 3600.f * cfg_.navTauS);
        const float angle =
            clampf(static_cast<float>(radToDeg(std::atan2(nav.crossTrackNm, lookNm))), -maxAngle,
                   maxAngle);
        const float wantTrack = nav.desiredTrackTrueDeg - angle;
        const float err = wrap180f(wantTrack - currentTrackDeg(ac));
        return turnLawBankDeg(ac, err, cfg_.headingTauS);
    }
    }
    return 0.f;
}

float Autopilot::verticalPitchCmd(const AircraftState& ac, const NavState& nav, double dtS) {
    float vsTarget = 0.f;
    float maxPitchDeg = cfg_.pitchMaxDeg;
    switch (st_.vertActive) {
    case VertMode::Off: return pitchDegOf(ac);
    case VertMode::Pitch: return pitchHoldDeg_;
    case VertMode::Vs: vsTarget = st_.vsBugFpm; break;
    case VertMode::Alt:
    case VertMode::AltCapture: {
        const bool capture = st_.vertActive == VertMode::AltCapture;
        const float target = capture ? captureAltFt_ : holdAltFt_;
        const float gain = capture ? cfg_.altCaptureGainFpmPerFt : cfg_.altHoldGainFpmPerFt;
        vsTarget =
            clampf(gain * (target - altIndFt(ac)), -cfg_.vsMaxDescentFpm, cfg_.vsMaxClimbFpm);
        break;
    }
    case VertMode::Gs: {
        const float vsPath =
            -ac.gsKt * kFpmPerKtPerRad * std::tan(static_cast<float>(degToRad(kPathDeg)));
        const float distFt = static_cast<float>(nmToFt(std::max(0.5f, nav.distToThresholdNm)));
        const float heightErrFt =
            nav.gsDeflection * static_cast<float>(degToRad(kGsFullScaleDeg)) * distFt;
        const float corr =
            clampf(heightErrFt / cfg_.gsTauS * 60.f, -cfg_.gsMaxCorrFpm, cfg_.gsMaxCorrFpm);
        vsTarget = vsPath + corr;
        break;
    }
    case VertMode::Flc: {
        // Pitch on speed, but never away from the bug: no climb with the bug below, no descent
        // with it above (the speed then waits for the thrust, as in a real FLC).
        vsTarget = speedOnPitchVs(ac, st_.spdBugKt);
        if (st_.altBugFt > altIndFt(ac))
            vsTarget = std::max(vsTarget, 0.f);
        else
            vsTarget = std::min(vsTarget, 0.f);
        break;
    }
    case VertMode::Ga:
        vsTarget = speedOnPitchVs(ac, gaSpeedKt_);
        // Stop the descent first: while the speed is within kGaSinkArrestKt of the target the
        // GA never asks for a descent (the thrust is still spooling up).
        if (ac.iasKt >= gaSpeedKt_ - kGaSinkArrestKt) vsTarget = std::max(vsTarget, 0.f);
        maxPitchDeg = std::min(maxPitchDeg, cfg_.gaPitchDeg);
        break;
    }
    // Underspeed protection without an autothrottle: below the minimum speed trade altitude for
    // speed (100 fpm less climb, or more descent, per knot of deficit). GS is exempt (it descends),
    // FLC and GA fly pitch on speed already.
    const bool speedMode = st_.vertActive == VertMode::Flc || st_.vertActive == VertMode::Ga;
    if (!st_.atEngaged && st_.vertActive != VertMode::Gs && !speedMode &&
        cfg_.minClimbIasKt > 0.f && ac.iasKt < cfg_.minClimbIasKt)
        vsTarget -= (cfg_.minClimbIasKt - ac.iasKt) * 100.f;
    return std::min(vsLoopPitch(ac, vsTarget, dtS, maxPitchDeg), maxPitchDeg);
}

// N7 pitch on speed (TECS-like): the VS that turns the speed error into the acceleration
// (target - IAS) / spdTauS. The energy balance gives the VS change for an acceleration error;
// the TAS / IAS factor converts the IAS rate to a TAS rate. The VS loop then flies it.
float Autopilot::speedOnPitchVs(const AircraftState& ac, float targetKt) const {
    const float iasKt = std::max(ac.iasKt, 30.f);
    const float tasKt = std::max(ac.tasKt, iasKt);
    const float accelErrKtps =
        apIasAccelKtps_ + (ac.iasKt - targetKt) / std::max(1.f, cfg_.spdTauS);
    const float dVs = clampf(kFpmPerKtPerKtps * tasKt * (tasKt / iasKt) * accelErrKtps,
                             -cfg_.spdMaxDeltaVsFpm, cfg_.spdMaxDeltaVsFpm);
    return clampf(ac.vsFpm + dVs, -kSpdLawMaxVsFpm, kSpdLawMaxVsFpm);
}

void Autopilot::updateIasAccel(const AircraftState& ac, double dtS) {
    const float dt = static_cast<float>(dtS);
    if (!ac.valid || !iasAccelInit_ || dt <= 0.f) {
        apLastIasKt_ = ac.iasKt;
        apIasAccelKtps_ = 0;
        iasAccelInit_ = ac.valid;
        return;
    }
    const float accel = (ac.iasKt - apLastIasKt_) / dt;
    apIasAccelKtps_ +=
        clampf(dt / std::max(0.05f, cfg_.spdAccelTauS), 0.f, 1.f) * (accel - apIasAccelKtps_);
    apLastIasKt_ = ac.iasKt;
}

float Autopilot::vsLoopPitch(const AircraftState& ac, float vsTargetFpm, double dtS,
                             float maxPitchDeg) {
    const float tasKt = std::max(ac.tasKt, 40.f);
    const float degPerFpm = 1.f / (tasKt * kFpmPerKtPerDeg);  // path angle per fpm of VS
    if (vsFresh_) {
        // Bumpless start with a feed-forward step to the new flight path angle.
        const float dGamma = (vsTargetFpm - ac.vsFpm) * degPerFpm;
        vsIntegPitchDeg_ = clampf(pitchDegOf(ac) + dGamma, cfg_.pitchMinDeg, maxPitchDeg);
        vsFresh_ = false;
    }
    const float err = vsTargetFpm - ac.vsFpm;
    vsIntegPitchDeg_ =
        clampf(vsIntegPitchDeg_ + cfg_.vsKiRel * degPerFpm * err * static_cast<float>(dtS),
               cfg_.pitchMinDeg, maxPitchDeg);
    return vsIntegPitchDeg_ + cfg_.vsKpRel * degPerFpm * err;
}

// ---------------------------------------------------------------------------------------------
// inner loops

void Autopilot::innerLoops(const ControlInputs& pilot, const AircraftState& ac, double dtS,
                           ControlInputs& out) {
    const float dt = static_cast<float>(dtS);
    const float rollDeg = rollDegOf(ac), pitchDeg = pitchDegOf(ac);
    const float pDeg = static_cast<float>(radToDeg(ac.pRadps));
    const float qDeg = static_cast<float>(radToDeg(ac.qRadps));
    const float rDeg = static_cast<float>(radToDeg(ac.rRadps));

    // The pilot trimmed while the AP was flying: take the new value.
    if (trimSynced_ && std::fabs(pilot.elevTrimNorm - lastSyncTrim_) > 1e-4f)
        apTrimNorm_ = pilot.elevTrimNorm;

    // roll: bank -> aileron
    const float bankErr = bankCmdDeg_ - rollDeg;
    ailInteg_ = clampf(ailInteg_ + cfg_.rollKi * bankErr * dt, -0.3f, 0.3f);
    out.aileronNorm = clampf(cfg_.rollKp * bankErr - cfg_.rollKd * pDeg + ailInteg_, -1.f, 1.f);

    // pitch: attitude -> elevator, integrator offloaded into trim
    const float pitchErr = pitchCmdDeg_ - pitchDeg;
    elevInteg_ = clampf(elevInteg_ + cfg_.pitchKi * pitchErr * dt, -0.5f, 0.5f);
    const float trimStep = clampf(cfg_.trimGainPerS * elevInteg_ * dt, -cfg_.trimMaxRatePerS * dt,
                                  cfg_.trimMaxRatePerS * dt);
    const float newTrim = clampf(apTrimNorm_ + trimStep, -1.f, 1.f);
    elevInteg_ -= newTrim - apTrimNorm_;
    apTrimNorm_ = newTrim;
    out.elevatorNorm =
        clampf(cfg_.pitchKp * pitchErr - cfg_.pitchKd * qDeg + elevInteg_, -1.f, 1.f);
    out.elevTrimNorm = apTrimNorm_;

    // turn coordination: damp sideslip and the yaw rate that the bank does not explain
    if (ac.tasKt > 40.f) {
        const float vMps = static_cast<float>(ktToMps(ac.tasKt));
        const float rollRad = static_cast<float>(ac.rollRad);
        const float rCoordDeg =
            static_cast<float>(radToDeg(kG * std::tan(rollRad) / vMps * std::cos(rollRad)));
        const float rud =
            cfg_.rudderBetaGain * ac.betaDeg - cfg_.rudderRateGain * (rDeg - rCoordDeg);
        out.rudderNorm = clampf(pilot.rudderNorm + rud, -1.f, 1.f);
    }
}

// ---------------------------------------------------------------------------------------------
// autothrottle

void Autopilot::updateAutothrottle(const ControlInputs& pilot, const AircraftState& ac, double dtS,
                                   ControlInputs& out) {
    if (!st_.atEngaged) {
        atInit_ = atSynced_ = false;
        return;
    }
    const float dt = static_cast<float>(dtS);
    if (ac.wowAny || ac.crashed) {  // A/T disconnects at touchdown
        setAt(false, ac);
        return;
    }
    // Throttle lever moved by the pilot: A/T off, the pilot's setting stays.
    if (atSynced_ && std::fabs(pilot.throttleNorm[0] - lastSyncThrottle_) > 0.002f) {
        setAt(false, ac);
        return;
    }
    if (!atInit_) {
        atThrottle_ = pilot.throttleNorm[0];
        atInteg_ = atThrottle_;
        lastIasKt_ = ac.iasKt;
        iasAccelKtps_ = 0;
        atInit_ = true;
    }
    if (dt > 0.f) {
        const float accel = (ac.iasKt - lastIasKt_) / dt;
        iasAccelKtps_ += clampf(dt / 2.0f, 0.f, 1.f) * (accel - iasAccelKtps_);  // 2 s low-pass
        lastIasKt_ = ac.iasKt;
    }

    // N7: GA and FLC hold a fixed throttle (THR) while the AP / FD flies pitch on speed.
    const bool thrust =
        modesLive() && (st_.vertActive == VertMode::Ga || st_.vertActive == VertMode::Flc);

    // RETARD in the flare on landing (descending, landing configuration, below ~30 ft RA).
    const bool landingConfig =
        ac.flapDetent >= approachFlapDetent_ && (!ac.gearRetractable || ac.gearPosNorm > 0.9f);
    if (ac.altAglFt < cfg_.retardAglFt && ac.vsFpm < -50.f && landingConfig) retardLatched_ = true;
    if (ac.altAglFt > 2.f * cfg_.retardAglFt || st_.vertActive == VertMode::Ga)
        retardLatched_ = false;

    float target;
    if (retardLatched_) {
        st_.spdActive = SpdMode::Retard;
        st_.thrustTargetNorm = 0;
        target = 0.f;
        atThrottle_ = slew(atThrottle_, target, cfg_.retardSlewPerS, dtS);
        atInteg_ = atThrottle_;
    } else if (thrust) {
        st_.spdActive = SpdMode::Thrust;
        st_.thrustTargetNorm = thrustTargetNorm(ac);
        atThrottle_ = slew(atThrottle_, st_.thrustTargetNorm, cfg_.atSlewPerS, dtS);
        atInteg_ = atThrottle_;  // SPD takes over from here without a jump
        thrustWas_ = true;
    } else {
        if (thrustWas_) atInteg_ = atThrottle_;
        thrustWas_ = false;
        st_.thrustTargetNorm = 0;
        st_.spdActive = SpdMode::Speed;
        const float err = st_.spdBugKt - ac.iasKt;
        target = atInteg_ + cfg_.atKp * err - cfg_.atKAccel * iasAccelKtps_;
        const bool saturated = (target > 1.f && err > 0.f) || (target < 0.f && err < 0.f);
        if (!saturated) atInteg_ = clampf(atInteg_ + cfg_.atKi * err * dt, 0.f, 1.f);
        atThrottle_ = slew(atThrottle_, clampf(target, 0.f, 1.f), cfg_.atSlewPerS, dtS);
    }
    const int n = std::min(kMaxEngines, ac.engineCount > 0 ? ac.engineCount : engineCount_);
    for (int i = 0; i < n; ++i) out.throttleNorm[i] = atThrottle_;
}

float Autopilot::thrustTargetNorm(const AircraftState& ac) const {
    if (st_.vertActive == VertMode::Ga) return clampf(cfg_.gaThrottleNorm, 0.f, 1.f);
    return st_.altBugFt > altIndFt(ac) ? clampf(cfg_.climbThrottleNorm, 0.f, 1.f) : 0.f;
}

}  // namespace fsim
