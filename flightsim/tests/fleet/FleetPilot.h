#pragma once
// Scripted "pilot" for the fleet flyability tests (WP F2): small controllers that fly a takeoff,
// a climb, a stabilised 3 degree approach, a flare and a rollout on top of FlightModel, using only
// the public ControlInputs / AircraftState interface. The same controller code flies every
// aircraft; the per-aircraft numbers live in FleetSpec (FleetSpecs.h).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "FleetSpecs.h"
#include "core/Geo.h"
#include "core/Units.h"
#include "sim/FlightHarness.h"

namespace fsim::test::fleet {

constexpr double kDt = 1.0 / kSimHz;
constexpr double kFtPerKtMin = 101.2686;  // fpm of descent rate per knot of ground speed per 1 tan

inline double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

// Runway 27 of XTST: threshold and far end, for along- and cross-track measurements.
struct RunwayRef {
    LatLon thr, end;
    double elevFt = 0;
    double headingTrueDeg = 270;
    static RunwayRef xtst27(const Flight& f) {
        const Runway* r = findRunway(f.nav.airport(), "27");
        RunwayRef out;
        out.thr = {r->thrLatDeg, r->thrLonDeg};
        out.end = {r->endLatDeg, r->endLonDeg};
        out.elevFt = r->thrElevFt;
        out.headingTrueDeg = r->headingTrueDeg;
        return out;
    }
    // Distance past the threshold along the runway, feet (negative before it).
    double alongFt(const AircraftState& s) const {
        return mToFt(alongTrackM({s.latDeg, s.lonDeg}, thr, end));
    }
    // Right-of-centerline distance, feet.
    double crossFt(const AircraftState& s) const {
        return mToFt(crossTrackM({s.latDeg, s.lonDeg}, thr, end));
    }
};

// Pitch attitude controller driven by a vertical-speed or speed demand. Output is the elevator
// stick (the start trim stays in place), so the first call must follow a trimmed start.
struct PitchPilot {
    double pitchCmdDeg = 0;
    void sync(const AircraftState& s) { pitchCmdDeg = radToDeg(s.pitchRad); }

    // Hold a vertical speed: integrate the vs error into a pitch demand, then hold the pitch.
    float vsHold(const AircraftState& s, const FleetSpec& sp, double vsTargetFpm,
                 double pitchMinDeg, double pitchMaxDeg) {
        const double err = vsTargetFpm - s.vsFpm;
        pitchCmdDeg = clampd(pitchCmdDeg + sp.vsIntegral * err * kDt, pitchMinDeg, pitchMaxDeg);
        const double target =
            clampd(pitchCmdDeg + sp.vsProportional * err, pitchMinDeg, pitchMaxDeg);
        return pitchHoldElevator(s, target, sp.pitchKp, sp.pitchKd);
    }
    // Hold a pitch attitude directly.
    float pitchHold(const AircraftState& s, const FleetSpec& sp, double pitchDeg) {
        pitchCmdDeg = pitchDeg;
        return pitchHoldElevator(s, pitchDeg, sp.pitchKp, sp.pitchKd);
    }
};

// Throttle controller holding an indicated airspeed around a base throttle setting.
struct SpeedPilot {
    double base = 0.5, integ = 0;
    void sync(const ControlInputs& c) {
        base = c.throttleNorm[0];
        integ = 0;
    }
    double throttle(const AircraftState& s, const FleetSpec& sp, double targetKt) {
        const double err = targetKt - s.iasKt;
        integ = clampd(integ + sp.thrIntegral * err * kDt, -0.4, 0.4);
        return clampd(base + sp.thrProportional * err + integ, 0.0, 1.0);
    }
};

inline void setAllThrottles(Flight& f, double v) {
    for (int i = 0; i < f.s().engineCount; ++i) f.ctl.throttleNorm[i] = static_cast<float>(v);
}

// Wings level / heading hold (true heading), bank limited.
inline float headingBank(const AircraftState& s, double headingTrueDeg, double crossFt = 0) {
    // Cross-track error steers the commanded heading by up to 8 degrees back to the centerline.
    const double hdgCmd = headingTrueDeg - clampd(crossFt * 0.03, -8.0, 8.0);
    const double err = wrap180(hdgCmd - radToDeg(s.headingTrueRad));
    const double bank = clampd(err * 1.2, -15.0, 15.0);
    return bankHoldAileron(s, bank);
}

// ---- takeoff and climb -------------------------------------------------------------------------

struct TakeoffLog {
    double liftoffIasKt = -1, liftoffDistFt = 0, liftoffTimeS = 0, liftoffAlphaDeg = 0;
    double maxHeadingErrDeg = 0, maxPitchDeg = 0, maxAlphaDeg = 0;
    double timeTo3000S = -1, timeGearUpS = -1, timeFlapsUpS = -1;
    double minClimbMarginKt = 1e9;  // min of (IAS - flap retract speed) while below 400 ft AGL
    double stallWarnS = 0;          // seconds of stall warning after liftoff
    double distTo3000Nm = 0;
    bool flapsUp = false, gearUp = false, crashed = false;
    char crashReason[48] = {0};
};

// Takes off from the runway start (rwy 27) and climbs to targetAglFt. Returns when reached,
// crashed, or after maxTimeS.
inline TakeoffLog flyTakeoffClimb(Flight& f, const FleetSpec& sp, double targetAglFt,
                                  double maxTimeS) {
    TakeoffLog log;
    const RunwayRef rwy = RunwayRef::xtst27(f);
    const int nDet = static_cast<int>(f.profile.flapDetents.size());
    f.ctl.parkingBrake = false;
    setAllThrottles(f, 1.0);
    PitchPilot pp;
    pp.sync(f.s());
    double distFt = 0, lastFlapStepS = -10, liftedS = -1;
    int flapDet = f.ctl.flapDetent;
    for (long i = 0; i < static_cast<long>(maxTimeS * kSimHz); ++i) {
        const AircraftState& s = f.s();
        const double aglFt = s.altAglFt - sp.cgOnGearFt;  // height of the wheels above the runway
        const double hdgErr = std::fabs(wrap180(radToDeg(s.headingTrueRad) - rwy.headingTrueDeg));
        log.maxHeadingErrDeg = std::max(log.maxHeadingErrDeg, hdgErr);
        log.maxPitchDeg = std::max(log.maxPitchDeg, radToDeg(s.pitchRad));
        if (!s.wowAny) log.maxAlphaDeg = std::max(log.maxAlphaDeg, static_cast<double>(s.alphaDeg));

        if (s.wowAny && liftedS < 0) {
            // Ground roll: centerline with rudder and nose-wheel steering, rotate at Vr.
            const float rud = headingHoldRudder(s, rwy.headingTrueDeg, sp.groundKp, sp.groundKd);
            f.ctl.rudderNorm = rud;
            f.ctl.steeringNorm = rud;
            f.ctl.aileronNorm = bankHoldAileron(s);
            f.ctl.elevatorNorm = s.iasKt > sp.rotateKt ? pitchHoldElevator(s, sp.rotatePitchDeg,
                                                                           sp.rotateKp, sp.rotateKd)
                                                       : 0.f;
            pp.pitchCmdDeg = sp.rotatePitchDeg;
        } else {
            if (liftedS < 0) {
                liftedS = s.simTimeS;
                log.liftoffIasKt = s.iasKt;
                log.liftoffDistFt = distFt;
                log.liftoffTimeS = s.simTimeS;
                log.liftoffAlphaDeg = s.alphaDeg;
            }
            f.ctl.steeringNorm = 0;
            f.ctl.rudderNorm = sp.airRudderKp > 0 ? headingHoldRudder(s, rwy.headingTrueDeg,
                                                                     sp.airRudderKp, 0.02)
                                                  : 0.f;
            f.ctl.aileronNorm = headingBank(s, rwy.headingTrueDeg, rwy.crossFt(s));
            // Speed-for-pitch law: hold the climb speed, never below the minimum pitch.
            const bool clean = flapDet == 0;
            const double target = (aglFt < 400.0 && !clean) ? sp.climbKt : sp.cleanKt;
            const double pitch = clampd(sp.climbPitchDeg + sp.climbPitchPerKt * (s.iasKt - target),
                                        sp.climbPitchMinDeg, sp.climbPitchMaxDeg);
            f.ctl.elevatorNorm = pitchHoldElevator(s, pitch, sp.pitchKp, sp.pitchKd);
            if (s.wowAny == false) {
                if (s.simTimeS - liftedS > 12.0) log.stallWarnS += s.stallWarning ? kDt : 0.0;
                if (aglFt < 400.0 && flapDet > 0)
                    log.minClimbMarginKt = std::min(log.minClimbMarginKt,
                                                    static_cast<double>(s.iasKt) - sp.stallCleanKt);
            }
            // Gear up once safely climbing.
            if (f.profile.hasRetractableGear && f.ctl.gearDown && s.vsFpm > 200 && aglFt > 50.0) {
                f.ctl.gearDown = false;
            }
            if (!f.ctl.gearDown && s.gearPosNorm < 0.01f && log.timeGearUpS < 0)
                log.timeGearUpS = s.simTimeS - liftedS;
            // Flaps up one detent at a time above the retraction speed.
            if (flapDet > 0 && aglFt > 150.0 && s.iasKt > sp.flapRetractKt &&
                s.simTimeS - lastFlapStepS > sp.flapStepS) {
                --flapDet;
                f.ctl.flapDetent = flapDet;
                lastFlapStepS = s.simTimeS;
            }
            if (flapDet == 0 && s.flapPosNorm < 0.01f && log.timeFlapsUpS < 0)
                log.timeFlapsUpS = s.simTimeS - liftedS;
        }
        (void)nDet;
        if (std::getenv("FLEET_TRACE") && i % kSimHz == 0)
            std::printf("  t %.0f wow %d agl %.0f ias %.1f hdgerr %.1f pitch %.1f roll %.1f vs %.0f "
                        "flap %.2f gear %.2f\n",
                        s.simTimeS, s.wowAny, aglFt, s.iasKt, hdgErr, radToDeg(s.pitchRad),
                        radToDeg(s.rollRad), s.vsFpm, s.flapPosNorm, s.gearPosNorm);
        f.step();
        distFt += f.s().gsKt * 1.68781 * kDt;
        if (f.s().crashed) {
            log.crashed = true;
            std::snprintf(log.crashReason, sizeof log.crashReason, "%s", f.s().crashReason);
            break;
        }
        if (liftedS >= 0 && f.s().altAglFt - sp.cgOnGearFt >= targetAglFt) {
            log.timeTo3000S = f.s().simTimeS - liftedS;
            break;
        }
    }
    log.flapsUp = f.ctl.flapDetent == 0 && f.s().flapPosNorm < 0.02f;
    log.gearUp = !f.profile.hasRetractableGear || f.s().gearPosNorm < 0.02f;
    return log;
}

// ---- approach, flare and rollout ---------------------------------------------------------------

struct LandingLog {
    double maxPathErrFt = 0;  // |altitude - 3 degree path| from the start to flareAglFt
    double maxPathErrBelow500Ft = 0;
    double maxIasErrKt = 0;  // |IAS - Vapp| while on the approach
    double minAglMarginFt = 1e9;
    double stallWarnS = 0;
    double touchdownVsFpm = 0, touchdownIasKt = 0, touchdownAlongFt = -1, touchdownPitchDeg = 0;
    double touchdownBankDeg = 0, touchdownCgAglFt = 0;
    double stopAlongFt = -1;  // distance from the threshold to the stop
    double rollFt = 0;        // rollout distance from touchdown
    double stopTimeS = -1;    // seconds from touchdown to stop
    double maxCrossFt = 0, maxRolloutHeadingErrDeg = 0, finalHeadingErrDeg = 0;
    bool touched = false, stopped = false, crashed = false, reachedFlare = false;
    char crashReason[48] = {0};
    double apchEndTimeS = 0;
};

// Flies the approach from the current (trimmed Final) state. When stopAtAglFt > 0 the flight ends
// there (approach-only test); otherwise it flares, lands and rolls out to a stop.
inline LandingLog flyApproachLanding(Flight& f, const FleetSpec& sp, double stopAtAglFt,
                                     double maxTimeS = 400.0) {
    LandingLog log;
    const RunwayRef rwy = RunwayRef::xtst27(f);
    const double vappKt = f.profile.vappKt;
    const double tanGlide = std::tan(degToRad(3.0));
    PitchPilot pp;
    pp.sync(f.s());
    SpeedPilot sps;
    sps.sync(f.ctl);
    bool wasAir = true, flareStarted = false;
    double flarePitchStartDeg = 0, flareHStartFt = 1;
    double touchS = -1, lastWowAllS = -1;
    double reverseThr = 0;
    for (long i = 0; i < static_cast<long>(maxTimeS * kSimHz); ++i) {
        const AircraftState& s = f.s();
        const double hFt = s.altAglFt - sp.gearTouchAglFt;  // main wheel height above the runway
        const double alongFt = rwy.alongFt(s);
        const double crossFt = rwy.crossFt(s);
        const double hdgErr = wrap180(radToDeg(s.headingTrueRad) - rwy.headingTrueDeg);

        if (!s.wowAny) {
            // ---- airborne: path hold, then flare ----
            const double pathAltFt = rwy.elevFt + 50.0 - std::min(alongFt, 0.0) * tanGlide;
            const double altErrFt = pathAltFt - s.altMslFt;  // + = below the path
            const bool flare = hFt < sp.flareAglFt;
            if (!flare) {
                log.maxPathErrFt = std::max(log.maxPathErrFt, std::fabs(altErrFt));
                if (hFt < 500.0)
                    log.maxPathErrBelow500Ft =
                        std::max(log.maxPathErrBelow500Ft, std::fabs(altErrFt));
                log.maxIasErrKt = std::max(log.maxIasErrKt, std::fabs(s.iasKt - vappKt));
                const double gsFps = std::max(30.0, static_cast<double>(s.gsKt) * 1.68781);
                const double vsPath = -gsFps * tanGlide * 60.0 + sp.pathGainFpmPerFt * altErrFt;
                f.ctl.elevatorNorm =
                    pp.vsHold(s, sp, vsPath, sp.apchPitchMinDeg, sp.apchPitchMaxDeg);
                setAllThrottles(f, sps.throttle(s, sp, vappKt));
                f.ctl.aileronNorm = headingBank(s, rwy.headingTrueDeg, crossFt);
                f.ctl.rudderNorm = 0;
            } else {
                log.reachedFlare = true;
                // Exponential flare toward a small sink rate; power to idle.
                const double vsFpm = -(hFt / sp.flareTauS + sp.touchdownSinkFps) * 60.0;
                if (!flareStarted) {
                    flareStarted = true;
                    flarePitchStartDeg = radToDeg(s.pitchRad);
                    flareHStartFt = hFt;
                }
                if (sp.flarePitchDeltaDeg > 0) {
                    // Height-scheduled round-out: pitch up smoothly as the wheels near the runway.
                    const double frac = clampd(1.0 - hFt / flareHStartFt, 0.0, 1.0);
                    const double target = flarePitchStartDeg + sp.flarePitchDeltaDeg * frac +
                                          sp.flareVsGain * (vsFpm - s.vsFpm);
                    f.ctl.elevatorNorm = pitchHoldElevator(
                        s, clampd(target, sp.apchPitchMinDeg, sp.flarePitchMaxDeg), sp.pitchKp,
                        sp.pitchKd);
                } else {
                    f.ctl.elevatorNorm =
                        pp.vsHold(s, sp, vsFpm, sp.apchPitchMinDeg, sp.flarePitchMaxDeg);
                }
                if (hFt < sp.idleAglFt) {
                    for (int e = 0; e < s.engineCount; ++e)
                        f.ctl.throttleNorm[e] = std::max(
                            0.f, f.ctl.throttleNorm[e] - static_cast<float>(sp.idleRatePerS * kDt));
                } else {
                    setAllThrottles(f, sps.throttle(s, sp, vappKt));
                }
                f.ctl.aileronNorm = headingBank(s, rwy.headingTrueDeg, crossFt);
                f.ctl.rudderNorm = 0;
            }
            if (s.stallWarning) log.stallWarnS += kDt;
            if (stopAtAglFt > 0 && hFt <= stopAtAglFt) {
                log.apchEndTimeS = s.simTimeS;
                return log;
            }
        } else {
            // ---- on the ground ----
            if (wasAir && !log.touched) {
                log.touched = true;
                touchS = s.simTimeS;
                log.touchdownAlongFt = alongFt;
                log.touchdownIasKt = s.iasKt;
                log.touchdownPitchDeg = radToDeg(s.pitchRad);
                log.touchdownBankDeg = radToDeg(s.rollRad);
                log.touchdownCgAglFt = s.altAglFt;
            }
            if (s.wowAll && lastWowAllS < 0) lastWowAllS = s.simTimeS;
            const double sinceS = s.simTimeS - touchS;
            setAllThrottles(f, 0.0);
            f.ctl.reverseThrust = false;
            if (f.profile.hasReversers && s.gsKt > sp.reverseOffKt && sinceS > sp.reverseDelayS) {
                f.ctl.reverseThrust = true;
                reverseThr = std::min(sp.reverseThrottle, reverseThr + 0.5 * kDt);
                // The reverser latches only at idle: if it was lost (a wheel bounce), bring the
                // throttles back to idle and select reverse again.
                if (reverseThr > 0.45 && !s.engines[0].reverserDeployed) reverseThr = 0;
                setAllThrottles(f, reverseThr);
            } else {
                reverseThr = 0;
            }
            f.ctl.speedbrakeNorm = f.profile.hasSpeedbrake ? 1.f : 0.f;
            // Aircraft without spoilers dump lift by retracting the flaps once rolling straight.
            if (!f.profile.hasSpeedbrake && sinceS > 2.0) f.ctl.flapDetent = 0;
            // Pitch down to the runway, then hold the centerline.
            f.ctl.elevatorNorm = pitchHoldElevator(s, sp.rolloutPitchDeg, sp.pitchKp, sp.pitchKd);
            const float rud =
                headingHoldRudder(s, rwy.headingTrueDeg - clampd(crossFt * 0.05, -5.0, 5.0),
                                  sp.groundKp, sp.groundKd);
            f.ctl.rudderNorm = rud;
            f.ctl.steeringNorm = rud;
            f.ctl.aileronNorm = bankHoldAileron(s);
            const bool brake = sinceS > sp.brakeDelayS;
            const float b = brake ? static_cast<float>(sp.brakeNorm) : 0.f;
            f.ctl.brakeLeftNorm = b;
            f.ctl.brakeRightNorm = b;
            log.maxCrossFt = std::max(log.maxCrossFt, std::fabs(crossFt));
            log.maxRolloutHeadingErrDeg = std::max(log.maxRolloutHeadingErrDeg, std::fabs(hdgErr));
            if (s.gsKt < 1.0f) {
                log.stopped = true;
                log.stopAlongFt = alongFt;
                log.rollFt = alongFt - log.touchdownAlongFt;
                log.stopTimeS = s.simTimeS - touchS;
                log.finalHeadingErrDeg = std::fabs(hdgErr);
                break;
            }
        }
        if (std::getenv("FLEET_TRACE") && i % kSimHz == 0)
            std::printf("  t %.0f wow %d h %.1f along %.0f ias %.1f gs %.1f vs %.0f pitch %.1f thr "
                        "%.2f brk %.2f cross %.1f hdg %.1f flap %.2f spd %.2f rev %d/%d thrust %.0f\n",
                        s.simTimeS, s.wowAny, hFt, alongFt, s.iasKt, s.gsKt, s.vsFpm,
                        radToDeg(s.pitchRad), f.ctl.throttleNorm[0], f.ctl.brakeLeftNorm, crossFt,
                        radToDeg(s.headingTrueRad), s.flapPosNorm, s.speedbrakePosNorm,
                        s.engines[0].reverserDeployed, f.ctl.reverseThrust, s.engines[0].thrustLbf);
        const float vsBefore = s.vsFpm;
        wasAir = !s.wowAny;
        f.step();
        if (wasAir && f.s().wowAny && log.touchdownVsFpm == 0) log.touchdownVsFpm = vsBefore;
        if (f.s().crashed) {
            log.crashed = true;
            std::snprintf(log.crashReason, sizeof log.crashReason, "%s", f.s().crashReason);
            break;
        }
    }
    return log;
}

}  // namespace fsim::test::fleet
