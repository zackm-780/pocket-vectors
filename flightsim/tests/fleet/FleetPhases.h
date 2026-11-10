#pragma once
// Phase functions of the fleet flyability suite (WP F2), shared by the core fleet tests
// (FleetTest.cpp) and the extended fleet tests (FleetExtTest.cpp, WP F4). Each function flies one
// aircraft through one phase and asserts the pass criteria from README.md.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "FleetPilot.h"

namespace fsim::test::fleet {

using namespace fsim;
using namespace fsim::test;

inline void report(const FleetSpec& sp, const char* key, double value) {
    std::printf("FLEET %s %s %.1f\n", sp.id, key, value);
}

// Interpolated flap-extended speed limit for a detent (profile Vfe when the detent has none).
inline double vfeOf(const AircraftProfile& p, size_t detent) {
    const float v = p.flapDetents[detent].vfeKt;
    return v > 0 ? v : p.vfeKt;
}

// ---- profile consistency ------------------------------------------------------------------------

inline void profileTest(const FleetSpec& sp) {
    Flight f;
    REQUIRE_MESSAGE(f.loadProfile(sp.id), f.error);
    const AircraftProfile& p = f.profile;
    CHECK(p.id == sp.id);
    CHECK_FALSE(p.displayName.empty());
    CHECK(p.engineCount >= 1);
    // F5: per-detent stall speeds and warnings, when present, have one value per detent; the stall
    // speed falls from clean to full flap and so does the warning alpha's ceiling.
    if (!p.vsByFlapKt.empty()) {
        REQUIRE(p.vsByFlapKt.size() == p.flapDetents.size());
        for (size_t d = 1; d < p.vsByFlapKt.size(); ++d)
            CHECK(p.vsByFlapKt[d] <= p.vsByFlapKt[d - 1]);
        CHECK(p.vsByFlapKt.front() == doctest::Approx(p.vs1Kt).epsilon(0.05));
    }
    if (!p.stallWarnAlphaByFlapDeg.empty())
        CHECK(p.stallWarnAlphaByFlapDeg.size() == p.flapDetents.size());

    // The model loads and the profile agrees with it.
    REQUIRE_MESSAGE(f.begin(sp.id, "runway"), f.error);
    CHECK(f.s().engineCount == p.engineCount);
    CHECK(f.s().gearRetractable == p.hasRetractableGear);
    CHECK(f.s().wowAll);
    CHECK(f.s().altAglFt == doctest::Approx(sp.cgOnGearFt).epsilon(0.03));
    CHECK_FALSE(f.s().crashed);

    // Flap detents: UP first, strictly increasing commands in 0..1, a label each, Vfe falling.
    REQUIRE(p.flapDetents.size() >= 2);
    CHECK(std::string(p.flapDetents[0].label) == "UP");
    CHECK(p.flapDetents[0].cmdNorm == 0.0f);
    CHECK(p.flapDetents.back().cmdNorm == doctest::Approx(1.0f));
    for (size_t i = 1; i < p.flapDetents.size(); ++i) {
        CHECK(p.flapDetents[i].cmdNorm > p.flapDetents[i - 1].cmdNorm);
        CHECK(std::string(p.flapDetents[i].label).size() > 0);
        CHECK(vfeOf(p, i) > 0);
        if (i > 1) CHECK(vfeOf(p, i) <= vfeOf(p, i - 1));
    }
    CHECK(p.vfeKt > 0);
    REQUIRE(p.takeoffFlapDetent >= 0);
    REQUIRE(p.takeoffFlapDetent < static_cast<int>(p.flapDetents.size()));
    REQUIRE(p.approachFlapDetent > 0);
    REQUIRE(p.approachFlapDetent < static_cast<int>(p.flapDetents.size()));
    CHECK(p.approachFlapDetent >= p.takeoffFlapDetent);

    // Speeds in a sane order: stall < Vr < Vref/Vapp, flaps limit above the approach speed.
    CHECK(p.vs0Kt > 0);
    CHECK(p.vs1Kt >= p.vs0Kt);
    CHECK(p.vrKt > 1.1f * p.vs0Kt);
    CHECK(p.vrefKt > p.vs0Kt);
    CHECK(p.vappKt >= p.vrefKt);
    CHECK(vfeOf(p, static_cast<size_t>(p.approachFlapDetent)) > p.vappKt);
    CHECK(p.cruiseKt > p.vappKt);
    CHECK(p.cruiseAltFt > 1000);
    if (p.category == AircraftCategory::Jet) {
        CHECK(p.vmoKt > p.cruiseKt);
        CHECK(p.mmo > 0.6f);
        CHECK(p.mmo < 1.0f);
    } else {
        CHECK(p.vnoKt > p.cruiseKt);
        CHECK(p.vneKt > p.vnoKt);
    }
    CHECK(p.stallWarnAlphaDeg > 3.0f);
    // Piston and turboprop aircraft have no reversers/speedbrakes unless a property says so.
    if (p.hasReversers) CHECK(p.category == AircraftCategory::Jet);

    // Controls the profile advertises are really wired: the speedbrake moves its surface.
    if (p.hasSpeedbrake) {
        REQUIRE(f.begin(sp.id, "cruise"));
        f.ctl.speedbrakeNorm = 1.0f;
        f.run(8.0);
        CHECK(f.s().speedbrakePosNorm > 0.9f);
    }
}

// ---- takeoff and climb -----------------------------------------------------------------------------

inline void takeoffClimbTest(const FleetSpec& sp) {
    Flight f;
    REQUIRE_MESSAGE(f.begin(sp.id, "runway"), f.error);
    const AircraftProfile& p = f.profile;
    CHECK(f.s().flapDetent == p.takeoffFlapDetent);
    const TakeoffLog log = flyTakeoffClimb(f, sp, 3000.0, 900.0);
    INFO("liftoff " << log.liftoffIasKt << " kt after " << log.liftoffDistFt << " ft, climb time "
                    << log.timeTo3000S << " s, crash '" << log.crashReason << "'");
    REQUIRE_FALSE(log.crashed);
    // Liftoff within the expected Vr band, on the centerline, inside the runway.
    CHECK(log.liftoffIasKt >= sp.liftMinKt);
    CHECK(log.liftoffIasKt <= sp.liftMaxKt);
    // The profile Vr is where rotation starts; liftoff follows within 20 kt.
    CHECK(log.liftoffIasKt >= p.vrKt - 5.0);
    CHECK(log.liftoffIasKt <= p.vrKt + 20.0);
    CHECK(log.liftoffDistFt < sp.maxLiftoffDistFt);
    CHECK(log.maxHeadingErrDeg < sp.maxTakeoffHdgErrDeg);
    CHECK(log.maxPitchDeg < 20.0);
    // Reaches 3,000 ft AGL in a sane time with the aircraft cleaned up.
    REQUIRE(log.timeTo3000S > 0);
    CHECK(log.timeTo3000S < sp.maxClimbTimeS);
    CHECK(log.flapsUp);
    CHECK(log.gearUp);
    CHECK(log.stallWarnS < 1.0);
    CHECK_FALSE(f.s().overspeed);
    CHECK(f.s().iasKt > sp.stallCleanKt * 1.2f);
    report(sp, "liftoff_kt", log.liftoffIasKt);
    report(sp, "liftoff_dist_ft", log.liftoffDistFt);
    report(sp, "liftoff_alpha_deg", log.liftoffAlphaDeg);
    report(sp, "climb_3000_s", log.timeTo3000S);
    report(sp, "max_alpha_deg", log.maxAlphaDeg);
}

// ---- trimmed cruise --------------------------------------------------------------------------------

inline void cruiseTest(const FleetSpec& sp) {
    Flight f;
    REQUIRE_MESSAGE(f.begin(sp.id, "cruise"), f.error);
    const AircraftProfile& p = f.profile;
    const double alt0 = f.s().altMslFt;
    CHECK(alt0 == doctest::Approx(p.cruiseAltFt).epsilon(0.001));
    CHECK(f.s().iasKt == doctest::Approx(p.cruiseKt).epsilon(0.01));
    CHECK(std::fabs(f.s().gearPosNorm - (p.hasRetractableGear ? 0.0f : 1.0f)) < 0.01f);
    CHECK(f.ctl.throttleNorm[0] < 0.95f);  // the profile cruise point is sustainable
    CHECK(f.ctl.throttleNorm[0] > 0.05f);
    double maxDevFt = 0, maxRollDeg = 0;
    for (long i = 0; i < 60 * kSimHz; ++i) {
        f.step();
        maxDevFt = std::max(maxDevFt, std::fabs(f.s().altMslFt - alt0));
        maxRollDeg = std::max(maxRollDeg, std::fabs(radToDeg(f.s().rollRad)));
    }
    INFO("max altitude deviation " << maxDevFt << " ft");
    CHECK(maxDevFt < 200.0);
    CHECK(maxRollDeg < 3.0);
    CHECK_FALSE(f.s().crashed);
    CHECK_FALSE(f.s().overspeed);
    CHECK_FALSE(f.s().stallWarning);
    CHECK(f.s().iasKt == doctest::Approx(p.cruiseKt).epsilon(0.05));
    if (p.mmo > 0) CHECK(f.s().mach < p.mmo);
    report(sp, "cruise_dev_ft", maxDevFt);
    report(sp, "cruise_throttle_pct", f.ctl.throttleNorm[0] * 100.0);
}

// ---- configuration changes -----------------------------------------------------------------------

inline void configTest(const FleetSpec& sp) {
    Flight f;
    REQUIRE_MESSAGE(f.resolve(sp.id, "cruise"), f.error);
    const AircraftProfile& p = f.profile;
    f.start.altMslFt = 12000.0;
    f.start.iasKt = sp.flapRetractKt;
    f.start.gearDown = true;
    REQUIRE(f.begin());
    const double alt0 = f.s().altMslFt;
    PitchPilot pp;
    pp.sync(f.s());
    SpeedPilot sps;
    sps.sync(f.ctl);
    double targetKt = sp.flapRetractKt;
    double maxDevFt = 0;
    // Holds altitude and speed while the configuration changes.
    auto fly = [&](double seconds, auto&& until) {
        const long n = static_cast<long>(seconds * kSimHz);
        for (long i = 0; i < n; ++i) {
            const AircraftState& s = f.s();
            f.ctl.elevatorNorm =
                pp.vsHold(s, sp, clampd(0.3 * (alt0 - s.altMslFt), -500, 500), -8, 14);
            setAllThrottles(f, sps.throttle(s, sp, targetKt));
            f.ctl.aileronNorm = headingBank(s, 270.0);
            f.step();
            maxDevFt = std::max(maxDevFt, std::fabs(f.s().altMslFt - alt0));
            if (f.s().crashed) return -1.0;
            if (until()) return f.s().simTimeS;
        }
        return -2.0;
    };
    // Retractable gear starts down; cycle it up and down.
    if (p.hasRetractableGear) {
        const double t0 = f.s().simTimeS;
        f.ctl.gearDown = false;
        const double t1 = fly(30, [&] { return f.s().gearPosNorm < 0.01f; });
        REQUIRE(t1 > 0);
        INFO("gear up took " << t1 - t0 << " s");
        CHECK(t1 - t0 > 1.0);
        CHECK(t1 - t0 < 15.0);
        report(sp, "gear_up_s", t1 - t0);
        const double t2 = f.s().simTimeS;
        f.ctl.gearDown = true;
        const double t3 = fly(30, [&] { return f.s().gearPosNorm > 0.99f; });
        REQUIRE(t3 > 0);
        CHECK(t3 - t2 > 1.0);
        CHECK(t3 - t2 < 15.0);
        report(sp, "gear_down_s", t3 - t2);
    }
    // Every flap detent in turn, at the speed its placard allows.
    double slowestFlapS = 0, prevPos = 0;
    for (size_t d = 1; d < p.flapDetents.size(); ++d) {
        targetKt = std::min(sp.flapRetractKt, vfeOf(p, d) - 5.0);
        const double cmd = p.flapDetents[d].cmdNorm;
        const double t0 = f.s().simTimeS;
        f.ctl.flapDetent = static_cast<int>(d);
        const double t1 = fly(60, [&] { return std::fabs(f.s().flapPosNorm - cmd) < 0.01; });
        INFO("flap detent " << p.flapDetents[d].label << " took " << t1 - t0 << " s");
        REQUIRE(t1 > 0);
        CHECK(t1 - t0 < 40.0);
        CHECK(f.s().flapDetent == static_cast<int>(d));
        CHECK(f.s().flapPosNorm > prevPos);
        prevPos = f.s().flapPosNorm;
        slowestFlapS = std::max(slowestFlapS, t1 - t0);
        fly(4, [] { return false; });
    }
    CHECK(std::fabs(f.s().flapPosNorm - 1.0f) < 0.01f);
    // And all the way back up.
    targetKt = sp.flapRetractKt;
    const double tUp0 = f.s().simTimeS;
    f.ctl.flapDetent = 0;
    const double tUp1 = fly(90, [&] { return f.s().flapPosNorm < 0.01f; });
    REQUIRE(tUp1 > 0);
    CHECK(tUp1 - tUp0 < 60.0);
    report(sp, "flaps_full_s", slowestFlapS);
    report(sp, "flaps_retract_s", tUp1 - tUp0);
    // Speedbrake out and in.
    if (p.hasSpeedbrake) {
        f.ctl.speedbrakeNorm = 1.0f;
        const double t1 = fly(15, [&] { return f.s().speedbrakePosNorm > 0.95f; });
        REQUIRE(t1 > 0);
        f.ctl.speedbrakeNorm = 0.0f;
        const double t2 = fly(15, [&] { return f.s().speedbrakePosNorm < 0.05f; });
        CHECK(t2 > 0);
    }
    CHECK_FALSE(f.s().crashed);
    CHECK_FALSE(f.s().overspeed);
    INFO("max altitude deviation " << maxDevFt << " ft");
    CHECK(maxDevFt < 800.0);
}

// ---- stabilised approach ---------------------------------------------------------------------------

inline void approachTest(const FleetSpec& sp) {
    Flight f;
    REQUIRE_MESSAGE(f.begin(sp.id, "final", "distance_nm = 8\n"), f.error);
    const AircraftProfile& p = f.profile;
    CHECK(f.s().flapDetent == p.approachFlapDetent);
    CHECK(f.s().iasKt == doctest::Approx(p.vappKt).epsilon(0.01));
    CHECK(std::fabs(f.s().gearPosNorm - 1.0f) < 0.01f);
    // Flown with the pilot down to 150 ft above the runway: 3 degree path within +-150 ft.
    const LandingLog log = flyApproachLanding(f, sp, 150.0, 700.0);
    INFO("max path error " << log.maxPathErrFt << " ft, IAS error " << log.maxIasErrKt << " kt");
    REQUIRE_FALSE(log.crashed);
    CHECK(log.apchEndTimeS > 0);
    CHECK(log.maxPathErrFt <= 150.0);
    CHECK(log.maxPathErrBelow500Ft <= 75.0);
    CHECK(log.maxIasErrKt <= 10.0);
    CHECK(log.stallWarnS == 0.0);
    report(sp, "apch_path_err_ft", log.maxPathErrFt);
    report(sp, "apch_ias_err_kt", log.maxIasErrKt);
}

inline void handsOffTest(const FleetSpec& sp) {
    Flight f;
    REQUIRE_MESSAGE(f.begin(sp.id, "final", "distance_nm = 6\n"), f.error);
    const RunwayRef rwy = RunwayRef::xtst27(f);
    double maxErrFt = 0;
    for (long i = 0; i < 60 * kSimHz; ++i) {
        f.step();
        const double pathAlt = rwy.elevFt + 50.0 - rwy.alongFt(f.s()) * std::tan(degToRad(3.0));
        maxErrFt = std::max(maxErrFt, std::fabs(f.s().altMslFt - pathAlt));
    }
    INFO("max path error " << maxErrFt << " ft");
    CHECK(maxErrFt <= 150.0);
    CHECK_FALSE(f.s().crashed);
    CHECK(f.s().iasKt == doctest::Approx(f.profile.vappKt).epsilon(0.05));
    report(sp, "handsoff_path_err_ft", maxErrFt);
}

// ---- landing and rollout -----------------------------------------------------------------------------

inline void landingTest(const FleetSpec& sp) {
    Flight f;
    REQUIRE_MESSAGE(
        f.begin(sp.id, "final", "distance_nm = " + std::to_string(sp.landStartNm) + "\n"), f.error);
    const LandingLog log = flyApproachLanding(f, sp, 0.0, 400.0);
    INFO("touchdown " << log.touchdownVsFpm << " fpm at " << log.touchdownIasKt << " kt, "
                      << log.touchdownAlongFt << " ft past the threshold, stop at "
                      << log.stopAlongFt << " ft, crash '" << log.crashReason << "'");
    REQUIRE_FALSE(log.crashed);
    REQUIRE(log.touched);
    CHECK(log.touchdownVsFpm > -sp.maxTouchdownSinkFpm);
    CHECK(log.touchdownAlongFt > 0.0);
    CHECK(log.touchdownAlongFt < 5000.0);
    CHECK(std::fabs(log.touchdownBankDeg) < 5.0);
    REQUIRE(log.stopped);
    CHECK(log.stopAlongFt < sp.maxLandingFt);
    CHECK(log.stopAlongFt < SyntheticNavDb::kRunwayLengthFt - 200.0);  // stops on the runway
    CHECK(log.maxCrossFt < 60.0);  // half the runway width is 75 ft
    CHECK(log.maxRolloutHeadingErrDeg < 15.0);
    CHECK_FALSE(f.s().crashed);
    report(sp, "td_vs_fpm", log.touchdownVsFpm);
    report(sp, "td_cg_agl_ft", log.touchdownCgAglFt);
    report(sp, "td_ias_kt", log.touchdownIasKt);
    report(sp, "td_along_ft", log.touchdownAlongFt);
    report(sp, "stop_along_ft", log.stopAlongFt);
    report(sp, "roll_ft", log.rollFt);
    report(sp, "roll_s", log.stopTimeS);
}

// ---- measured stall speeds and alpha ---------------------------------------------------------

struct StallResult {
    double iasKt = 0, alphaDeg = 0;
};

// Decelerates at idle from a trimmed start (1 g, pitch ramped at 0.15 deg/s) and reports the
// minimum indicated airspeed reached before the stall breaks (load factor falls below 0.93, or
// 12 s after the minimum) and the angle of attack there. At this slow rate the aircraft stays
// near 1 g, so this is the 1 g stall speed.
inline StallResult measureStall(const FleetSpec& sp, int detent, bool gearDown, double startKt,
                                double settleS = 5.0) {
    Flight f;
    REQUIRE(f.resolve(sp.id, "cruise"));
    f.start.iasKt = startKt;
    f.start.altMslFt = 9000.0;
    f.start.flapDetent = detent;
    f.start.gearDown = gearDown;
    REQUIRE(f.begin());
    setAllThrottles(f, 0.0);
    double pitchCmd = radToDeg(f.s().pitchRad), minIas = 1e9, tMin = 0;
    StallResult r;
    for (long i = 0; i < 400 * kSimHz && !f.s().crashed; ++i) {
        const AircraftState& s = f.s();
        pitchCmd += 0.15 / kSimHz;
        f.ctl.elevatorNorm = pitchHoldElevator(s, pitchCmd, 0.1, 0.1);
        f.ctl.aileronNorm = bankHoldAileron(s);
        f.ctl.rudderNorm = headingHoldRudder(s, 270.0, 0.02, 0.02);
        f.step();
        if (f.s().nzG < 0.93f && f.s().alphaDeg > 5.0f && f.s().simTimeS > settleS) break;
        if (f.s().iasKt < minIas) {
            minIas = f.s().iasKt;
            r.iasKt = minIas;
            r.alphaDeg = f.s().alphaDeg;
            tMin = f.s().simTimeS;
        }
        if (f.s().simTimeS - tMin > 12.0 || radToDeg(f.s().pitchRad) > 35) break;
    }
    return r;
}

// F5: elevator for a level turn. Integrates toward the load factor that keeps the turn level at
// the present bank (1 / cos bank), corrected by the vertical speed, with pitch-rate damping.
struct LevelTurnElevator {
    double integ = 0;
    double ki = 1.5, kp = 0.5;
    float update(const AircraftState& s) {
        const double bank = std::min(std::fabs(static_cast<double>(s.rollRad)), degToRad(70.0));
        const double nCmd = 1.0 / std::cos(bank) + std::clamp(-s.vsFpm / 2000.0, -0.2, 0.2);
        const double err = nCmd - s.nzG;
        integ = std::clamp(integ + ki * err / kSimHz, -1.0, 1.0);
        return static_cast<float>(
            std::clamp(integ + kp * err - 0.03 * radToDeg(s.qRadps), -1.0, 1.0));
    }
};

// F5: the same idle deceleration at one flap detent, optionally in a level banked turn, with the
// (IAS, alpha) trace and the first speed at which the profile's stall warning came on (the
// flight model's own latched warning, so the HUD and the model are checked together).
struct StallTrace {
    double vsKt = 0, alphaStallDeg = 0;  // minimum IAS before the break, and alpha there
    double firstWarnKt = 0;              // IAS when AircraftState::stallWarning first came on
    double nzAtWarn = 1;                 // load factor at that moment
    bool warned = false;
    double maxNz = 1;        // highest load factor seen after the settle time
    double maxAlphaDeg = 0;  // highest angle of attack after the start transient
    std::vector<std::pair<double, double>> iasAlpha;  // samples every 0.25 s after the settle
    // Alpha when the deceleration passed `kt` (the last sample at or above it).
    double alphaAtKt(double kt) const {
        double a = 0;
        for (const auto& [ias, alpha] : iasAlpha)
            if (ias >= kt) a = alpha;
        return a;
    }
};

inline StallTrace measureStallTrace(const FleetSpec& sp, int detent, bool gearDown, double startKt,
                                    double settleS = 5.0, double bankDeg = 0.0) {
    Flight f;
    REQUIRE(f.resolve(sp.id, "cruise"));
    f.start.iasKt = startKt;
    f.start.altMslFt = 9000.0;
    f.start.flapDetent = detent;
    f.start.gearDown = gearDown;
    REQUIRE(f.begin());
    setAllThrottles(f, 0.0);
    double pitchCmd = radToDeg(f.s().pitchRad), minIas = 1e9, tMin = 0, nextSampleS = 0;
    LevelTurnElevator turn;
    turn.integ = f.ctl.elevatorNorm;
    StallTrace r;
    for (long i = 0; i < 400 * kSimHz && !f.s().crashed; ++i) {
        const AircraftState& s = f.s();
        // Wings level: the slow pitch ramp. In a bank: a level turn (pitch follows the vertical
        // speed), so the load factor stays at 1 / cos(bank) while the speed bleeds off.
        pitchCmd += 0.15 / kSimHz;
        const double bankNow = std::min(bankDeg, 5.0 * s.simTimeS);
        f.ctl.elevatorNorm =
            bankDeg == 0.0 ? pitchHoldElevator(s, pitchCmd, 0.1, 0.1) : turn.update(s);
        f.ctl.aileronNorm =
            bankDeg == 0.0 ? bankHoldAileron(s) : bankHoldAileron(s, bankNow, 0.1, 0.05);
        f.ctl.rudderNorm = bankDeg == 0.0
                               ? headingHoldRudder(s, 270.0, 0.02, 0.02)
                               : static_cast<float>(std::clamp(-0.05 * s.betaDeg, -1.0, 1.0));
        f.step();
        const AircraftState& n = f.s();
        if (n.simTimeS > settleS) r.maxNz = std::max(r.maxNz, static_cast<double>(n.nzG));
        if (n.simTimeS > std::min(settleS, 5.0))
            r.maxAlphaDeg = std::max(r.maxAlphaDeg, static_cast<double>(n.alphaDeg));
        // The warning counts once the start transient is over (5 s): on a heavy jet it often
        // comes before the 30 s settle the break test needs.
        if (n.simTimeS > std::min(settleS, 5.0) && n.stallWarning && !r.warned) {
            r.warned = true;
            r.firstWarnKt = n.iasKt;
            r.nzAtWarn = n.nzG;
        }
        // The trace starts once the start transient is over (5 s), so heavy aircraft that need a
        // longer settle before the load-factor test still show the speeds above 1.07 Vs.
        if (n.simTimeS > std::min(settleS, 5.0) && n.simTimeS >= nextSampleS) {
            r.iasAlpha.emplace_back(n.iasKt, n.alphaDeg);
            nextSampleS = n.simTimeS + 0.25;
        }
        // The break: the load factor falls away (wings level: below 0.93 g; in a turn: below 85 %
        // of what the present bank needs).
        const double nNow =
            bankDeg == 0.0 ? 1.0 : 1.0 / std::cos(std::min(std::fabs(n.rollRad), 1.2f));
        const bool lost = bankDeg == 0.0 ? n.nzG < 0.93f : n.nzG < 0.85 * nNow;
        if (lost && n.alphaDeg > 5.0f && n.simTimeS > settleS) break;
        if (n.iasKt < minIas) {
            minIas = n.iasKt;
            r.vsKt = minIas;
            r.alphaStallDeg = n.alphaDeg;
            tMin = n.simTimeS;
        }
        if (n.simTimeS - tMin > 12.0 || radToDeg(n.pitchRad) > 35) break;
    }
    return r;
}

// F5: start speed and settle time for the per-detent deceleration (fast enough that the trace
// passes 1.2 x Vs after the settle, slow enough that the start transient does not fake a break).
inline double stallStartKt(const AircraftProfile& p, size_t detent) {
    const double vs = detent < p.vsByFlapKt.size() ? p.vsByFlapKt[detent] : p.vs1Kt;
    return p.category == AircraftCategory::Piston ? 1.3 * vs + 12.0 : 1.25 * vs + 15.0;
}
inline double stallSettleS(const AircraftProfile& p) {
    return p.category == AircraftCategory::Piston ? 5.0 : 30.0;
}

// F5: the stall at every flap detent. The profile's vs_by_flap matches the model, falls with flap,
// and the per-detent warning comes on at least 5 kt before the stall but not above 1.2 x Vs.
inline void stallByFlapTest(const FleetSpec& sp) {
    Flight f;
    REQUIRE(f.loadProfile(sp.id));
    const AircraftProfile& p = f.profile;
    const size_t n = p.flapDetents.size();
    REQUIRE(p.vsByFlapKt.size() == n);
    REQUIRE(p.stallWarnAlphaByFlapDeg.size() == n);
    char key[48];
    for (size_t d = 0; d < n; ++d) {
        if (d > 0) CHECK(p.vsByFlapKt[d] <= p.vsByFlapKt[d - 1]);
        const StallTrace t =
            measureStallTrace(sp, static_cast<int>(d), d > 0 || !p.hasRetractableGear,
                              stallStartKt(p, d), stallSettleS(p));
        INFO(sp.id << " flaps " << p.flapDetents[d].label << ": stall " << t.vsKt << " kt / "
                   << t.alphaStallDeg << " deg, warning " << p.stallWarnAlphaByFlapDeg[d]
                   << " deg first at " << t.firstWarnKt << " kt");
        std::snprintf(key, sizeof key, "vs_f%zu_kt", d);
        report(sp, key, t.vsKt);
        std::snprintf(key, sizeof key, "alpha_stall_f%zu_deg", d);
        report(sp, key, t.alphaStallDeg);
        std::snprintf(key, sizeof key, "warn_f%zu_kt", d);
        report(sp, key, t.firstWarnKt);
        CHECK(t.vsKt == doctest::Approx(p.vsByFlapKt[d]).epsilon(0.03));
        CHECK(p.stallWarnAlphaByFlapDeg[d] < t.alphaStallDeg - 0.5);
        REQUIRE(t.warned);
        CHECK(t.firstWarnKt >= t.vsKt + 5.0);
        CHECK(t.firstWarnKt <= 1.2 * t.vsKt);
    }
}

// The most extended detent, up to the approach detent, whose Vfe allows `iasKt` (the flap a pilot
// has out at that speed on the approach).
inline int matchingFlapDetent(const AircraftProfile& p, double iasKt) {
    int best = 0;
    for (int d = 1; d <= p.approachFlapDetent && d < static_cast<int>(p.flapDetents.size()); ++d)
        if (vfeOf(p, static_cast<size_t>(d)) >= iasKt) best = d;
    return best;
}

// Flies `holdS` seconds and reports whether the stall warning came on after a 3 s settle.
// Climb: full power, pitch up at 1.5 deg/s toward `pitchDeg`, less 1 deg per kt below the start
// speed (a pilot does not hold 15 degrees into a stall). Turn: level at the start speed, banked.
struct ManoeuvreResult {
    bool warned = false;
    double minIasKt = 0, maxAlphaDeg = 0, maxNz = 0, warnAlphaDeg = 0;
};
inline ManoeuvreResult flyManoeuvre(const FleetSpec& sp, int detent, double iasKt, double pitchDeg,
                                    double bankDeg, double holdS) {
    Flight f;
    REQUIRE(f.resolve(sp.id, "cruise"));
    f.start.iasKt = iasKt;
    f.start.altMslFt = 5000.0;
    f.start.flapDetent = detent;
    f.start.gearDown = detent > 0;
    REQUIRE(f.begin());
    const bool climb = pitchDeg > 0.0;
    if (climb) setAllThrottles(f, 1.0);
    const double pitch0 = radToDeg(f.s().pitchRad);
    double pitchCmd = pitch0, thr = f.ctl.throttleNorm[0];
    ManoeuvreResult r;
    r.minIasKt = 1e9;
    r.maxAlphaDeg = -90;
    for (long i = 0; i < static_cast<long>(holdS * kSimHz) && !f.s().crashed; ++i) {
        const AircraftState& s = f.s();
        const double t = s.simTimeS;
        if (climb) {
            pitchCmd = std::min(pitchDeg, pitch0 + 1.5 * t) - std::max(0.0, iasKt - s.iasKt);
            f.ctl.elevatorNorm = pitchHoldElevator(s, pitchCmd, 0.1, 0.1);
        } else {
            // A steady level turn: the pitch attitude follows the vertical speed slowly (a pilot's
            // scan, which every model here flies without the pitch oscillation an n-loop can start).
            thr = std::clamp(thr + 0.02 * (iasKt - s.iasKt) / kSimHz, 0.0, 1.0);
            setAllThrottles(f, thr);
            pitchCmd += std::clamp(-s.vsFpm / 300.0, -2.0, 2.0) / kSimHz;
            f.ctl.elevatorNorm = pitchHoldElevator(s, pitchCmd, 0.1, 0.1);
        }
        const double bankNow = std::min(bankDeg, 5.0 * t);
        f.ctl.aileronNorm = bankHoldAileron(s, bankNow, 0.1, 0.05);
        f.ctl.rudderNorm = bankDeg == 0.0
                               ? headingHoldRudder(s, 270.0, 0.02, 0.02)
                               : static_cast<float>(std::clamp(-0.05 * s.betaDeg, -1.0, 1.0));
        f.step();
        const AircraftState& n = f.s();
        if (n.simTimeS > 3.0) {
            r.warned = r.warned || n.stallWarning;
            r.minIasKt = std::min(r.minIasKt, static_cast<double>(n.iasKt));
            r.maxAlphaDeg = std::max(r.maxAlphaDeg, static_cast<double>(n.alphaDeg));
            r.maxNz = std::max(r.maxNz, static_cast<double>(n.nzG));
        }
    }
    CHECK_FALSE(f.s().crashed);
    r.warnAlphaDeg = stallWarnAlphaForFlapDeg(f.profile, f.s().flapPosNorm);
    return r;
}

// F5 (owner, 2026-11-09): no warning in a normal climb (up to 15 degrees of pitch, takeoff flaps
// then clean) or in a 30 degree bank at Vref + 20 kt with the matching flap; in a 45 degree
// level turn the warning comes only near the load-factor stall speed, Vs x sqrt(n).
inline void stallWarnManoeuvreTest(const FleetSpec& sp) {
    Flight f;
    REQUIRE(f.loadProfile(sp.id));
    const AircraftProfile& p = f.profile;
    const double turnKt = p.vrefKt + 20.0;
    const int turnFlap = matchingFlapDetent(p, turnKt);
    struct Case {
        const char* name;
        int detent;
        double kt, pitchDeg, bankDeg;
    };
    // Initial climb at V2 + 10 kt with H9's V2: 1.2 x the takeoff-flap stall, at least Vr + 3.
    const double v2Kt =
        std::max(1.2 * p.vsByFlapKt[static_cast<size_t>(p.takeoffFlapDetent)], p.vrKt + 3.0);
    const Case cases[] = {{"climb_takeoff_flaps", p.takeoffFlapDetent, v2Kt + 10.0, 15.0, 0.0},
                          {"climb_clean", 0, sp.cleanKt, 15.0, 0.0},
                          {"bank30_clean_1.3vs1", 0, 1.3 * p.vsByFlapKt[0], 0.0, 30.0},
                          {"bank30_vref20", turnFlap, turnKt, 0.0, 30.0}};
    for (const Case& c : cases) {
        const ManoeuvreResult r = flyManoeuvre(sp, c.detent, c.kt, c.pitchDeg, c.bankDeg, 40.0);
        INFO(sp.id << " " << c.name << " flaps "
                   << p.flapDetents[static_cast<size_t>(c.detent)].label << " from " << c.kt
                   << " kt: min " << r.minIasKt << " kt, max alpha " << r.maxAlphaDeg
                   << " (warning " << r.warnAlphaDeg << "), max nz " << r.maxNz);
        char key[48];
        std::snprintf(key, sizeof key, "%s_min_kt", c.name);
        report(sp, key, r.minIasKt);
        std::snprintf(key, sizeof key, "%s_max_alpha_deg", c.name);
        report(sp, key, r.maxAlphaDeg);
        CHECK_FALSE(r.warned);
    }
    // The load factor at the warning is the turn's own (about 1.4 at 45 degrees; the level-turn
    // pilot holds a little less while the speed bleeds off), so Vs x sqrt(n) uses that value.
    for (int d : {0, turnFlap}) {
        const double vs1g = p.vsByFlapKt[static_cast<size_t>(d)];
        const double startKt = 1.25 * vs1g * std::sqrt(1.0 / std::cos(degToRad(45.0))) + 15.0;
        const StallTrace t = measureStallTrace(sp, d, d > 0 || !p.hasRetractableGear, startKt,
                                               stallSettleS(p), 45.0);
        const double vsTurn = vs1g * std::sqrt(std::max(1.0, t.nzAtWarn));
        INFO(sp.id << " 45 deg bank, flaps " << p.flapDetents[static_cast<size_t>(d)].label
                   << ": first warning " << t.firstWarnKt << " kt at nz " << t.nzAtWarn
                   << ", Vs x sqrt(n) " << vsTurn << " kt, break " << t.vsKt << " kt");
        char key[48];
        std::snprintf(key, sizeof key, "bank45_f%d_warn_kt", d);
        report(sp, key, t.firstWarnKt);
        std::snprintf(key, sizeof key, "bank45_f%d_nz", d);
        report(sp, key, t.nzAtWarn);
        std::snprintf(key, sizeof key, "bank45_f%d_vs_sqrt_n_kt", d);
        report(sp, key, vsTurn);
        if (!t.warned) {
            // Some configurations run out of elevator in the turn before they reach the warning
            // (the 787-8 with flaps 30 tops out at 10.4 degrees and just descends): no warning
            // is right as long as the angle of attack never got to the threshold.
            const double warnDeg = p.stallWarnAlphaByFlapDeg[static_cast<size_t>(d)];
            MESSAGE(sp.id << " 45 deg bank flaps " << p.flapDetents[static_cast<size_t>(d)].label
                          << ": elevator-limited at " << t.maxAlphaDeg << " deg (warning "
                          << warnDeg << ")");
            CHECK(t.maxAlphaDeg < warnDeg);
            continue;
        }
        CHECK(t.nzAtWarn > 1.1);  // clearly in the turn (the level-turn pilot holds 1.1 to 1.8)
        CHECK(t.firstWarnKt >= vsTurn);
        CHECK(t.firstWarnKt <= 1.2 * vsTurn);
    }
}

inline void stallTest(const FleetSpec& sp, double settleS = 5.0) {
    Flight f;
    REQUIRE(f.loadProfile(sp.id));
    const AircraftProfile& p = f.profile;
    const size_t last = p.flapDetents.size() - 1;
    const StallResult clean =
        measureStall(sp, 0, !p.hasRetractableGear, sp.stallCleanKt * 1.5, settleS);
    const StallResult full =
        measureStall(sp, static_cast<int>(last), true, p.vappKt * 1.1, settleS);
    const StallResult appr = measureStall(sp, p.approachFlapDetent, true, p.vappKt * 1.1, settleS);
    INFO("clean " << clean.iasKt << " kt / " << clean.alphaDeg << " deg, full flaps " << full.iasKt
                  << " kt / " << full.alphaDeg << " deg, approach flaps " << appr.iasKt << " kt / "
                  << appr.alphaDeg << " deg");
    CHECK(clean.iasKt == doctest::Approx(sp.stallCleanKt).epsilon(0.03));
    // The profile's published Vs1 / Vs0 stay within 30% of the model (the c172p keeps POH numbers).
    CHECK(std::fabs(p.vs1Kt - clean.iasKt) / clean.iasKt < 0.30);
    CHECK(std::fabs(p.vs0Kt - full.iasKt) / full.iasKt < 0.30);
    if (!sp.stallElevatorLimited) CHECK(full.iasKt < clean.iasKt);
    // Approach speed leaves a real margin over the measured stall in the landing configuration.
    CHECK(p.vappKt >= 1.15 * appr.iasKt);
    // The stall warning comes before the stall in every measured configuration.
    CHECK(p.stallWarnAlphaDeg < clean.alphaDeg - 0.5);
    if (!sp.stallElevatorLimited) CHECK(p.stallWarnAlphaDeg < appr.alphaDeg - 0.5);
    report(sp, "vs_clean_kt", clean.iasKt);
    report(sp, "alpha_stall_clean_deg", clean.alphaDeg);
    report(sp, "vs_approach_kt", appr.iasKt);
    report(sp, "alpha_stall_approach_deg", appr.alphaDeg);
    report(sp, "vs_full_kt", full.iasKt);
    report(sp, "alpha_stall_full_deg", full.alphaDeg);
}

}  // namespace fsim::test::fleet
