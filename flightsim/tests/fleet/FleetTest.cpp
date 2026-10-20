// WP F2: flyability suite for the core fleet (c172p, c310, global5000, 737, B747).
//
// Every test flies the real JSBSim model headless through FlightModel with the scripted pilot in
// FleetPilot.h. Test names are "sim: fleet <id> <phase>", so ctest's sim_tests picks them up and
// one aircraft or phase can be selected with --test-case="sim: fleet 737 *". Each test also prints
// a "FLEET <id> <key> <value>" line (visible with -s) that feeds the table in README.md.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "FleetPilot.h"
#include "core/Log.h"

using namespace fsim;
using namespace fsim::test;
using namespace fsim::test::fleet;

namespace {

void report(const FleetSpec& sp, const char* key, double value) {
    std::printf("FLEET %s %s %.1f\n", sp.id, key, value);
}

// Interpolated flap-extended speed limit for a detent (profile Vfe when the detent has none).
double vfeOf(const AircraftProfile& p, size_t detent) {
    const float v = p.flapDetents[detent].vfeKt;
    return v > 0 ? v : p.vfeKt;
}

// ---- profile consistency ------------------------------------------------------------------------

void profileTest(const FleetSpec& sp) {
    Flight f;
    REQUIRE_MESSAGE(f.loadProfile(sp.id), f.error);
    const AircraftProfile& p = f.profile;
    CHECK(p.id == sp.id);
    CHECK_FALSE(p.displayName.empty());
    CHECK(p.engineCount >= 1);

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

void takeoffClimbTest(const FleetSpec& sp) {
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
    CHECK(log.maxHeadingErrDeg < 6.0);
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

void cruiseTest(const FleetSpec& sp) {
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

void configTest(const FleetSpec& sp) {
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

void approachTest(const FleetSpec& sp) {
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

void handsOffTest(const FleetSpec& sp) {
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

void landingTest(const FleetSpec& sp) {
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
StallResult measureStall(const FleetSpec& sp, int detent, bool gearDown, double startKt) {
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
        if (f.s().nzG < 0.93f && f.s().alphaDeg > 5.0f && f.s().simTimeS > 5.0) break;
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

void stallTest(const FleetSpec& sp) {
    Flight f;
    REQUIRE(f.loadProfile(sp.id));
    const AircraftProfile& p = f.profile;
    const size_t last = p.flapDetents.size() - 1;
    const StallResult clean = measureStall(sp, 0, !p.hasRetractableGear, sp.stallCleanKt * 1.5);
    const StallResult full = measureStall(sp, static_cast<int>(last), true, p.vappKt * 1.1);
    const StallResult appr = measureStall(sp, p.approachFlapDetent, true, p.vappKt * 1.1);
    INFO("clean " << clean.iasKt << " kt / " << clean.alphaDeg << " deg, full flaps " << full.iasKt
                  << " kt / " << full.alphaDeg << " deg, approach flaps " << appr.iasKt << " kt / "
                  << appr.alphaDeg << " deg");
    CHECK(clean.iasKt == doctest::Approx(sp.stallCleanKt).epsilon(0.03));
    // The profile's published Vs1 / Vs0 stay within 30% of the model (the c172p keeps POH numbers).
    CHECK(std::fabs(p.vs1Kt - clean.iasKt) / clean.iasKt < 0.30);
    CHECK(std::fabs(p.vs0Kt - full.iasKt) / full.iasKt < 0.30);
    CHECK(full.iasKt < clean.iasKt);
    // Approach speed leaves a real margin over the measured stall in the landing configuration.
    CHECK(p.vappKt >= 1.15 * appr.iasKt);
    // The stall warning comes before the stall in every measured configuration.
    CHECK(p.stallWarnAlphaDeg < clean.alphaDeg - 0.5);
    CHECK(p.stallWarnAlphaDeg < appr.alphaDeg - 0.5);
    report(sp, "vs_clean_kt", clean.iasKt);
    report(sp, "alpha_stall_clean_deg", clean.alphaDeg);
    report(sp, "vs_approach_kt", appr.iasKt);
    report(sp, "alpha_stall_approach_deg", appr.alphaDeg);
    report(sp, "vs_full_kt", full.iasKt);
    report(sp, "alpha_stall_full_deg", full.alphaDeg);
}

}  // namespace

// The overlay fixes (737 input sockets, global5000 csv output, c310 propeller) leave every core
// aircraft loading, starting and trimming without a JSBSim error or warning, and without stray
// output files.
TEST_CASE("sim: fleet overlay fixes load all aircraft without errors") {
    const std::string logPath = "fleet_overlay_log.txt";
    std::remove("global5000.csv");
    REQUIRE(log::openFile(logPath));
    log::setLevel(LogLevel::Warn);
    for (const FleetSpec& sp : kFleet) {
        for (const char* kind : {"runway", "cruise", "final"}) {
            Flight f;
            INFO(sp.id << " " << kind);
            REQUIRE_MESSAGE(f.begin(sp.id, kind), f.error);
            f.run(2.0);
            CHECK_FALSE(f.s().crashed);
        }
    }
    log::setLevel(LogLevel::Info);
    log::closeFile();
    std::ifstream in(logPath);
    std::stringstream text;
    text << in.rdbuf();
    INFO("log:\n" << text.str());
    CHECK(text.str().find("ERROR") == std::string::npos);
    CHECK(text.str().find("WARN") == std::string::npos);
    std::remove(logPath.c_str());
    CHECK_FALSE(std::ifstream("global5000.csv").good());
    // The overlay copies replaced the stock files in the runtime layout.
    for (const char* path :
         {"jsbsim/aircraft/737/737.xml", "jsbsim/aircraft/global5000/global5000.xml",
          "jsbsim/aircraft/c310/c310.xml"}) {
        std::ifstream xml(path);
        REQUIRE_MESSAGE(xml.good(), path);
        std::stringstream s;
        s << xml.rdbuf();
        CHECK_MESSAGE(s.str().find("OVERLAY (flightsim WP F2)") != std::string::npos, path);
    }
}

#define FLEET_CASES(PHASE, FN)                                                                     \
    TEST_CASE("sim: fleet c172p " PHASE) {                                                         \
        FN(kFleet[0]);                                                                             \
    }                                                                                              \
    TEST_CASE("sim: fleet c310 " PHASE) {                                                          \
        FN(kFleet[1]);                                                                             \
    }                                                                                              \
    TEST_CASE("sim: fleet global5000 " PHASE) {                                                    \
        FN(kFleet[2]);                                                                             \
    }                                                                                              \
    TEST_CASE("sim: fleet 737 " PHASE) {                                                           \
        FN(kFleet[3]);                                                                             \
    }                                                                                              \
    TEST_CASE("sim: fleet B747 " PHASE) {                                                          \
        FN(kFleet[4]);                                                                             \
    }

FLEET_CASES("profile", profileTest)
FLEET_CASES("takeoff and climb", takeoffClimbTest)
FLEET_CASES("cruise", cruiseTest)
FLEET_CASES("configuration changes", configTest)
FLEET_CASES("approach", approachTest)
FLEET_CASES("hands-off final", handsOffTest)
FLEET_CASES("landing", landingTest)
FLEET_CASES("stall", stallTest)
