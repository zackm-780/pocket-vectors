// WP F1 tests: the acceptance criteria (a)-(e) plus state mapping, inputs and crash rules.
// Tests named "sim: ..." run JSBSim (ctest sim_tests); "profile: ..." and "crash: ..." are pure.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "FlightHarness.h"
#include "core/FlatTerrain.h"
#include "sim/CrashRules.h"

using namespace fsim;
using namespace fsim::test;

namespace {

// Terrain stub for the water-contact rule: everything is water.
class WaterTerrain : public TerrainQuery {
public:
    float elevationM(double, double) const override { return 0; }
    LandClass landClass(double, double) const override { return LandClass::Water; }
    void addFlattenArea(const FlattenArea&) override {}
    void clearFlattenAreas() override {}
    void prefetch(double, double, float) override {}
    bool isDetailReady(double, double) const override { return true; }
};

// Pilot for a runway takeoff: full power, hold the centerline with rudder and nose-wheel
// steering, wings level, rotate gently to a target pitch at rotation speed.
void takeoffPilot(Flight& f, double rotateKt, double pitchDeg, double headingTrueDeg) {
    const AircraftState& s = f.s();
    const float rud = headingHoldRudder(s, headingTrueDeg);
    f.ctl.rudderNorm = rud;
    f.ctl.steeringNorm = rud;
    f.ctl.aileronNorm = bankHoldAileron(s);
    f.ctl.elevatorNorm = s.iasKt > rotateKt ? pitchHoldElevator(s, pitchDeg, 0.5, 0.1) : 0.f;
}

}  // namespace

// ---- profiles -------------------------------------------------------------------------------

TEST_CASE("profile: c172p.ini loads completely") {
    Flight f;
    REQUIRE_MESSAGE(f.loadProfile("c172p"), f.error);
    const AircraftProfile& p = f.profile;
    CHECK(p.jsbsimModel == "c172p");
    CHECK(p.category == AircraftCategory::Piston);
    CHECK(p.engineCount == 1);
    CHECK_FALSE(p.hasRetractableGear);
    REQUIRE(p.flapDetents.size() == 4);
    CHECK(std::string(p.flapDetents[0].label) == "UP");
    CHECK(std::string(p.flapDetents[1].label) == "10");
    CHECK(p.flapDetents[3].cmdNorm == doctest::Approx(1.0f));
    CHECK(p.flapDetents[1].vfeKt == doctest::Approx(110.0f));  // flaps 10
    CHECK(p.flapDetents[2].vfeKt == doctest::Approx(85.0f));   // flaps beyond 10
    CHECK(p.vs0Kt == doctest::Approx(33.0f));
    CHECK(p.vs1Kt == doctest::Approx(44.0f));
    CHECK(p.vrKt == doctest::Approx(55.0f));
    CHECK(p.vrefKt == doctest::Approx(65.0f));
    CHECK(p.vfeKt == doctest::Approx(85.0f));
    CHECK(p.vnoKt == doctest::Approx(129.0f));
    CHECK(p.vneKt > 150.0f);
    CHECK(p.cruiseAltFt == doctest::Approx(5500.0f));
    CHECK(p.stallWarnAlphaDeg > 5.0f);
    CHECK(p.approachFlapDetent == 2);
    CHECK(p.properties.count("crash.hard_landing_fpm") == 1);
}

TEST_CASE("profile: 737.ini maps flap detents onto the JSBSim 737 flap command") {
    Flight f;
    REQUIRE_MESSAGE(f.loadProfile("737"), f.error);
    const AircraftProfile& p = f.profile;
    CHECK(p.category == AircraftCategory::Jet);
    CHECK(p.engineCount == 2);
    CHECK(p.hasRetractableGear);
    CHECK(p.hasSpeedbrake);
    CHECK(p.hasReversers);
    CHECK(p.hasAutothrottle);
    REQUIRE(p.flapDetents.size() == 9);
    const char* labels[] = {"UP", "1", "2", "5", "10", "15", "25", "30", "40"};
    const float degrees[] = {0, 1, 2, 5, 10, 15, 25, 30, 40};
    for (size_t i = 0; i < 9; ++i) {
        CHECK(std::string(p.flapDetents[i].label) == labels[i]);
        // fcs/flap-cmd-norm spans 0..40 degrees of flap travel
        CHECK(p.flapDetents[i].cmdNorm == doctest::Approx(degrees[i] / 40.0f));
    }
    CHECK(std::string(p.flapDetents[p.approachFlapDetent].label) == "30");
    CHECK(std::string(p.flapDetents[p.takeoffFlapDetent].label) == "5");
    CHECK(p.vmoKt == doctest::Approx(340.0f));
    CHECK(p.mmo == doctest::Approx(0.82f));
    CHECK(p.properties.at("spoiler_cmd") == "fcs/spoiler-cmd-norm");
}

TEST_CASE("profile: parser rejects broken profiles with a message") {
    AircraftProfile p;
    std::string err;
    CHECK_FALSE(parseAircraftProfile("[flaps]\ndetents = UP:0\n", p, err));
    CHECK(err.find("[aircraft]") != std::string::npos);
    CHECK_FALSE(parseAircraftProfile("[aircraft]\nid = x\nengines = 9\n", p, err));
    CHECK_FALSE(
        parseAircraftProfile("[aircraft]\nid = x\n[flaps]\ndetents = UP:0, 10:1.5\n", p, err));
    CHECK_FALSE(parseAircraftProfile(
        "[aircraft]\nid = x\n[flaps]\ndetents = UP:0\napproach_detent = 3\n", p, err));
    CHECK(parseAircraftProfile("[aircraft]\nid = x\n", p, err));
    CHECK(p.flapDetents.size() == 1);  // a profile without flaps gets a single UP detent
}

TEST_CASE("profile: [crash] and [ground] sections travel in the properties map") {
    AircraftProfile p;
    std::string err;
    REQUIRE(parseAircraftProfile("[aircraft]\nid = x\n[crash]\nhard_landing_fpm = 700\n"
                                 "[ground]\nsteer_min_frac = 0.2\n[properties]\nreverser_on = 1\n",
                                 p, err));
    CHECK(p.properties.at("crash.hard_landing_fpm") == "700");
    CHECK(p.properties.at("ground.steer_min_frac") == "0.2");
    CHECK(p.properties.at("reverser_on") == "1");
    const CrashLimits c = crashLimitsFor(p, 4.0f);
    CHECK(c.hardLandingFpm == doctest::Approx(700.0f));
    CHECK(c.bellyAglFt == doctest::Approx(2.8f));
}

// ---- crash rule functions ---------------------------------------------------------------------

TEST_CASE("crash: limits depend on the category and can be overridden") {
    AircraftProfile piston, jet;
    piston.category = AircraftCategory::Piston;
    jet.category = AircraftCategory::Jet;
    const CrashLimits a = crashLimitsFor(piston, 4.0f), b = crashLimitsFor(jet, 4.0f);
    CHECK(a.hardLandingFpm < b.hardLandingFpm);
    CHECK(a.touchdownBankDeg > b.touchdownBankDeg);
    jet.properties["crash.touchdown_bank_deg"] = "5";
    CHECK(crashLimitsFor(jet, 4.0f).touchdownBankDeg == doctest::Approx(5.0f));
}

TEST_CASE("crash: touchdown verdicts") {
    AircraftProfile p;
    const CrashLimits c = crashLimitsFor(p, 4.0f);
    TouchdownSample s;
    s.sinkFpm = -300;
    s.closureFpm = -300;
    CHECK(evaluateTouchdownImpact(c, s) == nullptr);
    s.sinkFpm = -1500;
    CHECK(std::string(evaluateTouchdownImpact(c, s)) == "hard landing");
    s.sinkFpm = -300;
    s.closureFpm = -1500;  // closing on a slope faster than the limit
    CHECK(std::string(evaluateTouchdownImpact(c, s)) == "terrain impact");
    s.closureFpm = -300;
    s.rollRad = static_cast<float>(degToRad(40));
    CHECK(std::string(evaluateTouchdownImpact(c, s)) == "wing strike");
    s.rollRad = 0;
    s.pitchRad = static_cast<float>(degToRad(30));
    CHECK(std::string(evaluateTouchdownImpact(c, s)) == "tail strike");
    s.pitchRad = static_cast<float>(degToRad(-30));
    CHECK(std::string(evaluateTouchdownImpact(c, s)) == "nose strike");
}

TEST_CASE("crash: structure contact names map to reasons") {
    CHECK(std::string(structureCrashReason("LEFT_TIP")) == "wingtip strike");
    CHECK(std::string(structureCrashReason("TAIL_SKID")) == "tail strike");
    CHECK(std::string(structureCrashReason("NOSE_SKID")) == "nose strike");
    CHECK(std::string(structureCrashReason("ENG_1")) == "engine strike");
    CHECK(std::string(structureCrashReason("ROOF")) == "fuselage strike");
    CHECK(std::string(structureCrashReason("whatever")) == "structure strike");
    CHECK(std::string(structureCrashReason("VTAIL_TIP")) == "tail strike");
}

// ---- acceptance (a): takeoff ------------------------------------------------------------------

TEST_CASE("sim: c172p full-throttle takeoff lifts off between 50 and 65 kt with stick back") {
    Flight f;
    REQUIRE_MESSAGE(f.begin("c172p", "runway"), f.error);
    CHECK(f.s().wowAll);
    f.ctl.parkingBrake = false;
    f.ctl.throttleNorm[0] = 1.0f;
    double liftoffKt = -1;
    double maxHdgErrDeg = 0;
    for (long i = 0; i < 40 * kSimHz && liftoffKt < 0; ++i) {
        takeoffPilot(f, 52.0, 7.0, 270.0);
        f.step();
        const AircraftState& s = f.s();
        maxHdgErrDeg = std::max(maxHdgErrDeg, std::fabs(wrap180(radToDeg(s.headingTrueRad) - 270)));
        if (!s.wowAny) liftoffKt = s.iasKt;
    }
    INFO("liftoff IAS " << liftoffKt << " kt, max heading error " << maxHdgErrDeg << " deg");
    CHECK(liftoffKt >= 50.0);
    CHECK(liftoffKt <= 65.0);
    CHECK(maxHdgErrDeg < 5.0);
    f.run(10.0);
    CHECK_FALSE(f.s().crashed);
    CHECK(f.s().altAglFt > 40.0f);
    CHECK(f.s().vsFpm > 100.0f);
}

// ---- acceptance (c): reset without leaks -------------------------------------------------------

TEST_CASE("sim: 100 restarts do not grow resident memory") {
    Flight f;
    REQUIRE_MESSAGE(f.resolve("c172p", "runway"), f.error);
    // Warm up: the first loads allocate caches that are legitimately retained.
    for (int i = 0; i < 10; ++i) REQUIRE(f.model.start(f.profile, f.start));
    const size_t before = residentBytes();
    for (int i = 0; i < 100; ++i) {
        REQUIRE(f.model.restart());
        f.ctl = f.model.startControls();
        f.ctl.throttleNorm[0] = 0.5f;
        f.ctl.parkingBrake = false;
        for (int k = 0; k < 10; ++k) f.step();
    }
    const size_t after = residentBytes();
    INFO("RSS before " << before / 1024 << " KiB, after " << after / 1024 << " KiB");
    REQUIRE(before > 0);
    CHECK(after < before + 8u * 1024 * 1024);
    CHECK(f.s().valid);
    CHECK(f.s().simTimeS < 1.0);  // a restart really restarts
    // A different aircraft after many restarts.
    REQUIRE(f.begin("737", "cruise"));
    CHECK(f.s().engineCount == 2);
}

TEST_CASE("sim: starting an unknown model fails cleanly and a later start works") {
    Flight f;
    REQUIRE(f.resolve("c172p", "runway"));
    AircraftProfile bad = f.profile;
    bad.jsbsimModel = "no_such_aircraft";
    CHECK_FALSE(f.model.start(bad, f.start));
    CHECK_FALSE(f.model.started());
    CHECK(f.model.start(f.profile, f.start));
    CHECK(f.model.started());
}

// ---- acceptance (d): crashes -------------------------------------------------------------------

TEST_CASE("sim: a steep dive into the runway is a hard landing and freezes the model") {
    Flight f;
    REQUIRE_MESSAGE(f.begin("c172p", "final", "distance_nm = 0.4\n"), f.error);
    f.ctl.throttleNorm[0] = 0.2f;
    f.ctl.elevatorNorm = -0.35f;
    for (long i = 0; i < 30 * kSimHz && !f.s().crashed; ++i) f.step();
    REQUIRE(f.s().crashed);
    INFO("reason: " << f.s().crashReason);
    CHECK(std::string(f.s().crashReason) == "hard landing");
    const double t = f.s().simTimeS;
    f.run(2.0);
    CHECK(f.s().simTimeS == doctest::Approx(t));  // frozen
    CHECK(f.s().crashed);
}

TEST_CASE("sim: a hands-off 3 degree touchdown is firm but not a crash") {
    Flight f;
    REQUIRE_MESSAGE(f.begin("c172p", "final", "distance_nm = 0.25\n"), f.error);
    bool landed = false;
    for (long i = 0; i < 60 * kSimHz; ++i) {
        f.step();
        landed = landed || f.s().wowAny;
    }
    CHECK(landed);
    CHECK_FALSE(f.s().crashed);
}

TEST_CASE("sim: a flared c172p landing is not a crash and rolls out straight") {
    Flight f;
    REQUIRE_MESSAGE(f.begin("c172p", "final", "distance_nm = 0.6\n"), f.error);
    double touchdownVsFpm = 0;
    bool touched = false;
    for (long i = 0; i < 55 * kSimHz; ++i) {
        const AircraftState& s = f.s();
        const double aglFt = s.altAglFt - 4.3;  // CG height on the wheels is 4.3 ft
        f.ctl.aileronNorm = bankHoldAileron(s);
        if (!s.wowAny) {
            // Hands-off down the glidepath, then idle and a progressive flare below 30 ft.
            if (aglFt < 30.0) {
                f.ctl.throttleNorm[0] = 0.0f;
                const double pitchTargetDeg = -0.6 + (30.0 - std::max(aglFt, 0.0)) / 30.0 * 5.5;
                f.ctl.elevatorNorm = pitchHoldElevator(s, pitchTargetDeg, 0.05, 0.05);
            }
        } else {
            f.ctl.elevatorNorm = 0.1f;  // keep the nose wheel light on the roll-out
            f.ctl.rudderNorm = headingHoldRudder(s, 270.0);
            f.ctl.steeringNorm = f.ctl.rudderNorm;
        }
        const bool wasAir = !s.wowAny;
        const float vsBefore = s.vsFpm;
        f.step();
        if (wasAir && f.s().wowAny && !touched) {
            touched = true;
            touchdownVsFpm = vsBefore;
        }
        if (f.s().crashed) break;
    }
    INFO("touchdown vs " << touchdownVsFpm << " fpm, crash reason '" << f.s().crashReason << "'");
    CHECK(touched);
    CHECK_FALSE(f.s().crashed);
    CHECK(touchdownVsFpm > -400.0);
    CHECK(std::fabs(wrap180(radToDeg(f.s().headingTrueRad) - 270)) < 10.0);
}

TEST_CASE("sim: a gear-up 737 arriving at the ground is a gear-up landing") {
    Flight f;
    REQUIRE(f.resolve("737", "final", "distance_nm = 0.25\n"));
    f.start.gearDown = false;
    REQUIRE(f.begin());
    CHECK(f.s().gearPosNorm < 0.01f);
    CHECK_FALSE(f.s().wowAny);
    for (long i = 0; i < 30 * kSimHz && !f.s().crashed; ++i) f.step();
    REQUIRE(f.s().crashed);
    CHECK(std::string(f.s().crashReason) == "gear-up landing");
}

TEST_CASE("sim: touching down in a steep bank is a crash") {
    Flight f;
    REQUIRE(f.begin("c172p", "final", "distance_nm = 0.3\n"));
    for (long i = 0; i < 30 * kSimHz && !f.s().crashed; ++i) {
        // roll to 35 degrees of bank and hold it into the ground
        f.ctl.aileronNorm = bankHoldAileron(f.s(), 35.0);
        f.step();
    }
    REQUIRE(f.s().crashed);
    const std::string reason = f.s().crashReason;
    INFO("reason: " << reason);
    CHECK((reason == "wing strike" || reason == "wingtip strike" || reason == "hard landing"));
}

TEST_CASE("sim: touching down on water is a crash when a terrain source says so") {
    WaterTerrain water;
    Flight f;
    REQUIRE(f.begin("c172p", "final", "distance_nm = 0.25\n"));
    f.model.setTerrain(&water);
    for (long i = 0; i < 25 * kSimHz && !f.s().crashed; ++i) f.step();
    REQUIRE(f.s().crashed);
    CHECK(std::string(f.s().crashReason) == "water impact");
    // Same flight over land is fine (see the hands-off touchdown test).
    FlatTerrain land;
    REQUIRE(f.begin("c172p", "final", "distance_nm = 0.25\n"));
    f.model.setTerrain(&land);
    f.run(25.0);
    CHECK_FALSE(f.s().crashed);
}

TEST_CASE("sim: a parked aircraft with idle throttle never crashes") {
    for (const char* id : {"c172p", "737"}) {
        Flight f;
        REQUIRE_MESSAGE(f.begin(id, "runway"), f.error);
        f.run(20.0);
        CHECK_MESSAGE(!f.s().crashed, id << ": " << f.s().crashReason);
        CHECK(f.s().gsKt < 1.0f);
    }
}

// ---- acceptance (e): 737 cruise -----------------------------------------------------------------

TEST_CASE("sim: 737 cruise start holds altitude within 200 ft for 60 s") {
    Flight f;
    REQUIRE_MESSAGE(f.begin("737", "cruise"), f.error);
    const double alt0 = f.s().altMslFt;
    CHECK(alt0 == doctest::Approx(f.profile.cruiseAltFt).epsilon(0.001));
    CHECK(f.s().iasKt == doctest::Approx(f.profile.cruiseKt).epsilon(0.01));
    CHECK(f.s().engines[0].running);
    CHECK(f.s().engines[0].n1Pct > 60.0f);
    CHECK(f.s().engines[0].n2Pct > 60.0f);
    CHECK(f.s().engines[0].thrustLbf > 1000.0f);
    CHECK(f.s().engines[0].rpm == 0.0f);
    CHECK(f.s().mach > 0.7f);
    CHECK(f.s().mach < f.profile.mmo);
    double maxDevFt = 0;
    for (long i = 0; i < 60 * kSimHz; ++i) {
        f.step();
        maxDevFt = std::max(maxDevFt, std::fabs(f.s().altMslFt - alt0));
    }
    INFO("max altitude deviation " << maxDevFt << " ft");
    CHECK(maxDevFt < 200.0);
    CHECK_FALSE(f.s().crashed);
    CHECK_FALSE(f.s().overspeed);
    CHECK_FALSE(f.s().stallWarning);
    CHECK(std::fabs(radToDeg(f.s().rollRad)) < 2.0);
}

// ---- state mapping ------------------------------------------------------------------------------

TEST_CASE("sim: c172p runway start state") {
    Flight f;
    REQUIRE_MESSAGE(f.begin("c172p", "runway"), f.error);
    const AircraftState& s = f.s();
    CHECK(s.valid);
    CHECK(s.engineCount == 1);
    CHECK(s.engines[0].running);
    CHECK(s.engines[0].rpm > 500.0f);  // idling propeller
    CHECK(s.engines[0].n1Pct == 0.0f);
    CHECK(s.fuelTotalLbs == doctest::Approx(200.0f).epsilon(0.05));
    CHECK(s.terrainElevFt == doctest::Approx(2000.0f).epsilon(0.001));
    CHECK(s.altAglFt == doctest::Approx(4.4f).epsilon(0.1));
    CHECK(s.altMslFt == doctest::Approx(2004.4).epsilon(0.001));
    CHECK(s.parkingBrake);
    CHECK(s.brakeLeftNorm == doctest::Approx(1.0f));
    CHECK(s.wowAll);
    CHECK(s.wowAny);
    CHECK_FALSE(s.gearRetractable);
    CHECK(s.gearPosNorm == doctest::Approx(1.0f));
    CHECK(s.flapDetent == 0);
    CHECK(s.flapPosNorm == doctest::Approx(0.0f));
    CHECK(s.speedbrakePosNorm == 0.0f);
    CHECK(s.windKt == 0.0f);
    CHECK(s.nzG == doctest::Approx(1.0f).epsilon(0.05));
    CHECK(s.mach < 0.01f);
    // the eye sits a little forward of the CG and above it (body axes: x fwd, y right, z down)
    CHECK(s.eyeBodyM[0] > 0.0f);
    CHECK(s.eyeBodyM[0] < 0.4f);
    CHECK(s.eyeBodyM[2] < -0.15f);
    CHECK(s.eyeBodyM[2] > -0.5f);
    CHECK_FALSE(s.stallWarning);
    CHECK_FALSE(s.overspeed);
}

TEST_CASE("sim: 737 runway start state and per-engine data") {
    Flight f;
    REQUIRE_MESSAGE(f.begin("737", "runway"), f.error);
    f.ctl.parkingBrake = true;
    f.ctl.throttleNorm[0] = f.ctl.throttleNorm[1] = 0.5f;
    f.run(8.0);
    const AircraftState& s = f.s();
    CHECK(s.engineCount == 2);
    for (int i = 0; i < 2; ++i) {
        CHECK(s.engines[i].running);
        CHECK(s.engines[i].throttleCmdNorm == doctest::Approx(0.5f));
        CHECK(s.engines[i].n1Pct > 50.0f);
        CHECK(s.engines[i].n2Pct > 60.0f);
        CHECK(s.engines[i].thrustLbf > 3000.0f);
        CHECK_FALSE(s.engines[i].reverserDeployed);
    }
    CHECK(s.fuelTotalLbs > 20000.0f);
    CHECK(s.gearRetractable);
    CHECK(s.flapDetent == f.profile.takeoffFlapDetent);
    CHECK(s.flapPosNorm == doctest::Approx(0.125f).epsilon(0.05));
    CHECK(s.altAglFt == doctest::Approx(3.7f).epsilon(0.1));
    CHECK(s.gsKt < 1.0f);  // parking brake holds against half throttle
}

TEST_CASE("sim: fuel burns and the weight on wheels flags follow the aircraft") {
    Flight f;
    REQUIRE(f.begin("c172p", "runway"));
    const float fuel0 = f.s().fuelTotalLbs;
    f.ctl.parkingBrake = false;
    f.ctl.throttleNorm[0] = 1.0f;
    f.run(20.0);
    CHECK(f.s().fuelTotalLbs < fuel0);
    CHECK(f.s().wowAny);
    CHECK(f.s().engines[0].rpm > 2000.0f);
    CHECK(f.s().engines[0].thrustLbf > 100.0f);
}

TEST_CASE("sim: wind comes from the JSBSim atmosphere") {
    Flight f;
    REQUIRE(f.begin("c172p", "cruise", "", "[wind]\nfrom_true_deg = 330\nspeed_kt = 15\n"));
    f.run(2.0);
    const AircraftState& s = f.s();
    CHECK(s.windKt == doctest::Approx(15.0f).epsilon(0.02));
    CHECK(s.windFromTrueDeg == doctest::Approx(330.0f).epsilon(0.01));
    CHECK(s.iasKt == doctest::Approx(f.profile.cruiseKt).epsilon(0.02));
    CHECK(s.tasKt > s.iasKt);  // true airspeed above indicated at altitude
    // Heading is the runway heading (270); a wind from 330 is a left-front quartering headwind:
    // ground speed is below TAS.
    CHECK(s.gsKt < s.tasKt);
    CHECK(s.trackTrueRad != doctest::Approx(s.headingTrueRad).epsilon(0.0001));
}

TEST_CASE("sim: flaps, gear and speedbrake follow the controls with the right positions") {
    Flight f;
    REQUIRE(f.begin("737", "cruise", "altitude_ft = 12000\n"));
    f.ctl.flapDetent = 2;  // flaps 2
    f.ctl.speedbrakeNorm = 1.0f;
    f.run(6.0);
    CHECK(f.s().flapDetent == 2);
    CHECK(f.s().flapPosNorm == doctest::Approx(0.05f).epsilon(0.1));
    CHECK(f.s().speedbrakePosNorm == doctest::Approx(1.0f));
    f.ctl.speedbrakeNorm = 0.0f;
    f.ctl.gearDown = true;
    f.run(8.0);
    CHECK(f.s().speedbrakePosNorm == doctest::Approx(0.0f));
    CHECK(f.s().gearPosNorm == doctest::Approx(1.0f));
    f.ctl.gearDown = false;
    f.run(8.0);
    CHECK(f.s().gearPosNorm == doctest::Approx(0.0f));
    // Out-of-range detents are clamped
    f.ctl.flapDetent = 99;
    f.run(1.0);
    CHECK(f.s().flapDetent == 8);
}
