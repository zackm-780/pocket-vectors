// WP F1 tests: control inputs (reversers, mixture, steering, brakes, trims), warnings, wind and
// restart. Acceptance tests and crash rules are in FlightModelTest.cpp.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include "FlightHarness.h"

using namespace fsim;
using namespace fsim::test;

TEST_CASE("sim: reversers deploy only on the ground at idle and give negative thrust") {
    Flight f;
    REQUIRE(f.begin("737", "runway"));
    f.ctl.parkingBrake = false;
    f.ctl.throttleNorm[0] = f.ctl.throttleNorm[1] = 1.0f;
    auto holdCenterline = [&](double seconds) {
        for (long i = 0; i < static_cast<long>(seconds * kSimHz); ++i) {
            f.ctl.rudderNorm = f.ctl.steeringNorm = headingHoldRudder(f.s(), 270.0);
            f.step();
        }
    };
    // Asking for reverse with the throttles up is refused.
    f.ctl.reverseThrust = true;
    holdCenterline(12.0);
    CHECK_FALSE(f.s().engines[0].reverserDeployed);
    CHECK(f.s().engines[0].thrustLbf > 5000.0f);
    CHECK(f.s().gsKt > 30.0f);
    // Throttles to idle with the request still up: the reversers deploy.
    f.ctl.throttleNorm[0] = f.ctl.throttleNorm[1] = 0.0f;
    holdCenterline(3.0);
    CHECK(f.s().engines[0].reverserDeployed);
    CHECK(f.s().engines[1].reverserDeployed);
    // Reverse power: thrust goes negative and the aircraft decelerates.
    const float gs0 = f.s().gsKt;
    f.ctl.throttleNorm[0] = f.ctl.throttleNorm[1] = 0.9f;
    holdCenterline(5.0);
    CHECK(f.s().engines[0].thrustLbf < -1000.0f);
    CHECK(f.s().gsKt < gs0);
    // Releasing the request stows them again.
    f.ctl.reverseThrust = false;
    f.ctl.throttleNorm[0] = f.ctl.throttleNorm[1] = 0.0f;
    holdCenterline(3.0);
    CHECK_FALSE(f.s().engines[0].reverserDeployed);
    CHECK(f.s().engines[0].thrustLbf > -100.0f);
}

TEST_CASE("sim: reverse is ignored in the air") {
    Flight f;
    REQUIRE(f.begin("737", "cruise", "altitude_ft = 8000\n"));
    f.ctl.throttleNorm[0] = f.ctl.throttleNorm[1] = 0.0f;
    f.ctl.reverseThrust = true;
    f.run(3.0);
    CHECK_FALSE(f.s().engines[0].reverserDeployed);
    CHECK(f.s().engines[0].thrustLbf >= 0.0f);
}

TEST_CASE("sim: mixture cutoff stops a piston engine") {
    Flight f;
    REQUIRE(f.begin("c172p", "runway"));
    f.ctl.throttleNorm[0] = 0.3f;
    f.run(2.0);
    CHECK(f.s().engines[0].running);
    f.ctl.mixtureNorm = 0.0f;
    f.run(5.0);
    CHECK_FALSE(f.s().engines[0].running);
    CHECK(f.s().engines[0].thrustLbf < 5.0f);
}

TEST_CASE("sim: steering and differential brakes turn the right way") {
    auto headingChangeDeg = [](float steering, float leftBrake) {
        Flight f;
        REQUIRE(f.begin("c172p", "runway"));
        f.ctl.parkingBrake = false;
        f.ctl.throttleNorm[0] = 0.45f;
        f.run(5.0);  // get rolling at taxi speed
        f.ctl.throttleNorm[0] = 0.2f;
        f.ctl.steeringNorm = steering;
        f.ctl.brakeLeftNorm = leftBrake;
        f.run(3.0);
        return wrap180(radToDeg(f.s().headingTrueRad) - 270.0);
    };
    CHECK(std::fabs(headingChangeDeg(0.0f, 0.0f)) < 1.0);
    CHECK(headingChangeDeg(0.7f, 0.0f) > 3.0);  // + steering = right turn (heading increases)
    CHECK(headingChangeDeg(-0.7f, 0.0f) < -3.0);
    CHECK(headingChangeDeg(0.0f, 1.0f) < -1.5);  // left brake pulls left
}

TEST_CASE("sim: parking brake holds and brake positions show in the state") {
    Flight f;
    REQUIRE(f.begin("c172p", "runway"));
    f.ctl.throttleNorm[0] = 0.4f;
    f.run(3.0);
    CHECK(f.s().parkingBrake);
    CHECK(f.s().gsKt < 1.0f);
    f.ctl.parkingBrake = false;
    f.ctl.brakeLeftNorm = 0.3f;
    f.ctl.brakeRightNorm = 0.6f;
    f.run(0.5);
    CHECK_FALSE(f.s().parkingBrake);
    CHECK(f.s().brakeLeftNorm == doctest::Approx(0.3f));
    CHECK(f.s().brakeRightNorm == doctest::Approx(0.6f));
}

TEST_CASE("sim: trims are reported and the sign conversion is symmetric") {
    Flight f;
    REQUIRE(f.begin("c172p", "cruise"));
    f.ctl.elevTrimNorm = 0.2f;
    f.ctl.aileronTrimNorm = -0.1f;
    f.ctl.rudderTrimNorm = 0.15f;
    f.run(0.5);
    CHECK(f.s().elevTrimNorm == doctest::Approx(0.2f));
    CHECK(f.s().aileronTrimNorm == doctest::Approx(-0.1f));
    CHECK(f.s().rudderTrimNorm == doctest::Approx(0.15f));
    // JSBSim's own properties hold the opposite sign for elevator and rudder trim.
    CHECK(f.model.getProperty("fcs/pitch-trim-cmd-norm") == doctest::Approx(-0.2));
    CHECK(f.model.getProperty("fcs/yaw-trim-cmd-norm") == doctest::Approx(-0.15));
}

// ---- warnings -----------------------------------------------------------------------------------

TEST_CASE("sim: stall warning comes on before the stall and does not chatter") {
    Flight f;
    REQUIRE(f.begin("c172p", "cruise", "altitude_ft = 8000\n"));
    const float warnDeg = f.profile.stallWarnAlphaDeg;
    f.ctl.throttleNorm[0] = 0.0f;
    int risingEdges = 0;
    bool prev = false, warned = false;
    float alphaAtWarning = 0, maxAlpha = 0;
    for (long i = 0; i < 90 * kSimHz; ++i) {
        const double targetDeg = 2.0 + 0.1 * f.s().simTimeS;  // slow pitch ramp: speed bleeds off
        f.ctl.elevatorNorm = static_cast<float>(std::clamp(
            f.ctl.elevatorNorm + 0.0003 * (targetDeg - radToDeg(f.s().pitchRad)), -1.0, 1.0));
        f.ctl.aileronNorm = bankHoldAileron(f.s());
        f.step();
        const AircraftState& s = f.s();
        maxAlpha = std::max(maxAlpha, s.alphaDeg);
        if (s.stallWarning) {
            CHECK(s.alphaDeg > warnDeg - 1.01f);  // hysteresis band
            if (!prev) ++risingEdges;
            if (!warned) alphaAtWarning = s.alphaDeg;
            warned = true;
        } else {
            CHECK(s.alphaDeg <= warnDeg + 0.001f);
        }
        prev = s.stallWarning;
        if (s.vsFpm < -2500) break;
    }
    INFO("alpha at warning " << alphaAtWarning << ", max alpha " << maxAlpha);
    CHECK(warned);
    CHECK(alphaAtWarning < maxAlpha - 1.0f);  // the warning leads the stall
    CHECK(risingEdges <= 3);
}

TEST_CASE("sim: overspeed follows Vne for pistons and Mmo for jets") {
    Flight f;
    REQUIRE(f.resolve("c172p", "cruise"));
    f.profile.vneKt = 100.0f;  // cruise IAS is above this
    REQUIRE(f.begin());
    CHECK(f.s().overspeed);
    f.profile.vneKt = 163.0f;
    REQUIRE(f.begin());
    CHECK_FALSE(f.s().overspeed);
    Flight j;
    REQUIRE(j.resolve("737", "cruise"));
    j.profile.mmo = 0.70f;  // cruise is Mach 0.76
    REQUIRE(j.begin());
    CHECK(j.s().overspeed);
}

// ---- wind and restart -----------------------------------------------------------------------------

TEST_CASE("sim: a crosswind final crabs so the ground track stays on the centerline") {
    Flight f;
    REQUIRE_MESSAGE(f.begin("c172p", "final", "distance_nm = 2\n",
                            "[wind]\nfrom_true_deg = 180\nspeed_kt = 15\n"),
                    f.error);
    f.run(3.0);
    const AircraftState& s = f.s();
    CHECK(s.iasKt == doctest::Approx(65.0f).epsilon(0.02));
    CHECK(radToDeg(s.trackTrueRad) == doctest::Approx(270.0).epsilon(0.01));
    // Wind from the south, flying west: nose pointed into the wind, i.e. south of west.
    const double crabDeg = wrap180(radToDeg(s.headingTrueRad) - 270.0);
    CHECK(crabDeg < -8.0);
    CHECK(crabDeg > -20.0);
    CHECK(s.windKt == doctest::Approx(15.0f).epsilon(0.02));
}

TEST_CASE("sim: restart reproduces the same flight") {
    Flight f;
    REQUIRE(f.begin("c172p", "cruise"));
    f.run(5.0);
    const AircraftState a = f.s();
    REQUIRE(f.model.restart());
    f.ctl = f.model.startControls();
    f.run(5.0);
    const AircraftState& b = f.s();
    CHECK(a.latDeg == b.latDeg);
    CHECK(a.altMslFt == b.altMslFt);
    CHECK(a.iasKt == b.iasKt);
}
