// F10: armed speed brakes. State sequences for sim/SpeedbrakeArm, then headless 737 flights at
// XTST through Simulation: an armed landing (against the same landing unarmed) and a go-around.
#include <doctest/doctest.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "SimFixture.h"
#include "sim/SpeedbrakeArm.h"

using namespace fsim;
using namespace fsim::test;

namespace {

AircraftProfile jet() {
    AircraftProfile p;
    p.engineCount = 2;
    p.hasSpeedbrake = true;
    return p;
}

AircraftState air() {
    AircraftState s;
    s.valid = true;
    s.wowAny = false;
    return s;
}

AircraftState ground() {
    AircraftState s = air();
    s.wowAny = true;
    return s;
}

void throttles(ControlInputs& c, float v) {
    c.throttleNorm[0] = c.throttleNorm[1] = v;
}

}  // namespace

TEST_CASE("sim: F10 armed speed brakes deploy at touchdown at idle, within one step") {
    const AircraftProfile p = jet();
    SpeedbrakeArm arm;
    ControlInputs c;
    c.speedbrakeArmed = true;
    throttles(c, 0.0f);
    CHECK(arm.update(air(), p, c) == nullptr);  // on final, armed
    CHECK(c.speedbrakeNorm == 0.0f);
    const char* msg = arm.update(ground(), p, c);  // the first wheel contact
    REQUIRE(msg != nullptr);
    CHECK(std::string(msg) == "SPEEDBRAKE DEPLOYED");
    CHECK(c.speedbrakeNorm == 1.0f);
    CHECK(c.speedbrakeArmed);                      // still armed
    CHECK(arm.update(ground(), p, c) == nullptr);  // one message
    CHECK(c.speedbrakeNorm == 1.0f);
}

TEST_CASE("sim: F10 thrust above idle at touchdown: they deploy when it reaches idle") {
    const AircraftProfile p = jet();
    SpeedbrakeArm arm;
    ControlInputs c;
    c.speedbrakeArmed = true;
    throttles(c, 0.3f);
    arm.update(air(), p, c);
    CHECK(arm.update(ground(), p, c) == nullptr);
    CHECK(c.speedbrakeNorm == 0.0f);
    throttles(c, 0.05f);  // idle
    CHECK(std::string(arm.update(ground(), p, c)) == "SPEEDBRAKE DEPLOYED");
    CHECK(c.speedbrakeNorm == 1.0f);
}

TEST_CASE("sim: F10 deployed by the arm, thrust above half stows and disarms") {
    const AircraftProfile p = jet();
    SpeedbrakeArm arm;
    ControlInputs c;
    c.speedbrakeArmed = true;
    arm.update(air(), p, c);
    arm.update(ground(), p, c);
    REQUIRE(c.speedbrakeNorm == 1.0f);
    throttles(c, 0.4f);  // not yet a go-around
    CHECK(arm.update(ground(), p, c) == nullptr);
    CHECK(c.speedbrakeNorm == 1.0f);
    c.throttleNorm[1] = 0.6f;  // any lever above half
    CHECK(std::string(arm.update(ground(), p, c)) == "SPEEDBRAKE STOWED");
    CHECK(c.speedbrakeNorm == 0.0f);
    CHECK_FALSE(c.speedbrakeArmed);
    CHECK_FALSE(arm.deployedByArm());
    // After a bounce, in the air: the same.
    c.speedbrakeArmed = true;
    throttles(c, 0.0f);
    arm.update(air(), p, c);
    arm.update(ground(), p, c);
    REQUIRE(arm.deployedByArm());
    throttles(c, 1.0f);
    CHECK(std::string(arm.update(air(), p, c)) == "SPEEDBRAKE STOWED");
}

TEST_CASE("sim: F10 arming on the ground at idle does not deploy; no arm without speedbrakes") {
    const AircraftProfile p = jet();
    SpeedbrakeArm arm;
    ControlInputs c;
    c.speedbrakeArmed = true;
    throttles(c, 0.0f);
    for (int i = 0; i < 100; ++i) CHECK(arm.update(ground(), p, c) == nullptr);
    CHECK(c.speedbrakeNorm == 0.0f);
    CHECK_FALSE(arm.airborneSinceArmed());
    // Takeoff and landing: then it does.
    arm.update(air(), p, c);
    CHECK(std::string(arm.update(ground(), p, c)) == "SPEEDBRAKE DEPLOYED");
    // A disarm (X, a manual position) clears the latch; a new flight resets it.
    c.speedbrakeArmed = false;
    c.speedbrakeNorm = 0.0f;
    CHECK(arm.update(ground(), p, c) == nullptr);
    CHECK_FALSE(arm.deployedByArm());
    c.speedbrakeArmed = true;
    arm.update(air(), p, c);
    arm.reset();
    CHECK_FALSE(arm.airborneSinceArmed());
    // No speedbrakes: nothing happens even when armed.
    AircraftProfile piston;
    piston.engineCount = 1;
    SpeedbrakeArm a2;
    ControlInputs c2;
    c2.speedbrakeArmed = true;
    a2.update(air(), piston, c2);
    CHECK(a2.update(ground(), piston, c2) == nullptr);
    CHECK(c2.speedbrakeNorm == 0.0f);
}

namespace {

struct LandingRun {
    double touchdownS = -1, commandFullS = -1, surfaceFullS = -1, stopAlongFt = 0;
    double startLat = 0, startLon = 0, tdLat = 0, tdLon = 0, stopLat = 0, stopLon = 0;
    bool stopped = false;
    std::string firstMessage;
};

// 737_land_armed_xtst.ini, armed or not (the [inputs] arm line removed).
LandingRun land(bool armed) {
    SimFixture f;
    std::string text;
    {
        FILE* in =
            std::fopen(f.paths.asset("assets/scenarios/737_land_armed_xtst.ini").c_str(), "r");
        REQUIRE(in != nullptr);
        char line[256];
        while (std::fgets(line, sizeof line, in))
            if (armed || std::strstr(line, "speedbrake_arm") == nullptr) text += line;
        std::fclose(in);
    }
    REQUIRE_MESSAGE(f.initText(text), f.error);
    LandingRun r;
    unsigned events = f.sim.speedbrakeEventCount();
    for (long i = 0; i < 200L * kSimHz; ++i) {
        f.sim.stepOnce();
        const AircraftState& s = f.sim.aircraft();
        if (r.touchdownS < 0 && s.wowAny) {
            r.touchdownS = s.simTimeS;
            r.tdLat = s.latDeg;
            r.tdLon = s.lonDeg;
        }
        if (r.commandFullS < 0 && f.sim.pilot().speedbrakeNorm >= 1.0f) r.commandFullS = s.simTimeS;
        if (r.surfaceFullS < 0 && s.speedbrakePosNorm >= 0.99f) r.surfaceFullS = s.simTimeS;
        if (f.sim.speedbrakeEventCount() != events && r.firstMessage.empty()) {
            events = f.sim.speedbrakeEventCount();
            r.firstMessage = f.sim.lastSpeedbrakeEvent();
        }
        if (r.touchdownS > 0 && s.gsKt < 1.0f) {
            r.stopped = true;
            r.stopLat = s.latDeg;
            r.stopLon = s.lonDeg;
            break;
        }
        REQUIRE_FALSE(s.crashed);
    }
    return r;
}

double distFt(double lat1, double lon1, double lat2, double lon2) {
    return mToFt(greatCircleDistM(lat1, lon1, lat2, lon2));
}

}  // namespace

TEST_CASE("sim: F10 headless 737 landing: armed spoilers at touchdown, a shorter rollout") {
    const LandingRun armed = land(true);
    const LandingRun plain = land(false);
    REQUIRE(armed.touchdownS > 0);
    REQUIRE(armed.stopped);
    REQUIRE(plain.stopped);
    CHECK(armed.firstMessage == "SPEEDBRAKE DEPLOYED");
    CHECK(armed.commandFullS - armed.touchdownS <= 0.2);  // commanded at the first contact
    CHECK(armed.surfaceFullS > 0);
    const double rollArmed = distFt(armed.tdLat, armed.tdLon, armed.stopLat, armed.stopLon);
    const double rollPlain = distFt(plain.tdLat, plain.tdLon, plain.stopLat, plain.stopLon);
    std::printf("[F10] 737 landing: touchdown %.2f s, speed brake command 1.0 at +%.2f s, surface "
                "full at +%.2f s; rollout %.0f ft armed, %.0f ft unarmed\n",
                armed.touchdownS, armed.commandFullS - armed.touchdownS,
                armed.surfaceFullS - armed.touchdownS, rollArmed, rollPlain);
    CHECK(rollArmed < rollPlain);
    CHECK(plain.commandFullS < 0);  // unarmed: never out
}

TEST_CASE("sim: F10 headless 737 go-around: stowed within 0.2 s of the thrust, climbs away") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initFile("737_goaround_armed_xtst.ini"), f.error);
    double deployedS = -1, thrustS = -1, stowedS = -1, maxAglFt = 0;
    for (long i = 0; i < 100L * kSimHz; ++i) {
        f.sim.stepOnce();
        const AircraftState& s = f.sim.aircraft();
        const ControlInputs& c = f.sim.pilot();
        if (deployedS < 0 && c.speedbrakeNorm >= 1.0f) deployedS = s.simTimeS;
        if (deployedS > 0 && thrustS < 0 && c.throttleNorm[0] > 0.5f) thrustS = s.simTimeS;
        if (thrustS > 0 && stowedS < 0 && c.speedbrakeNorm == 0.0f) stowedS = s.simTimeS;
        if (thrustS > 0) maxAglFt = std::max(maxAglFt, static_cast<double>(s.altAglFt));
        REQUIRE_FALSE(s.crashed);
    }
    REQUIRE(deployedS > 0);
    REQUIRE(thrustS > 0);
    REQUIRE(stowedS > 0);
    std::printf("[F10] 737 go-around: deployed %.2f s, thrust up %.2f s, stowed +%.2f s, max AGL "
                "%.0f ft\n",
                deployedS, thrustS, stowedS - thrustS, maxAglFt);
    CHECK(stowedS - thrustS <= 0.2);
    CHECK_FALSE(f.sim.pilot().speedbrakeArmed);
    CHECK(f.sim.lastSpeedbrakeEvent() != nullptr);
    CHECK(std::string(f.sim.lastSpeedbrakeEvent()) == "SPEEDBRAKE STOWED");
    CHECK(maxAglFt > 500.0);  // it climbs away
}
