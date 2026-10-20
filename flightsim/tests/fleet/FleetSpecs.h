#pragma once
// Per-aircraft numbers for the fleet flyability tests (WP F2). The profile .ini files hold what the
// HUD and the rest of the game need; this table holds what the scripted test pilot needs (how
// hard to rotate, which speed to climb at, where to flare, ...) plus the pass/fail bands.
// Speeds are indicated airspeed in knots, heights in feet above the runway.

namespace fsim::test::fleet {

struct FleetSpec {
    const char* id;
    double cgOnGearFt;      // altAglFt of the parked aircraft (CG height on its wheels)
    double gearTouchAglFt;  // altAglFt at which the main wheels first touch (flare reference)

    // ---- takeoff (full power, flaps at the profile takeoff detent) ----
    double rotateKt, rotatePitchDeg;
    double rotateKp, rotateKd;    // pitch loop gains during rotation
    double groundKp, groundKd;    // heading loop gains (rudder and nose wheel) on the ground
    double liftMinKt, liftMaxKt;  // accepted liftoff speed band
    double maxLiftoffDistFt;

    // ---- climb ----
    double climbKt;  // speed target with takeoff flaps below 400 ft AGL
    double cleanKt;  // speed target above 400 ft AGL (flaps retract on the way)
    double climbPitchDeg, climbPitchPerKt, climbPitchMinDeg, climbPitchMaxDeg;
    double stallCleanKt;   // measured clean stall speed (slow idle deceleration, 1 g)
    double flapRetractKt;  // start retracting flaps above this speed
    double flapStepS;      // seconds between flap steps
    double maxClimbTimeS;  // to 3,000 ft AGL

    // ---- pitch / speed loops ----
    double pitchKp, pitchKd;
    double vsIntegral, vsProportional;  // deg/s per fpm, deg per fpm
    double thrProportional, thrIntegral;
    double pathGainFpmPerFt;
    double apchPitchMinDeg, apchPitchMaxDeg;

    // ---- flare and rollout ----
    double flareAglFt, flareTauS, touchdownSinkFps, flarePitchMaxDeg;
    double flarePitchDeltaDeg, flareVsGain;  // scheduled round-out (0 = vertical speed law)
    double idleAglFt, idleRatePerS;
    double reverseDelayS, reverseThrottle, reverseOffKt;
    double rolloutPitchDeg, brakeDelayS, brakeNorm;
    double landStartNm;   // final start distance for the landing test
    double maxLandingFt;  // threshold to stop
    double maxTouchdownSinkFpm;
};

// clang-format off
inline const FleetSpec kFleet[] = {
    {"c172p", 4.35, 4.35,
     52, 7, 0.5, 0.1, 0.05, 0.04, 50, 65, 3000,
     76, 80, 8, 0.3, 2, 14, 51, 62, 3, 600,
     0.08, 0.06, 0.001, 0.002, 0.01, 0.002, 3.0, -6, 8,
     20, 3.5, 1.0, 9, 0, 0, 12, 0.5, 0, 0, 0, 0, 1.0, 1.0, 2.5, 3000, 500},
    {"c310", 3.33, 3.33,
     80, 8, 0.5, 0.1, 0.05, 0.04, 75, 95, 4000,
     100, 115, 8, 0.3, 3, 13, 75, 95, 3, 600,
     0.08, 0.06, 0.001, 0.002, 0.01, 0.002, 3.0, -6, 9,
     20, 3.5, 1.0, 9, 0, 0, 12, 0.5, 0, 0, 0, 0, 1.0, 1.0, 2.5, 3500, 500},
    {"global5000", 8.79, 12.0,
     125, 10, 0.5, 0.1, 0.05, 0.04, 120, 145, 5000,
     160, 200, 9, 0.2, 3, 15, 150, 190, 3, 600,
     0.06, 0.05, 0.001, 0.002, 0.01, 0.002, 3.0, -6, 8,
     40, 4.5, 1.0, 7, 3.5, 0.004, 30, 0.5, 1.5, 0.6, 50, 0, 2.5, 0.8, 3.0, 6000, 600},
    {"737", 3.69, 4.1,
     145, 10, 0.5, 0.1, 0.05, 0.04, 140, 170, 6000,
     165, 225, 9, 0.2, 3, 15, 152, 190, 3, 600,
     0.06, 0.05, 0.001, 0.002, 0.01, 0.002, 3.0, -6, 8,
     40, 4.5, 1.0, 7, 3.5, 0.004, 30, 0.5, 1.5, 0.6, 50, 0, 2.5, 0.8, 3.0, 7000, 600},
    {"B747", 13.46, 21.0,
     140, 10, 0.5, 0.1, 0.05, 0.04, 140, 185, 8000,
     170, 230, 9, 0.2, 3, 12, 156, 200, 3, 800,
     0.06, 0.05, 0.002, 0.003, 0.01, 0.002, 3.0, -6, 7,
     50, 5.5, 1.0, 7, 5.0, 0.004, 35, 0.5, 1.5, 0.6, 50, 0, 2.5, 0.8, 3.0, 9000, 600},
};
// clang-format on

}  // namespace fsim::test::fleet
