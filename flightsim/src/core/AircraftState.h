#pragma once
// Snapshot of the simulated aircraft. Produced by sim/FlightModel, read by everyone else.
// A zero-initialised value means "invalid" (valid == false).

namespace fsim {

constexpr int kMaxEngines = 4;

struct EngineState {
    bool running = false;
    float throttleCmdNorm = 0;   // 0..1 as commanded
    float n1Pct = 0, n2Pct = 0;  // turbines (0 for pistons)
    float rpm = 0;               // pistons/props (0 for jets)
    float thrustLbf = 0;
    bool reverserDeployed = false;
};

struct AircraftState {
    bool valid = false;
    double simTimeS = 0;
    // position
    double latDeg = 0, lonDeg = 0, altMslFt = 0;
    float altAglFt = 0;  // above terrain/runway surface under the CG
    float terrainElevFt = 0;
    // attitude and rates (NED)
    float rollRad = 0, pitchRad = 0, headingTrueRad = 0;
    float pRadps = 0, qRadps = 0, rRadps = 0;
    // air data
    float iasKt = 0, tasKt = 0, mach = 0, alphaDeg = 0, betaDeg = 0, vsFpm = 0, nzG = 1;
    // ground data
    float gsKt = 0, trackTrueRad = 0;
    float velNorthMps = 0, velEastMps = 0, velDownMps = 0;
    // wind at aircraft
    float windFromTrueDeg = 0, windKt = 0;
    // eye point relative to CG, body axes (x fwd, y right, z down), meters
    float eyeBodyM[3] = {0, 0, 0};
    // configuration
    float flapPosNorm = 0;  // actual surface position 0..1
    int flapDetent = 0;     // currently selected detent index
    float gearPosNorm = 1;  // 1 = down and locked
    bool gearRetractable = false;
    bool wowAny = false, wowAll = false;  // weight on wheels
    float speedbrakePosNorm = 0;
    float elevTrimNorm = 0, aileronTrimNorm = 0, rudderTrimNorm = 0;  // -1..1
    float brakeLeftNorm = 0, brakeRightNorm = 0;
    bool parkingBrake = false;
    int engineCount = 0;
    EngineState engines[kMaxEngines];
    float fuelTotalLbs = 0;
    // warnings
    bool stallWarning = false, overspeed = false;
    bool crashed = false;
    char crashReason[48] = {0};
};

}  // namespace fsim
