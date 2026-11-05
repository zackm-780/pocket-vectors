#pragma once
// Autopilot gains and limits (WP N6). Every value has a category default (piston / turboprop /
// jet); a profile's [autopilot] section overrides any of them by key (see applyKey in
// ApConfig.cpp for the key names, documented in src/autopilot/README.md).

#include "core/AircraftProfile.h"

namespace fsim {

struct ApConfig {
    // ---- lateral ----
    float bankLimitDeg = 20;        // max commanded bank (25 for jets, NAVIGATION 5)
    float headingTauS = 3.5f;       // HDG: heading error closes with this time constant
    float navTauS = 20;             // NAV / LOC: look-ahead time, intercept = atan(xtk / (gs tau))
    float maxInterceptDeg = 45;     // NAV: largest intercept angle
    float locMaxInterceptDeg = 30;  // LOC: largest intercept angle
    float rollHoldWingsLevelDeg = 6;  // ROL: |bank| <= this holds wings level
    float bankRateLimitDegps = 5;     // slew limit of the bank command
    // roll inner loop: aileron per deg, per deg/s, per deg*s
    float rollKp = 0.05f, rollKd = 0.02f, rollKi = 0.004f;
    // turn coordination: rudder per deg of sideslip, per deg/s of yaw-rate error
    float rudderBetaGain = 0.02f, rudderRateGain = 0.02f;
    // ---- vertical ----
    float pitchMaxDeg = 15, pitchMinDeg = -12;
    float pitchRateLimitDegps = 2.5f;
    // pitch inner loop: elevator per deg, per deg/s, per deg*s
    float pitchKp = 0.08f, pitchKd = 0.06f, pitchKi = 0.01f;
    float trimGainPerS = 0.1f;      // elevator-integrator offload into trim (1/s)
    float trimMaxRatePerS = 0.02f;  // trim travel limit
    // VS outer loop, relative gains (pitch deg per deg of flight-path error / per second)
    float vsKpRel = 0.4f, vsKiRel = 0.2f;
    float vsMaxClimbFpm = 1000, vsMaxDescentFpm = 1500, vsDefaultFpm = 500;
    float altCaptureGainFpmPerFt = 6;  // |altErr| < |VS| / 6 starts the capture
    float altHoldGainFpmPerFt = 4;
    float altCaptureMinFt = 100, altHoldWithinFt = 25;
    float gsTauS = 12, gsMaxCorrFpm = 500;
    float minClimbIasKt = 0;  // underspeed protection without A/T (0 = 1.25 * vs1)
    float alphaLimitDeg = 0;  // pitch demand is cut above this angle of attack (0 = off)
    // ---- FLC / go-around (N7): pitch on speed ----
    float spdTauS = 5;              // speed error closes with this time constant (s)
    float spdAccelTauS = 1.0f;      // low-pass of the IAS acceleration the speed law uses (s)
    float spdMaxDeltaVsFpm = 2500;  // largest VS change the speed law asks for at once
    float gaPitchDeg = 10;          // go-around attitude: the GA pitch command never exceeds it
    float gaPitchRateDegps = 3;  // pitch command slew in GA (faster rotation than the normal limit)
    float climbThrottleNorm = 1.0f;  // A/T throttle in an FLC climb (THR CLB)
    float gaThrottleNorm = 1.0f;     // A/T throttle in GA (TOGA)
    // ---- autothrottle ----
    float atKp = 0.015f, atKi = 0.004f, atKAccel = 0.05f;  // per kt, per kt*s, per kt/s
    float atAvailable = 0;  // 1 = offer the A/T even if the profile has no autothrottle
    float atSlewPerS = 0.3f;
    float retardAglFt = 30, retardSlewPerS = 0.4f;
    // ---- override ----
    float stickDisconnect = 0.5f, stickDisconnectS = 0.5f;
    float bankDisconnectDeg = 45, pitchDisconnectDeg = 25;
    float minimumsDisconnectMarginFt = 50;
    // bug limits
    float spdBugMinKt = 40, spdBugMaxKt = 400;

    static ApConfig fromProfile(const AircraftProfile& profile);
};

}  // namespace fsim
