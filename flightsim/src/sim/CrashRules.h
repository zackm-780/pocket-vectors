#pragma once
// Crash rules (WP F1). Pure functions with no JSBSim dependency so the numbers can be unit
// tested. FlightModel gathers the inputs each step and asks these functions for a verdict.
//
// A crash is declared for:
//   - touchdown sink rate above the hard-landing limit (by category, profile override),
//   - bank or pitch beyond the limits while the wheels are making contact (touchdown window),
//   - any non-wheel structure contact (wingtip, tail, nose, engine, belly...),
//   - gear-up landing (retractable gear not down and locked, belly at the ground),
//   - terrain and water contact (TerrainQuery supplied by F3 through FlightModel::setTerrain).

#include <string>

#include "core/AircraftProfile.h"

namespace fsim {

struct CrashLimits {
    float hardLandingFpm = 900;        // sink rate at first wheel contact beyond which we crash
    float touchdownBankDeg = 25;       // |bank| limit during the touchdown window
    float touchdownPitchUpDeg = 20;    // nose-up limit (tail strike)
    float touchdownPitchDownDeg = 15;  // nose-down limit (nose strike)
    float bellyAglFt = 0;              // CG height above ground at which the belly touches
    float touchdownWindowS = 1.0f;     // how long after a wheel touches the attitude limits apply
};

// Category defaults, then profile overrides ([crash] section, stored as "crash.<key>" in
// AircraftProfile::properties). gearHeightFt is the CG height above the wheel contact points
// (used for the default belly height, 0.7 x gearHeightFt).
CrashLimits crashLimitsFor(const AircraftProfile& profile, float gearHeightFt);

struct TouchdownSample {
    float sinkFpm = 0;     // vertical speed at first contact, + = climbing (negative on impact)
    float closureFpm = 0;  // vertical speed relative to the terrain surface (same sign)
    float rollRad = 0;
    float pitchRad = 0;
};

// Verdict at the instant a wheel first touches. nullptr = fine.
const char* evaluateTouchdownImpact(const CrashLimits& limits, const TouchdownSample& s);
// Verdict for attitude while the touchdown window is open and a wheel carries weight.
const char* evaluateTouchdownAttitude(const CrashLimits& limits, const TouchdownSample& s);

// Crash reason for contact of a non-wheel (structure) contact point, from its name in the model.
const char* structureCrashReason(const std::string& contactName);

}  // namespace fsim
