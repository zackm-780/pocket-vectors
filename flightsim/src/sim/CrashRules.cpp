#include "sim/CrashRules.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>

#include "core/Units.h"

namespace fsim {
namespace {

float propFloat(const AircraftProfile& p, const char* key, float def) {
    const auto it = p.properties.find(key);
    if (it == p.properties.end() || it->second.empty()) return def;
    char* end = nullptr;
    const float v = std::strtof(it->second.c_str(), &end);
    return end == it->second.c_str() ? def : v;
}

bool contains(const std::string& upper, const char* needle) {
    return upper.find(needle) != std::string::npos;
}

}  // namespace

CrashLimits crashLimitsFor(const AircraftProfile& profile, float gearHeightFt) {
    CrashLimits c;
    switch (profile.category) {
    case AircraftCategory::Piston:
        c.hardLandingFpm = 900;
        c.touchdownBankDeg = 25;
        c.touchdownPitchUpDeg = 20;
        c.touchdownPitchDownDeg = 15;
        break;
    case AircraftCategory::Turboprop:
        c.hardLandingFpm = 1000;
        c.touchdownBankDeg = 18;
        c.touchdownPitchUpDeg = 14;
        c.touchdownPitchDownDeg = 10;
        break;
    case AircraftCategory::Jet:
        c.hardLandingFpm = 1100;
        c.touchdownBankDeg = 12;
        c.touchdownPitchUpDeg = 12;
        c.touchdownPitchDownDeg = 8;
        break;
    }
    c.bellyAglFt = 0.7f * std::max(gearHeightFt, 0.f);
    c.hardLandingFpm = propFloat(profile, "crash.hard_landing_fpm", c.hardLandingFpm);
    c.touchdownBankDeg = propFloat(profile, "crash.touchdown_bank_deg", c.touchdownBankDeg);
    c.touchdownPitchUpDeg =
        propFloat(profile, "crash.touchdown_pitch_up_deg", c.touchdownPitchUpDeg);
    c.touchdownPitchDownDeg =
        propFloat(profile, "crash.touchdown_pitch_down_deg", c.touchdownPitchDownDeg);
    c.bellyAglFt = propFloat(profile, "crash.belly_agl_ft", c.bellyAglFt);
    c.touchdownWindowS = propFloat(profile, "crash.touchdown_window_s", c.touchdownWindowS);
    return c;
}

const char* evaluateTouchdownImpact(const CrashLimits& limits, const TouchdownSample& s) {
    if (-s.sinkFpm > limits.hardLandingFpm) return "hard landing";
    // Closing on rising terrain faster than the limit is a terrain impact even when the vertical
    // speed alone is modest (flying the gear into a slope).
    if (-s.closureFpm > limits.hardLandingFpm) return "terrain impact";
    return evaluateTouchdownAttitude(limits, s);
}

const char* evaluateTouchdownAttitude(const CrashLimits& limits, const TouchdownSample& s) {
    if (std::fabs(radToDeg(s.rollRad)) > limits.touchdownBankDeg) return "wing strike";
    if (radToDeg(s.pitchRad) > limits.touchdownPitchUpDeg) return "tail strike";
    if (radToDeg(s.pitchRad) < -limits.touchdownPitchDownDeg) return "nose strike";
    return nullptr;
}

const char* structureCrashReason(const std::string& contactName) {
    std::string u = contactName;
    for (char& ch : u) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    if (contains(u, "TIP")) return contains(u, "TAIL") ? "tail strike" : "wingtip strike";
    if (contains(u, "TAIL")) return "tail strike";
    if (contains(u, "NOSE")) return "nose strike";
    if (contains(u, "ENG") || contains(u, "POD") || contains(u, "NACELLE")) return "engine strike";
    if (contains(u, "WING")) return "wing strike";
    if (contains(u, "ROOF") || contains(u, "CABIN") || contains(u, "BELLY") ||
        contains(u, "FUSELAGE"))
        return "fuselage strike";
    return "structure strike";
}

}  // namespace fsim
