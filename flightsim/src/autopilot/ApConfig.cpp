#include "autopilot/ApConfig.h"

#include <algorithm>

namespace fsim {

namespace {

void applyKey(ApConfig& c, const std::string& key, float v) {
    struct Entry {
        const char* name;
        float ApConfig::*field;
    };
    static const Entry kEntries[] = {
        {"bank_limit_deg", &ApConfig::bankLimitDeg},
        {"heading_tau_s", &ApConfig::headingTauS},
        {"nav_tau_s", &ApConfig::navTauS},
        {"max_intercept_deg", &ApConfig::maxInterceptDeg},
        {"loc_max_intercept_deg", &ApConfig::locMaxInterceptDeg},
        {"rol_wings_level_deg", &ApConfig::rollHoldWingsLevelDeg},
        {"bank_rate_limit_degps", &ApConfig::bankRateLimitDegps},
        {"roll_kp", &ApConfig::rollKp},
        {"roll_kd", &ApConfig::rollKd},
        {"roll_ki", &ApConfig::rollKi},
        {"rudder_beta_gain", &ApConfig::rudderBetaGain},
        {"rudder_rate_gain", &ApConfig::rudderRateGain},
        {"pitch_max_deg", &ApConfig::pitchMaxDeg},
        {"pitch_min_deg", &ApConfig::pitchMinDeg},
        {"pitch_rate_limit_degps", &ApConfig::pitchRateLimitDegps},
        {"pitch_kp", &ApConfig::pitchKp},
        {"pitch_kd", &ApConfig::pitchKd},
        {"pitch_ki", &ApConfig::pitchKi},
        {"trim_gain_per_s", &ApConfig::trimGainPerS},
        {"trim_max_rate_per_s", &ApConfig::trimMaxRatePerS},
        {"vs_kp_rel", &ApConfig::vsKpRel},
        {"vs_ki_rel", &ApConfig::vsKiRel},
        {"vs_max_climb_fpm", &ApConfig::vsMaxClimbFpm},
        {"vs_max_descent_fpm", &ApConfig::vsMaxDescentFpm},
        {"vs_default_fpm", &ApConfig::vsDefaultFpm},
        {"alt_capture_gain_fpm_per_ft", &ApConfig::altCaptureGainFpmPerFt},
        {"alt_hold_gain_fpm_per_ft", &ApConfig::altHoldGainFpmPerFt},
        {"alt_capture_min_ft", &ApConfig::altCaptureMinFt},
        {"alt_hold_within_ft", &ApConfig::altHoldWithinFt},
        {"gs_tau_s", &ApConfig::gsTauS},
        {"gs_max_corr_fpm", &ApConfig::gsMaxCorrFpm},
        {"min_climb_ias_kt", &ApConfig::minClimbIasKt},
        {"at_kp", &ApConfig::atKp},
        {"at_ki", &ApConfig::atKi},
        {"at_k_accel", &ApConfig::atKAccel},
        {"at_slew_per_s", &ApConfig::atSlewPerS},
        {"retard_agl_ft", &ApConfig::retardAglFt},
        {"retard_slew_per_s", &ApConfig::retardSlewPerS},
        {"stick_disconnect", &ApConfig::stickDisconnect},
        {"stick_disconnect_s", &ApConfig::stickDisconnectS},
        {"bank_disconnect_deg", &ApConfig::bankDisconnectDeg},
        {"pitch_disconnect_deg", &ApConfig::pitchDisconnectDeg},
        {"spd_bug_min_kt", &ApConfig::spdBugMinKt},
        {"spd_bug_max_kt", &ApConfig::spdBugMaxKt},
    };
    for (const Entry& e : kEntries)
        if (key == e.name) {
            c.*(e.field) = v;
            return;
        }
}

}  // namespace

ApConfig ApConfig::fromProfile(const AircraftProfile& p) {
    ApConfig c;
    switch (p.category) {
        case AircraftCategory::Jet:
            c.bankLimitDeg = 25;
            c.headingTauS = 4.5f;
            c.navTauS = 25;
            c.bankRateLimitDegps = 3.5f;
            c.pitchRateLimitDegps = 2.0f;
            c.pitchKp = 0.06f;
            c.pitchKd = 0.05f;
            c.vsMaxClimbFpm = 3000;
            c.vsMaxDescentFpm = 3000;
            c.vsDefaultFpm = 1500;
            c.pitchMaxDeg = 15;
            c.pitchMinDeg = -10;
            c.atSlewPerS = 0.25f;
            c.retardAglFt = 30;
            break;
        case AircraftCategory::Turboprop:
            c.bankLimitDeg = 25;
            c.vsMaxClimbFpm = 2000;
            c.vsMaxDescentFpm = 2000;
            c.vsDefaultFpm = 1000;
            break;
        case AircraftCategory::Piston:
            c.vsDefaultFpm = p.engineCount > 1 ? 800.f : 500.f;
            c.vsMaxClimbFpm = p.engineCount > 1 ? 1500.f : 1000.f;
            c.vsMaxDescentFpm = 1200;
            break;
    }
    if (p.vs1Kt > 0) c.minClimbIasKt = 1.25f * p.vs1Kt;
    for (const auto& [key, value] : p.autopilot) applyKey(c, key, value);
    return c;
}

}  // namespace fsim
