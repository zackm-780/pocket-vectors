#include "app/Trace.h"

#include "core/Units.h"

namespace fsim {

bool TraceWriter::open(const std::string& path) {
    return csv_.open(path, {"t_s",
                            "lat_deg",
                            "lon_deg",
                            "alt_msl_ft",
                            "alt_agl_ft",
                            "ias_kt",
                            "tas_kt",
                            "gs_kt",
                            "vs_fpm",
                            "heading_true_deg",
                            "track_true_deg",
                            "pitch_deg",
                            "roll_deg",
                            "alpha_deg",
                            "beta_deg",
                            "nz_g",
                            "p_degps",
                            "q_degps",
                            "r_degps",
                            "flap_pos_norm",
                            "gear_pos_norm",
                            "elev_trim_norm",
                            "wow_any",
                            "brake_l_norm",
                            "eng0_throttle_norm",
                            "eng0_n1_pct",
                            "eng0_rpm",
                            "eng0_thrust_lbf",
                            "fuel_lbs",
                            "stall_warn",
                            "crashed",
                            "crash_reason",
                            "nav_source",
                            "nav_phase",
                            "nav_dist_to_wpt_nm",
                            "nav_xtk_nm",
                            "nav_cdi_norm",
                            "nav_gs_defl_norm",
                            "ap_engaged",
                            "ap_lat_mode",
                            "ap_vert_mode",
                            "ap_spd_mode",
                            // N5: more NavState fields (appended; earlier columns keep their place)
                            "nav_wpt",
                            "nav_leg",
                            "nav_dtk_true_deg",
                            "nav_brg_to_wpt_true_deg",
                            "nav_dist_to_dest_nm",
                            "nav_dist_to_thr_nm",
                            "nav_cdi_scale_nm",
                            "nav_to_flag",
                            "nav_loc_valid",
                            "nav_gs_valid",
                            "nav_on_final",
                            "nav_target_alt_ft",
                            "nav_req_vs_fpm",
                            "nav_vnav_dev_ft",
                            "nav_mag_var_deg"});
}

void TraceWriter::write(const AircraftState& s, const NavState& nav, const AutopilotState& ap) {
    if (!csv_.isOpen()) return;
    csv_.addDouble(s.simTimeS, 2);
    csv_.addDouble(s.latDeg, 7);
    csv_.addDouble(s.lonDeg, 7);
    csv_.addDouble(s.altMslFt, 1);
    csv_.addDouble(s.altAglFt, 1);
    csv_.addDouble(s.iasKt, 1);
    csv_.addDouble(s.tasKt, 1);
    csv_.addDouble(s.gsKt, 1);
    csv_.addDouble(s.vsFpm, 0);
    csv_.addDouble(radToDeg(s.headingTrueRad), 2);
    csv_.addDouble(radToDeg(s.trackTrueRad), 2);
    csv_.addDouble(radToDeg(s.pitchRad), 2);
    csv_.addDouble(radToDeg(s.rollRad), 2);
    csv_.addDouble(s.alphaDeg, 2);
    csv_.addDouble(s.betaDeg, 2);
    csv_.addDouble(s.nzG, 3);
    csv_.addDouble(radToDeg(s.pRadps), 2);
    csv_.addDouble(radToDeg(s.qRadps), 2);
    csv_.addDouble(radToDeg(s.rRadps), 2);
    csv_.addDouble(s.flapPosNorm, 3);
    csv_.addDouble(s.gearPosNorm, 3);
    csv_.addDouble(s.elevTrimNorm, 3);
    csv_.addInt(s.wowAny ? 1 : 0);
    csv_.addDouble(s.brakeLeftNorm, 2);
    csv_.addDouble(s.engines[0].throttleCmdNorm, 3);
    csv_.addDouble(s.engines[0].n1Pct, 1);
    csv_.addDouble(s.engines[0].rpm, 0);
    csv_.addDouble(s.engines[0].thrustLbf, 1);
    csv_.addDouble(s.fuelTotalLbs, 1);
    csv_.addInt(s.stallWarning ? 1 : 0);
    csv_.addInt(s.crashed ? 1 : 0);
    csv_.addString(s.crashReason);
    csv_.addInt(static_cast<int>(nav.source));
    csv_.addInt(static_cast<int>(nav.phase));
    csv_.addDouble(nav.distToWptNm, 2);
    csv_.addDouble(nav.crossTrackNm, 3);
    csv_.addDouble(nav.cdiDeflection, 3);
    csv_.addDouble(nav.gsDeflection, 3);
    csv_.addInt(ap.apEngaged ? 1 : 0);
    csv_.addInt(static_cast<int>(ap.latActive));
    csv_.addInt(static_cast<int>(ap.vertActive));
    csv_.addInt(static_cast<int>(ap.spdActive));
    csv_.addString(nav.activeWptIdent);
    csv_.addInt(nav.activeLegIndex);
    csv_.addDouble(nav.desiredTrackTrueDeg, 2);
    csv_.addDouble(nav.bearingToWptTrueDeg, 2);
    csv_.addDouble(nav.distToDestNm, 2);
    csv_.addDouble(nav.distToThresholdNm, 3);
    csv_.addDouble(nav.cdiFullScaleNm, 2);
    csv_.addInt(nav.toFlag ? 1 : 0);
    csv_.addInt(nav.locValid ? 1 : 0);
    csv_.addInt(nav.gsValid ? 1 : 0);
    csv_.addInt(nav.onFinal ? 1 : 0);
    csv_.addDouble(nav.targetAltFt, 0);
    csv_.addDouble(nav.requiredVsFpm, 0);
    csv_.addDouble(nav.vnavDevFt, 0);
    csv_.addDouble(nav.magVarDeg, 2);
    csv_.endRow();
}

}  // namespace fsim
