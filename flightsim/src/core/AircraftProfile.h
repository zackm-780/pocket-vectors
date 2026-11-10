#pragma once
// Static description of an aircraft, parsed from assets/aircraft/<id>.ini by sim/.

#include <map>
#include <string>
#include <vector>

namespace fsim {

struct FlapDetent {
    float cmdNorm = 0;    // value written to fcs/flap-cmd-norm
    char label[8] = {0};  // "UP", "10", "15"...
    // --- additive (F1) ---
    float vfeKt = 0;  // max flaps-extended speed with this detent selected; 0 = use profile vfeKt
};

enum class AircraftCategory { Piston, Turboprop, Jet };

struct AircraftProfile {
    std::string id, displayName, jsbsimModel;
    AircraftCategory category = AircraftCategory::Piston;
    int engineCount = 1;
    std::vector<FlapDetent> flapDetents;
    bool hasRetractableGear = false, hasSpeedbrake = false, hasReversers = false,
         hasAutothrottle = false;
    // speeds, kt IAS
    float vs0Kt = 0, vs1Kt = 0, vrKt = 0, vrefKt = 0, vappKt = 0, vfeKt = 0, vnoKt = 0, vneKt = 0;
    float vmoKt = 0, mmo = 0, cruiseKt = 0, cruiseAltFt = 0;
    float stallWarnAlphaDeg = 0;
    // start configurations
    int approachFlapDetent = 0, takeoffFlapDetent = 0;
    // autopilot gains and limits (N6 defines the keys; generic map keeps core stable)
    std::map<std::string, float> autopilot;
    // JSBSim property overrides for non-standard models (F1 defines the keys)
    std::map<std::string, std::string> properties;

    // --- additive (WP-00) ---
    // Eye point offset from the CG in body axes (x fwd, y right, z down), meters. Used when the
    // JSBSim model has no EYEPOINT. Profile key: [view] eye_offset_m.
    float eyeOffsetBodyM[3] = {0, 0, 0};
    bool hasEyeOffset = false;

    // --- additive (V1) ---
    // External-view model built in code: [view] external_model = single | twin | bizjet |
    // airliner | heavy (empty: from category and engine count) and [view] wingspan_m (0: the
    // model's own span).
    std::string externalModel;
    float wingspanM = 0;
    // [view] gear_height_m: height of the JSBSim reference point above the runway on the wheels
    // (measured), so the external model's wheels touch the ground. 0 = unknown.
    float gearHeightM = 0;

    // --- additive (F6) ---
    // [cruise] fuel_flow_lbs_hr: all engines at the cruise altitude and speed (measured); AUTO fuel
    // plans with it. 0 = unknown (AUTO then loads full tanks).
    float cruiseFuelFlowLbsHr = 0;

    // --- additive (F5) ---
    // [warnings] stall_warn_alpha_by_flap_deg: the stall warning angle of attack at each flap
    // detent, in the order of flapDetents (empty: stallWarnAlphaDeg at every flap setting).
    std::vector<float> stallWarnAlphaByFlapDeg;
    // [speeds_kt] vs_by_flap: the 1 g stall speed (kt IAS) measured at each flap detent at the
    // model's shipped weight (empty: unknown). H9 places the flap bugs and V2 from it.
    std::vector<float> vsByFlapKt;
};

// F5: the stall warning angle of attack for the actual flap position (AircraftState::flapPosNorm),
// interpolated between the detents' commands; stallWarnAlphaDeg when the profile has no per-detent
// values. 0 = no stall warning. The flight model and the HUD both use this.
inline float stallWarnAlphaForFlapDeg(const AircraftProfile& p, float flapPosNorm) {
    const std::vector<float>& a = p.stallWarnAlphaByFlapDeg;
    const size_t n = a.size();
    if (n == 0 || n != p.flapDetents.size()) return p.stallWarnAlphaDeg;
    if (n == 1 || flapPosNorm <= p.flapDetents[0].cmdNorm) return a[0];
    for (size_t i = 1; i < n; ++i) {
        const float lo = p.flapDetents[i - 1].cmdNorm, hi = p.flapDetents[i].cmdNorm;
        if (flapPosNorm <= hi) {
            const float t = hi > lo ? (flapPosNorm - lo) / (hi - lo) : 1.0f;
            return a[i - 1] + t * (a[i] - a[i - 1]);
        }
    }
    return a[n - 1];
}

}  // namespace fsim
