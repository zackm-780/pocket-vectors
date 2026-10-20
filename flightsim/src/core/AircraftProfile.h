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
};

}  // namespace fsim
