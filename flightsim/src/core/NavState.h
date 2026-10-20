#pragma once
// Navigator output. Produced by nav/Navigator (N5), read by the HUD, the map and the autopilot.

namespace fsim {

enum class NavSource { None, Gps, Vor, Loc };
enum class NavPhase { None, Departure, Enroute, Terminal, Approach, Missed };

struct NavState {
    NavSource source = NavSource::None;
    NavPhase phase = NavPhase::None;
    char activeWptIdent[8] = {0};
    double wptLatDeg = 0, wptLonDeg = 0;
    char nextWptIdent[8] = {0};
    float distToWptNm = 0, eteToWptS = 0, distToDestNm = 0, eteToDestS = 0;
    float bearingToWptTrueDeg = 0, desiredTrackTrueDeg = 0;
    float crossTrackNm = 0;    // + = aircraft is right of course
    float cdiDeflection = 0;   // -1..1 = full scale (2 dots); + = course is to the RIGHT
    float cdiFullScaleNm = 0;  // GPS scaling in use (0 when angular)
    bool toFlag = true;
    // vertical
    bool gsValid = false;
    float gsDeflection = 0;  // -1..1 full scale; + = glidepath is ABOVE aircraft
    float targetAltFt = 0;
    float requiredVsFpm = 0;  // advisory VNAV
    // approach
    bool locValid = false;
    char approachName[16] = {0};
    float minimumsFt = 0;
    bool minimumsIsDa = true;  // DA (MSL) or MDA (MSL)
    float distToThresholdNm = 0;
    float magVarDeg = 0;  // at aircraft position, + = east
    // Additive (N5). Advisory VNAV and bookkeeping; all zero / false when not applicable.
    bool vnavValid = false;   // advisory 3 degree path to targetAltFt exists (aircraft above it)
    float vnavDevFt = 0;      // path altitude - aircraft altitude: + = path ABOVE the aircraft
    float todDistNm = 0;      // distance to the top of descent along the route; 0 = at/past it
    int activeLegIndex = -1;  // FlightPlan::legs index of the active leg, -1 = none
    bool onFinal = false;     // active leg lies on the final approach course (non-missed)
};

}  // namespace fsim
