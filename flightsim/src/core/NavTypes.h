#pragma once
// Navigation data types shared by nav/, the HUD, the map and the autopilot.

#include <optional>
#include <string>
#include <vector>

namespace fsim {

enum class AirportType { Large, Medium, Small, Seaplane, Unknown };

struct Ils {
    char ident[6] = {0};
    float freqMhz = 0;  // 0 = untuned (synthesized ILS)
    double locLatDeg = 0, locLonDeg = 0;
    float locCourseTrueDeg = 0;
    float locWidthDeg = 5;  // TOTAL course width (full-scale left to full-scale right)
    double gsLatDeg = 0, gsLonDeg = 0;
    float gsElevFt = 0;
    float gsAngleDeg = 3;
    float tchFt = 50;
    bool hasGs = true;
    bool synthesized = false;
};

// ONE LANDING DIRECTION (e.g. "28R").
struct Runway {
    std::string airportIdent, ident, oppositeIdent;
    double thrLatDeg = 0, thrLonDeg = 0;
    float thrElevFt = 0;                      // landing threshold (displacement applied)
    double startLatDeg = 0, startLonDeg = 0;  // physical start of pavement
    double endLatDeg = 0, endLonDeg = 0;
    float endElevFt = 0;  // far end
    float headingTrueDeg = 0, lengthFt = 0, widthFt = 0, displacedFt = 0;
    bool hard = true, lighted = false, approxGeometry = false;
    std::optional<Ils> ils;
};

struct Airport {
    std::string ident, name, iso;
    double latDeg = 0, lonDeg = 0;
    float elevFt = 0;
    AirportType type = AirportType::Unknown;
    std::vector<Runway> runways;  // both directions of every runway
};

enum class NavaidType { Vor, VorDme, Vortac, Tacan, Ndb, NdbDme, Dme };

struct Navaid {
    std::string ident, name;
    NavaidType type = NavaidType::Vor;
    float freqKhz = 0;
    double latDeg = 0, lonDeg = 0;
    float elevFt = 0;
    float magVarDeg = 0;
    float rangeNm = 0;
};

enum class WaypointKind { Airport, Runway, Vor, Ndb, Fix, LatLon, Synthetic };
enum class AltConstraintKind { None, At, AtOrAbove, AtOrBelow, Between };

struct AltConstraint {
    AltConstraintKind kind = AltConstraintKind::None;
    float altFt = 0, alt2Ft = 0;
};

struct Waypoint {
    std::string ident;
    WaypointKind kind = WaypointKind::Fix;
    double latDeg = 0, lonDeg = 0;
};

enum class LegType { IF, TF, CF, DF, CA, VA, FA, HM, HA, HF, RF };  // ARINC 424 subset

struct Leg {
    LegType type = LegType::TF;
    Waypoint to;
    float courseTrueDeg = 0;  // CF/CA/VA/FA
    AltConstraint alt;
    float speedLimitKt = 0;
    bool flyOver = false;
    bool missedApproach = false;
    double arcCenterLatDeg = 0, arcCenterLonDeg = 0;
    float arcRadiusNm = 0;
    bool arcRight = true;  // RF
};

enum class ApproachKind { None, Ils, Loc, Rnav, Vor, Ndb, Visual };

// One instrument/visual approach to one runway, as produced by nav/Procedures (N4, real FAA CIFP)
// or nav/ApproachBuilder (N5, synthesized for any runway). Inserted into a FlightPlan as
// transition legs + finalLegs + missedLegs (missedLegs all have missedApproach = true).
struct ApproachTransition {
    std::string name;  // IAF / transition ident; empty = "vectors" (straight to the final segment)
    std::vector<Leg> legs;
};
struct Approach {
    std::string
        id;  // CIFP-style ident without "RW": "I05", "R28L", "V10", "L13" (see NAVIGATION 2.1)
    std::string airportIdent, runwayIdent;  // runwayIdent in DB style ("05", "28L")
    ApproachKind kind = ApproachKind::None;
    std::vector<ApproachTransition> transitions;
    std::vector<Leg> finalLegs;   // IF/FAF ... MAP
    std::vector<Leg> missedLegs;  // after the MAP
    float minimumsFt = 0;         // DA or MDA, ft MSL; 0 = unknown
    bool minimumsIsDa = true;
    bool synthesized = false;
    // Real approaches only (N4): index into finalLegs of the FAF, -1 = none/unknown.
    int fafFinalLeg = -1;
    // Real approaches only (N4): the chart title says COPTER, so fixed-wing short ids skip it.
    bool helicopterOnly = false;
};

struct FlightPlan {
    std::string originIdent, destIdent, depRunway, arrRunway, approachId;
    ApproachKind approachKind = ApproachKind::None;
    float cruiseAltFt = 0;
    std::vector<Leg> legs;
    int activeLeg = -1;  // index into legs, -1 = none
    // Index of the first approach leg (transition, final and missed legs run to the end of legs);
    // -1 = no approach inserted. Maintained by nav/FlightPlan insertApproach() (N3).
    int approachStartLeg = -1;
};

}  // namespace fsim
