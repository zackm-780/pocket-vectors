#pragma once
// Flight setup model (WP H5): the selections made on the "New flight" screens (aircraft, airport,
// runway, start type, flight plan, approach) and the pure logic around them: discovering aircraft
// profiles and plan files, ordering airport search results, listing runways and approaches, and
// turning the selections into a Scenario for Simulation::init(). No GL, no UI state.

#include <string>
#include <vector>

#include "app/Scenario.h"
#include "core/AircraftProfile.h"
#include "core/NavDatabase.h"
#include "core/NavTypes.h"
#include "nav/ApproachBuilder.h"
#include "nav/Fms.h"
#include "nav/Procedures.h"

namespace fsim {

// ---- start types -------------------------------------------------------------------------------

enum class StartType { Runway, Final2Nm, Final6Nm, Final10Nm, InAir };
constexpr int kStartTypeCount = 5;
const char* startTypeLabel(StartType t);

// In-air altitude choices: 0 means "profile cruise altitude"; otherwise a multiple of 500 ft.
constexpr int kInAirAltStepFt = 500;
constexpr int kInAirAltMinFt = 1000;
constexpr int kInAirAltMaxFt = 45000;
// Moves the in-air altitude by `steps` of 500 ft (AUTO -> 3,000 -> ... and back down to AUTO).
// `fieldElevFt` keeps the minimum above the field.
int stepInAirAltitude(int currentFt, int steps, float fieldElevFt);

// ---- discovery ---------------------------------------------------------------------------------

struct AircraftEntry {
    std::string id, displayName;
    AircraftCategory category = AircraftCategory::Piston;
    int engineCount = 1;
    float cruiseAltFt = 0, cruiseKt = 0;
};
// Every assets/aircraft/*.ini that parses, sorted by display name. Unreadable files are skipped.
std::vector<AircraftEntry> discoverAircraft(const std::string& aircraftDir);
const char* categoryLabel(AircraftCategory c);

struct PlanFileEntry {
    std::string name;  // file name without directory, e.g. "KPDX-KOTH.fms"
    std::string path;
};
// *.fms files in the given directories (earlier directories win on equal names), sorted by name.
std::vector<PlanFileEntry> discoverPlanFiles(const std::vector<std::string>& dirs);

// ---- airports and runways ------------------------------------------------------------------------

struct AirportHit {
    const Airport* airport = nullptr;
    double distNm = -1;  // from the reference point, -1 = unknown
};
// Text search ordered like a pilot expects: exact ICAO ident first, then "K" + a 3 letter query
// (US FAA codes), then the database order (IATA, ident prefix, name). Airports without runways are
// left out. `refLat/LonDeg` fill distNm.
std::vector<AirportHit> searchAirportsOrdered(const NavDatabase& db, const std::string& query,
                                              int maxCount, double refLatDeg, double refLonDeg);
// Nearest airports that have runways, ascending distance.
std::vector<AirportHit> nearestAirportHits(const NavDatabase& db, double latDeg, double lonDeg,
                                           int maxCount, float maxDistNm);

// Runways (landing directions) sorted by number, then L, C, R.
std::vector<const Runway*> sortedRunways(const Airport& airport);
// Longest hard runway (ties: the first in sortedRunways order); any runway if none is hard.
const Runway* defaultRunway(const Airport& airport);
// "28R 294M 11000X150 ILS", magnetic heading from the true heading and the variation (+ east).
std::string runwayDescription(const Runway& runway, float magVarDeg);

// ---- approaches ----------------------------------------------------------------------------------

struct ApproachChoice {
    std::string id;     // "I05", "R28L", "VIS09", "I05-Z"
    std::string title;  // "ILS Z RWY 05" (real procedures: ApproachInfo::title)
    ApproachKind kind = ApproachKind::None;
    float minimumsFt = 0;
    bool minimumsIsDa = true;
    bool synthesized = false;
};
// Title for display: the CIFP title when `info` has one, else a name built from kind and runway.
std::string approachTitle(const Approach& approach, const ApproachInfo* info);
// Real approaches (through `provider`, N4) first, then synthesized ones (N5 availableApproaches).
// `navDir` lets real procedures show their ApproachInfo::title (empty = no titles from files).
std::vector<ApproachChoice> approachChoices(const NavDatabase& db, const std::string& airportIdent,
                                            const std::string& runwayIdent,
                                            const std::string& navDir,
                                            const RealApproachProvider& provider);

// ---- the selections ------------------------------------------------------------------------------

enum class PlanKind { None, File, Quick };

struct FlightSetup {
    std::string aircraftId = "c172p";
    std::string airportIdent = "XTST";  // start airport
    std::string runwayIdent = "27";
    StartType startType = StartType::Runway;
    int inAirAltFt = 0;  // StartType::InAir only, 0 = profile cruise altitude

    PlanKind planKind = PlanKind::None;
    std::string planName;  // file name or "QUICK KPDX-KOTH"
    FmsFile planFile;      // editable source of the plan (desrwy / app change here)
    FlightPlan plan;       // built from planFile
    // Approach: with a plan it is flown at the plan's destination, without one it is flown from the
    // start airport and runway (practice approach). Empty = none.
    std::string approachId;

    bool hasPlan() const { return planKind != PlanKind::None; }
    const std::string& destAirportIdent() const {
        return hasPlan() && !plan.destIdent.empty() ? plan.destIdent : airportIdent;
    }
    std::string destRunwayIdent() const { return hasPlan() ? plan.arrRunway : runwayIdent; }

    // Start airport: picks its default runway; clears a quick plan and the approach.
    bool setAirport(const NavDatabase& db, const std::string& ident);
    // Start runway (DB style). Clears the practice approach when there is no plan.
    void setRunway(const std::string& runwayIdentDb);
    // .fms file: the start airport and runway become the plan's origin, the approach its APP.
    bool loadPlanFile(const NavDatabase& db, const std::string& path, const std::string& name,
                      std::string& error);
    // Direct plan from the start airport and runway to `destIdent` (NAVIGATION 2.2).
    bool makeQuickPlan(const NavDatabase& db, const std::string& destIdent,
                       const std::string& destRunway, float cruiseAltFt, std::string& error);
    void clearPlan();
    // Plan destination runway (rebuilds the plan, drops the approach). No-op without a plan.
    bool setPlanDestRunway(const NavDatabase& db, const std::string& runwayIdentDb);
    void setApproach(const std::string& id);

    // Plan facts for display.
    std::string routeText() const;  // "KPDX UBG CVO EUG OTH KOTH"
    double planDistNm() const;

    // The scenario that starts this flight. With a plan, `planFilePath` is the .fms that
    // writePlanFile() wrote (the Scenario only references plans by path).
    Scenario buildScenario(const std::string& planFilePath) const;
    // Writes planFile (with the chosen approach) to `path`. False without a plan or on IO failure.
    bool writePlanFile(const std::string& path, std::string& error) const;
};

// Magnetic heading of a runway for display.
float runwayHeadingMagDeg(const Runway& runway, float magVarDeg);

}  // namespace fsim
