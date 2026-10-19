#pragma once
// Flight plan logic on top of the shared core types (core/NavTypes.h FlightPlan / Leg): building
// from .fms files, writing back, quick plans, editing, direct-to, distances and ETE, and the
// approach insertion API used by N4 (procedures) and N5 (approach builder).
//
// Plan layout (convention, see README_FMS.md):
//   legs[0]            IF  origin: runway start ("RW28L", kind Runway) if depRunway is known and
//                      resolves, else the airport ("KPDX", kind Airport). After a direct-to this is
//                      the present position (ident "PPOS", kind Synthetic).
//   legs[1..n-2]       TF  enroute waypoints (alt constraint = the .fms altitude).
//   last leg           TF  destination: runway threshold ("RW05") if arrRunway resolves, else the
//                      airport. Present only while no approach is inserted.
//   legs[approachStartLeg..]  approach + missed legs (replace the destination leg).
// All headings are TRUE; magnetic is only produced through an explicit magvar callback.

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/Geo.h"
#include "core/NavDatabase.h"
#include "core/NavTypes.h"
#include "nav/Fms.h"

namespace fsim {

// Magnetic variation at a point, degrees, + = east (same sign as NavDatabase::magVarDeg).
using MagVarFn = std::function<float(double latDeg, double lonDeg)>;
// Navaids within radiusNm of a point (not part of the core NavDatabase contract; N2 can supply it).
using NavaidsNearFn =
    std::function<std::vector<const Navaid*>(double latDeg, double lonDeg, float radiusNm)>;

// ---- ident resolution -----------------------------------------------------------------------

struct IdentMatch {
    Waypoint wp;
    double distNm = 0;        // from the reference point
    std::string description;  // "VOR Newberg", "Airport Portland Intl", "Fix"
};
// All airports, navaids and fixes called `ident`, nearest to (refLatDeg, refLonDeg) first. This is
// what the UI lists for an ambiguous ident.
std::vector<IdentMatch> resolveIdent(const NavDatabase& db, std::string_view ident,
                                     double refLatDeg, double refLonDeg);

// ---- .fms <-> FlightPlan ---------------------------------------------------------------------

struct PlanBuildResult {
    bool ok = false;
    FlightPlan plan;
    std::vector<std::string> errors;    // readable, "line N: ..." where a line applies
    std::vector<std::string> warnings;  // ambiguity decisions, unknown runways, ...
};

// Builds a plan from a parsed file. Idents are resolved through `db`; when several navaids or
// fixes share an ident the one closest to the file's coordinates (if within 2 NM) or, failing
// that, to the previous waypoint is used. Unknown idents use the file's coordinates; so does a
// DB match more than 25 NM from the file's coordinates.
PlanBuildResult buildPlanFromFms(const FmsFile& file, const NavDatabase& db);
// parseFms + buildPlanFromFms; parse errors are returned in `errors`. `fileOut` (optional) receives
// the parsed file so SID/STAR/CYCLE lines can be preserved by planToFms.
PlanBuildResult loadPlanFromFmsText(std::string_view text, const NavDatabase& db,
                                    FmsFile* fileOut = nullptr);
PlanBuildResult loadPlanFromFmsFile(const std::string& path, const NavDatabase& db,
                                    FmsFile* fileOut = nullptr);

// Plan -> file structure. Approach legs are not exported (the file carries only the APP name);
// SID/STAR/CYCLE/transition lines are copied from `headerTemplate` if given. Airways are not
// remembered, every leg is written as DRCT.
FmsFile planToFms(const FlightPlan& plan, const NavDatabase& db,
                  const FmsFile* headerTemplate = nullptr);
std::string writePlanFmsText(const FlightPlan& plan, const NavDatabase& db,
                             const FmsFile* headerTemplate = nullptr);
bool writePlanFmsFile(const std::string& path, const FlightPlan& plan, const NavDatabase& db,
                      const FmsFile* headerTemplate = nullptr, std::string* error = nullptr);

// ARINC 424 approach ident ("I05", "R29-Y", "VDM") -> kind.
ApproachKind approachKindFromId(std::string_view approachId);

// ---- quick plan ------------------------------------------------------------------------------

struct QuickPlanRequest {
    std::string originIdent;                // empty: start at presentPos
    std::string originRunway;               // DB style ("28L"), optional
    std::optional<LatLon> presentPos;       // used when originIdent is empty
    std::string destIdent;                  // required
    std::string destRunway;                 // DB style, optional
    float cruiseAltFt = 0;                  // 0 = none
    bool viaNavaids = false;                // NAVIGATION 2.2: VORs <= 15 NM off the great circle,
    NavaidsNearFn navaidsNear;              // <= 120 NM apart; needs this provider
    float viaMaxOffsetNm = 15, viaMaxLegNm = 120;
};
struct QuickPlanResult {
    bool ok = false;
    FlightPlan plan;
    std::string error;
    std::vector<std::string> warnings;
};
QuickPlanResult buildQuickPlan(const QuickPlanRequest& req, const NavDatabase& db);

// ---- editing ---------------------------------------------------------------------------------

// Index of the first leg after the enroute part: approachStartLeg when an approach is inserted,
// the destination leg otherwise (legs.size() if the plan has none).
int enrouteEndIndex(const FlightPlan& plan);
// Inserts a TF leg so that it becomes legs[index]; valid range is 1..enrouteEndIndex (0 for an
// empty plan). Keeps activeLeg and approachStartLeg consistent.
bool insertWaypoint(FlightPlan& plan, int index, const Waypoint& wp,
                    const AltConstraint& alt = AltConstraint());
// Deletes an enroute leg (1..enrouteEndIndex-1). The origin, destination and approach legs are
// removed through other calls.
bool deleteLeg(FlightPlan& plan, int index);
bool setAltConstraint(FlightPlan& plan, int index, const AltConstraint& alt);

// Direct-to legs[legIndex].to (any leg with a fix, except the origin): the plan becomes
// [PPOS IF][DF to target][rest]; activeLeg = 1.
bool directToLeg(FlightPlan& plan, int legIndex, const LatLon& present);
// Direct-to an arbitrary waypoint, then straight on to the plan tail (approach or destination).
void directToWaypoint(FlightPlan& plan, const Waypoint& wp, const LatLon& present);
// Direct-to an ident: a waypoint already in the plan wins, else the nearest DB match to `present`.
bool directToIdent(FlightPlan& plan, std::string_view ident, const NavDatabase& db,
                   const LatLon& present);

// ---- approach insertion (N4 / N5 call these) ------------------------------------------------

// Replaces everything after the last enroute waypoint (the destination leg or a previously
// inserted approach) with `approachLegs`, in order, keeping their missedApproach flags (missed
// legs must be last). Sets approachId, approachKind, arrRunway (DB style, if non-empty) and
// approachStartLeg. If the active leg was in the replaced tail it becomes the first approach leg.
bool insertApproach(FlightPlan& plan, std::string_view approachId, ApproachKind kind,
                    std::string_view arrRunway, std::vector<Leg> approachLegs,
                    std::string* error = nullptr);
// Removes the approach and restores the plain destination leg.
void removeApproach(FlightPlan& plan, const NavDatabase& db);

// ---- distance, time, per-leg data -----------------------------------------------------------

// Great-circle length of leg `index` from the previous fix (0 for legs[0], CA/VA legs, holds' extra
// pattern; RF legs use arc length).
double legDistNm(const FlightPlan& plan, int index);
// Desired track of the leg, true degrees [0,360); nullopt for legs[0].
std::optional<float> legDtkTrueDeg(const FlightPlan& plan, int index);
// Magnetic DTK using the variation at the leg start; nullopt like above.
std::optional<float> legDtkMagDeg(const FlightPlan& plan, int index, const MagVarFn& magVar);

// Sum of leg lengths from legs[fromLeg] on; missed-approach legs only if includeMissed.
double planDistNm(const FlightPlan& plan, int fromLeg = 0, bool includeMissed = false);
// Distance still to fly: present position -> legs[activeLeg].to, plus later legs.
double distRemainingNm(const FlightPlan& plan, const LatLon& present, bool includeMissed = false);
// Time en route at a constant ground speed, seconds (0 for gsKt <= 0).
double eteS(double distNm, double gsKt);

struct LegSummary {
    int index = 0;
    std::string ident;
    LegType type = LegType::TF;
    std::optional<float> dtkTrueDeg, dtkMagDeg;
    double distNm = 0;
    AltConstraint alt;
    bool missedApproach = false;
};
// One row per leg for the plan summary page; dtkMagDeg is filled only if magVar is non-empty.
std::vector<LegSummary> planSummary(const FlightPlan& plan, const MagVarFn& magVar = nullptr);

}  // namespace fsim
