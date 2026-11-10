#pragma once
// N11a: flight-plan edits with the behaviour of a Garmin GTN / G1000 or an airliner FMS
// (NAVIGATION.md 2.4). Every edit is a pure function: it takes the plan, the edit and where the
// aircraft is, and returns the new plan, one short message for the HUD and the map, and whether
// it threw work away (the HUD then offers the undo). Navigator::applyEdit applies a result
// (setPlan, never clearPlan, so an engaged NAV keeps its leg) and keeps one level of undo.
//
// Edits happen on a button press, so these functions may allocate.

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/AircraftState.h"
#include "core/Geo.h"
#include "core/NavDatabase.h"
#include "core/NavTypes.h"

namespace fsim {

struct PlanEditContext {
    LatLon present;
    float trackTrueDeg = 0;  // ground track (heading below 30 kt)
    float gsKt = 0;
    bool airborne = true;
};
// The context for an aircraft state.
PlanEditContext planEditContext(const AircraftState& aircraft);
// "ROUTE REPLACED UNDONE" for Navigator::undo()'s label; empty for an empty label.
std::string undoneMessage(const std::string& undoLabel);

struct PlanEditResult {
    bool ok = false;
    FlightPlan plan;
    std::string message;         // "DIRECT TO SEA, THEN OLM"; the reason when !ok
    bool destructive = false;    // the HUD offers the undo for these
    const char* undoLabel = "";  // "ROUTE REPLACED", "PLAN CLEARED", "DIRECT TO", ...
};

// Rules 1-4: Direct-To `target`. A waypoint already in the plan (same ident within 1 NM, the first
// at or after the active leg) flies direct to it and on through the rest of the plan; the
// destination airport keeps the STAR and the approach; another airport replaces the plan; any
// other fix rejoins the plan at the first waypoint ahead of it ("DIRECT TO X, THEN Y").
PlanEditResult directTo(const FlightPlan& plan, const Waypoint& target, const PlanEditContext& ctx);
// Rule 8: Direct-To the active waypoint if it is still ahead (more than 0.5 NM to go along its
// leg, or the bearing to it within 90 degrees of the track), else the first waypoint ahead.
PlanEditResult directToNext(const FlightPlan& plan, const PlanEditContext& ctx);
// Rule 8: makes leg `legIndex` active without a direct-to (the navigator intercepts its course).
PlanEditResult activateLeg(const FlightPlan& plan, int legIndex, const PlanEditContext& ctx);
// Rule 7: deletes waypoint `legIndex`. The active one: the next waypoint becomes active, direct
// from the present position. Approach legs and the destination are not deleted one by one.
PlanEditResult deleteWaypoint(const FlightPlan& plan, int legIndex, const PlanEditContext& ctx);
// Rule 5: a new destination. The old destination's STAR, approach and missed approach always go.
// keepRoute keeps the origin, SID and enroute legs and ends them at the new destination (its
// runway threshold when `runwayDb` resolves, else the airport); otherwise present position direct.
PlanEditResult changeDestination(const FlightPlan& plan, const NavDatabase& db,
                                 std::string_view airportIdent, std::string_view runwayDb,
                                 bool keepRoute, const PlanEditContext& ctx);
// Rule 6: a new departure runway (DB style, "10R"). legs[0] becomes the runway start; a SID is
// replaced by `sidLegs` (assembled for the new runway) when given, else removed.
PlanEditResult changeDepartureRunway(const FlightPlan& plan, const NavDatabase& db,
                                     std::string_view runwayDb,
                                     std::optional<std::vector<Leg>> sidLegs);

}  // namespace fsim
