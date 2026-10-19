#pragma once
// ApproachBuilder (WP N5): produces fsim::Approach for ANY runway (NAVIGATION 3.1, 3.3):
//   I<rwy>   synthesized ILS (lateral/vertical guidance comes from the runway's Ils record,
//            real NASR or synthesized by N2; these are the GPS legs that bring you to the final)
//   L<rwy>   localizer-only (Ils::hasGs = false)
//   R<rwy>   synthesized RNAV (GPS)
//   VIS<rwy> visual
// and merges them with real procedures (N4, FAA CIFP) supplied through a provider callback.
//
// Geometry (true headings, runway threshold = landing threshold):
//   IF   10 NM out on the extended final course, alt = ceil100(thr + 3200 ft)
//   FAF   5 NM out, alt = thr + 1600 ft
//   MAP  runway threshold fix "RWxx" (fly-over), alt = thr + 50 ft
//   Entry: transitions[0] is "vectors" (straight to the IF); transitions[1], [2] are "IAF-L" and
//          "IAF-R", a fix 4 NM abeam the IF on the left / right of the inbound course. See
//          selectTransition() for the > 90 degree rule.
//   Missed: CA on the final course to thr + 1500 ft, DF to the IF, HM (hold) at the IF, inbound on
//          the final course, right turns, 1 minute legs.
//   Visual: FAF 3 NM / thr + 1000 ft, MAP, missed = CA only; no IAF transitions.

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/Geo.h"
#include "core/NavDatabase.h"
#include "core/NavTypes.h"

namespace fsim {

// Returns real (CIFP) approaches that serve `runway` of `airport`; may return an empty list.
// N4 plugs its loader in here. Default: none.
using RealApproachProvider =
    std::function<std::vector<Approach>(const Airport& airport, const Runway& runway)>;

// Synthesized approaches. buildIlsApproach needs runway.ils (returns an Approach with kind None
// and empty legs otherwise).
Approach buildIlsApproach(const Airport& airport, const Runway& runway);
Approach buildRnavApproach(const Airport& airport, const Runway& runway);
Approach buildVisualApproach(const Airport& airport, const Runway& runway);

// Real approaches first (provider order), then synthesized ILS (or LOC) if the runway has an Ils,
// RNAV, visual. A synthesized approach whose id equals a real one's is left out.
std::vector<Approach> availableApproaches(const Airport& airport, const Runway& runway,
                                          const RealApproachProvider& realProvider = {});
std::vector<Approach> availableApproaches(const NavDatabase& db, std::string_view airportIdent,
                                          std::string_view runwayIdent,
                                          const RealApproachProvider& realProvider = {});
// One approach by id ("I05", "R28L", "VIS09"); case-insensitive. Real ones win.
std::optional<Approach> findApproach(const NavDatabase& db, std::string_view airportIdent,
                                     std::string_view runwayIdent, std::string_view approachId,
                                     const RealApproachProvider& realProvider = {});

// Runway ident (DB style) embedded in an approach id: "I05" -> "05", "R28L-Y" -> "28L",
// "VIS09" -> "09". Empty if it cannot be parsed.
std::string runwayFromApproachId(std::string_view approachId);

// Default minimums (MSL) for an approach kind at a threshold elevation (NAVIGATION 3.1):
// ILS DA = thr + 200; RNAV / LOC MDA = thr + 400 rounded up to 10; VOR / NDB MDA = thr + 500
// rounded up to 10; visual 0 (none).
float defaultMinimumsFt(ApproachKind kind, float thrElevFt, bool* isDa = nullptr);

// Inbound final course, true degrees, arriving at the last final leg's fix (the MAP); falls back to the course of a CF/CA leg, then 0.
float inboundCourseTrueDeg(const Approach& approach);

// Transition for an aircraft arriving from `prev` (the last enroute waypoint or the present
// position). Vectors (0) unless the course prev -> first final fix differs from the inbound
// course by more than 90 degrees; then the transition whose first fix is nearest to `prev`
// (the IAF on the arrival side for synthesized approaches). Always 0 if there is only one.
int selectTransition(const Approach& approach, const LatLon& prev);

// transitions[transitionIndex].legs + finalLegs + missedLegs, ready for insertApproach().
// Missed legs get missedApproach = true. An out-of-range index means "vectors" (no transition).
std::vector<Leg> approachPlanLegs(const Approach& approach, int transitionIndex);

}  // namespace fsim
