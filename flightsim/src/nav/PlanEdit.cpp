#include "nav/PlanEdit.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <utility>

#include "core/Units.h"
#include "nav/FlightPlan.h"

namespace fsim {
namespace {

constexpr double kSameFixNm = 1.0;        // a plan waypoint "is" the target within this distance
constexpr double kRejoinMaxTurnDeg = 90;  // rule 2: the turn at X onto the rejoin leg
constexpr double kAheadNm = 0.5;          // rule 8: still this far to go on its leg
constexpr double kAheadMaxOffDeg = 90;    // rule 8: or the bearing to it this close to the track

std::string upper(std::string_view s) {
    std::string r(s);
    for (char& c : r) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return r;
}

bool hasFix(const Leg& l) {
    return l.type != LegType::CA && l.type != LegType::VA;
}

double distNm(const LatLon& a, double latDeg, double lonDeg) {
    return mToNm(greatCircleDistM(a.latDeg, a.lonDeg, latDeg, lonDeg));
}

Leg pposLeg(const LatLon& present) {
    Leg l;
    l.type = LegType::IF;
    l.to = Waypoint{"PPOS", WaypointKind::Synthetic, present.latDeg, present.lonDeg};
    return l;
}

Leg dfLeg(const Waypoint& wp) {
    Leg l;
    l.type = LegType::DF;
    l.to = wp;
    return l;
}

// SID / STAR names stay only while some of their legs do (a Direct-To past them drops them).
void clearOrphanProcedureIds(FlightPlan& p) {
    bool sid = false, star = false;
    for (const Leg& l : p.legs) {
        sid = sid || l.source == LegSource::Sid;
        star = star || l.source == LegSource::Star;
    }
    if (!sid) {
        p.sidId.clear();
        p.sidTransition.clear();
    }
    if (!star) {
        p.starId.clear();
        p.starTransition.clear();
    }
}

int firstStarLeg(const FlightPlan& p) {
    for (size_t i = 0; i < p.legs.size(); ++i)
        if (p.legs[i].source == LegSource::Star) return static_cast<int>(i);
    return -1;
}

bool isDestLeg(const FlightPlan& p, int i) {
    if (p.approachStartLeg >= 0 || i != static_cast<int>(p.legs.size()) - 1 || i < 1) return false;
    const WaypointKind k = p.legs[static_cast<size_t>(i)].to.kind;
    return k == WaypointKind::Airport || k == WaypointKind::Runway;
}

PlanEditResult fail(const FlightPlan& plan, const char* why) {
    PlanEditResult r;
    r.plan = plan;
    r.message = why;
    return r;
}

PlanEditResult success(FlightPlan plan, std::string message, const char* undoLabel,
                       bool destructive = false) {
    PlanEditResult r;
    r.ok = true;
    clearOrphanProcedureIds(plan);
    r.plan = std::move(plan);
    r.message = std::move(message);
    r.undoLabel = undoLabel;
    r.destructive = destructive;
    return r;
}

// Rule 1 for a plan leg.
PlanEditResult directToPlanLeg(const FlightPlan& plan, int i, const PlanEditContext& ctx,
                               std::string_view shownIdent = {}) {
    FlightPlan p = plan;
    if (!fsim::directToLeg(p, i, ctx.present)) return fail(plan, "DIRECT TO FAILED");
    const std::string ident = shownIdent.empty() ? p.legs[static_cast<size_t>(p.activeLeg)].to.ident
                                                 : std::string(shownIdent);
    return success(std::move(p), "DIRECT TO " + ident, "DIRECT TO");
}

// The plan leg that is `target`: same ident within 1 NM, the first at or after the active leg.
int findPlanWaypoint(const FlightPlan& plan, const Waypoint& target) {
    const int n = static_cast<int>(plan.legs.size());
    const std::string id = upper(target.ident);
    auto matches = [&](int i) {
        const Leg& l = plan.legs[static_cast<size_t>(i)];
        if (!hasFix(l) || upper(l.to.ident) != id) return false;
        return distNm(LatLon{target.latDeg, target.lonDeg}, l.to.latDeg, l.to.lonDeg) <= kSameFixNm;
    };
    for (int i = std::max(1, plan.activeLeg); i < n; ++i)
        if (matches(i)) return i;
    for (int i = 1; i < n; ++i)
        if (matches(i)) return i;
    return -1;
}

double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
    return radToDeg(initialBearingTrueRad(lat1, lon1, lat2, lon2));
}

// Rule 2: X is not in the plan. Rejoin at the first waypoint from the active leg on whose turn at X
// is at most 90 degrees; none: the plan's tail (destination leg or approach).
PlanEditResult directToFix(const FlightPlan& plan, const Waypoint& x, const PlanEditContext& ctx) {
    const int n = static_cast<int>(plan.legs.size());
    FlightPlan p = plan;
    p.legs.clear();
    p.legs.push_back(pposLeg(ctx.present));
    p.legs.push_back(dfLeg(x));
    p.activeLeg = 1;
    p.approachStartLeg = -1;
    if (n < 2) return success(std::move(p), "DIRECT TO " + x.ident, "DIRECT TO");

    const bool inMissed = plan.activeLeg >= 0 && plan.activeLeg < n &&
                          plan.legs[static_cast<size_t>(plan.activeLeg)].missedApproach;
    const double toXDeg =
        distNm(ctx.present, x.latDeg, x.lonDeg) > 0.1
            ? bearingDeg(ctx.present.latDeg, ctx.present.lonDeg, x.latDeg, x.lonDeg)
            : static_cast<double>(ctx.trackTrueDeg);
    int join = -1;
    for (int j = std::max(1, plan.activeLeg); j < n; ++j) {
        const Leg& l = plan.legs[static_cast<size_t>(j)];
        if (!hasFix(l) || (l.missedApproach && !inMissed)) continue;
        if (distNm(LatLon{x.latDeg, x.lonDeg}, l.to.latDeg, l.to.lonDeg) < 0.1) continue;
        const double outDeg = bearingDeg(x.latDeg, x.lonDeg, l.to.latDeg, l.to.lonDeg);
        if (std::fabs(wrap180(outDeg - toXDeg)) <= kRejoinMaxTurnDeg) {
            join = j;
            break;
        }
    }
    if (join < 0) {
        const int tail = enrouteEndIndex(plan);
        if (tail >= n) return success(std::move(p), "DIRECT TO " + x.ident, "DIRECT TO");
        join = tail;
    }
    Leg y = plan.legs[static_cast<size_t>(join)];
    if (y.type == LegType::IF || y.type == LegType::DF || y.type == LegType::RF)
        y.type = LegType::TF;  // flown from X now
    const std::string yIdent = y.to.ident;
    p.legs.push_back(std::move(y));
    p.legs.insert(p.legs.end(), plan.legs.begin() + join + 1, plan.legs.end());
    if (plan.approachStartLeg >= 0)
        p.approachStartLeg = plan.approachStartLeg >= join ? 2 + (plan.approachStartLeg - join) : 2;
    return success(std::move(p), "DIRECT TO " + x.ident + ", THEN " + yIdent, "DIRECT TO");
}

}  // namespace

std::string undoneMessage(const std::string& undoLabel) {
    return undoLabel.empty() ? std::string() : undoLabel + " UNDONE";
}

PlanEditContext planEditContext(const AircraftState& a) {
    PlanEditContext c;
    c.present = LatLon{a.latDeg, a.lonDeg};
    c.trackTrueDeg =
        static_cast<float>(wrap360(radToDeg(a.gsKt > 30.0f ? a.trackTrueRad : a.headingTrueRad)));
    c.gsKt = a.gsKt;
    c.airborne = !a.wowAny;
    return c;
}

PlanEditResult directTo(const FlightPlan& plan, const Waypoint& target,
                        const PlanEditContext& ctx) {
    if (target.ident.empty()) return fail(plan, "DIRECT TO FAILED");
    const int inPlan = findPlanWaypoint(plan, target);
    if (inPlan >= 1) return directToPlanLeg(plan, inPlan, ctx);

    const std::string id = upper(target.ident);
    if (target.kind == WaypointKind::Airport && !plan.legs.empty() && id == upper(plan.destIdent)) {
        // Rule 3: the destination. Keep the STAR and the approach so they can still be flown.
        int keepFrom = firstStarLeg(plan);
        if (plan.approachStartLeg >= 0)
            keepFrom =
                keepFrom < 0 ? plan.approachStartLeg : std::min(keepFrom, plan.approachStartLeg);
        if (keepFrom < 0) {
            const int last = static_cast<int>(plan.legs.size()) - 1;
            if (isDestLeg(plan, last)) return directToPlanLeg(plan, last, ctx, id);
        } else {
            FlightPlan p = plan;
            p.legs.clear();
            p.legs.push_back(pposLeg(ctx.present));
            p.legs.push_back(dfLeg(target));
            p.legs.back().to.ident = id;  // the navigator matches it against destIdent exactly
            p.legs.insert(p.legs.end(), plan.legs.begin() + keepFrom, plan.legs.end());
            p.approachStartLeg =
                plan.approachStartLeg >= 0 ? 2 + (plan.approachStartLeg - keepFrom) : -1;
            p.activeLeg = 1;
            return success(std::move(p), "DIRECT TO " + id, "DIRECT TO");
        }
    }
    if (target.kind == WaypointKind::Airport) {
        // Rule 4: another airport replaces the plan.
        FlightPlan p;
        p.originIdent = plan.originIdent;
        p.depRunway = plan.depRunway;
        p.cruiseAltFt = plan.cruiseAltFt;
        p.destIdent = id;
        p.legs.push_back(pposLeg(ctx.present));
        p.legs.push_back(dfLeg(target));
        p.legs.back().to.ident = id;
        p.activeLeg = 1;
        const bool hadRoute = plan.legs.size() > 1;
        return success(std::move(p),
                       hadRoute ? "DIRECT TO " + id + ", ROUTE REPLACED" : "DIRECT TO " + id,
                       "ROUTE REPLACED", hadRoute);
    }
    return directToFix(plan, target, ctx);
}

PlanEditResult directToLegIndex(const FlightPlan& plan, int legIndex, const PlanEditContext& ctx) {
    if (legIndex < 1 || legIndex >= static_cast<int>(plan.legs.size()))
        return fail(plan, "DIRECT TO FAILED");
    return directToPlanLeg(plan, legIndex, ctx);
}

PlanEditResult directToNext(const FlightPlan& plan, const PlanEditContext& ctx) {
    const int n = static_cast<int>(plan.legs.size());
    const int a = plan.activeLeg;
    if (n < 2 || a < 1 || a >= n) return fail(plan, "NO ACTIVE WAYPOINT");
    const bool inMissed = plan.legs[static_cast<size_t>(a)].missedApproach;
    int lastFix = -1;
    for (int j = a; j < n; ++j) {
        const Leg& l = plan.legs[static_cast<size_t>(j)];
        if (!hasFix(l) || (l.missedApproach && !inMissed)) continue;
        lastFix = j;
        // Along-track distance still to go on leg j, from its previous fix (or the present
        // position when it has none).
        LatLon start = ctx.present;
        for (int k = j - 1; k >= 0; --k) {
            const Leg& pk = plan.legs[static_cast<size_t>(k)];
            if (!hasFix(pk)) continue;
            start = LatLon{pk.to.latDeg, pk.to.lonDeg};
            break;
        }
        const LatLon fix{l.to.latDeg, l.to.lonDeg};
        const double legNm = distNm(start, fix.latDeg, fix.lonDeg);
        const double toFixNm = distNm(ctx.present, fix.latDeg, fix.lonDeg);
        const double remNm =
            legNm < 0.05 ? toFixNm : legNm - mToNm(alongTrackM(ctx.present, start, fix));
        // Ahead: still more than 0.5 NM to go along its leg, or the aircraft is heading for it
        // (the bearing to it within 90 degrees of the track), e.g. vectored back toward it.
        const double brgDeg =
            bearingDeg(ctx.present.latDeg, ctx.present.lonDeg, fix.latDeg, fix.lonDeg);
        const bool headingFor =
            toFixNm > kAheadNm && std::fabs(wrap180(brgDeg - ctx.trackTrueDeg)) <= kAheadMaxOffDeg;
        if (remNm > kAheadNm || headingFor) return directToPlanLeg(plan, j, ctx);
    }
    if (lastFix >= 1) return directToPlanLeg(plan, lastFix, ctx);
    return fail(plan, "NO WAYPOINT AHEAD");
}

PlanEditResult activateLeg(const FlightPlan& plan, int legIndex, const PlanEditContext&) {
    if (legIndex < 1 || legIndex >= static_cast<int>(plan.legs.size()))
        return fail(plan, "CANNOT ACTIVATE");
    FlightPlan p = plan;
    p.activeLeg = legIndex;
    const std::string ident = p.legs[static_cast<size_t>(legIndex)].to.ident;
    return success(std::move(p), "ACTIVATE LEG " + ident, "ACTIVATE LEG");
}

PlanEditResult deleteWaypoint(const FlightPlan& plan, int legIndex, const PlanEditContext& ctx) {
    const int n = static_cast<int>(plan.legs.size());
    if (legIndex < 1 || legIndex >= n) return fail(plan, "CANNOT DELETE");
    if (plan.approachStartLeg >= 0 && legIndex >= plan.approachStartLeg)
        return fail(plan, "USE CHANGE APPROACH");
    if (isDestLeg(plan, legIndex)) return fail(plan, "USE NEW DESTINATION");
    const std::string ident = plan.legs[static_cast<size_t>(legIndex)].to.ident;
    if (legIndex == plan.activeLeg) {
        if (legIndex + 1 >= n) return fail(plan, "CANNOT DELETE");
        PlanEditResult r = directToPlanLeg(plan, legIndex + 1, ctx);
        if (r.ok) {
            r.message = ident + " DELETED, " + r.message;
            r.undoLabel = "DELETE";
        }
        return r;
    }
    FlightPlan p = plan;
    if (!fsim::deleteLeg(p, legIndex)) return fail(plan, "CANNOT DELETE");
    return success(std::move(p), ident + " DELETED", "DELETE");
}

PlanEditResult changeDestination(const FlightPlan& plan, const NavDatabase& db,
                                 std::string_view airportIdent, std::string_view runwayDb,
                                 bool keepRoute, const PlanEditContext& ctx) {
    const std::string id = upper(airportIdent);
    Waypoint dest;
    if (!planEndpointWaypoint(db, id, runwayDb, true, dest))
        return fail(plan, ("UNKNOWN AIRPORT " + id).c_str());
    FlightPlan p = plan;
    p.destIdent = id;
    p.arrRunway = dest.kind == WaypointKind::Runway ? std::string(runwayDb) : std::string();
    p.approachId.clear();
    p.approachKind = ApproachKind::None;
    p.approachTransition.clear();
    p.approachStartLeg = -1;
    p.starId.clear();
    p.starTransition.clear();
    Leg destLeg;
    destLeg.type = LegType::TF;
    destLeg.to = dest;
    const bool hadRoute = plan.legs.size() > 1;

    if (keepRoute && !plan.legs.empty()) {
        // Origin, SID and enroute legs stay; the old STAR, approach, missed and destination go.
        const int end = enrouteEndIndex(plan);
        p.legs.clear();
        int active = -1;
        for (int i = 0; i < end; ++i) {
            const Leg& l = plan.legs[static_cast<size_t>(i)];
            if (l.source == LegSource::Star) continue;
            if (i == plan.activeLeg) active = static_cast<int>(p.legs.size());
            p.legs.push_back(l);
        }
        p.legs.push_back(destLeg);
        if (p.legs.size() == 1) p.legs.insert(p.legs.begin(), pposLeg(ctx.present));
        const int last = static_cast<int>(p.legs.size()) - 1;
        if (active >= 1) {
            p.activeLeg = active;
        } else if (ctx.airborne) {
            if (!fsim::directToLeg(p, last, ctx.present))
                return fail(plan, "NEW DESTINATION FAILED");
        } else {
            p.activeLeg = std::min(1, last);
        }
        return success(std::move(p), "NEW DEST " + id + ", ROUTE KEPT", "ROUTE REPLACED", hadRoute);
    }
    // Clear the route: present position direct (on the ground: from the departure runway).
    p.legs.clear();
    if (!ctx.airborne && !plan.legs.empty())
        p.legs.push_back(plan.legs.front());
    else
        p.legs.push_back(pposLeg(ctx.present));
    if (ctx.airborne) destLeg.type = LegType::DF;
    p.legs.push_back(destLeg);
    p.activeLeg = 1;
    p.sidId.clear();
    p.sidTransition.clear();
    return success(std::move(p), "NEW DEST " + id + ", ROUTE CLEARED", "PLAN CLEARED", hadRoute);
}

PlanEditResult changeDepartureRunway(const FlightPlan& plan, const NavDatabase& db,
                                     std::string_view runwayDb,
                                     std::optional<std::vector<Leg>> sidLegs) {
    if (plan.legs.empty()) return fail(plan, "NO FLIGHT PLAN");
    Waypoint start;
    if (!planEndpointWaypoint(db, plan.originIdent, runwayDb, false, start) ||
        start.kind != WaypointKind::Runway)
        return fail(plan, "UNKNOWN RUNWAY");
    FlightPlan p = plan;
    Leg& origin = p.legs.front();
    origin.type = LegType::IF;
    origin.to = start;
    p.depRunway = std::string(runwayDb);
    std::string message = "DEPARTURE RWY " + std::string(runwayDb);
    if (!p.sidId.empty()) {
        const std::string sidId = p.sidId, sidTrans = p.sidTransition;
        if (sidLegs && !sidLegs->empty() &&
            insertSid(p, sidId, sidTrans, std::move(*sidLegs), nullptr)) {
            return success(std::move(p), message, "DEPARTURE RWY");
        }
        removeSid(p);
        if (p.activeLeg < 1 && p.legs.size() > 1) p.activeLeg = 1;
        return success(std::move(p), message + ", SID " + sidId + " REMOVED", "SID REMOVED", true);
    }
    return success(std::move(p), message, "DEPARTURE RWY");
}

}  // namespace fsim
